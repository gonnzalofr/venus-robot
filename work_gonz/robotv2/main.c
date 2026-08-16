#define _POSIX_C_SOURCE 200809L

/*
 *  TU/e 5EID0 - Venus Project
 *  Pynq robot firmware -- ROBOT 1 (starts facing +y).
 *
 *  AUTONOMOUS / MQTT-ONLY (run-on-boot). With no operator and no ws_bridge at boot,
 *  the stdin command interface and the stdout UI JSON stream are gone: the robot
 *  powers straight into autonomous EXPLORE and reports everything (pose, status,
 *  mountains/tape, scanned cubes) to mission control + the partner robot over the
 *  MQTT/UART0 link only. Manual / target tele-op went with the command interface.
 *  Diagnostics go to the boot log (firmware status on stderr; the sensor bring-up
 *  also prints to stdout).
 *
 *  Sensors come from the shared module in sensors.c / sensors.h:
 *     - 3x VL53L0X ToF  (front/left/right distance)   -- working driver
 *     - 2x TCRT5000 IR  (black-tape detection)
 *     - 1x TCS3200      (scannable colour)             -- frequency counter
 *
 *  Robot 2 is main2.c, which re-#includes this file with a -y start heading.
 *  Build ONE main per robot (never link both):
 *     robot 1:  gcc main.c  sensors.c vl53l0x.c mqtt.c -o robot1 \
 *                   $(pkg-config --cflags --libs libpynq) -lm
 *     robot 2:  gcc main2.c sensors.c vl53l0x.c mqtt.c -o robot2  (same flags)
 *  (use whatever include/-L flags your libpynq install needs.)
 *
 *  robotv4/ is the byte-for-byte same firmware consolidated into a single main.c.
 */

#include <libpynq.h>
#include <stepper.h>   /* stepper_init/enable/steps/... (not pulled in by libpynq.h) */

#include "sensors.h"   /* tof_read / ir_read / color_read_label / sensors_init_all */
#include "vl53l0x.h"
#include "mqtt.h"      /* send_mqtt / recv_mqtt -- partner-robot link over UART0 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>   /* SIGINT/SIGTERM clean shutdown */

#define AS_SECONDS(time_ticks) ((time_ticks) / 1.0e8)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef ROBOT_ID
#define ROBOT_ID "r1"
#endif

/* Fixed start pose, defining the shared world frame for BOTH robots. The origin
 * (0,0) is where the robot is switched on; the two robots start right next to each
 * other so they share that origin. Their headings are FIXED by placement: robot 1
 * (this file) faces +y (theta = +pi/2); robot 2 (main2.c) faces -y (theta = -pi/2).
 * Because both then integrate odometry in the same world frame, their cube
 * coordinates line up directly -- that is what makes the UI map and the
 * cross-robot dedup work with no per-robot transform. */
#ifndef START_X_M
#define START_X_M 0.0
#endif
#ifndef START_Y_M
#define START_Y_M 0.0
#endif
#ifndef START_THETA_RAD
#define START_THETA_RAD (M_PI / 2.0)   /* +pi/2: robot 1 looks along +y */
#endif

/* ===================== Pin map -- ROBOT 2 ==========================
 * sensors.c owns the sensor pins:
 *     IIC0 (ToF) .... IO_AR_SCL / IO_AR_SDA
 *     ToF XSHUT ..... AR5 (B = middle), AR4 (C = bottom); A (top) tied high @0x29
 *     TCS3200 ....... AR6 (OUT), AR7 (S2), AR8 (S3), AR11 (S0), AR12 (S1)
 *     IR tape ....... AR9 (left), AR10 (right)
 *
 * Comms now go ONLY over the board UART0 (AR0/AR1 -> ESP32 -> MQTT broker); the
 * stdio UI link was removed for the run-on-boot build. Odometry is dead-reckoning,
 * so the pulse-counter pins below are unused; they default to AR2/AR13 (both free
 * on robot 2) only so the disabled odometry block compiles without clashing with
 * the sensor pins (AR4-AR12).
 * =================================================================== */
#define LEFT_PULSE_PIN  IO_AR2    /* unused (odometry off); free pin */
#define RIGHT_PULSE_PIN IO_AR13   /* unused (odometry off); free pin */

/* Encoders aren't wired, so default to dead-reckoning: pose is integrated from
 * the COMMANDED steps. Set this to 1 only if real wheel encoders are added on
 * free pins (and update LEFT/RIGHT_PULSE_PIN to match). */
#define USE_PULSECOUNTER_ODOMETRY 0

/* --------------------------- Calibration --------------------------- *
 * Matches the measured wheel library: 1600 steps/rev, 23.56 cm wheel
 * circumference (=> Ø 7.5 cm), 12 cm wheel track. With these, this file's
 * meters_to_steps / update_odometry produce the exact same motion + pose
 * as your move_and_track / update_location.                             */
#define STEPS_PER_REV 1600.0
#define MICROSTEP_FACTOR 1.0
#define WHEEL_DIAMETER_M 0.075   /* 23.56 cm circumference / pi */
#define WHEEL_BASE_M 0.12        /* track */

#define CELL_M 0.03              /* 3 cm/cell: small cube = 1 cell, big cube = 2x2 */
#define BACKUP_M 0.08
#define TAPE_REPORT_OFFSET_M 0.05
#define HEADING_TOLERANCE_RAD 0.18

/* The robot publishes its telemetry to mission control over the MQTT/UART0 link
 * so that PC can map our location/orientation and every object. Publishes are
 * rate-limited so the UART link to the ESP32 isn't flooded -- mission control
 * only needs a steady trickle. */
#define POSE_MQTT_MIN_MS    400  /* min gap between pose publishes over MQTT      */
#define STATUS_MQTT_MIN_MS  600  /* min gap between status publishes over MQTT    */
/* Objects (mountains/tape) are de-duplicated by position, not time, so two
 * DISTINCT objects seen close together are never dropped (see send_observation). */
#define PUB_OBJ_MAX         128  /* distinct object cells remembered for the MQTT publish */

/* Stepper pulse delay in ticks (larger = slower). Matches the wheel
 * library's tested speeds: 20000 driving, 25000 turning. */
#define STEPPER_PULSE_DELAY_TICKS 20000
#define STEPPER_TURN_PULSE_DELAY_TICKS 25000

#define LOOP_SLEEP_MS 20
#define POSE_REPORT_PERIOD_MS 500
#define STATUS_REPORT_PERIOD_MS 2000

#define MOUNTAIN_STOP_M 0.15

/* --------------------- Autopilot (frontier + SLAM) ----------------- *
 * Mirrors the UI auto-pilot: build an occupancy grid from the forward
 * sensors, drive to the nearest frontier (free cell next to unknown), and
 * react to what the sensors see:
 *   - top ToF  -> mountains (boundaries): map them; if one is in the way, avoid.
 *   - IR tape  -> boundary in the way: avoid.
 *   - bottom ToF -> small 3x3 rock; bottom+middle ToF -> large 6x6 (within ROCK_DETECT_M,
 *     MEASURED ~6-11 cm). Approach, read colour, report the sample, then re-plan.
 * Bottom/middle readings beyond ROCK_DETECT_M are floor/mountain, not a cube.
 *
 * Mission layer on top of that (see the located-cube registry below):
 *   - On boot it scans in place and approaches the first cube it sees.
 *   - Every NEW cube is logged locally and shared with the partner / mission
 *     control over MQTT; a cube already located (by us or the partner) is
 *     ignored instead of re-scanned.
 *   - Hitting a boundary/crater (black tape) backtracks, re-scans, and -- if
 *     nothing new is found -- roams on instead of freezing.
 *   - Once all six cubes are known, the robot stops; since finds are shared,
 *     both robots stop together.                                            */
#define GRID_W 40                /* 40 * 3cm = 1.2 m square map (~1m arena + margin) */
#define GRID_H 40
#define GRID_CX 20               /* world (0,0) -> grid centre */
#define GRID_CY 20
#define ROCK_DETECT_M       0.15 /* bottom/middle ToF range that counts as a cube. Field runs
                                  * show a cube in front reads ~11-13 cm on the working (middle)
                                  * beam and sat right on the old 0.12 boundary, so ToF noise made
                                  * the trip intermittent and it kept missing -- 0.15 adds margin.
                                  * (Bench debug_tof saw ~6-11 cm; a mountain reads farther and
                                  * lights the TOP beam instead.) */
#define ROCK_APPROACH_M     0.02 /* final gap: stop ~this close to the cube. The ToF cannot
                                  * read the last few cm (short-range floor ~6 cm), so once the
                                  * cube is within COLOR_ALIGN_DIST_M the final approach is driven
                                  * OPEN-LOOP (commanded distance), not by waiting for a near read. */
#define COLOR_ALIGN_DIST_M    0.10   /* final-approach trigger: once the cube reads within this, drive the rest OPEN-LOOP. Kept above the ToF ~6cm short-range floor so it triggers reliably */
#define COLOR_SENSOR_OFFSET_M 0.02   /* colour sensor sits this far RIGHT of the bottom ToF; strafe LEFT this much to align it onto the cube */
#define CUBE_CONFIRM_TOL_M    0.05   /* burst reads must agree within this (m) */
#define CUBE_CONFIRM_READS     6      /* reads taken to confirm a cube candidate */
#define CUBE_CONFIRM_MIN_HITS  2      /* ...of which this many must agree to commit */
#define MOUNTAIN_MAP_MAX_M  0.45 /* map mountains the top ToF sees up to this range  */
/* Top ToF detection band. The top sensor is angled down, so on EMPTY floor it
 * reads its far baseline (measured ~0.57 m); a real mountain blocks the beam and
 * reads CLOSER than that. So a reading is only a mountain inside [MIN,MAX], where
 * MAX sits safely BELOW the empty-floor baseline. Beyond MAX = floor -> ignored,
 * exploration keeps going. TUNE MAX to ~0.10 m below the top sensor's empty read. */
#define MOUNTAIN_DETECT_MIN_M 0.05
#define MOUNTAIN_DETECT_MAX_M 0.45
#define NAV_TARGET_REACHED_M 0.06 /* within this of the target -> reached, re-plan    */
#define SENSE_HORIZON_M     0.25 /* trust open floor this far ahead when sensors read
                                  * clear, so the frontier map keeps growing          */
/* Frontier exploration thresholds, in GRID CELLS (ported from the UI prototype's
 * 100x100 algorithm, scaled to this 40-cell map). These are what make it actually
 * explore instead of crawling one cell straight ahead. */
#define FRONTIER_MIN_CELLS     4   /* a frontier target must be >= this many cells away */
#define DIST_FROM_LAST_CELLS   3   /* reject frontiers within this of the last target   */
#define BLACKLIST_RADIUS_CELLS 3   /* a failed frontier bans this radius around it      */
#define BLACKLIST_TTL_STEPS  250   /* ...for this many autopilot steps                  */
#define EDGE_BUFFER_CELLS      2   /* stay this far off the grid edge                   */
#define MEMORY_SCAN_CELLS   12.0   /* how far the side-scan probes the occupancy map    */
#define STUCK_STEPS            8   /* no-progress steps before blacklisting + recovery  */
#define SCAN_INCREMENTS       72   /* in-place sweep steps per 360deg = 5deg/step (fine scan; slower sweep) */
#define SCAN_SETTLE_MS       120   /* dwell after each scan rotation so wobble decays    */
#define APPROACH_MAX_STEPS    30   /* creep ticks before giving up on a phantom cube     */
#define ROAM_DISTANCE_M     0.40   /* straight-line distance per roam leg (longer = explores more ground) */
#define AVOID_TURN_RAD  (M_PI/3.0) /* turn aside this much to go around an obstacle      */

/* Cube colour commit (AP_EVALUATE). Average several reads, and only trust a
 * reading whose classifier confidence clears the floor before it is logged +
 * broadcast (a misread permanently fills a registry slot and can trip the
 * six-cube stop early). After EVAL_MAX_RETRIES low-confidence re-reads we accept
 * the best effort so a real cube is never abandoned and the loop always ends. */
#define COLOR_EVAL_SAMPLES     8   /* averaged reads per cube evaluation                 */
#define COLOR_CONFIDENCE_MIN 0.50  /* [0..1] min classifier confidence to commit a cube  */
#define EVAL_MAX_RETRIES       2   /* low-confidence re-approaches before accepting       */

#define CELL_UNKNOWN  0
#define CELL_FREE     1
#define CELL_MOUNTAIN 2
#define CELL_BOUNDARY 3

/* ----------------------------- Types ------------------------------- */

typedef enum { MODE_STOP = 0, MODE_EXPLORE, MODE_TARGET, MODE_MANUAL } robot_mode_t;

/* Autopilot state machine (same shape as the UI's). */
typedef enum {
  AP_SCAN = 0, AP_APPROACH, AP_EVALUATE, AP_ROAM
} ap_state_t;

typedef enum {
  COLOR_NONE = 0, COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW, COLOR_WHITE
} color_label_t;

typedef struct { double x, y, theta; } pose_t;

/* Vertical ToF stack: bottom -> 3x3 rocks, middle -> 6x6 rocks, top -> mountains. */
typedef struct {
  bool bottom_valid, middle_valid, top_valid;
  double bottom_m, middle_m, top_m;
} distance_readings_t;

typedef struct {
  bool detected;
  color_label_t label;
  double confidence;
} color_detection_t;

typedef struct {
  pulsecounter_index_t index;
  uint32_t past_count;
  uint32_t past_time;
  bool initialized;
} pulse_tracker_t;

typedef struct {
  pose_t pose;
  robot_mode_t mode;
  bool has_target;
  double target_x, target_y;
  uint64_t last_pose_report_ms;
  uint64_t last_status_report_ms;
  pulse_tracker_t left_counter;
  pulse_tracker_t right_counter;
  int active_left_dir;
  int active_right_dir;
  ap_state_t ap_state;   /* autopilot state machine */
  int scan_rotations;    /* in-place sweep steps taken while looking for a frontier */
  int last_target_gx, last_target_gy; /* last frontier picked (cells) */
  bool has_last_target;
  int stuck_counter;     /* navigate ticks with no distance progress */
  double last_dist_to_target;
  int rock_is_large;     /* 1 = 6x6 (middle sensor), 0 = 3x3 (bottom)  */
  int approach_steps;    /* creep ticks in the current AP_APPROACH (anti-livelock) */
  int eval_attempts;     /* low-confidence colour re-reads on the current cube      */
  bool scan_has_best;    /* a candidate cube was seen during the current 360 sweep  */
  double scan_best_dist; /* ...its ToF range (m); we approach the CLOSEST one       */
  double scan_best_theta;/* ...the heading (world rad) it was seen at               */
  int scan_best_large;   /* ...whether it read as a large (6x6) cube                */
} robot_state_t;

static void poll_partner_mqtt(robot_state_t *s);  /* drained during blocking moves too */

/* ----------------------------- Utils ------------------------------- */

/* Set false by SIGINT/SIGTERM so the main loop exits and cleanup_hardware()
 * actually runs (motors disabled, IIC/UART/pynq released) instead of the process
 * being killed mid-move with the steppers left energised. Read in the main loop
 * and inside the blocking step loop (execute_step_command) for a prompt halt. */
static volatile sig_atomic_t g_keep_running = 1;
static void handle_signal(int sig) { (void)sig; g_keep_running = 0; }

static uint64_t monotonic_ms(void) {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
  }
  return (uint64_t)((double)clock() * 1000.0 / (double)CLOCKS_PER_SEC);
}

static double normalize_angle(double a) {
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double steps_to_meters(int steps) {
  const double revs = (double)steps / (STEPS_PER_REV * MICROSTEP_FACTOR);
  return revs * M_PI * WHEEL_DIAMETER_M;
}

static int meters_to_steps(double meters) {
  const double revs = meters / (M_PI * WHEEL_DIAMETER_M);
  return (int)lround(revs * STEPS_PER_REV * MICROSTEP_FACTOR);
}

static int sign_of_steps(int steps) {
  return (steps > 0) - (steps < 0);
}

static void update_odometry(robot_state_t *s, int left_steps, int right_steps) {
  const double dl = steps_to_meters(left_steps);
  const double dr = steps_to_meters(right_steps);
  const double dc = 0.5 * (dl + dr);
  const double dtheta = (dr - dl) / WHEEL_BASE_M;
  const double mid = s->pose.theta + 0.5 * dtheta;
  s->pose.x += dc * cos(mid);
  s->pose.y += dc * sin(mid);
  s->pose.theta = normalize_angle(s->pose.theta + dtheta);
}

/* ------------------------- Pulse-counter odometry ------------------ */

static void pulse_tracker_init(pulse_tracker_t *t, pulsecounter_index_t idx) {
  t->index = idx; t->past_count = 0; t->past_time = 0; t->initialized = false;
}

#if USE_PULSECOUNTER_ODOMETRY
static int pulse_tracker_read_delta(pulse_tracker_t *t) {
  uint32_t time = 0;
  const uint32_t count = pulsecounter_get_count(t->index, &time);
  if (!t->initialized) {
    t->past_count = count; t->past_time = time; t->initialized = true;
    return 0;
  }
  const uint32_t d = count - t->past_count;
  t->past_count = count; t->past_time = time;
  return (int)d;
}
#endif

static void sync_odometry_from_pulsecounters(robot_state_t *s) {
#if USE_PULSECOUNTER_ODOMETRY
  const int ld = pulse_tracker_read_delta(&s->left_counter);
  const int rd = pulse_tracker_read_delta(&s->right_counter);
  if (ld == 0 && rd == 0) return;
  update_odometry(s, s->active_left_dir * ld, s->active_right_dir * rd);
#else
  (void)s;
#endif
}

/* ------------------------ Telemetry TX (MQTT-only) ---------------- *
 * All telemetry (pose / status / observations / scanned cubes) leaves the robot
 * over the MQTT/UART0 link only -- there is no stdout UI stream any more.
 * Diagnostics still go to stderr (-> the boot log).                      */

static const char *mode_name(robot_mode_t m) {
  switch (m) {
    case MODE_EXPLORE: return "explore";
    case MODE_TARGET:  return "target";
    case MODE_MANUAL:  return "manual";
    default:           return "stop";
  }
}

/* Capitalised names for the partner-robot MQTT schema (Red/Blue/Green/Yellow). */
static const char *color_label_to_mqtt(color_label_t c) {
  switch (c) {
    case COLOR_RED:    return "Red";
    case COLOR_GREEN:  return "Green";
    case COLOR_BLUE:   return "Blue";
    case COLOR_YELLOW: return "Yellow";
    case COLOR_WHITE:  return "White";
    default:           return "Unknown";
  }
}

/* True (and stamps *last_ms) at most once per min_ms -- throttles MQTT publishes
 * so the UART link to the ESP32 isn't flooded. */
static bool mqtt_rate_ok(uint64_t *last_ms, uint64_t min_ms) {
  const uint64_t now = monotonic_ms();
  if (now - *last_ms < min_ms) return false;
  *last_ms = now;
  return true;
}

static void send_pose(const robot_state_t *s) {

  /* Relay location + orientation to mission control over MQTT (rate-limited).
   * x/y in metres, theta in rad -- mission_control scales pose m->cm. */
  static uint64_t last = 0;
  if (mqtt_rate_ok(&last, POSE_MQTT_MIN_MS)) {
    char m[160];
    snprintf(m, sizeof(m),
             "{\"type\":\"pose\",\"robot\":\"%s\",\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f}",
             ROBOT_ID, s->pose.x, s->pose.y, s->pose.theta);
    mqtt_publish_json(m);
  }
}

static void send_status(const robot_state_t *s, const char *detail) {

  /* Publish mode/detail to mission control over MQTT (rate-limited) for its panel. */
  static uint64_t last = 0;
  if (mqtt_rate_ok(&last, STATUS_MQTT_MIN_MS)) {
    char m[224];
    snprintf(m, sizeof(m),
             "{\"type\":\"status\",\"robot\":\"%s\",\"mode\":\"%s\",\"detail\":\"%s\"}",
             ROBOT_ID, mode_name(s->mode), detail);
    mqtt_publish_json(m);
  }

  /* Every transition also publishes a fresh pose over MQTT (rate-limited) so
   * mission control's dead-reckoning trail stays current at turns/stops. */
  send_pose(s);
}

/* Objects already mirrored to mission control, keyed by kind + a ~5 cm cell, so
 * each distinct object is published exactly ONCE (a time gate would drop two
 * distinct objects seen in the same window, and re-spam one the robot lingers by).
 * Ring buffer: if more than PUB_OBJ_MAX distinct cells appear, the oldest is
 * forgotten and may be re-sent later (harmless -- mission control keys by pos). */
static struct { char k; int qx, qy; bool used; } g_pub_obj[PUB_OBJ_MAX];
static int g_pub_obj_head = 0;

static bool obj_publish_once(const char *kind, double x, double y) {
  const int qx = (int)lround(x * 20.0);   /* metres -> 5 cm cells */
  const int qy = (int)lround(y * 20.0);
  const char k = kind[0];
  for (int i = 0; i < PUB_OBJ_MAX; ++i) {
    if (g_pub_obj[i].used && g_pub_obj[i].k == k &&
        g_pub_obj[i].qx == qx && g_pub_obj[i].qy == qy) {
      return false;                         /* already mirrored this object */
    }
  }
  g_pub_obj[g_pub_obj_head].k = k;
  g_pub_obj[g_pub_obj_head].qx = qx;
  g_pub_obj[g_pub_obj_head].qy = qy;
  g_pub_obj[g_pub_obj_head].used = true;
  g_pub_obj_head = (g_pub_obj_head + 1) % PUB_OBJ_MAX;
  return true;
}

static void send_observation(const char *kind, double x, double y, double conf) {
  (void)conf;   /* confidence is UI-only; the MQTT object schema omits it */

  /* Relay each distinct object to mission control over MQTT so terrain
   * (mountains, boundaries/craters) maps too. The MQTT "type" is the kind, and
   * x/y are in cm to match mission_control's object/cube handling. */
  if (obj_publish_once(kind, x, y)) {
    char m[160];
    snprintf(m, sizeof(m),
             "{\"type\":\"%s\",\"robot\":\"%s\",\"x\":%d,\"y\":%d}",
             kind, ROBOT_ID, (int)lround(x * 100.0), (int)lround(y * 100.0));
    mqtt_publish_json(m);
  }
}

static void project_from_pose(const pose_t *p, double range_m, double bearing,
                              double *x, double *y) {
  const double wb = p->theta + bearing;
  *x = p->x + range_m * cos(wb);
  *y = p->y + range_m * sin(wb);
}

/* ----------------------- Located-cube registry --------------------- *
 * The mission is to find SIX cubes. We keep a local list of every cube located
 * so far -- both the ones THIS robot scans and the ones the partner reports over
 * MQTT (folded in by poll_partner_mqtt). It does two jobs:
 *   1. De-duplication: a cube whose position is already in the list is ignored,
 *      per the spec ("if the robot sees a cube already located by either itself
 *      or the other robot, it should be ignored").
 *   2. Completion: once SIX distinct cubes are known, the robot stops -- and
 *      because every find is shared over MQTT, BOTH robots independently reach
 *      six and stop together.
 * Positions are compared in metres; two finds within CUBE_DEDUP_RADIUS_M are
 * treated as the same cube (covers colour-sensor jitter + odometry drift). Both
 * robots run odometry in ONE shared world frame -- they start adjacent at (0,0)
 * with fixed opposite headings (r1 faces +y, r2 faces -y; see START_THETA_RAD and
 * main2.c) -- so the partner's x/y are already in our frame and register directly. */
#define MAX_CUBES           16
#define TARGET_CUBE_COUNT    6
#define CUBE_DEDUP_RADIUS_M  0.12   /* finds within 12 cm are the same cube */

static struct { double x, y; int hits; bool used; } g_cubes[MAX_CUBES];
static int g_cube_count = 0;

/* Index of the CLOSEST known cube within CUBE_DEDUP_RADIUS_M of (x,y), else -1. */
static int cube_nearest(double x, double y) {
  int best = -1;
  double best_d = CUBE_DEDUP_RADIUS_M;
  for (int i = 0; i < MAX_CUBES; ++i) {
    if (!g_cubes[i].used) continue;
    const double d = hypot(x - g_cubes[i].x, y - g_cubes[i].y);
    if (d <= best_d) { best_d = d; best = i; }
  }
  return best;
}

static bool cube_is_known(double x, double y) { return cube_nearest(x, y) >= 0; }

/* Add a cube at (x,y) unless it is already known. A repeat find (ours or the
 * partner's) instead refines the stored centre by running average -- which
 * smooths the inter-robot frame/odometry bias on the shared position. Returns
 * true ONLY for a genuinely new cube, so the caller reports + counts it once. */
static bool cube_register(double x, double y) {
  const int i = cube_nearest(x, y);
  if (i >= 0) {                                /* already known -> refine centre */
    g_cubes[i].x = (g_cubes[i].x * g_cubes[i].hits + x) / (g_cubes[i].hits + 1);
    g_cubes[i].y = (g_cubes[i].y * g_cubes[i].hits + y) / (g_cubes[i].hits + 1);
    ++g_cubes[i].hits;
    return false;
  }
  for (int j = 0; j < MAX_CUBES; ++j) {
    if (!g_cubes[j].used) {
      g_cubes[j].x = x; g_cubes[j].y = y; g_cubes[j].hits = 1; g_cubes[j].used = true;
      ++g_cube_count;
      return true;
    }
  }
  return false;   /* registry full (won't happen for six cubes) */
}

/* Halt once all six cubes are accounted for. Called after every new find --
 * local or from the partner -- so the robots stop together. */
static void stop_if_mission_complete(robot_state_t *s) {
  if (g_cube_count < TARGET_CUBE_COUNT) return;
  s->mode = MODE_STOP;
  stepper_reset();
  send_status(s, "mission_complete_all_cubes_found");
}

/* ----------------------- Sensor wrappers (real HW) ----------------- */

static distance_readings_t read_distance_sensors(void) {
  uint16_t b = 0, m = 0, t = 0;
  tof_read(&b, &m, &t);                 /* mm; 0 == invalid/out-of-range */
  distance_readings_t d = {0};
  d.bottom_valid = (b != 0); d.bottom_m = (double)b / 1000.0;  /* 3x3 rocks */
  d.middle_valid = (m != 0); d.middle_m = (double)m / 1000.0;  /* 6x6 rocks */
  d.top_valid    = (t != 0); d.top_m    = (double)t / 1000.0;  /* mountains */
  return d;
}

static bool tape_detected_now(void) {
  bool left = false, right = false;
  ir_read(&left, &right);
  return left || right;
}

static color_detection_t color_det_from_label(scan_color_t c, float conf) {
  color_detection_t det = {0};
  switch (c) {
    case SCAN_COLOR_RED:    det.label = COLOR_RED;    det.detected = true;  break;
    case SCAN_COLOR_GREEN:  det.label = COLOR_GREEN;  det.detected = true;  break;
    case SCAN_COLOR_BLUE:   det.label = COLOR_BLUE;   det.detected = true;  break;
    case SCAN_COLOR_YELLOW: det.label = COLOR_YELLOW; det.detected = true;  break;
    case SCAN_COLOR_WHITE:  det.label = COLOR_WHITE;  det.detected = true;  break;
    /* BLACK / NONE -> treat as background (floor/tape), not a scannable rock. */
    default:                det.label = COLOR_NONE;   det.detected = false; break;
  }
  det.confidence = (double)conf;
  return det;
}

/* Averaged read -- used only at the cube-commit point (AP_EVALUATE), where the
 * extra few hundred ms while stationary buys a far more reliable classification.
 * Built on the existing public sensor API (color_read + color_classify) so the
 * sensor driver itself needs no change: average several raw reads, discarding the
 * all-zero (timed-out) ones, then classify the mean. */
static color_detection_t read_color_detection_averaged(int samples) {
  uint64_t rs = 0, gs = 0, bs = 0;
  int valid = 0;
  if (samples < 1) samples = 1;
  for (int i = 0; i < samples; ++i) {
    uint32_t r = 0, g = 0, b = 0;
    color_read(&r, &g, &b);
    if (r == 0 && g == 0 && b == 0) continue;   /* timed-out read -> skip */
    rs += r; gs += g; bs += b; ++valid;
  }
  if (valid == 0) return color_det_from_label(SCAN_COLOR_NONE, 0.0f);
  float conf = 0.0f;
  const scan_color_t c = color_classify((uint32_t)(rs / valid), (uint32_t)(gs / valid),
                                        (uint32_t)(bs / valid), &conf);
  return color_det_from_label(c, conf);
}

/* --------------------------- Hazard helpers ------------------------ */

/* ------------------------------ Motion ----------------------------- */

static bool execute_step_command(robot_state_t *s, int left_steps, int right_steps,
                                 bool safety_enabled, uint16_t pulse_delay) {
  s->active_left_dir  = sign_of_steps(left_steps);
  s->active_right_dir = sign_of_steps(right_steps);
  stepper_set_speed(pulse_delay, pulse_delay);
  stepper_steps((int16_t)left_steps, (int16_t)right_steps);

  while (!stepper_steps_done()) {
    poll_partner_mqtt(s);   /* keep draining the partner link DURING the move so its
                             * relayed cube frames aren't lost to a UART FIFO overflow
                             * (a step can block for seconds); may also trip the
                             * six-cube stop, caught by the MODE_STOP check below. */
    sync_odometry_from_pulsecounters(s);

    if (!g_keep_running || s->mode == MODE_STOP) {   /* shutdown or stop: halt now */
      stepper_reset();
      sync_odometry_from_pulsecounters(s);
      s->active_left_dir = s->active_right_dir = 0;
      return false;
    }

    if (safety_enabled) {
      /* Tape = boundary: stop mid-move. (Mountains are handled at the state
       * level by the autopilot, not by aborting a step.) */
      if (tape_detected_now()) {
        stepper_reset();
        sync_odometry_from_pulsecounters(s);
        double x = 0.0, y = 0.0;
        project_from_pose(&s->pose, TAPE_REPORT_OFFSET_M, 0.0, &x, &y);
        send_observation("black_tape", x, y, 1.0);
        send_status(s, "tape_avoidance");
        s->active_left_dir = s->active_right_dir = 0;
        return false;
      }
    }
    sleep_msec(LOOP_SLEEP_MS);
  }

  sync_odometry_from_pulsecounters(s);
#if !USE_PULSECOUNTER_ODOMETRY
  /* Dead-reckoning: integrate pose from the commanded steps (no encoders). */
  update_odometry(s, left_steps, right_steps);
#endif
  s->active_left_dir = s->active_right_dir = 0;
  return true;
}

static bool move_forward_m(robot_state_t *s, double meters, bool safety) {
  const int steps = meters_to_steps(meters);
  return execute_step_command(s, steps, steps, safety, STEPPER_PULSE_DELAY_TICKS);
}

static bool rotate_rad(robot_state_t *s, double radians, bool safety) {
  const double wheel_m = 0.5 * WHEEL_BASE_M * radians;
  const int l = meters_to_steps(-wheel_m);
  const int r = meters_to_steps(wheel_m);
  return execute_step_command(s, l, r, safety, STEPPER_TURN_PULSE_DELAY_TICKS);
}


/* -------------------------- Reporting / scan ----------------------- */

static void report_periodic(robot_state_t *s) {
  const uint64_t now = monotonic_ms();
  if (now - s->last_pose_report_ms >= POSE_REPORT_PERIOD_MS) {
    send_pose(s);
    s->last_pose_report_ms = now;
  }
  if (now - s->last_status_report_ms >= STATUS_REPORT_PERIOD_MS) {
    send_status(s, "alive");
    s->last_status_report_ms = now;
  }
}

/* Poll the partner robot's MQTT link and fold any rock it reports into our shared
 * cube registry (dedup + the six-cube finish). cm -> m. */
static void poll_partner_mqtt(robot_state_t *s) {
  int px = 0, py = 0, psize = 0;
  float ptemp = 0.0f;
  char pcolor[32];
  pcolor[0] = '\0';

  /* Drain EVERY complete partner frame queued this tick. After a multi-second
   * blocking move several can be buffered; recv_mqtt parses one per call and
   * returns false once none remain (so finds aren't trickled out one-per-loop). */
  while (recv_mqtt(&px, &py, &psize, &ptemp, pcolor)) {
    if (pcolor[0] != '\0' && strcmp(pcolor, "Unknown") != 0) {
      /* Shared world frame (see the cube-registry note + START_THETA_RAD): the
       * partner's x/y (cm) are already in our frame, so register them directly. */
      const double wx = (double)px / 100.0, wy = (double)py / 100.0;

      /* Fold into the shared registry: this is how a cube the partner located is
       * ignored if we later see it, and how we count toward the six-cube finish.
       * Only a genuinely new cube advances the count. */
      if (cube_register(wx, wy)) stop_if_mission_complete(s);
    }
    pcolor[0] = '\0';   /* reset so a frame with no colour field can't reuse the last */
  }
}

/* Closest cube within ROCK_DETECT_M (-1 if none). Sizing follows the vertical beam
 * geometry MEASURED with debug_tof: a 3x3 cube only trips the BOTTOM beam, while a 6x6
 * is tall enough to ALSO trip the MIDDLE beam -- so "middle in range" => large. (A
 * mountain reads past ROCK_DETECT_M on bottom/middle and shows up on the TOP beam
 * instead, so it is handled by the mountain logic, not mistaken for a cube here.) */
static double ap_rock_dist(const distance_readings_t *d, int *is_large) {
  const bool b = d->bottom_valid && d->bottom_m > 0.0 && d->bottom_m <= ROCK_DETECT_M;
  const bool m = d->middle_valid && d->middle_m > 0.0 && d->middle_m <= ROCK_DETECT_M;
  const bool t = d->top_valid    && d->top_m    > 0.0 && d->top_m    <= ROCK_DETECT_M;
  /* A cube is seen by the BOTTOM and/or MIDDLE beam -- accept EITHER. On this rover
   * the bottom beam grazes the floor and often MISSES a cube the middle beam sees
   * cleanly (observed: bottom 8.19 m while middle reads ~9 cm on a cube right in
   * front), so requiring the bottom beam made it ignore real cubes. */
  if (!b && !m) return -1.0;
  /* The TOP beam (8 cm) is too high for a 3-6 cm cube, so a cube leaves it reading
   * the far background. If the top beam is ALSO blocked this close, it's a ~10 cm
   * mountain, not a cube -> reject. */
  if (t) return -1.0;
  if (is_large) *is_large = (b && m) ? 1 : 0;   /* both low beams blocked -> 6x6, just one -> 3x3 */
  if (b && m) return fmin(d->bottom_m, d->middle_m);
  return b ? d->bottom_m : d->middle_m;
}

/* Report the objects we sense to the map / mission control (localization). The
 * simplified roamer does NOT avoid mountains -- it only logs them so they appear
 * on the map -- and logs boundaries/craters too. Cubes are reported separately
 * when scanned (AP_EVALUATE). */
static void report_objects(robot_state_t *s, const distance_readings_t *d, bool tape) {
  if (d->top_valid && d->top_m >= MOUNTAIN_DETECT_MIN_M && d->top_m <= MOUNTAIN_DETECT_MAX_M) {
    double hx = 0.0, hy = 0.0;
    project_from_pose(&s->pose, d->top_m, 0.0, &hx, &hy);
    send_observation("mountain", hx, hy, 0.85);
  }
  if (tape) {
    double bx = 0.0, by = 0.0;
    project_from_pose(&s->pose, TAPE_REPORT_OFFSET_M, 0.0, &bx, &by);
    send_observation("black_tape", bx, by, 1.0);   /* boundary or crater */
  }
}

/* Read the cube's colour at the current spot. This one line is the ONLY per-robot
 * difference in the autopilot below: robotv2 averages several reads for a more
 * reliable classification (robot/ uses a single read). */
static color_detection_t read_cube_color(void) {
  return read_color_detection_averaged(COLOR_EVAL_SAMPLES);
}

/* The autopilot, run once per main-loop iteration in MODE_EXPLORE.
 *
 * State-machine roamer focused on object classification + localization:
 *   SCAN     - rotate 360deg in place; break off to the FIRST unscanned cube
 *              seen, remembering its position as the approach target.
 *   APPROACH - head to that cube while AVOIDING collisions: only cubes trigger
 *              this state (mountains never do), but a mountain or crater in the
 *              way is gone AROUND, keeping the cube as the goal.
 *   EVALUATE - read its colour, log + report colour/size/temperature, re-scan.
 *   ROAM     - a full sweep found nothing: turn a RANDOM direction, drive a set
 *              distance, then re-scan.
 * Evasive interrupt (any state): a boundary or crater (black tape) -> back off,
 * turn away a randomised angle, ABANDON any approach, and re-enter SCAN (which
 * re-scans for objects and roams if none). Runs until the six-cube stop. */
static void autopilot_step(robot_state_t *s) {
  const distance_readings_t d = read_distance_sensors();
  const bool tape = tape_detected_now();

  report_objects(s, &d, tape);   /* localise mountains + boundaries/craters */

  if (tape) {
    /* Boundary or crater (black tape): GO BACK. Reverse straight off the tape and
     * turn ~180deg to face the way we came (back toward the interior), then re-scan.
     * We deliberately do NOT drive forward here -- the old forward "step clear" kept
     * shoving the robot back onto the border and it got stuck there. If we're still
     * on tape next tick this interrupt fires again and reverses further until clear.
     * Any approach is dropped (a cube is never across a boundary; craters are avoided). */
    move_forward_m(s, -BACKUP_M, false);                            /* reverse straight off the tape */
    const double jitter = (((double)rand() / (double)RAND_MAX) - 0.5) * (M_PI / 3.0);  /* +/-30 deg */
    rotate_rad(s, M_PI + jitter, false);                            /* ~180deg -> face back to the interior */
    s->scan_rotations = 0;
    s->ap_state = AP_SCAN;                                          /* re-scan; nothing new -> roam */
    send_status(s, "evade_tape");
    return;
  }

  switch (s->ap_state) {
    case AP_SCAN: {
      /* 360deg in-place sweep; the FIRST not-yet-known cube seen wins. Remember
       * its position so APPROACH can steer back to it after dodging an obstacle. */
      int is_large = 0;
      const double rock0 = ap_rock_dist(&d, &is_large);
      if (rock0 > 0.0) {
        /* Confirm with a short BURST of reads, voting rather than demanding two in a
         * row. The ToF drops out intermittently -- especially the BOTTOM beam that a
         * 3 cm cube trips ALONE -- so a single re-read often lands on a dropout and
         * wrongly rejects a real (small) cube. Count reads that see a cube near the
         * same distance; commit once at least CUBE_CONFIRM_MIN_HITS agree. */
        int hits = 1, large_votes = is_large;
        double dist_sum = rock0;
        for (int i = 1; i < CUBE_CONFIRM_READS; ++i) {
          const distance_readings_t di = read_distance_sensors();
          int il = 0;
          const double ri = ap_rock_dist(&di, &il);
          if (ri > 0.0 && fabs(ri - dist_sum / (double)hits) <= CUBE_CONFIRM_TOL_M) {
            ++hits; dist_sum += ri; large_votes += il;
          }
        }
        if (hits >= CUBE_CONFIRM_MIN_HITS) {
          const double rock = dist_sum / (double)hits;
          double rx = 0.0, ry = 0.0;
          project_from_pose(&s->pose, rock, 0.0, &rx, &ry);
          if (!cube_is_known(rx, ry)) {
            s->rock_is_large = (large_votes * 2 >= hits);   /* >=half saw the middle beam -> 6cm */
            s->target_x = rx;
            s->target_y = ry;
            s->scan_rotations = 0;
            s->approach_steps = 0;
            s->ap_state = AP_APPROACH;
            send_status(s, "cube_detected");
            break;
          }
        }
      }
      if (s->scan_rotations < SCAN_INCREMENTS) {
        rotate_rad(s, 2.0 * M_PI / (double)SCAN_INCREMENTS, false);
        s->scan_rotations++;
      } else {
        s->scan_rotations = 0;
        s->ap_state = AP_ROAM;               /* full sweep, nothing new -> roam */
      }
      break;
    }

    case AP_APPROACH: {
      if (++s->approach_steps > APPROACH_MAX_STEPS) {   /* can't reach it -> give up */
        s->ap_state = AP_SCAN;
        s->scan_rotations = 0;
        send_status(s, "approach_timeout");
        break;
      }

      const double rock = ap_rock_dist(&d, NULL);

      /* Mountain in the way -> go AROUND it (avoid the collision), keep the cube.
       * Only if the mountain is NEARER than the cube (i.e. actually between us and
       * it); a mountain just behind the cube must not abort a reachable approach. */
      const bool mountain_ahead = d.top_valid &&
          d.top_m >= MOUNTAIN_DETECT_MIN_M && d.top_m < MOUNTAIN_STOP_M &&
          (rock < 0.0 || d.top_m < rock);
      if (mountain_ahead) {
        move_forward_m(s, -BACKUP_M, false);
        rotate_rad(s, AVOID_TURN_RAD, false);
        move_forward_m(s, CELL_M * 2.0, true);          /* sidestep; safety: don't cross tape */
        send_status(s, "approach_avoid_mountain");
        break;                                          /* stay AP_APPROACH; re-aim next tick */
      }

      /* How far still to go: prefer the live bottom ToF, fall back to the
       * remembered cube position when the (noisy) beam has dropped out. */
      const double tdx = s->target_x - s->pose.x, tdy = s->target_y - s->pose.y;
      const double to_target = hypot(tdx, tdy);
      const double near_m = (rock > 0.0) ? rock : to_target;

      if (near_m <= COLOR_ALIGN_DIST_M) {
        /* Close enough -> drive the last gap straight in and evaluate. */
        if (near_m > ROCK_APPROACH_M) {
          move_forward_m(s, near_m - ROCK_APPROACH_M, true);
        }
        s->ap_state = AP_EVALUATE;
        break;
      }

      if (rock > 0.0) {
        /* Cube in view straight ahead -> creep toward it. */
        move_forward_m(s, fmin(0.03, rock - ROCK_APPROACH_M), true);
      } else {
        /* Cube not in view (e.g. just dodged an obstacle) -> steer back toward
         * where we saw it. */
        const double err = normalize_angle(atan2(tdy, tdx) - s->pose.theta);
        if (fabs(err) > HEADING_TOLERANCE_RAD) rotate_rad(s, err, false);
        else move_forward_m(s, fmin(0.03, to_target), true);
      }
      break;
    }

    case AP_EVALUATE: {
      /* Register on the strength of the (burst-confirmed) DETECTION, not the colour
       * read -- the colour sensor is the weak link, so we never drop a confirmed cube
       * just because the colour is uncertain. We still read + report the best colour
       * we can ("Unknown" if none); position + size + temperature go out regardless. */
      const color_detection_t color = read_cube_color();
      double rx = 0.0, ry = 0.0;
      project_from_pose(&s->pose, ROCK_APPROACH_M, 0.0, &rx, &ry);
      if (cube_register(rx, ry)) {                              /* NEW cube -> log + report */
        const float temp_c = temp_read_celsius();               /* environment temp at the cube */
        send_mqtt((int)lround(rx * 100.0), (int)lround(ry * 100.0),       /* colour (best-effort) + size + temp */
                  color_label_to_mqtt(color.label),
                  s->rock_is_large ? 6 : 3, temp_c);
        send_status(s, "cube_located");
        stop_if_mission_complete(s);                            /* sixth cube -> stop */
      } else {
        send_status(s, "cube_duplicate_ignored");               /* already known -> ignore */
      }
      if (s->mode == MODE_STOP) break;                          /* mission complete: halt */
      move_forward_m(s, -0.05, false);                          /* back off the cube */
      s->scan_rotations = 0;
      s->ap_state = AP_SCAN;
      break;
    }

    case AP_ROAM: {
      /* Pick a RANDOM forward direction, drive a set distance, then re-scan. */
      const double ang = ((double)rand() / (double)RAND_MAX) * 2.0 * M_PI - M_PI;
      rotate_rad(s, ang, false);
      send_status(s, "roaming");
      move_forward_m(s, ROAM_DISTANCE_M, true);
      s->scan_rotations = 0;
      s->ap_state = AP_SCAN;
      break;
    }
  }
}

/* ----------------------------- Hardware ---------------------------- */

static void setup_hardware(void) {
  /* Headless / run-on-boot: no operator, no ws_bridge, no stdin. The only comms
   * link is MQTT over UART0 (brought up below); diagnostics go to the boot log. */
  pynq_init();
  fprintf(stderr, "[robot] pynq_init done\n");

  /* Steppers FIRST. stepper_init() resets the switchbox, so it must run BEFORE
   * the sensors route their pins -- otherwise it wipes the IR/colour GPIO
   * routing and the IR floats HIGH (reads tape forever; works in sensortest,
   * which never inits the stepper). sensors_init_all() only sets its own pins
   * and does not reset the whole switchbox, so the motor routing survives it. */
  stepper_init();
  stepper_enable();
  stepper_set_speed(STEPPER_PULSE_DELAY_TICKS, STEPPER_PULSE_DELAY_TICKS);
  fprintf(stderr, "[robot] steppers up\n");

  /* Sensor module brings up GPIO, IIC0 + the three ToF sensors (XSHUT on
   * AR4/AR5), the IR tape pair (AR9/AR10), and the TCS3200 colour sensor
   * (AR6/AR7/AR8). Runs after the stepper so its pin routing wins. */
  const int serr = sensors_init_all();
  if (serr != 0) {
    fprintf(stderr, "[robot] sensors_init_all returned 0x%X (some sensor offline)\n", serr);
  }
  fprintf(stderr, "[robot] sensors up -- autonomous MQTT-only mode.\n");

  /* UART0 on AR0/AR1 for the ESP32 / MQTT link to the partner robot. */
  switchbox_set_pin(IO_AR0, SWB_UART0_RX);
  switchbox_set_pin(IO_AR1, SWB_UART0_TX);
  uart_init(UART0);
  fprintf(stderr, "[robot] MQTT link up on UART0 (AR0/AR1).\n");

#if USE_PULSECOUNTER_ODOMETRY
  switchbox_set_pin(LEFT_PULSE_PIN,  SWB_TIMER_IC0);
  switchbox_set_pin(RIGHT_PULSE_PIN, SWB_TIMER_IC1);
  pulsecounter_init(PULSECOUNTER0);
  pulsecounter_init(PULSECOUNTER1);
  pulsecounter_set_edge(PULSECOUNTER0, GPIO_LEVEL_HIGH);
  pulsecounter_set_edge(PULSECOUNTER1, GPIO_LEVEL_HIGH);
  pulsecounter_reset_count(PULSECOUNTER0);
  pulsecounter_reset_count(PULSECOUNTER1);
#endif
}

static void cleanup_hardware(void) {
  stepper_reset();         /* stop any in-flight motion before disabling the driver */
  stepper_disable();
  stepper_destroy();
#if USE_PULSECOUNTER_ODOMETRY
  pulsecounter_destroy(PULSECOUNTER0);
  pulsecounter_destroy(PULSECOUNTER1);
#endif
  iic_destroy(IIC0);
  uart_reset_fifos(UART0); /* release UART0 (brought up in setup_hardware) */
  uart_destroy(UART0);
  pynq_destroy();
}

/* ------------------------------- main ------------------------------ */

int main(void) {
  robot_state_t state;
  memset(&state, 0, sizeof(state));
  state.pose.x = START_X_M;
  state.pose.y = START_Y_M;
  state.pose.theta = START_THETA_RAD;
  state.mode = MODE_EXPLORE;
  pulse_tracker_init(&state.left_counter, PULSECOUNTER0);
  pulse_tracker_init(&state.right_counter, PULSECOUNTER1);
  state.last_pose_report_ms = monotonic_ms();
  state.last_status_report_ms = monotonic_ms();

  signal(SIGINT, handle_signal);    /* Ctrl+C / kill -> clean shutdown (motors off) */
  signal(SIGTERM, handle_signal);

  /* Seed the RNG for the random roam direction. Mix in ROBOT_ID so the two
   * robots roam differently even if they boot in the same second. */
  {
    unsigned seed = (unsigned)time(NULL);
    for (const char *p = ROBOT_ID; *p; ++p) seed = seed * 31u + (unsigned char)*p;
    srand(seed);
  }

  setup_hardware();
  sync_odometry_from_pulsecounters(&state);

  send_status(&state, "boot");
  send_pose(&state);

  uint64_t last_loop_hb = 0;

  while (g_keep_running) {
    poll_partner_mqtt(&state);
    report_periodic(&state);

    /* Heartbeat to stderr (-> /tmp/ws.log): loop alive, mode, ap-state, pose,
     * the three ToF heights, tape, temp. */
    const uint64_t hb_now = monotonic_ms();
    if (hb_now - last_loop_hb >= 1000) {
      const distance_readings_t hb_d = read_distance_sensors();
      const bool hb_tape = tape_detected_now();
      fprintf(stderr,
              "[robot] tick: mode=%s ap=%d pose=(%.2f, %.2f, %.0fdeg) "
              "tof[bot %.2f mid %.2f top %.2f]m tape=%d temp=%.1fC\n",
              mode_name(state.mode), (int)state.ap_state, state.pose.x, state.pose.y,
              state.pose.theta * 180.0 / M_PI,
              hb_d.bottom_m, hb_d.middle_m, hb_d.top_m, hb_tape,
              temp_read_celsius());
      last_loop_hb = hb_now;
    }

    if (state.mode == MODE_STOP) {
      sleep_msec(LOOP_SLEEP_MS);
      continue;
    }
    autopilot_step(&state);   /* always autonomous EXPLORE: frontier + SLAM */
  }

  cleanup_hardware();
  return EXIT_SUCCESS;
}
