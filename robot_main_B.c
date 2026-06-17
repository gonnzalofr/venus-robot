#define _POSIX_C_SOURCE 200809L

/*
 *  TU/e 5EID0 - Venus Project
 *  Pynq robot firmware -- drives the rover and streams telemetry to the
 *  web UI (UI_live_robot.html) over stdio (forwarded to the browser by ws_bridge.py over WiFi).
 *
 *  Sensors come from the shared module in sensors.c / sensors.h:
 *     - 3x VL53L0X ToF  (front/left/right distance)   -- working driver
 *     - 2x TCRT5000 IR  (black-tape detection)
 *     - 1x TCS3200      (scannable colour)             -- frequency counter
 *
 *  Build (alongside sensors.c + vl53l0x.c):
 *     gcc robot_main.c sensors.c vl53l0x.c -o robot_main \
 *         $(pkg-config --cflags --libs libpynq) -lm
 *  (use whatever include/-L flags your libpynq install needs.)
 *
 *  Wire protocol (newline-terminated JSON, both directions):
 *     OUT  {"type":"pose","x":<m>,"y":<m>,"theta":<rad>, ...}
 *          {"type":"observation","kind":"mountain"|"black_tape"|"scannable",...}
 *          {"type":"status","mode":...,"detail":...}
 *     IN   {"type":"target","x":<m>,"y":<m>}
 *          {"type":"mode","mode":"explore"|"stop"}
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
#include <unistd.h>   /* read / STDIN_FILENO */
#include <fcntl.h>    /* non-blocking stdin   */

#define AS_SECONDS(time_ticks) ((time_ticks) / 1.0e8)

/* =====================================================================
 *  TWO-ROBOT CONFIG  --  the ONLY thing that differs between the two builds.
 *  Both robots start back-to-back at the centre and share ONE world frame
 *  (x = the direction robot A faces, "east"). Robot A faces +x and owns the
 *  x >= 0 half; robot B faces -x (theta = pi at boot) and owns the x <= 0 half.
 *  The virtual dividing line is x = 0; neither crosses it. Cubes are shared
 *  over MQTT in this common frame, so each robot can dedup the other's finds.
 *
 *  THIS FILE IS ROBOT A. The B build (robot_main_B.c) flips these three lines.
 * ===================================================================== */
#define ROBOT_LABEL   "B"     /* "A" or "B" -- UI/log id                       */
#define HALF_DIR      (-1)    /* +1 = own half is x>=0 (A); -1 = x<=0 (B)      */
#define START_THETA   (M_PI)  /* boot heading: 0 faces +x (A); M_PI faces -x(B)*/
#define LINE_MARGIN_M (0.03)  /* tolerance before the line counts as crossed   */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef ROBOT_ID
#define ROBOT_ID "r1"
#endif

#ifndef START_X_M
#define START_X_M 0.0
#endif
#ifndef START_Y_M
#define START_Y_M 0.0
#endif
#ifndef START_THETA_RAD
#define START_THETA_RAD START_THETA    /* per-robot boot heading (A: 0, B: pi) */
#endif

/* ============================= Pin map =============================
 * Confirmed wiring (sensors.c owns the sensor pins):
 *     IIC0 (ToF) .... IO_AR_SCL / IO_AR_SDA
 *     ToF XSHUT ..... AR4 (B), AR5 (C)
 *     TCS3200 ....... AR6 (OUT), AR7 (S2), AR8 (S3)
 *     IR tape ....... AR9 (left), AR10 (right)
 *
 * Comms no longer use a board UART: telemetry/commands go over stdio,
 * which ws_bridge.py forwards to the browser over WiFi. AR2/AR3 are the
 * odometry pulse-counter inputs.
 * =================================================================== */
#define LEFT_PULSE_PIN  IO_AR2   /* TODO confirm: left  wheel encoder */
#define RIGHT_PULSE_PIN IO_AR3   /* TODO confirm: right wheel encoder */

/* Encoders aren't confirmed wired (AR2/AR3), so default to dead-reckoning:
 * pose is integrated from the COMMANDED steps. Set this to 1 once real
 * wheel encoders are on AR2/AR3 to use closed-loop odometry instead. */
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
#define SCANNABLE_REPORT_RANGE_M 0.12
#define TARGET_REACHED_M 0.06
#define HEADING_TOLERANCE_RAD 0.18

/* Stepper pulse delay in ticks (larger = slower). Matches the wheel
 * library's tested speeds: 20000 driving, 25000 turning. */
#define STEPPER_PULSE_DELAY_TICKS 20000
#define STEPPER_TURN_PULSE_DELAY_TICKS 25000

#define LOOP_SLEEP_MS 20
#define POSE_REPORT_PERIOD_MS 500
#define STATUS_REPORT_PERIOD_MS 2000
#define SCANNABLE_MIN_PERIOD_MS 1000

#define MOUNTAIN_STOP_M 0.15
#define MOUNTAIN_REPORT_MAX_M 0.45

/* --------------------- Autopilot (frontier + SLAM) ----------------- *
 * Mirrors the UI auto-pilot: build an occupancy grid from the forward
 * sensors, drive to the nearest frontier (free cell next to unknown), and
 * react to what the sensors see:
 *   - top ToF  -> mountains (boundaries): map them; if one is in the way, avoid.
 *   - IR tape  -> boundary in the way: avoid.
 *   - bottom ToF (<=10cm) -> small 3x3 rock; middle ToF (<=10cm) -> large 6x6.
 *     Approach until 2cm away, read colour, report the sample, then re-plan.
 * Bottom/middle readings beyond 10cm are ignored (no rock there).            */
/* Arena is 100 cm (X, split between the two robots) x 130 cm (Y, lane length).
 * At 3 cm/cell that's ~34 x 44 cells; round up for margin and centre it. X spans
 * +-54 cm, Y spans +-69 cm; the dividing line is x=0 = column GRID_CX.            */
#define GRID_W 36                /* 36 * 3cm = 108 cm wide  (100 cm arena + margin)  */
#define GRID_H 46                /* 46 * 3cm = 138 cm tall  (130 cm arena + margin)  */
#define GRID_CX 18               /* world (0,0) -> grid centre */
#define GRID_CY 23
#define ROCK_DETECT_M       0.05 /* bottom/middle ToF: only within this is a real rock
                                  * (angled down, the floor reflects past this) -> scan */
#define ROCK_APPROACH_M     0.02 /* creep until the rock is this close, then read    */
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
#define BLACKLIST_RADIUS_CELLS 3   /* a handled object/obstacle bans this radius around it*/
#define BLACKLIST_TTL_STEPS  0     /* 0 = permanent: a scanned cube / avoided obstacle is
                                      handled for good, so we never re-target it          */
#define EDGE_BUFFER_CELLS      2   /* stay this far off the grid edge                   */
#define MEMORY_SCAN_CELLS   12.0   /* how far the side-scan probes the occupancy map    */
#define STUCK_STEPS            8   /* no-progress steps before blacklisting + recovery  */
#define SCAN_INCREMENTS        8   /* in-place sweep steps for one 360deg map scan      */
#ifndef SCAN_EVERY_N
#define SCAN_EVERY_N           3   /* do a map sweep every Nth waypoint (1=always)      */
#endif

/* ---- Sweep/seek mission (team algorithm, our sensors) -- TUNE on the robot --
 * The 3 ToF point forward at heights 2.5 / 5.5 / 8 cm. An object is "seen" by a
 * sensor when it is tall enough to block that beam, so WHICH sensors see it
 * gives the type directly:  bottom only = 3x3 cube,  bottom+middle = 6x6 cube,
 * bottom+middle+top = mountain (>=10 cm). IR black = boundary or crater.        */
#define OBJ_DETECT_M     0.45  /* a ToF reading nearer than this = an object ahead */
#define OBJ_MIN_M        0.02  /* below this = noise                               */
#define GOTO_CLOSE_M     0.12  /* switch GOTO->APPROACH when the object is this near*/
#define CLASSIFY_M       0.10  /* a sensor seeing the object within this = "sees it"*/
#define COLOR_READ_M     0.05  /* creep (via ToF) until the cube is this close       */
#define HUG_NUDGE_M      0.06  /* then drive THIS far blind to touch it -- temp reads
                                  need contact, so we close the gap + press lightly.
                                  Tune on-robot: bigger = firmer hug (pushes more).  */
#define REACH_TOL_M      0.01  /* "close enough" margin so drive-to-threshold can't
                                  asymptote-stall just shy of the target distance    */
#define SWEEP_INCREMENTS   12  /* steps in one in-place search sweep (~30 deg each) */
#define SWEEP_BACKOFF_M  0.05  /* back off after evaluating / before turning away   */
#define AVOID_TURN_RAD   (M_PI * 0.6)  /* ~108 deg away from a mountain/boundary    */
#define OBJ_BLACKLIST_CELLS 2  /* don't re-target an object within this many cells   */
/* lawnmower coverage */
#define ROW_SPACING_M    0.12  /* lateral gap between lanes (< ToF cone so they overlap)*/
#define ROW_STEP_M       0.05  /* forward step along a lane; small = stops fast at cubes*/
#define ROW_AIM_TOL      0.15  /* re-straighten the lane if heading drifts past this rad*/
#define LANE_X_TOL       0.02  /* return to the lane's absolute x if displaced past this */
#define TURN_STEP_RAD    0.30  /* turns are taken in steps THIS big (~17deg, < ToF cone)
                                  so the ToF samples mid-turn and a swept-past cube is
                                  caught by the reactive grab instead of skipped over   */
#define GOTO_MAX_STEPS     16  /* give up chasing a cube after this many steps         */
#define DETOUR_SIDESTEP_M 0.15 /* sidestep this far into our half to clear an obstacle */
#define DETOUR_PASS_M    0.35  /* advance this far along the lane to get fully past it  */
#define DETOUR_COOLDOWN     5  /* then drive this many lane steps before detouring again*/

#define CELL_UNKNOWN  0
#define CELL_FREE     1
#define CELL_MOUNTAIN 2
#define CELL_BOUNDARY 3

/* ----------------------------- Types ------------------------------- */

typedef enum { MODE_STOP = 0, MODE_EXPLORE, MODE_TARGET, MODE_MANUAL } robot_mode_t;

/* Autopilot state machine: LAWNMOWER (boustrophedon) coverage.
 *   Drive a straight ROW along +/-Y until the black boundary (or the dividing
 *   line); SHIFT one lane sideways into our own half and reverse; repeat, so the
 *   whole half gets swept in parallel lines. A cube seen on the way -> GOTO/
 *   APPROACH/EVALUATE (stop short + scan, never bump) then carry on; a mountain
 *   or already-handled obstacle -> DETOUR around it. Never spins in circles. */
typedef enum {
  AP_ROW = 0,     /* drive a straight lane until a boundary / obstacle / cube  */
  AP_GOTO,        /* drive up to a cube (stops short -- no bump)               */
  AP_APPROACH,    /* creep in until close enough to read colour                */
  AP_EVALUATE,    /* cube: read colour + size, report + share, then resume     */
  AP_AVOID,       /* (kept) mountain handler -> hands off to DETOUR            */
  AP_SHIFT,       /* U-turn: step one lane into our half, reverse direction    */
  AP_DETOUR,      /* obstacle in the lane -> crab around it, then resume        */
  AP_DONE         /* whole half swept -> hold                                  */
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
  uint64_t last_scannable_report_ms;
  pulse_tracker_t left_counter;
  pulse_tracker_t right_counter;
  int active_left_dir;
  int active_right_dir;
  int man_fwd;    /* manual drive intent: -1 back, 0 idle, +1 forward */
  int man_turn;   /* manual turn intent:  -1 left, 0 none, +1 right    */
  ap_state_t ap_state;       /* sweep/seek mission state machine */
  int sweep_step;            /* progress through the current in-place sweep */
  int row_dir;               /* lawnmower lane direction: +1 = +Y, -1 = -Y */
  int homed;                 /* 0 until we've reached a Y boundary to start from */
  int shift_step;            /* sub-step of the U-turn (0 turn,1 step over,2 reverse) */
  double shift_acc;          /* metres moved so far this shift / detour phase */
  double row_x;              /* x of the current lane */
  double lawn_max_x;         /* furthest-into-our-half lane reached so far */
  int stall_lanes;           /* consecutive lanes that reached no new ground */
  double y_lo, y_hi;         /* Y extent of the arena, learned from the first column */
  int bounds_known;          /* 1 once y_lo/y_hi are set (after the first full lane) */
  int goto_steps;            /* steps spent driving at the current cube (cap) */
  int detour_phase;          /* go-around phase: 0 turn out, 1 sidestep, 2 rejoin lane */
  int cooldown;              /* lane steps to ignore the just-passed obstacle */
  int rock_is_large;         /* 1 = 6x6 cube, 0 = 3x3 cube */
  int avoid_reason;          /* 0 = boundary/crater, 1 = mountain */
  char rx_line[256];
  size_t rx_len;
} robot_state_t;

static void ap_reset(robot_state_t *s);   /* defined in the autopilot section */

/* ----------------------------- Utils ------------------------------- */

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

/* ------------------------ Telemetry TX (stdout) -------------------- *
 * JSON telemetry goes to stdout, which ws_bridge.py forwards to the UI.
 * Diagnostics go to stderr so they never pollute the JSON stream.       */

/* Set 0 to stop mirroring the UI stream onto the MQTT link (e.g. if the ESP32 is
 * absent and you don't want any chance of the UART back-pressuring the loop). */
#define MQTT_MIRROR_UI 1

static void uart_send_string(const char *text) {
  fputs(text, stdout);          /* local transport: stdout -> ws_bridge -> browser */
  fflush(stdout);
#if MQTT_MIRROR_UI
  mqtt_publish_raw(text);       /* wireless transport: UART0 -> ESP32 -> MQTT broker */
#endif
}

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

static const char *color_label_to_str(color_label_t c) {
  switch (c) {
    case COLOR_RED:    return "red";
    case COLOR_GREEN:  return "green";
    case COLOR_BLUE:   return "blue";
    case COLOR_YELLOW: return "yellow";
    case COLOR_WHITE:  return "white";
    default:           return "none";
  }
}

static void send_pose(const robot_state_t *s) {
  char msg[192];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"pose\",\"robot\":\"%s\",\"t_ms\":%llu,"
           "\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f}\n",
           ROBOT_ID, (unsigned long long)monotonic_ms(),
           s->pose.x, s->pose.y, s->pose.theta);
  uart_send_string(msg);
}

static void send_status(const robot_state_t *s, const char *detail) {
  char msg[224];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"status\",\"robot\":\"%s\",\"t_ms\":%llu,"
           "\"mode\":\"%s\",\"detail\":\"%s\"}\n",
           ROBOT_ID, (unsigned long long)monotonic_ms(),
           mode_name(s->mode), detail);
  uart_send_string(msg);
}

/* The frontier the autopilot has chosen to drive to -> shown as a goal marker
 * in the UI (like the prototype's target). */
static void send_plan(double x, double y) {
  char msg[160];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"plan\",\"robot\":\"%s\",\"t_ms\":%llu,\"x\":%.4f,\"y\":%.4f}\n",
           ROBOT_ID, (unsigned long long)monotonic_ms(), x, y);
  uart_send_string(msg);
}

static void send_observation(const char *kind, double x, double y, double conf) {
  char msg[224];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"observation\",\"robot\":\"%s\",\"t_ms\":%llu,"
           "\"kind\":\"%s\",\"x\":%.4f,\"y\":%.4f,\"confidence\":%.2f}\n",
           ROBOT_ID, (unsigned long long)monotonic_ms(), kind, x, y, conf);
  uart_send_string(msg);
}

static void send_scannable(double x, double y, color_label_t color, double conf,
                           int cells, double temp_c) {
  char msg[256];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"observation\",\"robot\":\"%s\",\"t_ms\":%llu,"
           "\"kind\":\"scannable\",\"color\":\"%s\",\"x\":%.4f,"
           "\"y\":%.4f,\"size\":%d,\"temp_c\":%.1f,\"confidence\":%.2f}\n",
           ROBOT_ID, (unsigned long long)monotonic_ms(),
           color_label_to_str(color), x, y, cells, temp_c, conf);
  uart_send_string(msg);
}

/* Live sensor snapshot for the desktop monitor's sensor panel + ToF wall cloud.
 * Sent ~1x/second from the heartbeat (reuses its readings -- no extra I2C). */
static void send_telemetry(const distance_readings_t *d, bool ir_l, bool ir_r,
                           double temp_c, bool moving) {
  char msg[256];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"telemetry\",\"robot\":\"%s\",\"t_ms\":%llu,"
           "\"tof_bottom_mm\":%d,\"tof_middle_mm\":%d,\"tof_top_mm\":%d,"
           "\"ir_left\":%d,\"ir_right\":%d,\"temp_c\":%.1f,\"moving\":%s}\n",
           ROBOT_ID, (unsigned long long)monotonic_ms(),
           d->bottom_valid ? (int)lround(d->bottom_m * 1000.0) : 0,
           d->middle_valid ? (int)lround(d->middle_m * 1000.0) : 0,
           d->top_valid    ? (int)lround(d->top_m * 1000.0)    : 0,
           ir_l ? 1 : 0, ir_r ? 1 : 0, temp_c, moving ? "true" : "false");
  uart_send_string(msg);
}

static void project_from_pose(const pose_t *p, double range_m, double bearing,
                              double *x, double *y) {
  const double wb = p->theta + bearing;
  *x = p->x + range_m * cos(wb);
  *y = p->y + range_m * sin(wb);
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

static color_detection_t read_color_detection(void) {
  color_detection_t det = {0};
  float conf = 0.0f;
  const scan_color_t c = color_read_label(&conf);
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

/* -------------------------- Command RX (stdin) --------------------- */

static bool json_get_double(const char *json, const char *key, double *out) {
  const char *pos = strstr(json, key);
  if (!pos) return false;
  pos = strchr(pos, ':');
  if (!pos) return false;
  ++pos;
  char *end = NULL;
  const double v = strtod(pos, &end);
  if (end == pos) return false;
  *out = v;
  return true;
}

static bool json_get_string(const char *json, const char *key, char *out, size_t n) {
  const char *pos = strstr(json, key);
  if (!pos || n == 0) return false;
  pos = strchr(pos, ':'); if (!pos) return false;
  pos = strchr(pos, '"'); if (!pos) return false;
  ++pos;
  const char *end = strchr(pos, '"'); if (!end) return false;
  const size_t len = (size_t)(end - pos);
  const size_t cn = len < n - 1 ? len : n - 1;
  memcpy(out, pos, cn);
  out[cn] = '\0';
  return true;
}

static void handle_command_line(robot_state_t *s, const char *line) {
  fprintf(stderr, "[robot] cmd rx: %s\n", line);   /* diagnostic: shows commands arriving */

  if (strstr(line, "\"type\":\"drive\"") || strstr(line, "\"type\": \"drive\"")) {
    double fwd = 0.0, turn = 0.0;
    json_get_double(line, "\"fwd\"", &fwd);
    json_get_double(line, "\"turn\"", &turn);
    s->man_fwd  = (fwd > 0.5) - (fwd < -0.5);
    s->man_turn = (turn > 0.5) - (turn < -0.5);
    s->has_target = false;
    s->mode = MODE_MANUAL;
    return;
  }

  if (strstr(line, "\"type\":\"target\"") || strstr(line, "\"type\": \"target\"")) {
    double x = 0.0, y = 0.0;
    if (json_get_double(line, "\"x\"", &x) && json_get_double(line, "\"y\"", &y)) {
      s->target_x = x; s->target_y = y;
      s->has_target = true; s->mode = MODE_TARGET;
      send_status(s, "target_received");
    }
    return;
  }
  if (strstr(line, "\"type\":\"mode\"") || strstr(line, "\"type\": \"mode\"")) {
    char mode[24];
    if (json_get_string(line, "\"mode\"", mode, sizeof(mode))) {
      if (strcmp(mode, "stop") == 0) {
        s->mode = MODE_STOP;
        stepper_reset();
        ap_reset(s);
        send_status(s, "mode_stop");
      } else if (strcmp(mode, "explore") == 0) {
        s->mode = MODE_EXPLORE;
        ap_reset(s);                 /* fresh frontier search, no stale state */
        send_status(s, "mode_explore");
      } else {
        fprintf(stderr, "[robot] unknown mode: %s\n", mode);
      }
    }
  }
}

/* Commands (target / mode) arrive on stdin from ws_bridge.py. stdin is set
 * non-blocking in setup_hardware, so read() returns <=0 when nothing waits. */
static void poll_commands(robot_state_t *s) {
  char buf[128];
  ssize_t n;
  while ((n = read(STDIN_FILENO, buf, sizeof(buf))) > 0) {
    for (ssize_t i = 0; i < n; ++i) {
      const char c = buf[i];
      if (c == '\n' || c == '\r') {
        if (s->rx_len > 0) {
          s->rx_line[s->rx_len] = '\0';
          handle_command_line(s, s->rx_line);
          s->rx_len = 0;
        }
      } else if (s->rx_len + 1 < sizeof(s->rx_line)) {
        s->rx_line[s->rx_len++] = c;
      } else {
        s->rx_len = 0;
      }
    }
  }
}

/* --------------------------- Hazard helpers ------------------------ */

/* Report a mountain seen by the TOP sensor (used by manual/target scanning). */
static void report_front_mountain(const robot_state_t *s, const distance_readings_t *d) {
  if (!d->top_valid || d->top_m < MOUNTAIN_DETECT_MIN_M || d->top_m > MOUNTAIN_REPORT_MAX_M) return;
  double x = 0.0, y = 0.0;
  project_from_pose(&s->pose, d->top_m, 0.0, &x, &y);
  send_observation("mountain", x, y, 0.85);
}

/* ------------------------------ Motion ----------------------------- */

static bool execute_step_command(robot_state_t *s, int left_steps, int right_steps,
                                 bool safety_enabled, uint16_t pulse_delay) {
  s->active_left_dir  = sign_of_steps(left_steps);
  s->active_right_dir = sign_of_steps(right_steps);
  stepper_set_speed(pulse_delay, pulse_delay);
  stepper_steps((int16_t)left_steps, (int16_t)right_steps);

  while (!stepper_steps_done()) {
    poll_commands(s);
    sync_odometry_from_pulsecounters(s);

    if (s->mode == MODE_STOP) {
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
  /* HARD dividing-line guard (coordinate based): never let any move -- forward or
   * backward -- put us across x=0 into the partner's half. Clamp the distance so
   * we stop exactly on the line. This is the "under no circumstance cross" rule. */
  const double cth = cos(s->pose.theta);
  if (fabs(cth) > 1e-9) {
    const double dest_x = s->pose.x + cth * meters;
    if ((double)HALF_DIR * dest_x < 0.0) meters = (0.0 - s->pose.x) / cth;
  }
  const int steps = meters_to_steps(meters);
  return execute_step_command(s, steps, steps, safety, STEPPER_PULSE_DELAY_TICKS);
}

static bool rotate_rad(robot_state_t *s, double radians, bool safety) {
  const double wheel_m = 0.5 * WHEEL_BASE_M * radians;
  const int l = meters_to_steps(-wheel_m);
  const int r = meters_to_steps(wheel_m);
  return execute_step_command(s, l, r, safety, STEPPER_TURN_PULSE_DELAY_TICKS);
}

static void avoid_after_block(robot_state_t *s) {
  move_forward_m(s, -BACKUP_M, false);
  rotate_rad(s, M_PI / 2.0, false);
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

/* Continuous scan used in MANUAL / TARGET modes (the autopilot does its own
 * sensing in EXPLORE). Reports mountains from the top ToF and colour samples. */
static void scan_and_report(robot_state_t *s) {
  const distance_readings_t d = read_distance_sensors();
  if (d.top_valid && d.top_m >= MOUNTAIN_DETECT_MIN_M && d.top_m < MOUNTAIN_REPORT_MAX_M) {
    report_front_mountain(s, &d);
  }

  const color_detection_t color = read_color_detection();
  const uint64_t now = monotonic_ms();
  if (color.detected &&
      now - s->last_scannable_report_ms > SCANNABLE_MIN_PERIOD_MS) {
    /* big 6x6 rock if the middle ToF sees it within range, else small 3x3 */
    const int cells = (d.middle_valid && d.middle_m > 0.0 && d.middle_m <= ROCK_DETECT_M) ? 2 : 1;
    double x = 0.0, y = 0.0;
    project_from_pose(&s->pose, SCANNABLE_REPORT_RANGE_M, 0.0, &x, &y);
    send_scannable(x, y, color.label, color.confidence, cells, temp_read_celsius());  /* -> UI */

    /* Share this rock with the partner robot over MQTT (x/y in cm), with the
     * real NTC temperature. TODO: rock size (needs ToF bottom/middle logic). */
    const float temp_c = temp_read_celsius();
    send_mqtt((int)lround(x * 100.0), (int)lround(y * 100.0),
              color_label_to_mqtt(color.label),
              0 /* size cm: TODO */, temp_c);

    s->last_scannable_report_ms = now;
  }
}

static bool cube_remember(double x, double y);   /* shared cube store (defined below) */

/* Poll the partner robot's MQTT link; plot any rock it reports on our map too
 * (so the dashboard shows both robots' finds). cm -> m for the UI. */
static void poll_partner_mqtt(robot_state_t *s) {
  int px = 0, py = 0, psize = 0;
  float ptemp = 0.0f;
  char pcolor[32] = "Unknown";
  recv_mqtt(&px, &py, &psize, &ptemp, pcolor);
  if (pcolor[0] != '\0' && strcmp(pcolor, "Unknown") != 0) {
    /* Same world frame for both robots -> store the partner's cube directly so we
     * never go scan a cube it already did (dedup by location in cube_remember). */
    cube_remember((double)px / 100.0, (double)py / 100.0);
    char msg[256];
    snprintf(msg, sizeof(msg),
             "{\"type\":\"observation\",\"robot\":\"partner\",\"t_ms\":%llu,"
             "\"kind\":\"scannable\",\"color\":\"%s\",\"x\":%.4f,\"y\":%.4f,"
             "\"confidence\":1.00}\n",
             (unsigned long long)monotonic_ms(), pcolor,
             (double)px / 100.0, (double)py / 100.0);
    uart_send_string(msg);   /* -> our UI on stdout */
  }
  (void)s;
}

/* ----------------------------- Navigation -------------------------- */

static double distance_to_target(const robot_state_t *s) {
  return hypot(s->target_x - s->pose.x, s->target_y - s->pose.y);
}

static void target_step(robot_state_t *s) {
  if (!s->has_target || distance_to_target(s) < TARGET_REACHED_M) {
    s->mode = MODE_EXPLORE;
    ap_reset(s);                 /* clean frontier search after reaching the target */
    send_status(s, "target_reached");
    return;
  }
  const double dx = s->target_x - s->pose.x;
  const double dy = s->target_y - s->pose.y;
  const double desired = atan2(dy, dx);
  const double err = normalize_angle(desired - s->pose.theta);

  if (fabs(err) > HEADING_TOLERANCE_RAD) {
    if (!rotate_rad(s, err, true) && s->mode != MODE_STOP) avoid_after_block(s);
    return;
  }
  const double step_m = fmin(CELL_M, distance_to_target(s));
  if (!move_forward_m(s, step_m, true) && s->mode != MODE_STOP) avoid_after_block(s);
}

/* ===================== Autopilot: frontier + SLAM ================== */

static uint8_t g_grid[GRID_W][GRID_H];     /* zero-init -> all CELL_UNKNOWN */
static uint8_t g_path[GRID_W][GRID_H];     /* cells the robot has driven through */
static uint64_t g_ap_tick = 0;             /* autopilot step counter (blacklist TTL) */

/* TTL blacklist of frontiers we failed to reach -- mirrors the UI's failedFrontiers
 * so the robot doesn't keep retrying an unreachable target (rock behind a mountain,
 * a corner it can't enter, etc.). */
static struct { int gx, gy; uint64_t tick; bool used; } g_blacklist[64];
static int g_blacklist_head = 0;

static bool ap_is_blacklisted(int gx, int gy);

static void ap_blacklist_add(int gx, int gy) {
  if (ap_is_blacklisted(gx, gy)) return;          /* dedupe: don't fill the ring with the
                                                     same object/obstacle over and over    */
  g_blacklist[g_blacklist_head].gx = gx;
  g_blacklist[g_blacklist_head].gy = gy;
  g_blacklist[g_blacklist_head].tick = g_ap_tick;
  g_blacklist[g_blacklist_head].used = true;
  g_blacklist_head = (g_blacklist_head + 1) % (int)(sizeof(g_blacklist) / sizeof(g_blacklist[0]));
}
static bool ap_is_blacklisted(int gx, int gy) {
  for (size_t i = 0; i < sizeof(g_blacklist) / sizeof(g_blacklist[0]); ++i) {
    if (!g_blacklist[i].used) continue;
    if (BLACKLIST_TTL_STEPS > 0 && g_ap_tick - g_blacklist[i].tick > BLACKLIST_TTL_STEPS) continue;
    const int dx = gx - g_blacklist[i].gx, dy = gy - g_blacklist[i].gy;
    if (dx * dx + dy * dy <= BLACKLIST_RADIUS_CELLS * BLACKLIST_RADIUS_CELLS) return true;
  }
  return false;
}

/* Put the autopilot into a clean state. Call on every transition INTO explore
 * so stale GOTO/APPROACH/AVOID data from a prior session can't carry. */
static void ap_reset(robot_state_t *s) {
  s->ap_state = AP_ROW;
  s->row_dir = -1;            /* first drive toward -Y to reach a boundary (home) */
  s->homed = 0;
  s->shift_step = 0;
  s->shift_acc = 0.0;
  s->row_x = (double)HALF_DIR * ROW_SPACING_M * 0.5;  /* lane 0 is half a lane INTO our half,
                                  so the lane's tolerance band never reaches the x=0 line   */
  s->lawn_max_x = (double)(-HALF_DIR) * 1e9;   /* nothing reached yet */
  s->stall_lanes = 0;
  s->y_lo = -1e9; s->y_hi = 1e9;
  s->bounds_known = 0;
  s->goto_steps = 0;
  s->detour_phase = 0;
  s->cooldown = 0;
  s->has_target = false;
  s->rock_is_large = 0;
  s->avoid_reason = 0;
}

static int ap_gx(double x) {
  long g = lround((double)GRID_CX + x / CELL_M);
  return g < 0 ? 0 : (g >= GRID_W ? GRID_W - 1 : (int)g);
}
static int ap_gy(double y) {
  long g = lround((double)GRID_CY + y / CELL_M);
  return g < 0 ? 0 : (g >= GRID_H ? GRID_H - 1 : (int)g);
}

static void ap_set(int gx, int gy, uint8_t v) {
  if (gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H) g_grid[gx][gy] = v;
}

/* ---------------- shared cube store (mine + the partner's) ----------------
 * Every cube this robot scans, AND every cube the partner broadcasts over MQTT,
 * is remembered here in the common world frame. Before targeting a rock we check
 * the store, so neither robot ever scans a cube that's already known. */
#define MAX_KNOWN_CUBES   64
#define CUBE_SAME_M      0.12   /* two finds within this are the same cube */
typedef struct { double x, y; bool used; } known_cube_t;
static known_cube_t g_known_cubes[MAX_KNOWN_CUBES];
static int g_known_cube_count = 0;

/* Is there already a known cube within CUBE_SAME_M of (x,y)? */
static bool cube_is_known(double x, double y) {
  for (int i = 0; i < MAX_KNOWN_CUBES; ++i) {
    if (!g_known_cubes[i].used) continue;
    if (hypot(x - g_known_cubes[i].x, y - g_known_cubes[i].y) < CUBE_SAME_M) return true;
  }
  return false;
}

/* Record a cube (ours or the partner's) and blacklist its cell so the frontier
 * search / sweep never re-targets it. Idempotent. Returns true if newly added. */
static bool cube_remember(double x, double y) {
  ap_blacklist_add(ap_gx(x), ap_gy(y));   /* keep navigation from re-visiting it */
  if (cube_is_known(x, y)) return false;
  for (int i = 0; i < MAX_KNOWN_CUBES; ++i) {
    if (!g_known_cubes[i].used) {
      g_known_cubes[i].x = x; g_known_cubes[i].y = y; g_known_cubes[i].used = true;
      if (g_known_cube_count < MAX_KNOWN_CUBES) g_known_cube_count++;
      return true;
    }
  }
  return false;                            /* store full -- still blacklisted */
}

/* Signed distance of a world-x into THIS robot's half (>=0 = own side). */
static double ap_half_pos(double x) { return (double)HALF_DIR * x; }

/* Fold the current pose + sensor readings into the occupancy grid, and stream
 * mountains / tape to the UI as observations. */
static void slam_update(robot_state_t *s, const distance_readings_t *d, bool tape) {
  const double th = s->pose.theta;
  const int pgx = ap_gx(s->pose.x), pgy = ap_gy(s->pose.y);
  ap_set(pgx, pgy, CELL_FREE);
  g_path[pgx][pgy] = 1;          /* remember where we've driven (don't re-target it) */

  /* Nearest thing blocking straight ahead. Bottom/middle only count as a wall
   * when they see a rock within ROCK_DETECT_M (beyond that = open floor, per
   * spec); top counts at any range up to the mountain map limit; tape blocks
   * just ahead. Nothing seen -> assume clear out to the sensing horizon, so the
   * frontier map grows and the robot always has somewhere to drive. */
  const bool top_is_mountain = d->top_valid &&
      d->top_m >= MOUNTAIN_DETECT_MIN_M && d->top_m <= MOUNTAIN_DETECT_MAX_M;
  double clear = SENSE_HORIZON_M;
  if (d->bottom_valid && d->bottom_m > 0.0 && d->bottom_m <= ROCK_DETECT_M && d->bottom_m < clear) clear = d->bottom_m;
  if (d->middle_valid && d->middle_m > 0.0 && d->middle_m <= ROCK_DETECT_M && d->middle_m < clear) clear = d->middle_m;
  if (top_is_mountain && d->top_m < clear) clear = d->top_m;   /* floor noise (<MIN) ignored */
  if (tape && TAPE_REPORT_OFFSET_M < clear) clear = TAPE_REPORT_OFFSET_M;

  /* Carve free cells from the robot up to just short of that obstacle. Carve a
   * ~3-cell-wide swath (centre + one cell each perpendicular side) so a 360 scan
   * fills a solid explored disk instead of a thin 12-spoke star -- the frontier
   * detector needs a connected free region to find its edges. */
  const double px = -sin(th), py = cos(th);   /* unit vector perpendicular to heading */
  for (double dd = 0.0; dd < clear - CELL_M * 0.5; dd += CELL_M * 0.5) {
    const double bx = s->pose.x + cos(th) * dd, by = s->pose.y + sin(th) * dd;
    for (int side = -1; side <= 1; side++) {
      const int cx = ap_gx(bx + px * CELL_M * side);
      const int cy = ap_gy(by + py * CELL_M * side);
      if (g_grid[cx][cy] == CELL_UNKNOWN) g_grid[cx][cy] = CELL_FREE;
    }
  }

  if (top_is_mountain) {                                 /* TOP in band -> real mountain */
    const double hx = s->pose.x + cos(th) * d->top_m;
    const double hy = s->pose.y + sin(th) * d->top_m;
    ap_set(ap_gx(hx), ap_gy(hy), CELL_MOUNTAIN);
    send_observation("mountain", hx, hy, 0.85);
  }

  if (tape) {                                           /* IR -> boundary */
    const double bx = s->pose.x + cos(th) * TAPE_REPORT_OFFSET_M;
    const double by = s->pose.y + sin(th) * TAPE_REPORT_OFFSET_M;
    ap_set(ap_gx(bx), ap_gy(by), CELL_BOUNDARY);
    send_observation("black_tape", bx, by, 1.0);
  }
}

/* ---- object detection from the 3 forward ToF (heights 2.5/5.5/8 cm) ---- */

/* Distance (m) to the nearest object straight ahead, or -1 if just open floor.
 * The bottom ToF (lowest) sees every object >= 3 cm tall, so it's the universal
 * detector; middle/top can catch one it grazes. */
static double object_front_dist(const distance_readings_t *d) {
  double best = -1.0;
  if (d->bottom_valid && d->bottom_m > OBJ_MIN_M && d->bottom_m < OBJ_DETECT_M) best = d->bottom_m;
  if (d->middle_valid && d->middle_m > OBJ_MIN_M && d->middle_m < OBJ_DETECT_M)
    if (best < 0.0 || d->middle_m < best) best = d->middle_m;
  if (d->top_valid && d->top_m > OBJ_MIN_M && d->top_m < OBJ_DETECT_M)
    if (best < 0.0 || d->top_m < best) best = d->top_m;
  return best;
}

/* Same, but ONLY the bottom + middle ToF (the cube detectors -- every cube is >=3cm
 * so the bottom always sees it). Used for the reactive grab: the instant one of
 * these catches something, we go for it. -1 = open floor. */
static double object_front_dist_low(const distance_readings_t *d) {
  double best = -1.0;
  if (d->bottom_valid && d->bottom_m > OBJ_MIN_M && d->bottom_m < OBJ_DETECT_M) best = d->bottom_m;
  if (d->middle_valid && d->middle_m > OBJ_MIN_M && d->middle_m < OBJ_DETECT_M)
    if (best < 0.0 || d->middle_m < best) best = d->middle_m;
  return best;
}

/* Type of the object we're right in front of, by which heights see it within
 * CLASSIFY_M:  2 = mountain (>=8 cm),  1 = 6x6 cube (>=5.5 cm),  0 = 3x3 cube. */
static int classify_object(const distance_readings_t *d) {
  const bool top    = d->top_valid    && d->top_m    > OBJ_MIN_M && d->top_m    < CLASSIFY_M;
  const bool middle = d->middle_valid && d->middle_m > OBJ_MIN_M && d->middle_m < CLASSIFY_M;
  if (top)    return 2;
  if (middle) return 1;
  return 0;
}

/* Should the object 'dist' ahead be left alone? True if it's already handled
 * (blacklisted), already in the shared store (we OR the partner scanned it), or in
 * the partner's half. The ToF is a ~25deg CONE, so the object could be anywhere
 * within +-12deg of dead-ahead -- we sample across that arc, otherwise an obstacle
 * caught at the cone's edge projects off its own blacklist and gets grabbed forever. */
static bool object_ahead_blacklisted(const robot_state_t *s, double dist) {
  const double lat = dist * 0.22;                    /* half the cone width at this range */
  for (int k = -1; k <= 1; k++) {
    double ox = 0.0, oy = 0.0;
    project_from_pose(&s->pose, dist, (double)k * lat, &ox, &oy);
    if (ap_is_blacklisted(ap_gx(ox), ap_gy(oy))) return true;
    if (cube_is_known(ox, oy)) return true;          /* mine or partner's already */
    if (ap_half_pos(ox) < 0.0) return true;          /* across the line -> partner's job */
  }
  return false;
}

/* ----------------------------- lawnmower geometry ----------------------------- */

/* Absolute heading for a lane: +1 drives +Y (north), -1 drives -Y (south). */
static double ap_row_heading(int row_dir) { return (row_dir > 0) ? (M_PI / 2.0) : (-M_PI / 2.0); }

/* Absolute heading that points INTO our own half (the U-turn / sidestep direction):
 * +x for robot A, -x for robot B. */
static double ap_shift_heading(void) { return (HALF_DIR > 0) ? 0.0 : M_PI; }

/* Turn TOWARD an absolute heading by at most one TURN_STEP_RAD step. Returns true once
 * we're aligned (within ROW_AIM_TOL). Stepping in small chunks (instead of one big
 * rotate) means the main loop -- and the reactive cube grab -- runs between each chunk,
 * so a cube the beam sweeps past during the turn is caught immediately. */
static bool ap_face(robot_state_t *s, double target) {
  const double err = normalize_angle(target - s->pose.theta);
  if (fabs(err) <= ROW_AIM_TOL) { return true; }
  rotate_rad(s, (fabs(err) <= TURN_STEP_RAD) ? err
                                             : (err > 0 ? TURN_STEP_RAD : -TURN_STEP_RAD), false);
  return false;
}

/* Map the boundary just ahead (for the UI) and report it. */
static void ap_mark_boundary(robot_state_t *s) {
  double bx = 0.0, by = 0.0;
  project_from_pose(&s->pose, TAPE_REPORT_OFFSET_M, 0.0, &bx, &by);
  ap_set(ap_gx(bx), ap_gy(by), CELL_BOUNDARY);
  send_observation("black_tape", bx, by, 1.0);
}

/* The autopilot, run once per main-loop iteration in MODE_EXPLORE. */
static void autopilot_step(robot_state_t *s) {
  g_ap_tick++;
  const distance_readings_t d = read_distance_sensors();
  const bool black = tape_detected_now();      /* IR: boundary OR crater (both black) */
  slam_update(s, &d, black);                   /* keep mapping the surroundings for the UI */

  /* ---- reactive black = the arena boundary / a crater ----
   * In a lawnmower this is the END of a lane: back off the line, then U-turn into
   * the next lane. If it happens while SHIFTing sideways we've reached the far edge
   * of our half -> the whole half is swept, so we're done. */
  if (black && s->ap_state != AP_EVALUATE) {
    /* Classify the black BEFORE backing off -- the back-off shifts our y and would
     * otherwise push a crater hit down into the "boundary" band. Interior black (well
     * within the known Y extent) is a crater to go around; black near a Y end is the
     * arena boundary = lane end. */
    const bool interior = s->homed && s->bounds_known &&
        s->pose.y > s->y_lo + 2.0 * CELL_M && s->pose.y < s->y_hi - 2.0 * CELL_M;
    const bool facing_x = fabs(cos(s->pose.theta)) > 0.7;   /* moving in x, not along a lane */
    const double hit_y = s->pose.y;
    move_forward_m(s, -SWEEP_BACKOFF_M, false);     /* never sit on the black */
    ap_mark_boundary(s);
    if (s->ap_state == AP_DETOUR) {
      /* hit black mid-detour: skip to the rejoin-lane phase so we end cleanly. */
      s->detour_phase = 2;
      s->shift_acc = 0.0;
    } else if (s->homed && facing_x) {
      /* sliding sideways onto a new lane and hit black = the far X edge of our half
       * is past here -> the whole half is swept. */
      send_status(s, "coverage_complete");
      s->ap_state = AP_DONE;
    } else if (!s->homed) {                          /* first boundary: lane 0 starts at x=0 */
      s->homed = 1;
      s->y_lo = hit_y;                               /* bottom of the arena (we drove -Y to it) */
      s->row_dir = -s->row_dir;                      /* reverse and sweep back up lane 0 */
      s->ap_state = AP_ROW;
    } else if (!s->bounds_known) {                   /* top of lane 0 -> Y extent known */
      s->y_hi = hit_y;
      s->bounds_known = 1;
      s->ap_state = AP_SHIFT;
      send_status(s, "row_end");
    } else if (interior) {
      /* crater (interior black) -- can't drive through it, and if it sits on the lane
       * centre we can't return to the lane either. Move to the next lane but KEEP the
       * sweep direction (pre-flip cancels SHIFT's flip) so the next lane covers the Y
       * range PAST the crater rather than re-covering what we just did. */
      s->row_dir = -s->row_dir;
      s->ap_state = AP_SHIFT;
      send_status(s, "crater_avoided");
    } else {                                         /* reached a Y boundary -> next lane */
      s->ap_state = AP_SHIFT;
      send_status(s, "row_end");
    }
    return;
  }

  /* ===================== REACTIVE CUBE GRAB =====================
   * The instant the bottom/middle ToF catches an unhandled object in our half, drop
   * whatever lane-keeping / turning / detouring we're doing and go straight at it.
   * Because turns are now taken in small ToF-sampled steps, this fires even while
   * spinning -- so a cube the beam sweeps past during a U-turn is approached + hugged
   * + scanned immediately, never skipped. (Not while already approaching/scanning a
   * cube, avoiding, or done; the just-scanned cube is blacklisted so it won't re-grab.) */
  if (s->homed && (s->ap_state == AP_ROW || s->ap_state == AP_SHIFT || s->ap_state == AP_DETOUR)) {
    const double lo = object_front_dist_low(&d);
    if (lo > 0.0 && lo < OBJ_DETECT_M && !object_ahead_blacklisted(s, lo)) {
      double ox = 0.0, oy = 0.0;
      project_from_pose(&s->pose, lo, 0.0, &ox, &oy);
      send_plan(ox, oy);
      s->goto_steps = 0;
      s->ap_state = AP_GOTO;          /* GOTO -> APPROACH -> EVALUATE hugs + scans it */
      return;
    }
  }

  switch (s->ap_state) {
    case AP_ROW: {
      /* Lanes live on an ABSOLUTE grid (x = row_x = k*spacing from the dividing line).
       * Whatever knocked us off the lane -- a grabbed cube, a detour round the mountain
       * -- we slide back onto row_x FIRST, so coverage stays a clean boustrophedon no
       * matter how much the diversions moved us. Then turn onto the lane heading and
       * drive a step. (Sliding +/-x that runs into the far X boundary = half swept; the
       * black handler turns that into DONE.) */
      const double dx = s->row_x - s->pose.x;
      if (fabs(dx) > LANE_X_TOL) {                  /* off the lane -> return to its x */
        if (!ap_face(s, (dx > 0.0) ? 0.0 : M_PI)) break;
        move_forward_m(s, fabs(dx) < ROW_STEP_M ? fabs(dx) : ROW_STEP_M, true);
        break;
      }
      const double rh = ap_row_heading(s->row_dir);
      if (!ap_face(s, rh)) break;                /* still turning onto the lane */

      const double front = object_front_dist(&d);
      if (front > 0.0 && front < OBJ_DETECT_M) {
        if (!object_ahead_blacklisted(s, front)) {   /* a NEW object dead ahead -> go to it */
          s->goto_steps = 0;
          send_plan(s->pose.x + cos(rh) * front, s->pose.y + sin(rh) * front);
          s->ap_state = AP_GOTO;
          break;
        }
        /* mountain / already-handled obstacle ahead -> go around (unless we just did) */
        if (s->cooldown == 0 && front < GOTO_CLOSE_M + ROW_STEP_M) {
          s->detour_phase = 0;
          s->shift_acc = 0.0;
          s->ap_state = AP_DETOUR;
          break;
        }
      }
      if (s->cooldown > 0) s->cooldown--;             /* counting down past the last obstacle */
      move_forward_m(s, ROW_STEP_M, true);            /* safety on: abort instantly if IR sees black */
      break;
    }

    case AP_SHIFT: {
      /* Advance to the next lane (one ROW_SPACING deeper into our half) and reverse the
       * sweep direction. AP_ROW slides over to the new lane's x and drives it; if that
       * sideways move runs into the far boundary, the black handler ends with DONE. */
      s->row_x += (double)HALF_DIR * ROW_SPACING_M;
      s->row_dir = -s->row_dir;
      /* progress / stall safety net (in case obstacles keep us from reaching new ground) */
      if ((double)HALF_DIR * s->row_x > (double)HALF_DIR * s->lawn_max_x + 0.01) {
        s->lawn_max_x = s->row_x;
        s->stall_lanes = 0;
      } else if (++s->stall_lanes >= 3) {
        send_status(s, "coverage_complete");
        s->ap_state = AP_DONE;
        break;
      }
      s->ap_state = AP_ROW;
      break;
    }

    case AP_GOTO: {
      /* Drive up to the object, stopping SHORT at GOTO_CLOSE -> APPROACH (never a bump).
       * If it slips off the narrow beam (it was off to the side -- belongs to another
       * lane) or we chase too long, give up and carry on with the lane. */
      const double front = object_front_dist(&d);
      if (front < 0.0 || ++s->goto_steps > GOTO_MAX_STEPS) { s->ap_state = AP_ROW; break; }
      if (front <= GOTO_CLOSE_M + REACH_TOL_M) { s->ap_state = AP_APPROACH; break; }
      const double step = front - GOTO_CLOSE_M;
      move_forward_m(s, step > CELL_M ? CELL_M : (step > REACH_TOL_M ? step : REACH_TOL_M), false);
      break;
    }

    case AP_APPROACH: {
      /* Creep to colour-read range, classifying by sensor height. A mountain (top
       * ToF sees it) is not a cube -> hand off to the obstacle path. */
      const double front = object_front_dist(&d);
      if (front < 0.0) { s->ap_state = AP_ROW; break; }
      const int kind = classify_object(&d);
      if (kind == 2) { s->avoid_reason = 1; s->ap_state = AP_AVOID; send_status(s, "mountain_detected"); break; }
      s->rock_is_large = (kind == 1) ? 1 : 0;
      if (front <= COLOR_READ_M + REACH_TOL_M) { s->ap_state = AP_EVALUATE; break; }
      const double step = front - COLOR_READ_M;
      move_forward_m(s, step > 0.02 ? 0.02 : (step > REACH_TOL_M ? step : REACH_TOL_M), false);
      break;
    }

    case AP_EVALUATE: {
      /* Drive RIGHT up to the cube (hug it) so the NTC actually contacts it -- the
       * temperature read is only accurate touching. Then read colour + temp, report
       * to the UI + share over MQTT, remember + blacklist the cube, back off, resume. */
      /* Compute the cube's CENTRE first (near-edge ToF distance + half its width) so
       * the blacklist lands ON the cube regardless of how far the hug actually reaches
       * -- otherwise a large cube read from one side gets re-scanned from the other. */
      double near = object_front_dist(&d);
      if (near < 0.0) near = COLOR_READ_M;
      double bx = 0.0, by = 0.0;
      project_from_pose(&s->pose, near + (s->rock_is_large ? 0.03 : 0.015), 0.0, &bx, &by);
      move_forward_m(s, HUG_NUDGE_M, false);     /* close the last gap + press gently */
      const color_detection_t color = read_color_detection();
      const float temp_c = temp_read_celsius();
      if (color.detected) {
        send_scannable(bx, by, color.label, color.confidence, s->rock_is_large ? 2 : 1, temp_c);
        send_mqtt((int)lround(bx * 100.0), (int)lround(by * 100.0),
                  color_label_to_mqtt(color.label), s->rock_is_large ? 6 : 3, temp_c);
        send_status(s, "cube_scanned");
      }
      cube_remember(bx, by);
      move_forward_m(s, -(SWEEP_BACKOFF_M + HUG_NUDGE_M), false);  /* back off cube + hug */
      s->ap_state = AP_ROW;
      break;
    }

    case AP_AVOID: {
      /* Mountain: map + blacklist it, back off, then go around it (DETOUR). */
      double ox = 0.0, oy = 0.0;
      project_from_pose(&s->pose, GOTO_CLOSE_M, 0.0, &ox, &oy);
      ap_set(ap_gx(ox), ap_gy(oy), CELL_MOUNTAIN);
      send_observation("mountain", ox, oy, 0.85);
      ap_blacklist_add(ap_gx(ox), ap_gy(oy));
      move_forward_m(s, -SWEEP_BACKOFF_M, false);
      s->detour_phase = 0;
      s->shift_acc = 0.0;
      s->ap_state = AP_DETOUR;
      break;
    }

    case AP_DETOUR: {
      /* Go around an obstacle and RETURN to the same lane, keeping the grid intact:
       *   (0) sidestep into our half to clear it,
       *   (1) advance along the lane FAR enough to be fully past it,
       *   (2) step back to the lane's x and resume (with a short cooldown).
       * Every phase is COUNT-based (shift_acc), so it always terminates -- it can never
       * freeze, even if a step is physically blocked. Because (1) gets us past the
       * obstacle in Y before (2) steps back, the return is always over clear ground
       * (no ram -- important since the open-loop motors can't feel a block). */
      const double rh   = ap_row_heading(s->row_dir);
      const double into = ap_shift_heading();                 /* +HALF_DIR x (into our half)  */
      const double back = (HALF_DIR > 0) ? M_PI : 0.0;        /* -HALF_DIR x (toward the lane) */
      if (s->detour_phase == 0) {                             /* sidestep clear */
        if (!ap_face(s, into)) break;                         /* turn (ToF-sampled) */
        move_forward_m(s, ROW_STEP_M, true);
        s->shift_acc += ROW_STEP_M;
        if (s->shift_acc >= DETOUR_SIDESTEP_M) { s->shift_acc = 0.0; s->detour_phase = 1; }
        break;
      }
      if (s->detour_phase == 1) {                             /* advance past it along the lane */
        if (!ap_face(s, rh)) break;
        move_forward_m(s, ROW_STEP_M, true);
        s->shift_acc += ROW_STEP_M;
        if (s->shift_acc >= DETOUR_PASS_M) { s->shift_acc = 0.0; s->detour_phase = 2; }
        break;
      }
      if (!ap_face(s, back)) break;                           /* step back to the lane */
      move_forward_m(s, ROW_STEP_M, true);
      s->shift_acc += ROW_STEP_M;
      if (s->shift_acc >= DETOUR_SIDESTEP_M) { s->cooldown = DETOUR_COOLDOWN; s->ap_state = AP_ROW; }
      break;
    }

    case AP_DONE: {
      sleep_msec(LOOP_SLEEP_MS);                       /* whole half swept -> hold */
      break;
    }
  }
}

/* Manual drive: small step per loop while a direction is held (turn has
 * priority over forward). Driven by {"type":"drive","fwd":..,"turn":..}
 * sent from the UI's d-pad / WASD. Idle (no key) just waits. */
#define MANUAL_STEP_M    0.04
#define MANUAL_TURN_RAD  0.12

static void manual_step(robot_state_t *s) {
  if (s->man_turn != 0) {
    rotate_rad(s, (double)s->man_turn * MANUAL_TURN_RAD, false);
  } else if (s->man_fwd > 0) {
    move_forward_m(s, MANUAL_STEP_M, true);     /* forward: tape/mountain safety on */
  } else if (s->man_fwd < 0) {
    move_forward_m(s, -MANUAL_STEP_M, false);   /* reverse: nothing watching behind */
  } else {
    sleep_msec(LOOP_SLEEP_MS);                  /* key released: hold position */
  }
}

/* ----------------------------- Hardware ---------------------------- */

static void setup_hardware(void) {
  /* Comms run over stdio (ws_bridge.py pipes them to the browser):
   *   JSON telemetry -> stdout,  commands -> stdin (non-blocking),
   *   diagnostics    -> stderr.                                        */
  setvbuf(stdout, NULL, _IOLBF, 0);
  fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);

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
  fprintf(stderr, "[robot] sensors up -- streaming JSON on stdout. Ctrl+C to stop.\n");

  /* UART0 on AR0/AR1 for the ESP32 / MQTT link to the partner robot. */
  switchbox_set_pin(IO_AR0, SWB_UART0_RX);
  switchbox_set_pin(IO_AR1, SWB_UART0_TX);
  uart_init(UART0);
  mqtt_enable();                 /* UART0 is up -> publishes (rocks + mirrored UI) allowed */
  fprintf(stderr, "[robot " ROBOT_LABEL "] MQTT link up on UART0 (AR0/AR1); "
                  "own half = x%s0, dividing line x=0.\n", (HALF_DIR > 0) ? ">=" : "<=");

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
  stepper_disable();
  stepper_destroy();
#if USE_PULSECOUNTER_ODOMETRY
  pulsecounter_destroy(PULSECOUNTER0);
  pulsecounter_destroy(PULSECOUNTER1);
#endif
  iic_destroy(IIC0);
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

  setup_hardware();
  sync_odometry_from_pulsecounters(&state);

  send_status(&state, "boot");
  send_pose(&state);

  uint64_t last_loop_hb = 0;

  while (true) {
    poll_commands(&state);
    poll_partner_mqtt(&state);
    report_periodic(&state);

    /* Heartbeat to stderr (-> /tmp/ws.log): loop alive, mode, ap-state, pose,
     * the three ToF heights, tape, temp. */
    const uint64_t hb_now = monotonic_ms();
    if (hb_now - last_loop_hb >= 1000) {
      const distance_readings_t hb_d = read_distance_sensors();
      bool hb_irl = false, hb_irr = false;
      ir_read(&hb_irl, &hb_irr);
      const bool hb_tape = hb_irl || hb_irr;
      const double hb_temp = temp_read_celsius();
      fprintf(stderr,
              "[robot] tick: mode=%s ap=%d pose=(%.2f, %.2f, %.0fdeg) "
              "tof[bot %.2f mid %.2f top %.2f]m tape=%d temp=%.1fC\n",
              mode_name(state.mode), (int)state.ap_state, state.pose.x, state.pose.y,
              state.pose.theta * 180.0 / M_PI,
              hb_d.bottom_m, hb_d.middle_m, hb_d.top_m, hb_tape, hb_temp);
      /* live sensor snapshot -> desktop monitor's sensor panel + ToF cloud */
      send_telemetry(&hb_d, hb_irl, hb_irr, hb_temp,
                     state.mode == MODE_EXPLORE || state.mode == MODE_TARGET ||
                     state.mode == MODE_MANUAL);
      last_loop_hb = hb_now;
    }

    if (state.mode == MODE_STOP) {
      sleep_msec(LOOP_SLEEP_MS);
      continue;
    }
    if (state.mode == MODE_MANUAL) {
      scan_and_report(&state);
      manual_step(&state);
    } else if (state.mode == MODE_TARGET) {
      scan_and_report(&state);
      target_step(&state);
    } else {
      autopilot_step(&state);   /* MODE_EXPLORE: frontier + SLAM */
    }
  }

  cleanup_hardware();
  return EXIT_SUCCESS;
}
