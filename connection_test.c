#define _POSIX_C_SOURCE 200809L

/*
 *  TU/e 5EID0 - Venus Project
 *  CONNECTION TEST firmware -- NO sensors / NO motors required.
 *
 *  Purpose: verify the UART <-> ESP32 <-> browser (UI_live_robot.html)
 *  link end-to-end without any peripherals wired up. It:
 *     - drives a *simulated* rover around a small arena,
 *     - streams the exact same JSON the real firmware does (pose / status /
 *       observation), so the UI map + sample list populate,
 *     - "discovers" a handful of predefined fake objects as the sim rover
 *       passes near them,
 *     - honours incoming {"type":"target",...} and {"type":"mode",...}
 *       commands so you can confirm the browser->robot direction too.
 *
 *  Build (no sensors.c / vl53l0x.c needed):
 *     gcc connection_test.c -I<libpynq include> -L<libpynq lib> -lpynq -lm \
 *         -o connection_test
 *
 *  The ONLY hardware this touches is the Pynq UART0 routed to the ESP32.
 *  Set the two UART pins below to match your wiring.
 */

#include <libpynq.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef ROBOT_ID
#define ROBOT_ID "r1"
#endif

/* UART0 to the ESP32 bridge. These are the pins from the libpynq UART
 * example (IO_AR0 = RX, IO_AR1 = TX). The connection test uses NO ToF
 * sensors, so there is no XSHUT conflict here -- this is the right place
 * for the link. (On the full robot, ToF wants AR0/AR1 for XSHUT, so the
 * UART has to move and the wiring must be reconciled with the HW team.) */
#define UART_RX_PIN IO_AR0
#define UART_TX_PIN IO_AR1

/* Simulation tuning */
#define SIM_DT_MS            50      /* loop period                     */
#define SIM_SPEED_MPS        0.18    /* forward speed while moving       */
#define SIM_TURN_RATE_RPS    1.2     /* max turn rate (rad/s)            */
#define SIM_ARENA_LIMIT_M    1.4     /* steer back toward centre past this */
#define POSE_REPORT_MS       300
#define STATUS_REPORT_MS     2000
#define DISCOVER_RANGE_M     0.22    /* how close to "discover" an object */
#define TARGET_REACHED_M     0.06

typedef enum { MODE_STOP = 0, MODE_EXPLORE, MODE_TARGET } robot_mode_t;

typedef struct { double x, y, theta; } pose_t;

typedef struct {
  double x, y;
  const char *kind;   /* "mountain" | "black_tape" | "scannable" */
  const char *color;  /* only for scannable, else NULL           */
  bool reported;
} world_object_t;

/* Fake objects scattered around the arena (metres). These stand in for the
 * rocks/walls that will physically be there with the real robot. */
static world_object_t g_objects[] = {
  { 0.60,  0.40, "scannable", "red",    false },
  {-0.50,  0.85, "scannable", "green",  false },
  { 0.95, -0.60, "scannable", "blue",   false },
  {-0.80, -0.70, "scannable", "yellow", false },
  { 1.20,  0.20, "mountain",  NULL,     false },
  { 0.00,  1.30, "mountain",  NULL,     false },
  {-1.30,  0.10, "mountain",  NULL,     false },
  { 0.30, -1.20, "black_tape",NULL,     false },
};
static const int g_object_count = (int)(sizeof(g_objects) / sizeof(g_objects[0]));

/* ----------------------------- helpers ----------------------------- */

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

static void uart_send_string(const char *t) {
  for (size_t i = 0; t[i]; ++i) uart_send(UART0, (uint8_t)t[i]);
}

static const char *mode_name(robot_mode_t m) {
  switch (m) {
    case MODE_EXPLORE: return "explore";
    case MODE_TARGET:  return "target";
    default:           return "stop";
  }
}

/* ----------------------------- JSON out ---------------------------- */

static void send_pose(const pose_t *p) {
  char msg[192];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"pose\",\"robot\":\"%s\",\"t_ms\":%llu,"
           "\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f}\n",
           ROBOT_ID, (unsigned long long)monotonic_ms(), p->x, p->y, p->theta);
  uart_send_string(msg);
}

static void send_status(robot_mode_t mode, const char *detail) {
  char msg[224];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"status\",\"robot\":\"%s\",\"t_ms\":%llu,"
           "\"mode\":\"%s\",\"detail\":\"%s\"}\n",
           ROBOT_ID, (unsigned long long)monotonic_ms(), mode_name(mode), detail);
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

static void send_scannable(double x, double y, const char *color, double conf) {
  char msg[256];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"observation\",\"robot\":\"%s\",\"t_ms\":%llu,"
           "\"kind\":\"scannable\",\"color\":\"%s\",\"x\":%.4f,"
           "\"y\":%.4f,\"confidence\":%.2f}\n",
           ROBOT_ID, (unsigned long long)monotonic_ms(), color, x, y, conf);
  uart_send_string(msg);
}

/* ----------------------------- JSON in ----------------------------- */

static bool json_get_double(const char *json, const char *key, double *out) {
  const char *pos = strstr(json, key);
  if (!pos) return false;
  pos = strchr(pos, ':'); if (!pos) return false;
  ++pos;
  char *end = NULL;
  const double v = strtod(pos, &end);
  if (end == pos) return false;
  *out = v; return true;
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
  memcpy(out, pos, cn); out[cn] = '\0';
  return true;
}

typedef struct {
  robot_mode_t mode;
  bool has_target;
  double target_x, target_y;
} ctrl_t;

static void handle_command_line(ctrl_t *c, const char *line) {
  if (strstr(line, "\"type\":\"target\"") || strstr(line, "\"type\": \"target\"")) {
    double x = 0.0, y = 0.0;
    if (json_get_double(line, "\"x\"", &x) && json_get_double(line, "\"y\"", &y)) {
      c->target_x = x; c->target_y = y;
      c->has_target = true; c->mode = MODE_TARGET;
      send_status(c->mode, "target_received");
    }
    return;
  }
  if (strstr(line, "\"type\":\"mode\"") || strstr(line, "\"type\": \"mode\"")) {
    char mode[24];
    if (json_get_string(line, "\"mode\"", mode, sizeof(mode))) {
      if (strcmp(mode, "stop") == 0) {
        c->mode = MODE_STOP; c->has_target = false;
        send_status(c->mode, "mode_stop");
      } else if (strcmp(mode, "explore") == 0) {
        c->mode = MODE_EXPLORE; c->has_target = false;
        send_status(c->mode, "mode_explore");
      }
    }
  }
}

static void poll_uart(ctrl_t *c) {
  static char line[256];
  static size_t len = 0;
  while (uart_has_data(UART0)) {
    const char ch = (char)uart_recv(UART0);
    if (ch == '\n' || ch == '\r') {
      if (len > 0) { line[len] = '\0'; handle_command_line(c, line); len = 0; }
    } else if (len + 1 < sizeof(line)) {
      line[len++] = ch;
    } else {
      len = 0;
    }
  }
}

/* ------------------------- simulated motion ------------------------ */

/* Steer `pose` toward `desired_heading` at most SIM_TURN_RATE, then advance
 * forward by SIM_SPEED * dt. Returns distance advanced. */
static void sim_drive(pose_t *p, double desired_heading, double dt_s) {
  double err = normalize_angle(desired_heading - p->theta);
  const double max_turn = SIM_TURN_RATE_RPS * dt_s;
  if (err >  max_turn) err =  max_turn;
  if (err < -max_turn) err = -max_turn;
  p->theta = normalize_angle(p->theta + err);

  const double step = SIM_SPEED_MPS * dt_s;
  p->x += step * cos(p->theta);
  p->y += step * sin(p->theta);
}

static void maybe_discover(const pose_t *p) {
  for (int i = 0; i < g_object_count; ++i) {
    world_object_t *o = &g_objects[i];
    if (o->reported) continue;
    if (hypot(o->x - p->x, o->y - p->y) <= DISCOVER_RANGE_M) {
      o->reported = true;
      if (strcmp(o->kind, "scannable") == 0) {
        send_scannable(o->x, o->y, o->color, 0.92);
      } else {
        send_observation(o->kind, o->x, o->y, 0.80);
      }
    }
  }
}

/* ------------------------------- main ------------------------------ */

int main(void) {
  pynq_init();
  printf("[conn-test] pynq_init done\n"); fflush(stdout);

  switchbox_set_pin(UART_RX_PIN, SWB_UART0_RX);
  switchbox_set_pin(UART_TX_PIN, SWB_UART0_TX);
  uart_init(UART0);
  uart_reset_fifos(UART0);
  printf("[conn-test] UART0 up on AR0(RX)/AR1(TX) @115200 -- streaming JSON.\n");
  printf("[conn-test] This loops forever (Ctrl+C to stop). Heartbeat below:\n");
  fflush(stdout);

  pose_t pose = { 0.0, 0.0, 0.0 };
  ctrl_t ctrl = { MODE_EXPLORE, false, 0.0, 0.0 };

  send_status(ctrl.mode, "boot_connection_test");
  send_pose(&pose);
  uint64_t last_heartbeat_ms = 0;

  uint64_t last_pose_ms = monotonic_ms();
  uint64_t last_status_ms = monotonic_ms();
  const double dt_s = SIM_DT_MS / 1000.0;

  /* gentle wander state */
  double wander_phase = 0.0;

  while (true) {
    poll_uart(&ctrl);

    if (ctrl.mode == MODE_TARGET && ctrl.has_target) {
      const double dx = ctrl.target_x - pose.x;
      const double dy = ctrl.target_y - pose.y;
      if (hypot(dx, dy) < TARGET_REACHED_M) {
        ctrl.has_target = false;
        ctrl.mode = MODE_EXPLORE;
        send_status(ctrl.mode, "target_reached");
      } else {
        sim_drive(&pose, atan2(dy, dx), dt_s);
      }
    } else if (ctrl.mode == MODE_EXPLORE) {
      /* Wander: gentle curve, but steer back to centre near the edges so
       * the sim rover keeps sweeping past the fake objects. */
      double desired;
      if (hypot(pose.x, pose.y) > SIM_ARENA_LIMIT_M) {
        desired = atan2(-pose.y, -pose.x);
      } else {
        wander_phase += 0.6 * dt_s;
        desired = pose.theta + 0.5 * sin(wander_phase);
      }
      sim_drive(&pose, desired, dt_s);
    }
    /* MODE_STOP: hold position. */

    maybe_discover(&pose);

    const uint64_t now = monotonic_ms();
    if (now - last_pose_ms >= POSE_REPORT_MS) {
      send_pose(&pose);
      last_pose_ms = now;
    }
    if (now - last_status_ms >= STATUS_REPORT_MS) {
      send_status(ctrl.mode, "alive");
      last_status_ms = now;
    }

    /* Heartbeat to the SSH console so you can see it is running, not dead. */
    if (now - last_heartbeat_ms >= 1000) {
      printf("[conn-test] alive  mode=%s  pose=(%.2f, %.2f, %.2f)\n",
             mode_name(ctrl.mode), pose.x, pose.y, pose.theta);
      fflush(stdout);
      last_heartbeat_ms = now;
    }

    sleep_msec(SIM_DT_MS);
  }

  uart_destroy(UART0);
  pynq_destroy();
  return EXIT_SUCCESS;
}
