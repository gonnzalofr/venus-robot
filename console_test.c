#define _POSIX_C_SOURCE 200809L

/*
 *  TU/e 5EID0 - Venus Project
 *  CONSOLE connection test -- streams the rover UI JSON straight out the
 *  Pynq's USB serial console (/dev/ttyPS0), so the browser can read it via
 *  Web Serial as "cu.debug-console". NO sensors, NO motors, NO libpynq --
 *  pure userspace, can't touch any hardware.
 *
 *  This is for proving the UART <-> browser link over the USB cable you
 *  already have, with no ESP32 in the loop.
 *
 *  Build (drop in as applications/UIcode_test/main.c, then `make`):
 *     the stock libpynq app Makefile compiles it fine -- it's plain C.
 *
 *  Run:
 *     sudo ./main                 # writes to /dev/ttyPS0
 *     sudo ./main /dev/ttyPS1     # if your console is a different device
 *
 *  Then in the browser open the UI, CONNECT, pick "cu.debug-console".
 *
 *  Heartbeat goes to stderr (your SSH terminal); JSON goes to the console
 *  device only, so the two never mix.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef ROBOT_ID
#define ROBOT_ID "r1"
#endif

#define DEFAULT_CONSOLE_DEV "/dev/ttyPS0"

#define SIM_DT_MS            50
#define SIM_SPEED_MPS        0.18
#define SIM_TURN_RATE_RPS    1.2
#define SIM_ARENA_LIMIT_M    1.4
#define POSE_REPORT_MS       300
#define STATUS_REPORT_MS     2000
#define DISCOVER_RANGE_M     0.22

typedef enum { MODE_STOP = 0, MODE_EXPLORE, MODE_TARGET } robot_mode_t;
typedef struct { double x, y, theta; } pose_t;
typedef struct {
  double x, y;
  const char *kind;   /* "mountain" | "black_tape" | "scannable" */
  const char *color;  /* scannable only, else NULL               */
  bool reported;
} world_object_t;

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

static int g_fd = -1;   /* console device fd, or -1 -> stdout */

/* nanosleep wrapper (usleep is not declared under _POSIX_C_SOURCE 200809L). */
static void sleep_ms(long ms) {
  struct timespec ts;
  ts.tv_sec  = ms / 1000;
  ts.tv_nsec = (ms % 1000) * 1000000L;
  nanosleep(&ts, NULL);
}

/* ----------------------------- output ------------------------------ */

static void out_str(const char *s) {
  const size_t n = strlen(s);
  if (g_fd >= 0) {
    if (write(g_fd, s, n) < 0) { /* ignore transient write errors */ }
  } else {
    fwrite(s, 1, n, stdout);
    fflush(stdout);
  }
}

/* ----------------------------- helpers ----------------------------- */

static uint64_t monotonic_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static double normalize_angle(double a) {
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
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
  out_str(msg);
}

static void send_status(robot_mode_t mode, const char *detail) {
  char msg[224];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"status\",\"robot\":\"%s\",\"t_ms\":%llu,"
           "\"mode\":\"%s\",\"detail\":\"%s\"}\n",
           ROBOT_ID, (unsigned long long)monotonic_ms(), mode_name(mode), detail);
  out_str(msg);
}

static void send_observation(const char *kind, double x, double y, double conf) {
  char msg[224];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"observation\",\"robot\":\"%s\",\"t_ms\":%llu,"
           "\"kind\":\"%s\",\"x\":%.4f,\"y\":%.4f,\"confidence\":%.2f}\n",
           ROBOT_ID, (unsigned long long)monotonic_ms(), kind, x, y, conf);
  out_str(msg);
}

static void send_scannable(double x, double y, const char *color, double conf) {
  char msg[256];
  snprintf(msg, sizeof(msg),
           "{\"type\":\"observation\",\"robot\":\"%s\",\"t_ms\":%llu,"
           "\"kind\":\"scannable\",\"color\":\"%s\",\"x\":%.4f,"
           "\"y\":%.4f,\"confidence\":%.2f}\n",
           ROBOT_ID, (unsigned long long)monotonic_ms(), color, x, y, conf);
  out_str(msg);
}

/* ------------------------- simulated motion ------------------------ */

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
      if (strcmp(o->kind, "scannable") == 0)
        send_scannable(o->x, o->y, o->color, 0.92);
      else
        send_observation(o->kind, o->x, o->y, 0.80);
    }
  }
}

/* ------------------------------- main ------------------------------ */

int main(int argc, char **argv) {
  const char *dev = (argc > 1) ? argv[1] : DEFAULT_CONSOLE_DEV;

  g_fd = open(dev, O_WRONLY | O_NOCTTY);
  if (g_fd < 0) {
    fprintf(stderr, "[console-test] could not open %s (%s) -- writing JSON to stdout instead.\n",
            dev, strerror(errno));
  } else {
    fprintf(stderr, "[console-test] JSON -> %s  (open this as 'cu.debug-console' in the browser)\n", dev);
  }
  fprintf(stderr, "[console-test] streaming... loops forever, Ctrl+C to stop. Heartbeat below:\n");

  pose_t pose = { 0.0, 0.0, 0.0 };
  robot_mode_t mode = MODE_EXPLORE;
  const double dt_s = SIM_DT_MS / 1000.0;
  double wander_phase = 0.0;

  send_status(mode, "boot_console_test");
  send_pose(&pose);

  uint64_t last_pose_ms = 0, last_status_ms = 0, last_hb_ms = 0;

  while (1) {
    /* wander, biased back to centre near the edges so it sweeps the objects */
    double desired;
    if (hypot(pose.x, pose.y) > SIM_ARENA_LIMIT_M) {
      desired = atan2(-pose.y, -pose.x);
    } else {
      wander_phase += 0.6 * dt_s;
      desired = pose.theta + 0.5 * sin(wander_phase);
    }
    sim_drive(&pose, desired, dt_s);
    maybe_discover(&pose);

    const uint64_t now = monotonic_ms();
    if (now - last_pose_ms   >= POSE_REPORT_MS)   { send_pose(&pose);          last_pose_ms = now; }
    if (now - last_status_ms >= STATUS_REPORT_MS) { send_status(mode, "alive"); last_status_ms = now; }
    if (now - last_hb_ms     >= 1000) {
      fprintf(stderr, "[console-test] alive  pose=(%.2f, %.2f, %.2f)\n", pose.x, pose.y, pose.theta);
      last_hb_ms = now;
    }

    sleep_ms(SIM_DT_MS);
  }

  if (g_fd >= 0) close(g_fd);
  return 0;
}
