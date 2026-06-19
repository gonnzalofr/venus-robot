/*
 *  TU/e 5EID0 - Venus Project -- ToF CALIBRATION / DEBUG tool.
 *
 *  Purpose: put known objects in front of the stacked ToF sensors and read the
 *  three distances so the firmware thresholds can be calibrated. For each prompt
 *  you place the object, press ENTER, and the bottom / middle / top ToF are
 *  sampled for ~10 s; paste the output back to tune the robot code.
 *
 *  It uses the SAME sensor module as the robots (sensors.c / vl53l0x.c), so the
 *  readings are exactly what the firmware sees. Build it against either robot --
 *  the sensor module is identical in robot/ and robotv2/:
 *
 *     gcc debugging/ToF/main.c robotv2/sensors.c robotv2/vl53l0x.c -Irobotv2 \
 *         $(pkg-config --cflags --libs libpynq) -lm -o tof_debug
 *     ./tof_debug
 *
 *  Sensor geometry the firmware ASSUMES (this tool checks it):
 *     3x3x3 cube  -> BOTTOM sensor only
 *     6x6x6 cube  -> BOTTOM + MIDDLE
 *     obstacle / mountain (10 cm) -> ALL THREE
 *     empty floor -> the TOP sensor reads its far baseline (MEASURE it -- ~50-57 cm
 *                    on these rigs); a nearer object reads CLOSER, so "top reads
 *                    well below baseline" = a real object.
 *
 *  Firmware thresholds these readings calibrate (keep in sync with main.c):
 *     ROCK_DETECT_M = 0.05      bottom/middle: a cube only counts within 5 cm
 *     MOUNTAIN_DETECT_MIN_M = 0.05, MAX_M = 0.45   top: the "this is a mountain" band
 *     MOUNTAIN_STOP_M = 0.15    top: a mountain this close is blocking the path
 */

#include <libpynq.h>
#include <iic.h>

#include "sensors.h"
#include "vl53l0x.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* ----------------------------- sampling ---------------------------- */
#define SAMPLE_MS      500U                       /* one ToF read every ... */
#define DURATION_MS  10000U                       /* ... for this long per prompt */
#define NSAMP        (DURATION_MS / SAMPLE_MS)

/* ---- firmware thresholds being calibrated, in mm (keep in sync w/ main.c) ---- */
#define ROCK_DETECT_MM     50     /* 0.05 m: bottom/middle cube band */
#define MOUNTAIN_MIN_MM    50     /* 0.05 m: top mountain band, lower edge */
#define MOUNTAIN_MAX_MM   450     /* 0.45 m: top mountain band, upper edge */
#define MOUNTAIN_STOP_MM  150     /* 0.15 m: top "mountain blocking the path" */

typedef struct {
  const char *title;
  const char *expected;
  const char *note;
} scenario_t;

static void wait_enter(void) {
  printf("  >> place the object, then press ENTER to sample for %u s ...",
         (unsigned)(DURATION_MS / 1000U));
  fflush(stdout);
  int c;
  while ((c = getchar()) != '\n' && c != EOF) { }
}

/* One summary line for a single sensor over the sampling window. */
static void summarise(const char *name, uint32_t cnt, uint64_t sum,
                      uint16_t mn, uint16_t mx, uint32_t trig, const char *band) {
  if (cnt == 0U) {
    printf("    %-12s valid 0/%u  -> NO reading (out of range / nothing there)\n",
           name, (unsigned)NSAMP);
    return;
  }
  printf("    %-12s valid %u/%u  min %u  mean %u  max %u mm   [in %s: %u/%u]\n",
         name, (unsigned)cnt, (unsigned)NSAMP, (unsigned)mn,
         (unsigned)(sum / cnt), (unsigned)mx, band, (unsigned)trig, (unsigned)cnt);
}

static void run_scenario(const scenario_t *sc) {
  printf("\n============================================================\n");
  printf("SCENARIO: %s\n", sc->title);
  printf("EXPECTED: %s\n", sc->expected);
  wait_enter();

  printf("\n   t(s) | bottom 3x3 | middle 6x6 |  top  mtn  | trig\n");
  printf("  ------+------------+------------+------------+------\n");

  uint32_t bc = 0, mc = 0, tc = 0;          /* valid (non-zero) counts */
  uint32_t br = 0, mr = 0, tb = 0;          /* in-band (triggered) counts */
  uint64_t bs = 0, ms = 0, ts = 0;          /* sums for mean */
  uint16_t bmn = 0xFFFF, mmn = 0xFFFF, tmn = 0xFFFF;
  uint16_t bmx = 0, mmx = 0, tmx = 0;

  for (uint32_t i = 0; i < NSAMP; ++i) {
    uint16_t b = 0, m = 0, t = 0;
    tof_read(&b, &m, &t);                   /* mm; 0 = invalid / out of range */

    const char bt = (b && b <= ROCK_DETECT_MM) ? 'B' : '.';
    const char mt = (m && m <= ROCK_DETECT_MM) ? 'M' : '.';
    const char tt = (t && t >= MOUNTAIN_MIN_MM && t <= MOUNTAIN_MAX_MM) ? 'T' : '.';
    printf("  %4.1f  | %6u mm  | %6u mm  | %6u mm  | %c%c%c\n",
           (double)(i * SAMPLE_MS) / 1000.0,
           (unsigned)b, (unsigned)m, (unsigned)t, bt, mt, tt);

    if (b) { bc++; bs += b; if (b < bmn) bmn = b; if (b > bmx) bmx = b; if (b <= ROCK_DETECT_MM) br++; }
    if (m) { mc++; ms += m; if (m < mmn) mmn = m; if (m > mmx) mmx = m; if (m <= ROCK_DETECT_MM) mr++; }
    if (t) { tc++; ts += t; if (t < tmn) tmn = t; if (t > tmx) tmx = t;
             if (t >= MOUNTAIN_MIN_MM && t <= MOUNTAIN_MAX_MM) tb++; }

    sleep_msec(SAMPLE_MS);
  }

  printf("\n  SUMMARY (valid = non-zero reading):\n");
  summarise("bottom(3x3)", bc, bs, bmn, bmx, br, "<=5cm");
  summarise("middle(6x6)", mc, ms, mmn, mmx, mr, "<=5cm");
  summarise("top(mtn)",    tc, ts, tmn, tmx, tb, "5-45cm");

  printf("  TRIGGERED by firmware bands: %s%s%s%s\n",
         br ? "bottom " : "", mr ? "middle " : "", tb ? "top " : "",
         (!br && !mr && !tb) ? "(none)" : "");
  if (sc->note && sc->note[0]) printf("  NOTE: %s\n", sc->note);
}

int main(void) {
  printf("=== TU/e Venus -- ToF calibration tool ===\n");
  printf("Geometry the firmware assumes:\n");
  printf("  3x3x3 -> bottom only | 6x6x6 -> bottom+middle | obstacle -> all three\n");
  printf("  empty floor -> top reads its far baseline (~50-57cm; MEASURE it below); a nearer object reads closer.\n");
  printf("Bands being calibrated: cube <=%dmm (bottom/middle); mountain %d-%dmm (top); "
         "mountain-stop %dmm.\n",
         ROCK_DETECT_MM, MOUNTAIN_MIN_MM, MOUNTAIN_MAX_MM, MOUNTAIN_STOP_MM);

  pynq_init();
  const int err = sensors_init_all();   /* brings up the ToF (+ other sensors, harmless) */
  printf("sensors_init_all() = 0x%X  (bit0=ToF bit1=IR bit2=colour bit3=temp; 0 = all up)\n", (unsigned)err);
  if (err & 1) printf("WARNING: ToF reported a problem at init; readings may all be 0.\n");

  static const scenario_t scenarios[] = {
    { "Clear the area -- NOTHING in front (baseline)",
      "All sensors read their empty-floor baseline; top shows its far value (~50-57 cm -- MEASURE it here).",
      "This measured top baseline is the key number: set MOUNTAIN_DETECT_MAX_M to ~10 cm BELOW it so empty "
      "floor never reads as a mountain (firmware currently assumes ~57 cm -> MAX = 0.45 m). Also note any "
      "bottom/middle floor reflection." },

    { "Place a 3x3x3 cube in front",
      "BOTTOM sensor only (cube within 5 cm). middle/top: no trigger.",
      "bottom 'mean' = how close the 3x3 reads; ROCK_DETECT_M must be >= that (+ a small margin). "
      "If middle ALSO reads <=5cm here, the 3x3 is reaching the middle beam (check the sensor stack heights)." },

    { "Place a 6x6x6 cube in front",
      "BOTTOM + MIDDLE (both within 5 cm). top: no trigger.",
      "Both bottom & middle should read <=5cm. If middle reads >5cm, raise ROCK_DETECT_M or re-aim the middle sensor. "
      "If top also triggers, the 6x6 is tall enough to hit the top beam." },

    { "Place an obstacle / mountain (10 cm) in front",
      "ALL THREE trigger; top reads the mountain inside 5-45 cm.",
      "Compare top's distance here with the empty-floor baseline from scenario 1. When the obstacle is close, "
      "top should be < MOUNTAIN_STOP (15 cm). These two numbers set MOUNTAIN_DETECT_MAX_M and MOUNTAIN_STOP_M." },

    { "Place a 3x3x3 cube IN FRONT OF a 6x6x6 cube",
      "bottom = the NEAR 3x3; middle = the 6x6 behind (unless the 3x3 occludes it).",
      "ap_rock_dist() picks the NEAREST of bottom/middle <=5cm. Check bottom reads the near 3x3 and whether middle "
      "still sees the 6x6 at a different (larger) distance -- that difference is how small-vs-large is told apart." },

    { "Place a 3x3x3 cube IN FRONT OF an obstacle",
      "bottom = the NEAR 3x3; middle/top = the obstacle behind.",
      "Confirm the near 3x3 reads on bottom (<=5cm) and that top STILL flags the mountain behind (in 5-45cm band) "
      "so the obstacle isn't hidden by the small cube in front." },

    { "Place a 6x6x6 cube IN FRONT OF an obstacle",
      "bottom + middle = the NEAR 6x6; top = the obstacle behind.",
      "Confirm bottom+middle read the near 6x6 (<=5cm) and top still reads the mountain in-band. If the 6x6 blocks the "
      "top beam (top goes invalid/0 or jumps), the mountain behind is hidden -- important for the mapping logic." },
  };

  const int nsc = (int)(sizeof(scenarios) / sizeof(scenarios[0]));
  for (int i = 0; i < nsc; ++i) run_scenario(&scenarios[i]);

  printf("\n=== DONE -- please paste the output back. Key takeaways: ===\n");
  printf("  * cube distances (3x3 bottom; 6x6 bottom+middle) -> set ROCK_DETECT_M just above the max seen.\n");
  printf("  * empty-floor TOP baseline + obstacle TOP distance -> set MOUNTAIN_DETECT_MIN/MAX_M and MOUNTAIN_STOP_M.\n");
  printf("  * any sensor that triggers when it should not (or vice-versa) -> mounting/threshold fix.\n");
  printf("  * min-vs-max spread per sensor -> reading noise (whether to add ToF median/averaging).\n");

  iic_destroy(IIC0);
  pynq_destroy();
  return EXIT_SUCCESS;
}
