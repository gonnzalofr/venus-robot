/*
 *  TU/e 5EID0 - Venus Project -- COLOUR CALIBRATION / DEBUG tool.
 *
 *  Purpose: put known colour swatches / cubes in front of the TCS3200 colour
 *  sensor to (1) capture the white/black reference counts that calibrate it and
 *  (2) check that each cube colour classifies correctly. For each prompt you
 *  place the swatch, press ENTER, and the sensor is sampled for ~10 s; paste the
 *  output back to tune the firmware.
 *
 *  It uses the SAME sensor module as the robots (sensors.c), so the raw counts
 *  and the classification match what the firmware sees. Build against either
 *  robot -- the sensor module is identical in robot/ and robotv2/ (sensors.c
 *  pulls in the ToF driver, so vl53l0x.c must be linked too):
 *
 *     gcc debugging/color/main.c robotv2/sensors.c robotv2/vl53l0x.c -Irobotv2 \
 *         $(pkg-config --cflags --libs libpynq) -lm -o color_debug
 *     ./color_debug
 *
 *  How the sensor works (so the numbers make sense):
 *     - color_read() returns a raw HIGH-pulse loop count per channel (R/G/B).
 *     - MORE light / more reflective surface -> LOWER count.
 *       White reads LOW (~89,99,91), black reads HIGH (~200,305,281).
 *     - color_classify() maps each raw count to 0..255 using the per-channel
 *       white/black calibration, then picks the nearest palette colour.
 *
 *  WHAT YOU CALIBRATE (the six constants in sensors.c):
 *     COL_R_WHITE / COL_G_WHITE / COL_B_WHITE  <- the WHITE run's mean counts
 *     COL_R_BLACK / COL_G_BLACK / COL_B_BLACK  <- the BLACK run's mean counts
 *  Then the RED/GREEN/BLUE/YELLOW runs verify classification.
 *
 *  IMPORTANT: calibrate at the SAME distance the robot reads a cube
 *  (ROCK_APPROACH_M ~ 2 cm) and under the SAME lighting -- the counts are only
 *  valid for that geometry. Keep the S0/S1 frequency scaling as color_init() sets
 *  it (20%); changing it rescales every count. The two physical sensors (r1 vs
 *  r2) may differ slightly, so calibrate each robot.
 */

#include <libpynq.h>
#include <iic.h>

#include "sensors.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* ----------------------------- sampling ---------------------------- */
#define SAMPLE_MS      500U
#define DURATION_MS  10000U
#define NSAMP        (DURATION_MS / SAMPLE_MS)

/* Current calibration in sensors.c, shown for reference so you can compare your
 * measured means against it (update sensors.c if they differ). */
#define CUR_R_WHITE  89
#define CUR_R_BLACK 200
#define CUR_G_WHITE  99
#define CUR_G_BLACK 305
#define CUR_B_WHITE  91
#define CUR_B_BLACK 281

typedef struct {
  const char *title;
  const char *expected;     /* the label we expect color_classify() to return */
  const char *ref_kind;     /* "WHITE", "BLACK", or NULL (verification only) */
} cscenario_t;

static void wait_enter(void) {
  printf("  >> place the swatch, then press ENTER to sample for %u s ...",
         (unsigned)(DURATION_MS / 1000U));
  fflush(stdout);
  int c;
  while ((c = getchar()) != '\n' && c != EOF) { }
}

static void run_cscenario(const cscenario_t *sc) {
  printf("\n============================================================\n");
  printf("SCENARIO: %s\n", sc->title);
  printf("EXPECTED classification: %s\n", sc->expected);
  wait_enter();

  printf("\n   t(s) |   R   |   G   |   B   | classified (confidence)\n");
  printf("  ------+-------+-------+-------+-------------------------\n");

  uint64_t rs = 0, gs = 0, bs = 0;
  uint32_t cnt = 0;
  int label_count[7] = {0};   /* indexed by scan_color_t (0..6) */
  double conf_sum = 0.0;

  for (uint32_t i = 0; i < NSAMP; ++i) {
    uint32_t r = 0, g = 0, b = 0;
    color_read(&r, &g, &b);                 /* raw HIGH-pulse counts */
    float conf = 0.0f;
    const scan_color_t c = color_classify(r, g, b, &conf);

    printf("  %4.1f  | %5u | %5u | %5u | %-7s (%.2f)\n",
           (double)(i * SAMPLE_MS) / 1000.0,
           (unsigned)r, (unsigned)g, (unsigned)b, color_label_name(c), (double)conf);

    rs += r; gs += g; bs += b; cnt++;
    if ((int)c >= 0 && (int)c < 7) label_count[(int)c]++;
    conf_sum += (double)conf;
    sleep_msec(SAMPLE_MS);
  }

  if (cnt == 0U) { printf("  (no samples)\n"); return; }

  const uint32_t rm = (uint32_t)(rs / cnt), gm = (uint32_t)(gs / cnt), bm = (uint32_t)(bs / cnt);

  /* most frequent classification across the window */
  int best = 0;
  for (int k = 1; k < 7; ++k) if (label_count[k] > label_count[best]) best = k;

  printf("\n  MEAN raw counts:  R=%u  G=%u  B=%u   (mean confidence %.2f)\n",
         (unsigned)rm, (unsigned)gm, (unsigned)bm, conf_sum / (double)cnt);
  printf("  Classified as '%s' in %d/%u samples.\n",
         color_label_name((scan_color_t)best), label_count[best], (unsigned)NSAMP);

  if (sc->ref_kind) {
    printf("  -> CALIBRATION: set in sensors.c   COL_R_%s=%u  COL_G_%s=%u  COL_B_%s=%u\n",
           sc->ref_kind, (unsigned)rm, sc->ref_kind, (unsigned)gm, sc->ref_kind, (unsigned)bm);
    printf("     (current sensors.c values: R/G/B %s = %d/%d/%d)\n",
           sc->ref_kind,
           (sc->ref_kind[0] == 'W') ? CUR_R_WHITE : CUR_R_BLACK,
           (sc->ref_kind[0] == 'W') ? CUR_G_WHITE : CUR_G_BLACK,
           (sc->ref_kind[0] == 'W') ? CUR_B_WHITE : CUR_B_BLACK);
  } else {
    printf("  -> VERIFY: expected '%s'. If it misclassifies or confidence is low, "
           "re-check the white/black calibration, distance and lighting.\n", sc->expected);
  }
}

int main(void) {
  printf("=== TU/e Venus -- colour calibration tool ===\n");
  printf("Raw counts: brighter surface -> LOWER count (white ~%d/%d/%d, black ~%d/%d/%d).\n",
         CUR_R_WHITE, CUR_G_WHITE, CUR_B_WHITE, CUR_R_BLACK, CUR_G_BLACK, CUR_B_BLACK);
  printf("Do the WHITE and BLACK runs first -> their means become the COL_*_WHITE/BLACK\n");
  printf("constants in sensors.c; the colour runs then verify classification.\n");
  printf("Calibrate at the cube-read distance (~2 cm) under the real lighting.\n");

  pynq_init();
  const int err = sensors_init_all();   /* brings up the colour sensor (+ others, harmless) */
  printf("sensors_init_all() = 0x%X  (bit0=ToF bit1=IR bit2=colour bit3=temp; 0 = all up)\n", (unsigned)err);
  if (err & 4) printf("WARNING: colour sensor reported a problem at init; counts may be 0.\n");

  static const cscenario_t scenarios[] = {
    { "Floor / background (no cube)", "none / black (background)", NULL },
    { "WHITE reference swatch",        "white", "WHITE" },
    { "BLACK reference swatch",        "black", "BLACK" },
    { "RED cube",                      "red",   NULL },
    { "GREEN cube",                    "green", NULL },
    { "BLUE cube",                     "blue",  NULL },
    { "YELLOW cube",                   "yellow", NULL },
  };

  const int nsc = (int)(sizeof(scenarios) / sizeof(scenarios[0]));
  for (int i = 0; i < nsc; ++i) run_cscenario(&scenarios[i]);

  printf("\n=== DONE -- please paste the output back. Key takeaways: ===\n");
  printf("  * WHITE / BLACK mean counts -> the six COL_*_WHITE / COL_*_BLACK constants in sensors.c.\n");
  printf("  * each colour run's classification + confidence -> does the palette resolve correctly?\n");
  printf("  * if a colour is consistently wrong, note its raw R/G/B means so the palette can be retuned.\n");
  printf("  * low / erratic confidence -> uncertain classification; improve calibration, distance or\n");
  printf("    lighting (the firmware reports confidence with each find but does not gate on it).\n");

  iic_destroy(IIC0);
  pynq_destroy();
  return EXIT_SUCCESS;
}
