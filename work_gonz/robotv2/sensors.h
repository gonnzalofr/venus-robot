#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

/*
 *  TU/e 5EID0 - Venus Project, Group 10
 *  Embedded Software - Sensor Module
 *
 *  Public API used by the robot main loop. Each underlying sensor
 *  (3x VL53L0X ToF, 2x TCRT5000 IR, 1x TCS3200 colour, 1x NTC-10K
 *  thermistor) has its own init + read function, plus there is a
 *  single update_sensor_data() that polls everything in one go.
 */

/* This is THE struct the robot main code uses. Replace the local
 * definition in robot main.c with `#include "sensors.h"`.        */
typedef struct {
    /* The three ToF sensors are stacked vertically and aimed forward:
     *   bottom -> small (3x3) rocks, middle -> large (6x6) rocks,
     *   top    -> mountains. mm; 0 means out-of-range / invalid.        */
    uint16_t tof_bottom_mm;
    uint16_t tof_middle_mm;
    uint16_t tof_top_mm;
    bool     ir_left_detects_tape;
    bool     ir_right_detects_tape;
    uint32_t color_freq_r;
    uint32_t color_freq_g;
    uint32_t color_freq_b;
    float    temp_celsius;
} SensorData;

/* ---------- Top-level ---------- */

/* Brings up every sensor in the right order.
 * Returns 0 if everything came online, non-zero if any sensor failed. */
int  sensors_init_all(void);

/* Polls every sensor and writes the readings into `out`. Never blocks
 * for long; safe to call once per main-loop iteration.                */
void update_sensor_data(SensorData *out);

/* ---------- Time-of-Flight (3x VL53L0X on IIC0) ---------- */

/* Brings the three ToF sensors up via the XSHUT address-reassign
 * sequence (A is always-on, B on AR0, C on AR1).                */
int  tof_init(void);

/* Reads all three ToF distances in mm. Vertical stack aimed forward:
 *   bottom -> small (3x3) rocks, middle -> large (6x6) rocks, top -> mountains.
 * Zero means out-of-range / read error - treat 0 as "invalid". */
void tof_read(uint16_t *bottom, uint16_t *middle, uint16_t *top);

/* ---------- IR tape detector (2x TCRT5000) ---------- */

int  ir_init(void);
void ir_read(bool *left_tape, bool *right_tape);

/* ---------- Colour (TCS3200) ---------- */

/* Discrete colour label produced by the classifier. SCAN_COLOR_NONE means
 * "nothing scannable in view" (too dark / no dominant channel).            */
typedef enum {
    SCAN_COLOR_NONE = 0,
    SCAN_COLOR_RED,
    SCAN_COLOR_GREEN,
    SCAN_COLOR_BLUE,
    SCAN_COLOR_YELLOW,
    SCAN_COLOR_WHITE,
    SCAN_COLOR_BLACK
} scan_color_t;

int  color_init(void);

/* Raw per-filter HIGH-pulse counts (longer = darker through that filter). */
void color_read(uint32_t *r, uint32_t *g, uint32_t *b);

/* Turn three filter frequencies into a colour label. When non-NULL,
 * *confidence is filled in [0..1] (share of total the winning channel holds).
 * Tunables COLOR_MIN_TOTAL_HZ / COLOR_DOMINANT_MARGIN live in sensors.c.    */
scan_color_t color_classify(uint32_t r, uint32_t g, uint32_t b, float *confidence);

/* Convenience: one shot read + classify. */
scan_color_t color_read_label(float *confidence);

/* Human-readable name for a label (e.g. for JSON / logging). */
const char  *color_label_name(scan_color_t label);

/* ---------- Temperature (NTC-10K via ADC, on ADC5) ---------- */

int  temp_init(void);

/* Read the NTC and return degrees Celsius directly (Beta equation:
 * B = 4050, R0 = 10k @ 25C, 10k series resistor, 3.3V, divider on ADC5).
 * Returns a large negative value if the reading is invalid. */
float temp_read_celsius(void);

#endif /* SENSORS_H */
