#include "sensors.h"
#include "vl53l0x.h"

#include <libpynq.h>
#include <iic.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

/*
 *  TU/e 5EID0 - Venus Project, Group 10
 *  Embedded Software - Sensor Module Implementation
 *
 *  - ToF (3x VL53L0X)        : fully implemented (uses the working
 *                              XSHUT-staggered bring-up sequence)
 *  - IR  (2x TCRT5000)       : skeleton, GPIO pin numbers are TODO
 *  - Colour (TCS3200)        : skeleton, frequency counting is TODO
 *  - Temperature (NTC-10K)   : ADC read skeleton + full Celsius math
 *
 *  Pin assignments are gathered here at the top so they are easy to
 *  edit when the electronics sub-team finalises wiring.
 */

/* =====================================================================
 * Pin map (edit here, not scattered through the code)
 * ===================================================================== */

/* --- VL53L0X ToF --- (ROBOT 2 wiring) */
#define TOF_XSHUT_B   IO_AR5   /* XSHUT of ToF sensor B (middle) -> AR5 */
#define TOF_XSHUT_C   IO_AR4   /* XSHUT of ToF sensor C (bottom) -> AR4 */
/* Sensor A's XSHUT is tied high on the board (always on) -> top.
 * Mapping below: A->top, B->middle, C->bottom (verify in sensortest). */

/* --- TCRT5000 IR tape detectors --- (ROBOT 2 wiring) */
#define IR_LEFT_PIN   IO_AR9    /* IR left  DO -> AR9  */
#define IR_RIGHT_PIN  IO_AR10   /* IR right DO -> AR10 */

/* --- TCS3200 colour --- (ROBOT 2 wiring; S0/S1 now wired) */
#define COLOR_S0_PIN  IO_AR11  /* output frequency-scaling select S0 -> AR11 */
#define COLOR_S1_PIN  IO_AR12  /* output frequency-scaling select S1 -> AR12 */
#define COLOR_S2_PIN  IO_AR7   /* photodiode filter-select line S2 -> AR7 */
#define COLOR_S3_PIN  IO_AR8   /* photodiode filter-select line S3 -> AR8 */
#define COLOR_OUT_PIN IO_AR6   /* frequency output -> AR6 */
#define COLOR_SETTLE_MS 10 /* filter settle after S2/S3 flip */

/* Pulse-width method + map_to_rgb calibration (raw HIGH-pulse loop counts).
 * Per channel: <= WHITE -> 255, >= BLACK -> 0, linear in between.
 * Measured 2026-06-18 (debug_color, ~2 cm, mission lighting):
 * white block (10,13,12), black block (17,32,30).
 * Verified red/green/blue all classify correctly with these. */
#define COL_R_WHITE  10u
#define COL_R_BLACK  17u
#define COL_G_WHITE  13u
#define COL_G_BLACK  32u
#define COL_B_WHITE  12u
#define COL_B_BLACK  30u

/* Guard so a stuck OUT pin can't hang the read (returns 0 on timeout). */
#define COL_PULSE_TIMEOUT 1000000u

/* --- NTC-10K thermistor --- (confirmed on ADC5 / A5) */
#define TEMP_ADC_CHANNEL ADC5

/* NTC Beta-equation constants (matches design report eq. 4.1). */
#define NTC_R0      10000.0f
#define NTC_T0_K    298.15f
#define NTC_BETA    4050.0f
#define NTC_SERIES  10000.0f
#define NTC_VREF    3.3f
#define ADC_MAX     4095.0f   /* 12-bit ADC assumed; adjust if wrong   */

/* ToF bring-up addresses. Each sensor ends up at a unique address; we use the
 * team convention (0x68/0x69/0x6A) so sensors left powered between software runs
 * are already where we expect them. */
#define TOF_ADDR_DEFAULT 0x29
#define TOF_ADDR_A       0x68   /* top    (always-on, XSHUT tied high) */
#define TOF_ADDR_B       0x69   /* middle (XSHUT on AR5) */
#define TOF_ADDR_C       0x6A   /* bottom (XSHUT on AR4) */

#define XSHUT_SETTLE_MS 100

/* =====================================================================
 * Module state (static so it is not visible outside this file)
 * ===================================================================== */

static vl53x s_tof_a, s_tof_b, s_tof_c;
static bool  s_tof_a_ok = false, s_tof_b_ok = false, s_tof_c_ok = false;
static bool  s_tof_ready = false;
static bool  s_ir_ready    = false;
static bool  s_color_ready = false;
static bool  s_temp_ready  = false;

/* =====================================================================
 * Internal helpers: VL53L0X bring-up.
 *
 * KEY: the sensors keep their reassigned I2C address as long as they stay
 * powered, so a plain software restart does NOT put them back at 0x29. The
 * bring-up therefore NEVER assumes 0x29 and NEVER pauses/fails the system --
 * it finds each sensor wherever it currently answers, (re)assigns it to its
 * target, and continues regardless.
 * ===================================================================== */

/* Addresses a freshly-powered VL53L0X could be sitting at, in probe order. */
static const uint8_t TOF_ADDR_CANDIDATES[] = {
    TOF_ADDR_DEFAULT,                       /* 0x29 fresh / after a power-cycle */
    TOF_ADDR_A, TOF_ADDR_B, TOF_ADDR_C,     /* our targets (this code, prior run) */
    0x30, 0x31, 0x32                        /* older-firmware targets, just in case */
};

/* Always-on sensor (XSHUT tied high): we can't reset it to 0x29, so find it at
 * whatever address it answers (it's the ONLY device on the bus while B/C are
 * held off) and move it to target. Idempotent + non-fatal. */
/* Try to bring a sensor that answers at cur_addr up at target_addr, printing
 * every step so we can see exactly where it fails. Returns true on success. */
static bool tof_try_at(const char *label, uint8_t cur_addr, uint8_t target_addr, vl53x *handle)
{
    if (cur_addr != target_addr) {
        const int s = tofSetAddress(IIC0, cur_addr, target_addr);
        printf("[%s]   setAddress 0x%02X -> 0x%02X = %d\n", label, cur_addr, target_addr, s);
        fflush(NULL);
    }
    const int r = tofInit(handle, IIC0, target_addr, 0);
    printf("[%s]   tofInit 0x%02X = %d\n", label, target_addr, r);
    fflush(NULL);
    return (r == 0);
}

static bool tof_bring_up_always_on(const char *label, uint8_t target_addr, vl53x *handle)
{
    for (size_t i = 0; i < sizeof(TOF_ADDR_CANDIDATES) / sizeof(TOF_ADDR_CANDIDATES[0]); ++i) {
        const uint8_t a = TOF_ADDR_CANDIDATES[i];
        const int p = tofPing(IIC0, a);
        printf("[%s] ping 0x%02X = %d\n", label, a, p);
        fflush(NULL);
        if (p != 0) continue;
        if (tof_try_at(label, a, target_addr, handle)) {
            printf("[%s] READY at 0x%02X\n", label, target_addr);
            fflush(NULL);
            return true;
        }
    }
    printf("[%s] offline (skipped)\n", label);
    fflush(NULL);
    return false;
}

static bool tof_bring_up_xshut(const char *label, uint8_t target_addr, vl53x *handle)
{
    const int p29 = tofPing(IIC0, TOF_ADDR_DEFAULT);
    printf("[%s] ping 0x29 = %d\n", label, p29);
    fflush(NULL);
    if (p29 == 0 && tof_try_at(label, TOF_ADDR_DEFAULT, target_addr, handle)) {
        printf("[%s] READY at 0x%02X\n", label, target_addr);
        fflush(NULL);
        return true;
    }
    const int pt = tofPing(IIC0, target_addr);
    printf("[%s] ping 0x%02X = %d\n", label, target_addr, pt);
    fflush(NULL);
    if (pt == 0 && tof_try_at(label, target_addr, target_addr, handle)) {
        printf("[%s] READY at 0x%02X (already)\n", label, target_addr);
        fflush(NULL);
        return true;
    }
    printf("[%s] offline (skipped)\n", label);
    fflush(NULL);
    return false;
}

/* =====================================================================
 * Time-of-Flight
 * ===================================================================== */

int tof_init(void)
{
    /* Switchbox + GPIO direction for the XSHUT lines on B and C. */
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    switchbox_set_pin(TOF_XSHUT_B, SWB_GPIO);   /* AR5 (middle) */
    switchbox_set_pin(TOF_XSHUT_C, SWB_GPIO);   /* AR4 (bottom) */

    gpio_set_direction(TOF_XSHUT_B, GPIO_DIR_OUTPUT);
    gpio_set_direction(TOF_XSHUT_C, GPIO_DIR_OUTPUT);

    /* Isolate A: hold B and C in shutdown (XSHUT low resets them to 0x29
     * when later re-enabled, so any stale address on B/C is cleared). */
    gpio_set_level(TOF_XSHUT_B, GPIO_LEVEL_LOW);
    gpio_set_level(TOF_XSHUT_C, GPIO_LEVEL_LOW);
    sleep_msec(30);

    iic_init(IIC0);

    /* A = top, always-on. B/C are held off, so A is the only device on the bus;
     * find it wherever it is and move it to TOF_ADDR_A. */
    s_tof_a_ok = tof_bring_up_always_on("ToF-A", TOF_ADDR_A, &s_tof_a);

    /* B = middle: wake via AR5 (resets it to 0x29), then assign TOF_ADDR_B. */
    gpio_set_level(TOF_XSHUT_B, GPIO_LEVEL_HIGH);
    sleep_msec(XSHUT_SETTLE_MS);
    s_tof_b_ok = tof_bring_up_xshut("ToF-B", TOF_ADDR_B, &s_tof_b);

    /* C = bottom: wake via AR4 (resets it to 0x29), then assign TOF_ADDR_C. */
    gpio_set_level(TOF_XSHUT_C, GPIO_LEVEL_HIGH);
    sleep_msec(XSHUT_SETTLE_MS);
    s_tof_c_ok = tof_bring_up_xshut("ToF-C", TOF_ADDR_C, &s_tof_c);

    s_tof_ready = s_tof_a_ok || s_tof_b_ok || s_tof_c_ok;
    printf("[ToF] online: A=%d B=%d C=%d\n", s_tof_a_ok, s_tof_b_ok, s_tof_c_ok);
    fflush(NULL);

    /* OK as long as at least one sensor came up. */
    return s_tof_ready ? 0 : 1;
}

/*
 * Physical-to-logical mapping for the vertical ToF stack (confirmed by test).
 * EDIT these three assignments if the layout changes.
 *   A -> top     (mountains)
 *   B -> middle  (large 6x6 rocks)
 *   C -> bottom  (small 3x3 rocks)
 */
void tof_read(uint16_t *bottom, uint16_t *middle, uint16_t *top)
{
    /* Read only the sensors that actually came up; others report 0 (invalid). */
    uint32_t dA = s_tof_a_ok ? tofReadDistance(&s_tof_a) : (uint32_t)-1;
    uint32_t dB = s_tof_b_ok ? tofReadDistance(&s_tof_b) : (uint32_t)-1;
    uint32_t dC = s_tof_c_ok ? tofReadDistance(&s_tof_c) : (uint32_t)-1;

    /* tofReadDistance returns (uint32_t)-1 on error. Map that to 0
     * so the main loop's "0 == invalid" convention works.            */
    if (dA == (uint32_t)-1) dA = 0;
    if (dB == (uint32_t)-1) dB = 0;
    if (dC == (uint32_t)-1) dC = 0;

    if (top)    *top    = (uint16_t)(dA > 65535 ? 65535 : dA);   /* A -> top    */
    if (middle) *middle = (uint16_t)(dB > 65535 ? 65535 : dB);   /* B -> middle */
    if (bottom) *bottom = (uint16_t)(dC > 65535 ? 65535 : dC);   /* C -> bottom */
}

/* =====================================================================
 * IR tape detectors (TCRT5000)
 * Most breakouts expose a digital "DO" pin that goes LOW when the IR
 * is reflected back (light surface) and HIGH on dark surface (= tape).
 * Verify polarity with the electronics team.
 * ===================================================================== */

int ir_init(void)
{
    switchbox_set_pin(IR_LEFT_PIN,  SWB_GPIO);   /* AR9  */
    switchbox_set_pin(IR_RIGHT_PIN, SWB_GPIO);   /* AR10 */
    gpio_set_direction(IR_LEFT_PIN,  GPIO_DIR_INPUT);
    gpio_set_direction(IR_RIGHT_PIN, GPIO_DIR_INPUT);

    s_ir_ready = true;
    return 0;
}

void ir_read(bool *left_tape, bool *right_tape)
{
    if (!s_ir_ready) {
        if (left_tape)  *left_tape  = false;
        if (right_tape) *right_tape = false;
        return;
    }

    /* TODO: confirm active-high vs active-low. Currently assumes HIGH
     * on DO = tape (dark surface = no reflection).                    */
    if (left_tape)
        *left_tape  = (gpio_get_level(IR_LEFT_PIN)  == GPIO_LEVEL_HIGH);
    if (right_tape)
        *right_tape = (gpio_get_level(IR_RIGHT_PIN) == GPIO_LEVEL_HIGH);
}

/* =====================================================================
 * Colour sensor (TCS3200)
 * Frequency-output sensor. S2/S3 select the filter:
 *      S2 S3   Filter
 *       0  0   Red
 *       0  1   Blue
 *       1  0   Clear (no filter)
 *       1  1   Green
 * OUT pin emits a square wave whose frequency is proportional to the
 * light intensity through the selected filter. Count edges over a
 * fixed window to get a "frequency" reading.
 * ===================================================================== */

static void color_select_filter(int s2, int s3)
{
    gpio_set_level(COLOR_S2_PIN, s2 ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
    gpio_set_level(COLOR_S3_PIN, s3 ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
}

/*
 * Measure one HIGH-pulse duration on OUT in loop counts. Longer pulse =
 * lower frequency = less light through the selected filter. Guarded against a
 * stuck pin so it can never hang the caller; returns 0 on timeout.
 */
static uint32_t color_measure_pulse(void)
{
    uint32_t guard = 0;
    while (gpio_get_level(COLOR_OUT_PIN) == GPIO_LEVEL_HIGH) {
        if (++guard > COL_PULSE_TIMEOUT) return 0;
    }
    guard = 0;
    while (gpio_get_level(COLOR_OUT_PIN) == GPIO_LEVEL_LOW) {
        if (++guard > COL_PULSE_TIMEOUT) return 0;
    }
    uint32_t count = 0;
    while (gpio_get_level(COLOR_OUT_PIN) == GPIO_LEVEL_HIGH) {
        if (++count > COL_PULSE_TIMEOUT) return 0;
    }
    return count;
}

/* Raw pulse count -> 0..255 using per-channel white/black calibration. */
static int color_map_to_rgb(uint32_t raw, uint32_t white, uint32_t black)
{
    if (raw <= white) return 255;
    if (raw >= black) return 0;
    const float intensity = (float)(black - raw) / (float)(black - white);
    return (int)(intensity * 255.0f);
}

int color_init(void)
{
    switchbox_set_pin(COLOR_S0_PIN,  SWB_GPIO);   /* AR11 */
    switchbox_set_pin(COLOR_S1_PIN,  SWB_GPIO);   /* AR12 */
    switchbox_set_pin(COLOR_S2_PIN,  SWB_GPIO);   /* AR7 */
    switchbox_set_pin(COLOR_S3_PIN,  SWB_GPIO);   /* AR8 */
    switchbox_set_pin(COLOR_OUT_PIN, SWB_GPIO);   /* AR6 */
    gpio_set_direction(COLOR_S0_PIN,  GPIO_DIR_OUTPUT);
    gpio_set_direction(COLOR_S1_PIN,  GPIO_DIR_OUTPUT);
    gpio_set_direction(COLOR_S2_PIN,  GPIO_DIR_OUTPUT);
    gpio_set_direction(COLOR_S3_PIN,  GPIO_DIR_OUTPUT);
    gpio_set_direction(COLOR_OUT_PIN, GPIO_DIR_INPUT);

    /* Output frequency scaling via S0/S1: 20% (S0=HIGH, S1=LOW). Slower pulses
     * = more loop counts in color_measure_pulse() = better resolution for the
     * busy-loop method. (S0=L,S1=L powers the sensor DOWN -- never do that.)
     * NOTE: this scaling sets the raw counts, so the white/black calibration
     * (COL_*_WHITE / COL_*_BLACK) MUST be redone for this robot. */
    gpio_set_level(COLOR_S0_PIN, GPIO_LEVEL_HIGH);
    gpio_set_level(COLOR_S1_PIN, GPIO_LEVEL_LOW);

    s_color_ready = true;
    return 0;
}

void color_read(uint32_t *r, uint32_t *g, uint32_t *b)
{
    if (!s_color_ready) {
        if (r) *r = 0;
        if (g) *g = 0;
        if (b) *b = 0;
        return;
    }

    color_select_filter(0, 0);                          /* Red    */
    sleep_msec(COLOR_SETTLE_MS);
    uint32_t fr = color_measure_pulse();

    color_select_filter(1, 1);                          /* Green  */
    sleep_msec(COLOR_SETTLE_MS);
    uint32_t fg = color_measure_pulse();

    color_select_filter(0, 1);                          /* Blue   */
    sleep_msec(COLOR_SETTLE_MS);
    uint32_t fb = color_measure_pulse();

    if (r) *r = fr;   /* raw HIGH-pulse counts (not Hz) */
    if (g) *g = fg;
    if (b) *b = fb;
}

/* ---------------------------------------------------------------------
 * Colour classification: raw pulse counts -> 0..255 RGB (via the white/black
 * calibration), then nearest palette colour by Euclidean distance. Includes
 * WHITE and BLACK so neutral surfaces classify instead of returning NONE.
 * ------------------------------------------------------------------- */
scan_color_t color_classify(uint32_t r_raw, uint32_t g_raw, uint32_t b_raw, float *confidence)
{
    /* All-zero raw = pulse-measure timeout / dead OUT pin. Report NONE rather
     * than letting it map to (255,255,255) and masquerade as a WHITE rock. */
    if (r_raw == 0 && g_raw == 0 && b_raw == 0) {
        if (confidence) *confidence = 0.0f;
        return SCAN_COLOR_NONE;
    }

    const int r = color_map_to_rgb(r_raw, COL_R_WHITE, COL_R_BLACK);
    const int g = color_map_to_rgb(g_raw, COL_G_WHITE, COL_G_BLACK);
    const int b = color_map_to_rgb(b_raw, COL_B_WHITE, COL_B_BLACK);

    /* Anchors are the MEASURED normalised RGB of each real cube (debug_color
     * 2026-06-18), NOT idealised corners: at ~2 cm the cubes read desaturated,
     * so pure-corner anchors collapse every cube onto WHITE. No yellow cube in
     * this mission, so yellow is intentionally absent from the palette. */
    static const struct { scan_color_t label; int r, g, b; } palette[] = {
        { SCAN_COLOR_RED,    255, 161, 170 },
        { SCAN_COLOR_GREEN,  182, 174, 127 },
        { SCAN_COLOR_BLUE,   255, 174, 212 },
        { SCAN_COLOR_WHITE,  255, 255, 255 },
        { SCAN_COLOR_BLACK,    0,   0,   0 },
    };
    const int n = (int)(sizeof(palette) / sizeof(palette[0]));

    int best = 0;
    long best_d = 0x7fffffffL;
    for (int i = 0; i < n; ++i) {
        const long dr = (long)r - palette[i].r;
        const long dg = (long)g - palette[i].g;
        const long db = (long)b - palette[i].b;
        const long d = dr * dr + dg * dg + db * db;
        if (d < best_d) { best_d = d; best = i; }
    }

    if (confidence) {
        const float far = 3.0f * 255.0f * 255.0f;   /* corner-to-corner */
        *confidence = 1.0f - (float)best_d / far;
    }
    return palette[best].label;
}

scan_color_t color_read_label(float *confidence)
{
    uint32_t r = 0, g = 0, b = 0;
    color_read(&r, &g, &b);
    return color_classify(r, g, b, confidence);
}

const char *color_label_name(scan_color_t label)
{
    switch (label) {
        case SCAN_COLOR_RED:    return "red";
        case SCAN_COLOR_GREEN:  return "green";
        case SCAN_COLOR_BLUE:   return "blue";
        case SCAN_COLOR_YELLOW: return "yellow";
        case SCAN_COLOR_WHITE:  return "white";
        case SCAN_COLOR_BLACK:  return "black";
        case SCAN_COLOR_NONE:
        default:                return "none";
    }
}

/* =====================================================================
 * NTC-10K temperature
 * Voltage divider: 3.3V -- 10k(fixed) -- Vout -- NTC -- GND
 * Vout sampled through unity-gain op-amp buffer into the ADC.
 * ===================================================================== */

int temp_init(void)
{
    /* NTC divider buffered into ADC5. Matches the working stand-alone test
     * (which also called buttons_init alongside adc_init). */
    adc_init();
    buttons_init();
    s_temp_ready = true;
    return 0;
}

/*
 * Read the NTC and return degrees Celsius.
 * adc_read_channel() returns the channel VOLTAGE directly. Divider is
 *   3.3V -- 10k(fixed) -- Vout -- NTC -- GND, so R_ntc = R2 * Vout/(Vref-Vout),
 * then the Beta equation gives temperature.
 */
float temp_read_celsius(void)
{
    if (!s_temp_ready) return -273.15f;

    const float v_out = (float)adc_read_channel(TEMP_ADC_CHANNEL);

    /* Guard against rail values that would blow up the maths. */
    if (v_out <= 0.001f || v_out >= (NTC_VREF - 0.001f)) {
        return -273.15f;
    }

    const float r_ntc = NTC_SERIES * (v_out / (NTC_VREF - v_out));
    const float inv_T = (1.0f / NTC_T0_K)
                      + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0);
    return (1.0f / inv_T) - 273.15f;
}

/* =====================================================================
 * Top-level wrappers used by the robot main loop
 * ===================================================================== */

int sensors_init_all(void)
{
    int err = 0;

    gpio_init();

    if (tof_init()   != 0) { printf("ToF init failed\n");    err |= 1; }
    if (ir_init()    != 0) { printf("IR init failed\n");     err |= 2; }
    if (color_init() != 0) { printf("Colour init failed\n"); err |= 4; }
    if (temp_init()  != 0) { printf("Temp init failed\n");   err |= 8; }

    return err;
}

void update_sensor_data(SensorData *out)
{
    if (out == NULL) return;

    tof_read(&out->tof_bottom_mm,
             &out->tof_middle_mm,
             &out->tof_top_mm);

    ir_read(&out->ir_left_detects_tape,
            &out->ir_right_detects_tape);

    color_read(&out->color_freq_r,
               &out->color_freq_g,
               &out->color_freq_b);

    out->temp_celsius = temp_read_celsius();
}
