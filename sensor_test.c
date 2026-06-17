#include <libpynq.h>
#include <iic.h>
#include "sensors.h"
#include "vl53l0x.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Scan the I2C bus for VL53L0X sensors at every address, so we can tell
 * "nothing on the bus" (wiring/power) from "stuck at a reassigned address"
 * (needs a power-cycle). Run before sensors_init_all(). */
static void i2c_scan(void) {
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    iic_init(IIC0);

    printf("--- I2C scan for VL53L0X (0x08..0x77) ---\n");
    int found = 0;
    for (int a = 0x08; a <= 0x77; ++a) {
        if (tofPing(IIC0, (uint8_t)a) == 0) {
            printf("  VL53L0X responds at 0x%02X\n", a);
            ++found;
        }
    }
    if (found == 0) {
        printf("  none found -> check 5V/GND power and SDA/SCL wiring.\n");
    } else {
        printf("  found %d. If they are at 0x30/0x31/0x32 (not 0x29),\n"
               "  power-cycle the sensors to reset them to 0x29.\n", found);
    }
    printf("-----------------------------------------\n");
    fflush(stdout);
}

/*
 *  Sensor bring-up test -- TU/e Venus.
 *  Initializes the sensor module and prints, ~5x/second:
 *     - 3x VL53L0X ToF distances (left / centre / right) in mm  (0 = invalid)
 *     - TCS3200 colour raw frequencies (R/G/B in Hz) + classified colour
 *     - 2x TCRT5000 IR tape booleans
 *
 *  Build: drop in as applications/sensortest/main.c alongside
 *         sensors.c sensors.h vl53l0x.c vl53l0x.h, then `make` and `sudo ./main`.
 *
 *  The ToF bring-up prints its own per-sensor ping/init status (from
 *  sensors.c) at startup, so a failure shows exactly which sensor/step failed.
 */

int main(void) {
    pynq_init();

    i2c_scan();

    printf("=== sensor bring-up ===\n");
    fflush(stdout);

    const int err = sensors_init_all();
    printf("sensors_init_all -> 0x%X  (bit0=ToF bit1=IR bit2=Colour bit3=Temp; 0 = all OK)\n", err);
    printf("=======================\n\n");
    fflush(stdout);

    while (1) {
        /* --- distance (vertical stack: bottom=3x3 rocks, middle=6x6, top=mtn) --- */
        uint16_t d_bot = 0, d_mid = 0, d_top = 0;
        tof_read(&d_bot, &d_mid, &d_top);

        /* --- colour --- */
        uint32_t cr = 0, cg = 0, cb = 0;
        color_read(&cr, &cg, &cb);
        float conf = 0.0f;
        const scan_color_t label = color_classify(cr, cg, cb, &conf);

        /* --- IR tape --- */
        bool tape_l = false, tape_r = false;
        ir_read(&tape_l, &tape_r);

        /* --- temperature --- */
        const float temp_c = temp_read_celsius();

        printf("ToF[Bot %4u  Mid %4u  Top %4u] mm | "
               "Colour raw[R %5u  G %5u  B %5u] -> %-6s %.2f | "
               "IR[L %d  R %d] | Temp %6.2f C\n",
               d_bot, d_mid, d_top,
               cr, cg, cb, color_label_name(label), conf,
               tape_l, tape_r, temp_c);
        fflush(stdout);

        sleep_msec(200);
    }

    iic_destroy(IIC0);
    pynq_destroy();
    return 0;
}
