#include <libpynq.h>
#include <stdio.h>

#define S0 IO_AR0
#define S1 IO_AR1
#define S2 IO_AR2
#define S3 IO_AR3
#define OUT_PIN IO_AR4

#define R_WHITE 225
#define R_BLACK 600
#define G_WHITE 220
#define G_BLACK 800
#define B_WHITE 240
#define B_BLACK 750

struct Colors {
    const char *name;
    int r,g,b;
};

struct Colors colorValues[] ={
    {"RED", 255, 0, 0},
    {"GREEN", 0, 255, 0},
    {"BLUE", 0, 0, 255},
    {"WHITE", 255, 255, 255,},
    {"BLACK", 0, 0, 0}
};

int colorSize = 5;

const char* identifyColor(int r, int g, int b) {
    int bestIndex = 0;
    long minDist = 100000;

    for(int i = 0; i < colorSize; i++) {
        long dr = r - colorValues[i].r;
        long dg = g - colorValues[i].g;
        long db = b - colorValues[i].b;
        long dist = dr*dr + dg*dg + db*db;

        if(dist < minDist) {
            bestIndex = i;
            minDist = dist;
        }
    }
    return colorValues[bestIndex].name;
}

int map_to_rgb(uint32_t raw, uint32_t white, uint32_t black) {
    if (raw <= white) return 255;
    if (raw >= black) return 0;

    float intensity = (float)(black - raw) / (float)(black - white);
    return (int)(intensity * 255.0);
}

uint32_t measure_pulse() {
    uint32_t count = 0;

    while (gpio_get_level(OUT_PIN) == GPIO_LEVEL_HIGH);
    while (gpio_get_level(OUT_PIN) == GPIO_LEVEL_LOW);
    while (gpio_get_level(OUT_PIN) == GPIO_LEVEL_HIGH) {
        count++;
        if(count > 10000000) return 0;
    }
    return count;
}

void setup_sensor() {
    gpio_set_direction(S0, GPIO_DIR_OUTPUT);
    gpio_set_direction(S1, GPIO_DIR_OUTPUT);
    gpio_set_direction(S2, GPIO_DIR_OUTPUT);
    gpio_set_direction(S3, GPIO_DIR_OUTPUT);
    gpio_set_direction(OUT_PIN, GPIO_DIR_INPUT);

    gpio_set_level(S0, GPIO_LEVEL_HIGH);
    gpio_set_level(S1, GPIO_LEVEL_LOW);
}

int main() {
    pynq_init();
    setup_sensor();
    printf("%d", gpio_get_level(OUT_PIN));
    fflush(stdout);
    while (1) {
        // RED
        gpio_set_level(S2, GPIO_LEVEL_LOW);
        gpio_set_level(S3, GPIO_LEVEL_LOW);
        sleep_msec(10);
        uint32_t r_raw = measure_pulse();
        int r = map_to_rgb(r_raw, R_WHITE, R_BLACK);

        // GREEN
        gpio_set_level(S2, GPIO_LEVEL_HIGH);
        gpio_set_level(S3, GPIO_LEVEL_HIGH);
        sleep_msec(10);
        uint32_t g_raw = measure_pulse();
        int g = map_to_rgb(g_raw, G_WHITE, G_BLACK);

        //BLUE
        gpio_set_level(S2, GPIO_LEVEL_LOW);
        gpio_set_level(S3, GPIO_LEVEL_HIGH);
        sleep_msec(10);
        uint32_t b_raw = measure_pulse();
        int b = map_to_rgb(b_raw, B_WHITE, B_BLACK);

        const char* color_name;
        color_name = identifyColor(r, g, b);
        
        printf("\rRGB: [%3d, %3d, %3d] | Raw: R:%u G:%u B:%u %-10s   ", r, g, b, r_raw, g_raw, b_raw, color_name);
       
        fflush(stdout);
        
        sleep_msec(500);
    }

    pynq_destroy();
    return 0;
}