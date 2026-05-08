#include <libpynq.h>
#include <math.h>

#define B       4050.0
#define T0      298.15      // 25°C in Kelvin
#define R0      10000.0     // 10kOhm at 25°C
#define V_REF   3.3         // V
#define R2      10000     // 10kOhm fixed resistor

float resistance_to_celsius(float R);

int main(void) {

    pynq_init();
    adc_init();
    buttons_init();

    double v_out;
    double r_t;
    double t_c;
    

    while(1) {
        v_out = adc_read_channel(ADC5); 
        r_t = R2 * (v_out / (V_REF - v_out));
            
        t_c = resistance_to_celsius((float)r_t);

        printf("V_out: %f, V_reef: %f r_t: %f, t_c: %f\n", 
                v_out, V_REF, r_t, t_c);
        sleep_msec(1000);
    }

    adc_destroy();
    buttons_destroy();
    pynq_destroy();
    return(0);

}

float resistance_to_celsius(float R) {
    // Steinhart-Hart simplified (Beta) equation
    float T = 1.0 / (1.0/T0 + (1.0/B) * log(R / R0));
    return T - 273.15;
}