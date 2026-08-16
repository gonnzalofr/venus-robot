#ifndef L
#define L
#include <stdint.h>
#include <stdbool.h>
typedef int iic_index_t; typedef int pulsecounter_index_t;
#define IO_AR0 0
#define IO_AR1 1
#define IO_AR2 2
#define IO_AR4 4
#define IO_AR5 5
#define IO_AR6 6
#define IO_AR7 7
#define IO_AR8 8
#define IO_AR9 9
#define IO_AR10 10
#define IO_AR11 11
#define IO_AR12 12
#define IO_AR13 13
#define IO_AR_SCL 100
#define IO_AR_SDA 101
#define SWB_GPIO 0
#define SWB_UART0_TX 1
#define SWB_UART0_RX 2
#define SWB_IIC0_SCL 3
#define SWB_IIC0_SDA 4
#define SWB_TIMER_IC0 5
#define SWB_TIMER_IC1 6
#define GPIO_DIR_OUTPUT 1
#define GPIO_DIR_INPUT 0
#define GPIO_LEVEL_HIGH 1
#define GPIO_LEVEL_LOW 0
#define IIC0 0
#define UART0 0
#define ADC5 5
#define PULSECOUNTER0 0
#define PULSECOUNTER1 1
int pynq_init();int pynq_destroy();int gpio_init();int gpio_set_direction();int gpio_set_level();int gpio_get_level();
int switchbox_init();int switchbox_set_pin();int iic_init();int iic_destroy();int iic_read_register();int iic_write_register();
int uart_init();int uart_send();int uart_recv();int uart_has_data();int uart_reset_fifos();int uart_destroy();
int adc_init();int adc_read_channel();int buttons_init();int sleep_msec();
int pulsecounter_init();int pulsecounter_set_edge();int pulsecounter_reset_count();int pulsecounter_get_count();int pulsecounter_destroy();
#endif
