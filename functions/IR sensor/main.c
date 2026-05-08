#include <libpynq.h>
#include <stdio.h>


int main(void) {
    // Initialize the library
    pynq_init();

    // Set Arduino Pin 0 (AR0) to Input mode
    // Using the Switchbox to route the GPIO
    gpio_set_direction(IO_AR0, GPIO_DIR_INPUT);

    printf("Starting Digital Detection (D0)...\n");
    printf("Press CTRL+C to exit.\n\n");

    while (1) {
        // Read the digital level (0 or 1)
        int level;
        while(1){
        sleep_msec(50);
        level = gpio_get_level(IO_AR0);
        if (level == GPIO_LEVEL_LOW) {
            printf("Surface: WHITE  \n");
        } else {
            printf("Surface: BLACK  \n");
        }
        }   
        
        fflush(stdout); // Ensure the line updates in the terminal
        sleep_msec(100);
    }

    pynq_destroy();
    return 0;
}