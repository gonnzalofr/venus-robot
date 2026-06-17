#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>
#include <stdlib.h>

 /*
 * TU/e 5EID0::LIBPYNQ Driver for VL53L0X TOF Sensor
 * Example Code 
 * * Original: Larry Bank
 * Adapted for PYNQ: Walthzer
 * */

int vl53l0x_triple(void) {
    uint8_t addrA = 0x68;
    uint8_t addrB = 0x69;
    uint8_t addrC = 0x6A;

    // 0. Set pins as OUTPUTS (Crucial for driving XSHUT)
    gpio_set_direction(IO_AR0, GPIO_DIR_OUTPUT);
    gpio_set_direction(IO_AR1, GPIO_DIR_OUTPUT);

    // 1. Turn OFF Sensor B (AR0) and Sensor C (AR1)
    // Sensor A remains ON (assuming its XSHUT is tied high or uncontrolled)
    gpio_set_level(IO_AR0, GPIO_LEVEL_LOW); 
    gpio_set_level(IO_AR1, GPIO_LEVEL_LOW); 
    sleep_msec(20); // Give them time to power down

    vl53x sensorA;
    vl53x sensorB;
    vl53x sensorC;

    // ==========================================
    // SENSOR A: Ping, Address Change, Initialize
    // ==========================================
    printf("Pinging Sensor A (default address 0x29)...\n");
    if (tofPing(IIC0, 0x29) == 0) {
        printf(" -> Sensor A Ping Successful!\n");
    } else {
        printf(" -> Sensor A Ping Failed!\n");
    }

    printf("Changing Sensor A address to 0x%02X...\n", addrA);
    tofSetAddress(IIC0, 0x29, addrA);
    
    printf("Initializing Sensor A...\n");
    tofInit(&sensorA, IIC0, addrA, 0);
    printf(" -> Sensor A Initialized!\n\n");

    // ==========================================
    // SENSOR B: Wake up, Ping, Address Change, Initialize
    // ==========================================
    printf("Turning ON Sensor B (AR0 HIGH)...\n");
    gpio_set_level(IO_AR0, GPIO_LEVEL_HIGH);  
    sleep_msec(20); // Give the sensor a moment to boot

    printf("Pinging Sensor B (default address 0x29)...\n");
    if (tofPing(IIC0, 0x29) == 0) {
        printf(" -> Sensor B Ping Successful!\n");
    } else {
        printf(" -> Sensor B Ping Failed!\n");
    }

    printf("Changing Sensor B address to 0x%02X...\n", addrB);
    tofSetAddress(IIC0, 0x29, addrB);       

    printf("Initializing Sensor B...\n");
    tofInit(&sensorB, IIC0, addrB, 0);
    printf(" -> Sensor B Initialized!\n\n");

    // ==========================================
    // SENSOR C: Wake up, Ping, Address Change, Initialize
    // ==========================================
    printf("Turning ON Sensor C (AR1 HIGH)...\n");
    gpio_set_level(IO_AR1, GPIO_LEVEL_HIGH); 
    sleep_msec(20); // Give the sensor a moment to boot

    printf("Pinging Sensor C (default address 0x29)...\n");
    if (tofPing(IIC0, 0x29) == 0) {
        printf(" -> Sensor C Ping Successful!\n");
    } else {
        printf(" -> Sensor C Ping Failed!\n");
    }

    printf("Changing Sensor C address to 0x%02X...\n", addrC);
    tofSetAddress(IIC0, 0x29, addrC);       

    printf("Initializing Sensor C...\n");
    tofInit(&sensorC, IIC0, addrC, 0);
    printf(" -> Sensor C Initialized!\n\n");
    
    // ==========================================
    // DATA COLLECTION LOOP
    // ==========================================
    uint32_t iDistance1;
    uint32_t iDistance2;
    uint32_t iDistance3;
    
    printf("Starting continuous distance measurements...\n");
    for (int i = 0; i < 1200; i++)
    {
        iDistance1 = tofReadDistance(&sensorA);
        iDistance2 = tofReadDistance(&sensorB);
        iDistance3 = tofReadDistance(&sensorC);
        
        printf("A => %4dmm -- B => %4dmm -- C => %4dmm\n", iDistance1, iDistance2, iDistance3);
        sleep_msec(100);
    }

    // Note: main.c also calls these destroys. Be careful of double-free errors 
    // depending on how your main.c is ultimately structured.
    iic_destroy(IIC0);
    pynq_destroy();
    return EXIT_SUCCESS;
}