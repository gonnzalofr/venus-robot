#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>

 /*
 *  TU/e 5EID0::LIBPYNQ Driver for VL53L0X TOF Sensor
 *  Example Code 
 * 
 *  Original: Larry Bank
 *  Adapted for PYNQ: Walthzer
 * 
 */

int vl53l0x_triple(void) {
    uint8_t addrA = 0x68;
    uint8_t addrB = 0x69;
    uint8_t addrC = 0x6A;

    gpio_set_level(AR0, GPIO_LEVEL_LOW); 
    gpio_set_level(AR1, GPIO_LEVEL_LOW); 
    sleep_msec(10);

    tofsetaddress(IIC0, 0x29, addrA);
    gpio_set_level(AR0, GPIO_LEVEL_HIGH);  
    sleep_msec(10);

    tofSetAddress(IIC0, 0x29, addrB);       
    gpio_set_level(AR1, GPIO_LEVEL_HIGH); 
    sleep_msec(10);

    tofSetAddress(IIC0, 0x29, addrC);      

	
	vl53x sensorA;
    vl53x sensorB;
    vl53x sensorC;

	tofInit(&sensorA, IIC0, addrA, 0);
    tofInit(&sensorB, IIC0, addrB, 0);
    tofInit(&sensorC, IIC0, addrC, 0);
	
	uint32_t iDistance1;
    uint32_t iDistance2;
    uint32_t iDistance3;
    
	for (i=0; i<1200; i++)
	{
		iDistance1 = tofReadDistance(&sensorA);
		printf("A => %dmm -- ", iDistance1);
		iDistance2 = tofReadDistance(&sensorB);
		printf("B => %dmm\n", iDistance2);
        iDistance3 = tofReadDistance(&sensorC);
		printf("C => %dmm\n", iDistance3);
		sleep_msec(100);
	}

	iic_destroy(IIC0);
	pynq_destroy();
	return EXIT_SUCCESS;
}
