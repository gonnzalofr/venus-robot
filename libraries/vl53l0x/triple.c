#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>

#define AR0 54  // Replace 54 with the actual GPIO number for your overlay
#define AR1 55
 /*
 *  TU/e 5EID0::LIBPYNQ Driver for VL53L0X TOF Sensor
 *  Example Code 
 * 
 *  Original: Larry Bank
 *  Adapted for PYNQ: Walthzer
 * 
 */

int vl53l0x_triple(void) {
    int i;
    uint8_t addrA = 0x68;
    uint8_t addrB = 0x69;
    uint8_t addrC = 0x6A;

    gpio_set_level(AR0, GPIO_LEVEL_LOW); 
    gpio_set_level(AR1, GPIO_LEVEL_LOW); 
    sleep_msec(10);
 	i = tofPing(IIC0, 0x29);
	printf("Sensor Ping: ");
	if(i != 0)
	{
		printf("Fail\n");
		return 1;
	}
	printf("Succes\n");
    i = tofSetAddress(IIC0, 0x29, addrA);
    printf("---Address Change: ");
	if(i != 0)
	{
		printf("Fail A\n");
		return 1;
	}
	printf("Succes\n");
    gpio_set_level(AR0, GPIO_LEVEL_HIGH);  
    sleep_msec(100);

    i = tofSetAddress(IIC0, 0x29, addrB);  
    printf("---Address Change: ");
	if(i != 0)
	{
		printf("Fail B\n");
		return 1;
	}
	printf("Succes\n");    
    gpio_set_level(AR1, GPIO_LEVEL_HIGH); 
    sleep_msec(100);

    i = tofSetAddress(IIC0, 0x29, addrC);      
    printf("---Address Change: ");
	if(i != 0)
	{
		printf("Fail C\n");
		return 1;
	}
	printf("Succes\n");

	
	vl53x sensorA;
    vl53x sensorB;
    vl53x sensorC;

	i = tofPing(IIC0, addrA);
	printf("---Sensor Ping - A: ");
	if(i != 0)
	{
		printf("Fail - A\n");
		return 1;
	}
	printf("Succes\n");

	i = tofPing(IIC0, addrB);
	printf("---Sensor Ping - B: ");
	if(i != 0)
	{
		printf("Fail - B\n");
		return 1;
	}
	printf("Succes\n");

	i = tofPing(IIC0, addrB);
	printf("---Sensor Ping - C: ");
	if(i != 0)
	{
		printf("Fail - C\n");
		return 1;
	}
	printf("Succes\n");


	i = tofInit(&sensorA, IIC0, addrA, 0);

		if (i != 0)
	{
		printf("---Init: Fail\n");
		return 1;
	}
    sleep_msec(10);
    i = tofInit(&sensorB, IIC0, addrB, 0);

		if (i != 0)
	{
		printf("---Init: Fail\n");
		return 1;
	}
    sleep_msec(10);
    i = tofInit(&sensorC, IIC0, addrC, 0);

		if (i != 0)
	{
		printf("---Init: Fail\n");
		return 1;
	}

    uint8_t model1, revision1;
    uint8_t model2, revision2;
    uint8_t model3, revision3;
    printf("VL53L0X-A device successfully opened.\n");
	tofGetModel(&sensorA, &model1, &revision1);
	printf("Model1 ID - %d\n", model1);
	printf("Revision1 ID - %d\n", revision1);
	fflush(NULL);

    printf("VL53L0X-B device successfully opened.\n");
	tofGetModel(&sensorB, &model2, &revision2);
	printf("Model2 ID - %d\n", model2);
	printf("Revision2 ID - %d\n", revision2);
	fflush(NULL);

    printf("VL53L0X-C device successfully opened.\n");
	tofGetModel(&sensorC, &model3, &revision3);
	printf("Model3 ID - %d\n", model3);
	printf("Revision3 ID - %d\n", revision3);
	fflush(NULL);

	
	uint32_t iDistance1;
    uint32_t iDistance2;
    uint32_t iDistance3;
    
	for (i=0; i<1200; i++)
	{
		iDistance1 = tofReadDistance(&sensorA);
		printf("A => %dmm\n -- ", iDistance1);
        sleep_msec(10);
		iDistance2 = tofReadDistance(&sensorB);
		printf("B => %dmm\n", iDistance2);
        sleep_msec(10);
        iDistance3 = tofReadDistance(&sensorC);
		printf("C => %dmm\n", iDistance3);
		sleep_msec(100);
	}

	return EXIT_SUCCESS;
}
