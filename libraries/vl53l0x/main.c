#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>
#define AR0 54 
#define AR1 55
 /*
 *  TU/e 5EID0::LIBPYNQ Driver for VL53L0X TOF Sensor
 *  Example Code 
 * 
 *  Original: Larry Bank
 *  Adapted for PYNQ: Walthzer
 * 
 */

int vl53l0x_triple(void);
extern int vl53l0x_example_single();
/** This Example program REQUIRES ALL OF:
 * - single.c
 * - dual.c
 * To be present.
**/

int main(void) {
  	pynq_init();
	printf("Test1\n");
	//Setting up the buttons & LEDs
	//Init the IIC pins
	switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
	switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

	iic_init(IIC0);

	/**Test Scripts: Select ONE!
	 * - Single Sensor -> vl53l0x_example_single();
	 * - Dual Sensor -> vl53l0x_example_dual();
	**/	

	/** Connect one sensor to the IIC bus and enjoy! **/
	//vl53l0x_triple();
	vl53l0x_example_single();
	/** Connect two sensors to the IIC bus
	 * ONLY CONNECT ONE TO 5V and GND
	 * --If the second sensor is connected to either 5v or GND
	 * --Before the first is initialized, they conflict.
	 * Run the program and follow instructions! **/
	//vl53l0x_example_dual();

	iic_destroy(IIC0);
	pynq_destroy();
	return EXIT_SUCCESS;
}

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

	gpio_set_level(AR0, GPIO_LEVEL_HIGH);
sleep_msec(100);

	int p = tofPing(IIC0, 0x29);
	printf("Post-AR0-HIGH ping 0x29: %s\n", p == 0 ? "OK" : "FAIL");

	i = tofSetAddress(IIC0, 0x29, addrB);

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
