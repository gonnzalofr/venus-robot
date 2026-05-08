#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>

int initialize_sensors(void) {
    uint8_t addr1 = 0x68;
    uint8_t addr2 = 0x69;
    uint8_t addr3 = 0x6A;

    gpio_set_level(IO0, GPIO_LEVEL_LOW); 
    gpio_set_level(IO1, GPIO_LEVEL_LOW); 
    sleep_msec(10);

    tofsetaddress(IIC0, 0x29, addrA);
    gpio_set_level(IO0, GPIO_LEVEL_HIGH);  
    sleep_msec(10);

    tofSetAddress(IIC0, 0x29, addrB);       
    gpio_set_level(IO1, GPIO_LEVEL_HIGH); 
    sleep_msec(10);

    tofSetAddress(IIC0, 0x29, addrC);      
    printf("Sensor B ready at 0x69\n");
	
	vl53x sensor1;
    vl53x sensor2;
    vl53x sensor3;

	tofInit(&sensor1, IIC0, addr1, 0);
    tofInit(&sensor2, IIC0, addr2, 0);
    tofInit(&sensor3, IIC0, addr3, 0);
}
