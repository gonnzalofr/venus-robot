#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>

int fetch_distances(int sensor_count) {
    case(sensor_count) {
        1: {
            uint8_t addr1 = 0x68;
            uint32_t iDistance1 = tofReadDistance(&sensor1);
		    return iDistance;
        }
        2: {
            uint8_t addr2 = 0x69;
            uint32_t iDistance2 = tofReadDistance(&sensor2);
		    return iDistance2;
        }
        3: {
            uint8_t addr3 = 0x6A;
            uint32_t iDistance3 = tofReadDistance(&sensor3);
		    return iDistance3;
}
