#ifndef _SETUPINF_H_
#define _SETUPINF_H_

#include "stm32g0xx_hal.h"
#include "string.h"

typedef struct setupdata{
	uint32_t 	flag;				//'HaHa' 0x48618461
	float 		zero_elec_angle;
	float		zero_offset;
}setupdata_t;

typedef struct setupinf{
    setupdata_t data;

    int8_t (*readFromE2prom)();
    int8_t (*writeToE2prom)();
}setupinf_t;



uint8_t initSetupInfo(void);

int8_t _readFromE2prom(void);
int8_t _writeToE2prom(void);

extern I2C_HandleTypeDef hi2c2;
extern setupinf_t setup;

#endif


