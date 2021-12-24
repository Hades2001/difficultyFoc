#ifndef _MAGNETICSENSOR_H_
#define _MAGNETICSENSOR_H_

#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_system.h"
//#include "stm32g0xx_hal.h"
#include "user_define.h"
#include "userTimer.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <limits.h>
#include <float.h>

#define SPI_TX_MODE 0
#define SPI_RX_MODE 1

#define SYSTEM_ERROR_MASK           0x4000
#define INTERFACE_ERROR_MASK        0x2000
#define INV_ANGLE_ERROR_MASK        0x1000

typedef struct
{
	float angle_last;
	float angle_new;
	float angle_offset;

	float velocity;

	uint64_t vel_time;
	uint64_t vel_time_last;
	float vel_sendor;
	float vel_angle_last;
	int vel_circle_last;
	int circle;
	int8_t dir;

}magnetic_sensor_t;

typedef enum
{
	kSensorCW = 1,
	kSensorCCW = -1,
	kSensorUnknown = 0
}sensor_direction_t;

//extern SPI_HandleTypeDef hspi1;

extern int initSensor(magnetic_sensor_t *sensor);
extern int updataSensor(magnetic_sensor_t *sensor);
extern float getVelocity(magnetic_sensor_t *sensor);
extern float getAngle(magnetic_sensor_t *sensor);
extern float getMechanicalAngle(magnetic_sensor_t *sensor);

#endif


