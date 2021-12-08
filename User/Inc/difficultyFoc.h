#ifndef _DIFFICULTYFOC_H_
#define _DIFFICULTYFOC_H_

#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_system.h"

#include "magneticSensor.h"
#include "user_serial.h"
#include "user_define.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <limits.h>
#include <float.h>

typedef enum
{
	kFOCSuccessful = 0,
	kFOCFault = -1,
}foc_state_t;

typedef struct motor
{
	float Ta;
	float Tb;
	float Tc;
	float voltage_limit;
	float voltage_dc;
	float Ua;
	float Ub;
	float Uc;

	int pole_pairs;
	int sensor_dir;
	float zero_elec_angle;
}motor_t;

extern float getElectricaAngle(motor_t *dev,magnetic_sensor_t *sensor);
extern int DiffcultyFOCAlignSendor(motor_t *device,magnetic_sensor_t *sensor);
extern int setTimerPWMVal(motor_t *device);
extern int DifficultySeting(float Uq, float Ud, float angle_el, motor_t *device);
extern int DifficultyFOCInit(motor_t *device);

#endif
