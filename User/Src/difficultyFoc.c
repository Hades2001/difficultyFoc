#include "difficultyFoc.h"

motor_t motor;
extern serial_t serial;

double _normalizeAngle(double angle)
{
	double a = fmod(angle, _PI * 2);
	return a >= 0 ? a : (a + _PI * 2);
}

float getElectricaAngle(motor_t *dev,magnetic_sensor_t *sensor)
{
	return _normalizeAngle((float)(dev->sensor_dir * dev->pole_pairs) * getMechanicalAngle(sensor) - dev->zero_elec_angle);
}

int DifficultyFOCInit(motor_t *device)
{
	device->pole_pairs = 7;
	device->zero_elec_angle = 0.0;

	return kFOCSuccessful;
}

int DifficultySeting(float Uq, float Ud, float angle_el, motor_t *device)
{
	if (device == nullptr)
		return -1;
	double Uout = 0;
	if (fabs(Ud) > 1e-6)
	{
		Uout = sqrtf(Uq * Uq + Ud * Ud) / device->voltage_limit;
		angle_el = _normalizeAngle(angle_el + atan2f(Uq, Ud));
	}
	else
	{
		Uout = Uq / device->voltage_limit;
		angle_el = _normalizeAngle(angle_el + (_PI));
	}

	int sector = (int)(floor(angle_el / (_PI / 3)) + 1);

	float T1 = _SQRT3 * sinf(sector * _PI_3 - angle_el) * Uout;
	float T2 = _SQRT3 * sinf(angle_el - (sector - 1.0) * _PI_3) * Uout;
	float T0 = 1 - T1 - T2;
	float Ta, Tb, Tc;
	switch (sector)
	{
	case 1:
		Ta = T1 + T2 + T0 / 2;
		Tb = T2 + T0 / 2;
		Tc = T0 / 2;
		break;
	case 2:
		Ta = T1 + T0 / 2;
		Tb = T1 + T2 + T0 / 2;
		Tc = T0 / 2;
		break;
	case 3:
		Ta = T0 / 2;
		Tb = T1 + T2 + T0 / 2;
		Tc = T2 + T0 / 2;
		break;
	case 4:
		Ta = T0 / 2;
		Tb = T1 + T0 / 2;
		Tc = T1 + T2 + T0 / 2;
		break;
	case 5:
		Ta = T2 + T0 / 2;
		Tb = T0 / 2;
		Tc = T1 + T2 + T0 / 2;
		break;
	case 6:
		Ta = T1 + T2 + T0 / 2;
		Tb = T0 / 2;
		Tc = T1 + T0 / 2;
		break;
	default:
		Ta = 0;
		Tb = 0;
		Tc = 0;
	}
	device->Ta = Ta;
	device->Tb = Tb;
	device->Tc = Tc;
	return 0;
}

int setTimerPWMVal(motor_t *device)
{
	device->Ua = device->Ta * device->voltage_limit;
	device->Ub = device->Tb * device->voltage_limit;
	device->Uc = device->Tc * device->voltage_limit;

	device->Ua = _CONSTRAIN(device->Ua, 0.0f, device->voltage_limit);
	device->Ub = _CONSTRAIN(device->Ub, 0.0f, device->voltage_limit);
	device->Uc = _CONSTRAIN(device->Uc, 0.0f, device->voltage_limit);

	uint8_t CCRa = _CONSTRAIN(device->Ua * 255 / device->voltage_dc, 0, 255);
	uint8_t CCRb = _CONSTRAIN(device->Ub * 255 / device->voltage_dc, 0, 255);
	uint8_t CCRc = _CONSTRAIN(device->Uc * 255 / device->voltage_dc, 0, 255);

	TIM1->CCR1 = CCRa;
	TIM1->CCR2 = CCRb;
	TIM1->CCR3 = CCRc;

	return 0;
}


int DiffcultyFOCAlignSendor(motor_t *device,magnetic_sensor_t *sensor)
{
	int cnt = 0;
	float angle = 0.0f;
	float sensor_val[2] = {0.0f, 0.0f};
	for (cnt = 0; cnt < 500; cnt++)
	{
		angle = _3PI_2 +(_2PI * ((float)cnt)) / 500;
		DifficultySeting(2.0, 0.0, angle, device);
		setTimerPWMVal(device);
		LL_mDelay(1);
	}
	LL_mDelay(100);
	updataSensor(sensor);
	sensor_val[0] = getAngle(sensor);

	for (cnt = 500; cnt >= 0; cnt--)
	{
		angle = _3PI_2 +(_2PI * ((float)cnt)) / 500;
		DifficultySeting(2.0, 0.0, angle, device);
		setTimerPWMVal(device);
		LL_mDelay(1);
	}
	LL_mDelay(100);
	updataSensor(sensor);
	sensor_val[1] = getAngle(sensor);

	DifficultySeting(0.0f, 0.0f, 0.0f, device);
	setTimerPWMVal(device);
	LL_mDelay(100);

	if (sensor_val[0] == sensor_val[1])
	{
		LOG_PRINTF(LOG_ERROR,"[FOC] Aligning sendor but motor isn't movement!!\r\n");
		return kFOCFault;
	}
	else if (sensor_val[0] < sensor_val[1])
	{
		LOG_PRINTF(LOG_INFO,"[FOC] sensor direction is CCW\r\n");
		device->sensor_dir = kSensorCCW;
	}
	else if (sensor_val[0] > sensor_val[1])
	{
		LOG_PRINTF(LOG_INFO,"[FOC] sensor direction is CW\r\n");
		device->sensor_dir = kSensorCW;
	}

	float moved = fabs(sensor_val[0] - sensor_val[1]);
	if (fabs(moved * device->pole_pairs - _2PI) > 0.5f)
	{
		LOG_PRINTF(LOG_ERROR,"[FOC] Aligning sendor failed (v:%.4f)!!\r\n", moved);
	}
	else
	{
		LOG_PRINTF(LOG_INFO,"[FOC] Aligning sendor successful\r\n");
	}

	DifficultySeting(1.0, 0.0, _3PI_2, device);
	setTimerPWMVal(device);
	LL_mDelay(200);

	updataSensor(sensor);
	device->zero_elec_angle = 0;
	device->zero_elec_angle = getElectricaAngle(device,sensor);
	LL_mDelay(20);
	LOG_PRINTF(LOG_INFO,"[FOC] Zero electrica angle : %.4f\r\n", _R2D(device->zero_elec_angle));

	DifficultySeting(0.0f, 0.0f, 0.0f, device);
	setTimerPWMVal(device);
	LL_mDelay(10);

	return kFOCSuccessful;
}
