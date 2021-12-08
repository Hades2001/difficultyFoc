#include "magneticSensor.h"

#define SET_SPI_MOSI_MODE(SPIx,MODE) LL_SPI_SetTransferDirection(SPIx,(MODE == SPI_TX_MODE ) ? LL_SPI_HALF_DUPLEX_TX : LL_SPI_HALF_DUPLEX_RX)

magnetic_sensor_t tle5012;

uint16_t writeAndRead16Bit(SPI_TypeDef *SPIx,uint16_t data)
{
	uint8_t retry = 0;				 
	do{
		retry++;
	} while ((retry < 200)&&(!LL_SPI_IsActiveFlag_TXE(SPIx)));
			  
	LL_SPI_TransmitData16(SPIx, data);
	retry = 0;
	do{
		retry++;
	} while ((retry < 200)&&(!LL_SPI_IsActiveFlag_RXNE(SPIx)));  						    
	return LL_SPI_ReceiveData16(SPIx);
}

uint16_t readTLE5012e(uint16_t cmd)
{
	uint16_t received[3];

	LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_0);
	__nop();__nop();__nop();
	writeAndRead16Bit(SPI1,cmd);
	SET_SPI_MOSI_MODE(SPI1,SPI_RX_MODE);
	__nop();__nop();__nop();
	received[0] = writeAndRead16Bit(SPI1,0xffff);
	received[1] = writeAndRead16Bit(SPI1,0xffff);
	LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_0);
	SET_SPI_MOSI_MODE(SPI1,SPI_TX_MODE);
	return ((received[0] & 0x7fff) << 1) >> 3;
}

int initSensor(magnetic_sensor_t *sensor)
{
	SET_SPI_MOSI_MODE(SPI1,SPI_TX_MODE);
	uint16_t val = 8192 - readTLE5012e(0x8021);
	sensor->angle_last = (val * _2PI / 8192.0);
	sensor->vel_angle_last = (val * _2PI / 8192.0);
	sensor->vel_time_last = sys_ticks.micros();
	sensor->vel_circle_last = 0;

	val = 8192 - readTLE5012e(0x8021);
	sensor->angle_new = (val * _2PI / 8192.0);
	sensor->vel_time = sys_ticks.micros();
	sensor->dir = -1;
	sensor->circle = 0;
	sensor->velocity = 0.0f;
	return 0;
}

int updataSensor(magnetic_sensor_t *sensor)
{
	uint16_t val = 8192 - readTLE5012e(0x8021);
	sensor->angle_new = (val * _2PI / 8192.0);
	sensor->vel_time = sys_ticks.micros();

	float angle_d = sensor->angle_new - sensor->angle_last;
	if( fabs(angle_d) > 5.0265 ) sensor->circle += ( angle_d > 0 ) ? -1 : 1;

	sensor->angle_last = sensor->angle_new;
	sensor->dir = ( angle_d > 0 ) ? 1 : -1;
	return 0;
}

float getVelocity(magnetic_sensor_t *sensor)
{
	float deltaT = ( sensor->vel_time - sensor->vel_time_last );
	if( deltaT > 0 ){
		sensor->velocity = (((float)(sensor->circle - sensor->vel_circle_last )) * _2PI + (sensor->angle_last - sensor->vel_angle_last )) / deltaT;
	}
	sensor->vel_circle_last = sensor->circle;
	sensor->vel_angle_last = sensor->angle_last;
	sensor->vel_time_last = sensor->vel_time;

	return sensor->velocity;
}

float getAngle(magnetic_sensor_t *sensor)
{
	return (float)sensor->circle * _2PI + sensor->angle_last;
}

float getMechanicalAngle(magnetic_sensor_t *sensor)
{
	return sensor->angle_last;
}

