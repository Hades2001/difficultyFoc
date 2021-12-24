#include "magneticSensor.h"
#include "user_serial.h"

#define SET_SPI_MOSI_MODE(SPIx,MODE) LL_SPI_SetTransferDirection(SPIx,(MODE == SPI_TX_MODE ) ? LL_SPI_HALF_DUPLEX_TX : LL_SPI_HALF_DUPLEX_RX)

uint8_t crc_j1850_tab[256] = {
	0x00,0x1d,0x3a,0x27,0x74,0x69,0x4e,0x53,0xe8,0xf5,0xd2,0xcf,0x9c,0x81,0xa6,0xbb,
	0xcd,0xd0,0xf7,0xea,0xb9,0xa4,0x83,0x9e,0x25,0x38,0x1f,0x02,0x51,0x4c,0x6b,0x76,
	0x87,0x9a,0xbd,0xa0,0xf3,0xee,0xc9,0xd4,0x6f,0x72,0x55,0x48,0x1b,0x06,0x21,0x3c,
	0x4a,0x57,0x70,0x6d,0x3e,0x23,0x04,0x19,0xa2,0xbf,0x98,0x85,0xd6,0xcb,0xec,0xf1,
	0x13,0x0e,0x29,0x34,0x67,0x7a,0x5d,0x40,0xfb,0xe6,0xc1,0xdc,0x8f,0x92,0xb5,0xa8,
	0xde,0xc3,0xe4,0xf9,0xaa,0xb7,0x90,0x8d,0x36,0x2b,0x0c,0x11,0x42,0x5f,0x78,0x65,
	0x94,0x89,0xae,0xb3,0xe0,0xfd,0xda,0xc7,0x7c,0x61,0x46,0x5b,0x08,0x15,0x32,0x2f,
	0x59,0x44,0x63,0x7e,0x2d,0x30,0x17,0x0a,0xb1,0xac,0x8b,0x96,0xc5,0xd8,0xff,0xe2,
	0x26,0x3b,0x1c,0x01,0x52,0x4f,0x68,0x75,0xce,0xd3,0xf4,0xe9,0xba,0xa7,0x80,0x9d,
	0xeb,0xf6,0xd1,0xcc,0x9f,0x82,0xa5,0xb8,0x03,0x1e,0x39,0x24,0x77,0x6a,0x4d,0x50,
	0xa1,0xbc,0x9b,0x86,0xd5,0xc8,0xef,0xf2,0x49,0x54,0x73,0x6e,0x3d,0x20,0x07,0x1a,
	0x6c,0x71,0x56,0x4b,0x18,0x05,0x22,0x3f,0x84,0x99,0xbe,0xa3,0xf0,0xed,0xca,0xd7,
	0x35,0x28,0x0f,0x12,0x41,0x5c,0x7b,0x66,0xdd,0xc0,0xe7,0xfa,0xa9,0xb4,0x93,0x8e,
	0xf8,0xe5,0xc2,0xdf,0x8c,0x91,0xb6,0xab,0x10,0x0d,0x2a,0x37,0x64,0x79,0x5e,0x43,
	0xb2,0xaf,0x88,0x95,0xc6,0xdb,0xfc,0xe1,0x5a,0x47,0x60,0x7d,0x2e,0x33,0x14,0x09,
	0x7f,0x62,0x45,0x58,0x0b,0x16,0x31,0x2c,0x97,0x8a,0xad,0xb0,0xe3,0xfe,0xd9,0xc4,
};

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

#define CRC_1850(c,d) 	crc_j1850_tab[c^d]
#define GET_MSB_BYTE(d)	((d >> 8 )&0xff)
#define GET_LSB_BYTE(d)	((d & 0xff))

extern serial_t serial;

int checkSafetyWord(uint16_t cmd16,uint16_t data16,uint16_t safetyWord )
{
	uint8_t crc = 0xff;

    if (!((safetyWord)&SYSTEM_ERROR_MASK)) return -1;
    else if (!((safetyWord)&INTERFACE_ERROR_MASK)) return -2;
    else if (!((safetyWord)&INV_ANGLE_ERROR_MASK)) return -3;

	crc = CRC_1850(crc,GET_MSB_BYTE(cmd16));
	crc = CRC_1850(crc,GET_LSB_BYTE(cmd16));
	crc = CRC_1850(crc,GET_MSB_BYTE(data16));
	crc = CRC_1850(crc,GET_LSB_BYTE(data16));

	return (GET_LSB_BYTE(safetyWord) == ((~crc)&0xff)) ? 1 : 0;
}

uint8_t readTLE5012e(uint16_t cmd,uint16_t* data)
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

	int check = checkSafetyWord(cmd,received[0],received[1]);
	*data = 8192 - (((received[0] & 0x7fff) << 1) >> 3);

	return check;
}

uint8_t readTLE5012e_raw(uint16_t cmd,uint16_t* data)
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

	int check = checkSafetyWord(cmd,received[0],received[1]);
	*data = received[0];

	return check;
}


int initSensor(magnetic_sensor_t *sensor)
{
	uint16_t val;
	uint8_t res = 0;
	SET_SPI_MOSI_MODE(SPI1,SPI_TX_MODE);
	do
	{
		res = readTLE5012e(0x8021,&val);
	} while (res != 1);
	
	sensor->angle_last = (val * _2PI / 8192.0);
	sensor->vel_angle_last = (val * _2PI / 8192.0);
	sensor->vel_time_last = sys_ticks.micros();
	sensor->vel_circle_last = 0;

	do
	{
		res = readTLE5012e(0x8021,&val);
	} while (res != 1);

	sensor->angle_new = (val * _2PI / 8192.0);
	sensor->vel_time = sys_ticks.micros();
	sensor->dir = -1;
	sensor->circle = 0;
	sensor->velocity = 0.0f;

	sensor->angle_offset = 0.0f;
	return 0;
}

int getanglespeed(magnetic_sensor_t *sensor)
{
	int16_t val;
	uint8_t res = 0;
	do
	{
		res = readTLE5012e_raw(0x8031,&val);
	} while (res != 1);

	val = (val & 0x7fff);
	val = ( val & 0x4000 ) ? ( val - 32768 ) : val;
	sensor->vel_sendor = -(float)val;
	return 0;
}

int updataSensor(magnetic_sensor_t *sensor)
{
	uint16_t val;
	uint8_t res = 0;
	do
	{
		res = readTLE5012e(0x8021,&val);
	} while (res != 1);

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
	float deltaT = ((float)( sensor->vel_time - sensor->vel_time_last )) / 1e3;
	if( deltaT > 0 ){
		sensor->velocity = (((float)(sensor->circle - sensor->vel_circle_last )) * _2PI + (sensor->angle_last - sensor->vel_angle_last )) * 1000 / deltaT;
		//sensor->velocity = ((sensor->circle != sensor->vel_circle_last)||(fabs(sensor->angle_last - sensor->vel_angle_last) > 0.0005f)) ? (((float)(sensor->circle - sensor->vel_circle_last )) * _2PI + (sensor->angle_last - sensor->vel_angle_last )) / deltaT : 0.0f;
	}
	sensor->vel_circle_last = sensor->circle;
	sensor->vel_angle_last = sensor->angle_last;
	sensor->vel_time_last = sensor->vel_time;

	return sensor->velocity;
}

float getAngle(magnetic_sensor_t *sensor)
{
	return (float)sensor->circle * _2PI + sensor->angle_last - sensor->angle_offset;
}

float getMechanicalAngle(magnetic_sensor_t *sensor)
{
	return sensor->angle_last;
}

