/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_define.h"
#include "difficultyFoc.h"
#include "magneticSensor.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "userTimer.h"
#include "user_serial.h"
#include "setupInf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

extern serial_t serial;
extern motor_t motor;
extern magnetic_sensor_t tle5012;
extern setupinf_t setup;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int c, FILE *stream)
{
	LL_USART_TransmitData8(USART2,c);
	while(!LL_USART_IsActiveFlag_TXE(USART2));
	return 1;
}

typedef struct pid_controller
{
	float p;
	float i;
	float d;

	float error_prev;
	float output_prev;
	float integral_prev;
	float output_rmap;
	uint64_t time_prve;

	float limit_up;
	float limit_down;
} pid_controller_t;

typedef struct FO_filter
{
	float lase;
	float alpha;
	float _alpha_n;
} FO_filter_t;

typedef struct system
{
	system_state_t state;
    
    float motor_p;
    float error;
    float target;

	float angle_now;
	float velocity_now;
	float elecangle_now;

	pid_controller_t velocity_pid;
	pid_controller_t angle_pid;
	pid_controller_t current_pid;

	float* used_parm;
	pid_controller_t* used_pid;

	//************************
	int8_t setupstate;

} system_t;

system_t sys;

#define DECLARE_FO_FILTER(name, alpha) FO_filter_t name = {0, alpha, (1 - alpha)}

DECLARE_FO_FILTER(angle_filter, 0.8f);
DECLARE_FO_FILTER(velocity_filter, 0.8f);

void initFOFilter(FO_filter_t *p, float alpha)
{
	p->lase = 0;
	p->alpha = alpha;
	p->_alpha_n = 1 - alpha;
}
float FOFilter(FO_filter_t *p, float data)
{
	p->lase = (data * p->alpha + (p->_alpha_n) * p->lase);
	return p->lase;
}

void PIDInit(pid_controller_t *parm)
{
	if (parm == NULL)
		return;
	parm->p = 1;
	parm->i = 0;
	parm->d = 0;

	parm->error_prev = 0;
	parm->output_prev = 0;
	parm->integral_prev = 0;

	parm->limit_up = FLT_MAX;
	parm->limit_down = FLT_MIN;
}

void PIDInitWithParm(pid_controller_t *parm, float _p, float _i, float _d,float _up, float _down, float _remap)
{
	if (parm == NULL)
		return;
	parm->p = _p;
	parm->i = _i;
	parm->d = _d;

	parm->limit_up = _up;
	parm->limit_down = _down;
	parm->output_rmap = _remap;

	parm->error_prev = 0;
	parm->output_prev = 0;
	parm->integral_prev = 0;
	parm->output_rmap = 0;
	parm->time_prve = sys_ticks.micros();
}

void PIDSetLimit(pid_controller_t *parm, float _up, float _down, float _remap)
{
	parm->limit_up = _up;
	parm->limit_down = _down;
	parm->output_rmap = _remap;
}

void PIDSetParm(pid_controller_t *parm, float _p, float _i, float _d)
{
	if (parm == NULL)
		return;

	parm->p = _p;
	parm->i = _i;
	parm->d = _d;
}

float PIDOperate(pid_controller_t *parm, float error)
{
	uint64_t time = sys_ticks.micros();
	float deltaTs = ( time - parm->time_prve ) * 1e-6;
	if(( deltaTs <= 0.0f )||( deltaTs > 0.5f )) deltaTs = 0.001;

	float proportional = parm->p * error;
	float integral = parm->integral_prev + parm->i * deltaTs * 0.5f *(error + parm->error_prev);
	integral = _CONSTRAIN(integral, parm->limit_down, parm->limit_up);

	float derivative = parm->d * (error - parm->error_prev) / deltaTs;
	float output = proportional + integral + derivative;

	output = _CONSTRAIN(output, parm->limit_down, parm->limit_up);

	if( parm->output_rmap > 0 )
	{
        float output_rate = (output - parm-> output_prev) / deltaTs;
        if (output_rate > parm->output_rmap)
            output = parm->output_prev + parm->output_rmap * deltaTs;
        else if (output_rate < -parm->output_rmap)
            output = parm->output_prev - parm->output_rmap * deltaTs;
	}

	parm->error_prev = error;
	parm->integral_prev = integral;
	parm->output_prev = output;
	parm->time_prve = time;

	return output;
}

uint32_t adc_val_dma[10];

void configADCDMA()
{
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3,(uint32_t)adc_val_dma);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3,LL_ADC_DMA_GetRegAddr(ADC1,LL_ADC_DMA_REG_REGULAR_DATA));
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 1);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	
	LL_ADC_Enable(ADC1);
	while(LL_ADC_IsActiveFlag_ADRDY(ADC1) != SET);
	LL_ADC_REG_StartConversion(ADC1);
}

void readADC()
{
	//adc_val_dma[0] = HAL_ADC_GetValue(&hadc1);

	printf("adc value is %d \r\n", adc_val_dma[0]);
}

uint8_t inRangeF(float value, float down, float up)
{
	return ((value >= down) && (value <= up)) ? 1 : 0;
}

void sendMotorParm(float sensorAngle,float velocity,uint32_t current)
{
	volatile uint32_t sendbuff[3] = {
		*(uint32_t*)&sensorAngle,
		*(uint32_t*)&velocity,
		*(uint32_t*)&current,
	};
	serial.sendPack(0xff,0x11,sizeof(sendbuff),(void*)sendbuff);
}

void sendDebugMessage(const char *format, ...)
{
	char str_buff[256];
	va_list ap;
	va_start(ap,format);
	int length = vsprintf(str_buff,format,ap);
	serial.sendPack(0xff,0x31,length,(void*)str_buff);
	va_end(ap);
}

float parm2Angle(uint8_t *p)
{
	volatile uint8_t databuff_0[8];//0x20000CB4
	memcpy((uint8_t*)databuff_0,(uint8_t*)p,sizeof(uint8_t)*8);
	volatile int32_t circle = *(int32_t*)&databuff_0[0]; // 0x00000000
	volatile int32_t angle = *(int32_t*)&databuff_0[4];  // 0x200003CC
	volatile float anglef = *(float*)&angle;
	return (float)circle * _2PI + anglef;
}

float parm2Velocity(uint8_t *p)
{
	return *(float*)&p[0];
}

void serialReceive()
{
	if(serial.isAvailable() > 0 )
	{
		uint8_t flag = 0;
		uint8_t mode_change_flag = 0;
		uint8_t id = serial.revice_pack.id;
		switch( serial.revice_pack.cmd )
		{
			case CMD_SET_IDLE : 
				sys.state = STATE_IDLE;
				sys.target = 0.0f;
				sys.used_pid = &sys.angle_pid;
				sys.used_parm = &sys.angle_now;	
				mode_change_flag = 1;
				break;
			case CMD_SET_VELOCITY : 
				sys.state = STATE_VELOCITY;
				sys.target = parm2Velocity(&serial.revice_pack.data[0]);
				sys.used_pid = &sys.velocity_pid;
				sys.used_parm = &sys.velocity_now;	
				mode_change_flag = 1;
				break;
			case CMD_SET_ANGLE : 
				sys.state = STATE_ANGLE;
				sys.target = parm2Angle(&serial.revice_pack.data[0]);
				sys.used_pid = &sys.angle_pid;
				sys.used_parm = &sys.angle_now;	
				mode_change_flag = 1;	
				break;
		}
		serial.sendPack(id,0x00,1,&flag);
		serial.clearRDFlag();

		if( mode_change_flag == 1 )
		{
			sys.error = 0.0f;
			sys.motor_p = 0.0f;
		}
	}
}

DECLARE_FO_FILTER(angle_LPF,0.9);
DECLARE_FO_FILTER(velocity_LPF,0.2);

void testEEPROM()
{
	uint8_t read_buff[10];
	HAL_I2C_Mem_Write(&hi2c2, 0x00a0, 0, I2C_MEMADD_SIZE_8BIT, "Fuck stc", 8, 100);
	HAL_Delay(10);
	memset(read_buff, '\0', sizeof(read_buff));
	HAL_I2C_Mem_Read(&hi2c2, 0x00a1, 0, I2C_MEMADD_SIZE_8BIT, read_buff, 8, 100);
	serial.printf("[FOC] Read from EEPROM is %s\r\n", read_buff);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  	SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	MX_TIM3_Init();
	MX_TIM14_Init();
	MX_ADC1_Init();
	MX_I2C2_Init();
  /* USER CODE BEGIN 2 */


	LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_0);
	LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_1);

	TIM1->CCR1 = 127;
	TIM1->CCR2 = 127;
	TIM1->CCR3 = 127;

	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);

	LL_SPI_Enable(SPI1);
	initTicks(TIM14);

	LL_TIM_EnableIT_UPDATE(TIM14);
	LL_TIM_EnableCounter(TIM14);

	InitTransmission(USART2);
	initSetupInfo();
	initSensor(&tle5012);

	//configADCDMA();
	//testEEPROM();
	//HAL_ADC_Start(&hadc1);
	//HAL_ADC_Start_DMA(&hadc1, &adc_val_dma[0], 1);

	motor.voltage_dc = 12.0;
	motor.voltage_limit = 10.0;
	DifficultyFOCInit(&motor);


	PIDInitWithParm(&sys.velocity_pid, 	0.09f, 1.0f, 0.0f, 5.0f, -5.0f, 1000.0f);
	PIDInitWithParm(&sys.angle_pid, 	5.0f,  0.0f, 0.0f, 5.0f, -5.0f, 0.0f);
	PIDInitWithParm(&sys.current_pid, 	1.0f,  0.0f, 0.0f, 5.0f, -5.0f, 0.0f);

	LL_mDelay(100);

	sys.setupstate = setup.readFromE2prom();
	if( sys.setupstate != 0 )
	{
		if( sys.setupstate == -1 ) 
			serial.printf("[FOC] read eeprom fault\r\n");
		
		serial.printf("[FOC] Start time %lld\r\n", sys_ticks.micros());
		DiffcultyFOCAlignSendor(&motor, &tle5012);
		serial.printf("[FOC] End time %lld\r\n",sys_ticks.micros());
		if( sys.setupstate == -2 )
		{
			setup.data.zero_elec_angle = motor.zero_elec_angle;
			setup.writeToE2prom();
		}
	}
	else
	{
		serial.printf("[FOC] zero elec Angle = %.2f\r\n", _R2D(setup.data.zero_elec_angle));
		serial.printf("[FOC] zero offset %.2f\r\n", _R2D(setup.data.zero_offset));
		motor.zero_elec_angle = setup.data.zero_elec_angle;
		motor.sensor_dir = kSensorCW;

		tle5012.angle_offset = _D2R(60);
	}
		
	updataSensor(&tle5012);
	angle_LPF.lase = getAngle(&tle5012);
	velocity_LPF.lase = getVelocity(&tle5012);

	sys.error = 0.0f;
	sys.state = STATE_IDLE;
	sys.target = 0.0f;
	sys.motor_p = 0.0f;
	
	sys.angle_now = getAngle(&tle5012);
	sys.velocity_now = getVelocity(&tle5012);

	sys.used_pid = &sys.angle_pid;
	sys.used_parm = &sys.angle_now;

	sys.elecangle_now = getElectricaAngle(&motor, &tle5012);
	DifficultySeting(sys.motor_p, 0.0f, sys.elecangle_now, &motor);
	setTimerPWMVal(&motor);

	sendMotorParm(1.25f,2.54f,0.0f);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		updataSensor(&tle5012);
		
		sys.angle_now = FOFilter(&angle_LPF,getAngle(&tle5012));
		sys.velocity_now = FOFilter(&velocity_LPF,getVelocity(&tle5012));

		if( sys.state != STATE_IDLE )
		{
			sys.error = sys.target - *sys.used_parm;
			sys.motor_p = PIDOperate(sys.used_pid, sys.error);
			sys.elecangle_now = getElectricaAngle(&motor, &tle5012);
			DifficultySeting(sys.motor_p, 0.0f, sys.elecangle_now, &motor);
			setTimerPWMVal(&motor);
		}
		else
		{
			DifficultySeting(0.0f, 0.0f, sys.elecangle_now, &motor);
			setTimerPWMVal(&motor);
		}
		//getanglespeed(&tle5012);
		
		serialReceive();
		sendMotorParm(sys.angle_now,sys.velocity_now,tle5012.vel_sendor);

		/*
		velocity_error = ( velocity_target - velocity_af );
		motor_p = PIDOperate(&global_pid, velocity_error);
		motor_p = _CONSTRAIN(motor_p, -5.0f, 5.0f);
		electrical_angle = getElectricaAngle(&motor, &tle5012);
		DifficultySeting(motor_p, 0.0f, electrical_angle, &motor);
		setTimerPWMVal(&motor);
		*/
		//LL_mDelay(100);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)

	{
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
		//angle++;
		//angle = angle % 360;
		//DifficultySeting(2.0,0.0,angle * _PI / 180,&motor);
		//setTimerPWMVal(&motor);
		//SetAngle(angle,100);
	}
	else if (htim == &htim14)
	{
		_sys.microscnt += 1000;
	}
}
*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

