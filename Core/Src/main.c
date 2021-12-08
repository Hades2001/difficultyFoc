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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int c, FILE *stream)
{
	LL_USART_TransmitData8(USART2,c);
	while(!LL_USART_IsActiveFlag_TXE(USART2));
	return 1;
}

typedef struct
{
	float p;
	float i;
	float d;

	float error_prev;
	float output_prev;
	float integral_prev;

	float limit_up;
	float limit_down;
} pid_controller_t;

typedef struct
{
	float lase;
	float alpha;
	float _alpha_n;
} FO_filter_t;

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
	return (data * p->alpha + (p->_alpha_n) * p->lase);
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

void PIDInitWithParm(pid_controller_t *parm, float _p, float _i, float _d)
{
	if (parm == NULL)
		return;
	parm->p = _p;
	parm->i = _i;
	parm->d = _d;

	parm->error_prev = 0;
	parm->output_prev = 0;
	parm->integral_prev = 0;
}

void PIDSetLimit(pid_controller_t *parm, float _up, float _down)
{
	parm->limit_up = _up;
	parm->limit_down = _down;
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
	float proportional = parm->p * error;
	float integral = parm->integral_prev + parm->i * (error + parm->error_prev);
	integral = _CONSTRAIN(integral, parm->limit_down, parm->limit_up);
	float derivative = parm->d * (error - parm->error_prev);
	float output = proportional + integral + derivative;
	output = _CONSTRAIN(output, parm->limit_down, parm->limit_up);

	parm->error_prev = error;
	parm->integral_prev = integral;
	parm->output_prev = output;

	return output;
}

pid_controller_t global_pid;

void testEEPROM()
{
	
	uint8_t read_buff[10];
	HAL_I2C_Mem_Write(&hi2c2, 0x00a0, 0, I2C_MEMADD_SIZE_8BIT, "Fuck stc", 8, 100);
	HAL_Delay(10);
	memset(read_buff, '\0', sizeof(read_buff));
	HAL_I2C_Mem_Read(&hi2c2, 0x00a1, 0, I2C_MEMADD_SIZE_8BIT, read_buff, 8, 100);
	printf("Read from EEPROM is %s\r\n", read_buff);
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

uint8_t tx_buff[256];
void uartDMATransmit(uint8_t *data,uint8_t length)
{
	memcpy(tx_buff,data,sizeof(uint8_t)*length);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2,(uint32_t)(tx_buff));
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2,LL_USART_DMA_GetRegAddr(USART2,LL_USART_DMA_REG_DATA_TRANSMIT));
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, length);
	LL_USART_EnableDMAReq_TX(USART2);
	
	LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_2);
	LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_2,length);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}

void sendMotorParm(float sensorAngle,float velocity,float current)
{
	uint32_t sendbuff[3] = {
		*(uint32_t*)&sensorAngle,
		*(uint32_t*)&velocity,
		*(uint32_t*)&current,
	};
	serial.sendPack(0x11,sizeof(sendbuff),(void*)sendbuff);
}

void sendDebugMessage(const char *format, ...)
{
	char str_buff[256];
	va_list ap;
	va_start(ap,format);
	int length = vsprintf(str_buff,format,ap);
	serial.sendPack(0x31,length,(void*)str_buff);
	va_end(ap);
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
	initSensor(&tle5012);

	initTicks(TIM14);
	LL_TIM_EnableIT_UPDATE(TIM14);
	LL_TIM_EnableCounter(TIM14);

	InitTransmission(USART2);


	//configADCDMA();
	//testEEPROM();

	//HAL_ADC_Start(&hadc1);

	//HAL_ADC_Start_DMA(&hadc1, &adc_val_dma[0], 1);

	motor.voltage_dc = 12.0;
	motor.voltage_limit = 10.0;
	DifficultyFOCInit(&motor);

	PIDInitWithParm(&global_pid, 1.0f, 0.0f, 0.0f);
	PIDSetLimit(&global_pid, 2520.0f, -2520.0f);

	LL_mDelay(100);

	serial.printf("[FOC] Start time %lld\r\n", sys_ticks.micros());
	DiffcultyFOCAlignSendor(&motor, &tle5012);
	serial.printf("[FOC] End time %lld\r\n",sys_ticks.micros());

	float electrical_angle = getElectricaAngle(&motor, &tle5012);
	float motor_p = 1.0f;
	float target = _D2R(120);
	float angle_error = 0.0f;
	uint64_t timepoint = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		timepoint = sys_ticks.micros();
		updataSensor(&tle5012);
		angle_error = target - getAngle(&tle5012);
		motor_p = PIDOperate(&global_pid, angle_error);
		motor_p = _CONSTRAIN(motor_p, -1.0f, 1.0f);
		electrical_angle = getElectricaAngle(&motor, &tle5012);
		DifficultySeting(motor_p, 0.0f, electrical_angle, &motor);
		setTimerPWMVal(&motor);
		sendMotorParm(getAngle(&tle5012),0.0f,0.0f);
		//serial.printf("[FOC] Cycle time %lld\r\n", sys_ticks.micros() - timepoint);
		
	/*
		updataSensor(&tle5012);
		LL_mDelay(100);
		printf("Angle : %.2f\r\n",_R2D(getAngle(&tle5012)));
		*/
		/*
		if( serial.isAvailable() > 0 )
		{
			printf("revicd data %s\r\n",serial.revice_pack.data);
			serial.available = 0;
		}
		*/
		//readADC();

		/*
		updataSensor(&tle5012);
		if( inRangeF(getAngle(&tle5012),_D2R(10),_D2R(30)))
		{
			if( tle5012.dir == 1 )
			{
				angle_error = ( _D2R(10) - fabs(( getAngle(&tle5012) - _D2R(20)))) * 10.0f;
				printf("angle_error = %.2f\r\n",angle_error);
			}
			//else if( tle5012.dir == -1 )
			//{
			//	angle_error = ( getAngle(&tle5012) - _D2R(10)) * 5.0f;
			//}
			motor_p = _CONSTRAIN( angle_error ,-2.0f,2.0f);

		}
		else
		{
			motor_p = 0.0f;
		}
		electrical_angle = getElectricaAngle(&motor,&tle5012);
		DifficultySeting(-motor_p, 0.0f, electrical_angle, &motor);
		setTimerPWMVal(&motor);
		*/
		//readADC();

		/*
	sensor_read = getAngleR() - sensorAngle;
	sensor_read = ( sensor_read > 0 ) ? sensor_read : ( 2520.0 + sensor_read );
	sensor_read = ( sensor_read > 1260 ) ? ( sensor_read - 2520 ) : sensor_read;

	angle_erroe = angle_set - sensor_read;

	angle_out = sensor_read + PIDOperate(&global_pid,angle_erroe);
	DifficultySeting(1.0,0.0,((float)( fmod(angle_out, 360 ))) * _PI / 180,&motor);
	setTimerPWMVal(&motor);
	printf("Angle : %.2f,%.2f\r\n",sensor_read,angle_out);
	*/
		/*

		*/
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

