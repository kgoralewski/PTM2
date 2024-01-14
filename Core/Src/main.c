/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2c.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STOP = 0,
  RUN = 1
} state_t;

struct Motor {
	int dir, actRpm, setRpm, count, actCount, lastCount, pwm, err, lastErr, temp;
	bool overheat;
	state_t state;
	uint16_t tempRead;
	double integral, derivative;
} motor;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PWM 999
#define ENCODER_FREQ 20.0
#define ENCODER_PULSE 960

#define KP 7.0
#define KI 3.1
#define KD 0.08

#define TEMP_MIN 45
#define TEMP_MAX 50

#define RxBuffer_Size 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t RxBuffer[RxBuffer_Size];
struct lcd_disp disp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void displayNormal();
void displayOverheat();
void sendData();
void setDir();
void temp();
void pid();
void countRPM();
void procesData();
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	bool startReading, upReading, downReading;
	bool lastUpReading = 1;
	bool lastDownReading = 1;
	bool lastStartReading = 1;

	motor.state = 0;
	motor.dir = 1;
	motor.setRpm = 0;
	motor.overheat = 0;
	disp.addr = (0x3f << 1);
	disp.bl = true;

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
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_ADC_Start(&hadc2);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuffer, RxBuffer_Size);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	lcd_init(&disp);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	while (1) {


		sendData();
		temp();

		if(motor.temp > TEMP_MAX){
			motor.overheat = 1;
			motor.state = 0;
			displayOverheat();
		}
		else if(motor.temp < TEMP_MIN){
			motor.overheat = 0;
			displayNormal();
		}

		lcd_display(&disp);

		startReading = HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin);
		if (startReading == 1 && lastStartReading == 0) {
			if (motor.state == 0 && motor.overheat == 0)
				motor.state = 1;
			else
				motor.state = 0;
		}
		lastStartReading = startReading;

		downReading = HAL_GPIO_ReadPin(RPM_DOWN_GPIO_Port, RPM_DOWN_Pin);
		if (downReading == 1 && lastDownReading == 0) {
			motor.setRpm = motor.setRpm - 20;
		}
		lastDownReading = downReading;

		upReading = HAL_GPIO_ReadPin(RPM_UP_GPIO_Port, RPM_UP_Pin);
		if (upReading == 1 && lastUpReading == 0) {
			motor.setRpm = motor.setRpm + 20;
		}
		lastUpReading = upReading;

		if (motor.state) {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			setDir();
		} else {
			HAL_GPIO_WritePin(RIGHT_GPIO_Port, RIGHT_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LEFT_GPIO_Port, LEFT_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			motor.err = 0;
			motor.lastErr = 0;
			motor.derivative = 0;
			motor.integral = 0;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM16
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void displayNormal(){
	char statusName[2][7] = {{"STOP  "}, {"START "}};
	sprintf((char *)disp.first_line, "Status: %s", statusName[motor.state]);
	sprintf((char *)disp.second_line, "Set RPM: %d    ", motor.setRpm);
	sprintf((char *)disp.third_line, "Actual RPM: %d    ", motor.actRpm);
	sprintf((char *)disp.fourth_line, "Temperature: %d%cC ", motor.temp, 223);
}

void displayOverheat(){
	sprintf((char *)disp.first_line, "                    ");
	sprintf((char *)disp.second_line, "     !OVERHEAT!   ");
	sprintf((char *)disp.third_line,  "       !%d%cC!   ", motor.temp, 223);
	sprintf((char *)disp.fourth_line, "                    ");
}

void sendData(){
	char msg[32];
	sprintf((char*) msg, "%d, %d, %d, %d\n", motor.actRpm, motor.setRpm, motor.err, 0);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
}

void setDir(){
	if(motor.setRpm < 0){
		motor.dir = -1;
		HAL_GPIO_WritePin(RIGHT_GPIO_Port, RIGHT_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEFT_GPIO_Port, LEFT_Pin, GPIO_PIN_SET);
	}
	else {
		motor.dir = 1;
		HAL_GPIO_WritePin(LEFT_GPIO_Port, LEFT_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RIGHT_GPIO_Port, RIGHT_Pin, GPIO_PIN_SET);
	}
}

void temp(){
	if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
		motor.tempRead = HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Start(&hadc2);
	}

	motor.temp = 33000/(3.3*motor.tempRead/4096)-10000;
	motor.temp = motor.temp*-0.00438+69.5;
}

void pid() {
	if(motor.state == 1){
	motor.err = (motor.setRpm - motor.actRpm)*motor.dir;
	motor.integral += (double)(motor.err / ENCODER_FREQ);
	motor.derivative = (double) (motor.err - motor.lastErr) * ENCODER_FREQ;
	motor.pwm = (int) KP * (motor.err + KI* motor.integral + KD * motor.derivative);
	motor.lastErr = motor.err;
	if (motor.pwm < 0)
		motor.pwm = 0;
	else if (motor.pwm > MAX_PWM)
		motor.pwm = MAX_PWM;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, motor.pwm);
	}
}

void countRPM(){
	if (motor.actCount < motor.lastCount) {
		motor.count = motor.actCount + ENCODER_PULSE - motor.lastCount;
	} else {
		motor.count = motor.actCount - motor.lastCount;
	}
	motor.lastCount = motor.actCount;
	motor.actRpm = (int) motor.count * (60 * ENCODER_FREQ / ENCODER_PULSE);
	if(motor.actRpm>500)
		motor.actRpm =  motor.actRpm-1200;
}

    void procesData(){
	switch (RxBuffer[0]) {
		case 'R':
			motor.state = RUN;
			break;
		case 'S':
			motor.state = STOP;
			break;
		case 'P':
			motor.setRpm = atoi((const char*)RxBuffer+2);
			break;
		default:
			break;
	}
	memset(RxBuffer, 0, sizeof(RxBuffer));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART2){
		procesData();
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuffer, RxBuffer_Size);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM16) {
		motor.actCount = __HAL_TIM_GET_COUNTER(&htim2);
		countRPM();
		pid();
	}
}
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
	while (1) {
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
