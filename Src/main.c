/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mdbt42q.h"
#include "ws2812b.h"
#include <stdio.h>
#include <string.h>
#include "ema_filter.h"
#include "dwt_stm32_delay.h"
//#include "pid.h"
#ifdef USE_DMP
#include "mpu6050_dmp.h"
#else
#include "mpu6050.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_GREEN_TOGGLE	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t last_time=0, mpu_last_time=0, led_last_time=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void power_en(void)
{
  // POWER Controler
  HAL_GPIO_WritePin(PERI_3V3_PWR_nEN_GPIO_Port, PERI_3V3_PWR_nEN_Pin, GPIO_PIN_RESET); // PERI_3V3_PWR_nEN
  HAL_GPIO_WritePin(LED_LMIT_EN_GPIO_Port, 			LED_LMIT_EN_Pin, 			GPIO_PIN_SET); // LED_LMIT_EN, Hight Enable, Low Disable
  HAL_GPIO_WritePin(LED_3V3_PWR_nEN_GPIO_Port, 	LED_3V3_PWR_nEN_Pin, 	GPIO_PIN_SET); // LED_3V3_PWR_nEN, High Enable, Low Disable
}

//float ledPos = 0;
float ledPos_before = 0.0f;
float pidControl = 0.0f;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t buff[256];
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
  MX_I2C1_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM11_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  printf("Booting LittleCat Board!!!!221\r\n\n");
  power_en();
  ble_gpio_init();
  initLEDMOSI();
  time_setup();
  DWT_Delay_Init();
//  pidData = (PidData_t *)calloc(1, sizeof(PidData_t));
  //! Initialization of PID factors
//  PID_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , pidData);


  Cal_Filter = (MovingFilter_t *)calloc(4, sizeof(MovingFilter_t));
  EMA_FILTER_Init(EMA_Alpha, Cal_Filter);

  DMP_Init();

//  printf("Calibration ready\r\n");
  // Waiting the device status until the stable state
//  for(register int i=0; i<1000; i++) {
//	  Read_DMP();
//	  HAL_Delay(5);
//	  if( (i%100) == 0 ) HAL_UART_Transmit(&huart1, (uint8_t *)&".", 1, 100);
//  }
//  printf("\r\nCalibration start\r\n");
  // Calibration of the mpu6050
//  for(register int i=0; i<2000; i++)
//  {
//	  Read_DMP();
//	  DEMA_Filter( Roll, 		&Cal_Filter[0] );
////	  DEMA_Filter( Pitch, 		&Cal_Filter[1] );
////	  DEMA_Filter( Yaw,	 		&Cal_Filter[2] );
////	  DEMA_Filter( Roll_reverse, &Cal_Filter[3] );
////	  vt100SetCursorPos( 3, 0);
////	  vt100ClearLinetoEnd();
////	  printf("\rRoll : %f\r\n", Roll);
////	  printf("\rDEMA : %f\r\n", Cal_Filter[0].DEMA);
//	  HAL_Delay(5);
//	  if( (i%100) == 0 ) HAL_UART_Transmit(&huart1, (uint8_t *)&".", 1, 100);
//  }
//  base_roll			= Cal_Filter[0].DEMA;
//  base_pitch		= Cal_Filter[1].DEMA;
//  base_yaw			= Cal_Filter[2].DEMA;
//  base_roll_reverse	= Cal_Filter[3].DEMA;
//  printf("\r\nCalibration is done.\r\n");
  //HAL_Delay(2000);
  Cal_done = 1;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uart_recv_int_enable();
  HAL_Delay(1000);

  vt100ClearScreen();
  HAL_TIM_Base_Start_IT(&htim10);

  targetAnglel -= base_roll;
  if (targetAnglel < 0) targetAnglel = 360.0 + targetAnglel;
  targetLedPos = (LED_TOTAL / 360.0f) * roundf(targetAnglel);
  ledPos =  (LED_TOTAL / 360.0f) * roundf(Roll);
  ledPos = ledPos - targetLedPos;
  if (ledPos < 0) ledPos = LED_TOTAL + ledPos;

  printf("\r\nCalibration is done.\r\n");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (ledPos_before != ledPos) {

		  setAllPixelColor(0, 0, 0);
//		  setPixelColor( (uint16_t)DEMA_Filter( ledPos, &Cal_Filter[0] ), 0, 50, 0 );
		  setPixelColor( (uint16_t)ledPos, 0, 50, 0 );
		  ledPos_before = ledPos;
//		  memset(buff, 0, sizeof(buff));
//		  sprintf(buff, "roll : %d, pos : %d\r\n", (uint16_t)Roll, (uint16_t)ledPos);
//		  HAL_UART_Transmit(&huart2, buff, strlen(buff), 100);
//		  printf("roll : %d, pos : %d\r\n", (uint16_t)Roll, (uint16_t)ledPos);
	  }
	  DWT_Delay_us(1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/*
  if (htim->Instance ==TIM10)
  {
    HAL_TIM_Base_Stop_IT(&htim10);
    Read_DMP();
    //mpu_last_time = time_ms();
    HAL_TIM_Base_Start_IT(&htim10);
  }
*/
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == MPU6050_INT1_X_Pin)
  {
    // To do
	  if(Cal_done) {
		  Read_DMP();
	  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
