/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "vt100.h"
#include "vt100.h"
#include "time.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t ledPosUser;
extern uint8_t led_control_mode;
extern uint8_t auto_time_off_mode;
extern uint32_t ntime_auto_off_mode;
extern uint8_t running_mode;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//#define USE_I2C_DMA
//#define USE_DMP
#define LED_TOTAL	212
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MPU6050_INT1_X_Pin GPIO_PIN_13
#define MPU6050_INT1_X_GPIO_Port GPIOC
#define MPU6050_INT1_X_EXTI_IRQn EXTI15_10_IRQn
#define BLE_WAKEUP_Pin GPIO_PIN_0
#define BLE_WAKEUP_GPIO_Port GPIOC
#define BLE_UART2_PD_Pin GPIO_PIN_1
#define BLE_UART2_PD_GPIO_Port GPIOC
#define BLE_RESET_Pin GPIO_PIN_2
#define BLE_RESET_GPIO_Port GPIOC
#define BLE_FLASHED_DEFAULT_Pin GPIO_PIN_3
#define BLE_FLASHED_DEFAULT_GPIO_Port GPIOC
#define BLE_UART2_CTS_Pin GPIO_PIN_0
#define BLE_UART2_CTS_GPIO_Port GPIOA
#define BLE_UART2_RTS_Pin GPIO_PIN_1
#define BLE_UART2_RTS_GPIO_Port GPIOA
#define BLE_UART2_TX_Pin GPIO_PIN_2
#define BLE_UART2_TX_GPIO_Port GPIOA
#define BLE_UART2_RX_Pin GPIO_PIN_3
#define BLE_UART2_RX_GPIO_Port GPIOA
#define DEBUG_UART1_TX_Pin GPIO_PIN_9
#define DEBUG_UART1_TX_GPIO_Port GPIOA
#define DEBUG_UART1_RX_Pin GPIO_PIN_10
#define DEBUG_UART1_RX_GPIO_Port GPIOA
#define MCU_RUN_Pin GPIO_PIN_11
#define MCU_RUN_GPIO_Port GPIOA
#define LED_3V3_PWR_EN_Pin GPIO_PIN_10
#define LED_3V3_PWR_EN_GPIO_Port GPIOC
#define BATT_ALRT_INT_Pin GPIO_PIN_12
#define BATT_ALRT_INT_GPIO_Port GPIOC
#define BATT_ALRT_INT_EXTI_IRQn EXTI15_10_IRQn
#define MPU6050_SCL1_Pin GPIO_PIN_8
#define MPU6050_SCL1_GPIO_Port GPIOB
#define MPU6050_SDA1_Pin GPIO_PIN_9
#define MPU6050_SDA1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define STAT_SLEEP 0x01
#define STAT_RUNNING 0x02
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
