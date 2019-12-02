/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define GET_STATUS  0x01
#define SET_WAKEUP	0x02
#define SET_SLEEP 0x03
#define GET_DEGREE  0x04
#define SET_LED_POS	0x05
#define SET_LED_COLOR 0x06
#define SET_RAND_LED_MODE 0x07
#define SET_AUTO_TIME_OFF_MODE  0x08
#define SET_N_TIME_AUTO_OFF 0x09
#define GET_N_TIME_AUTO_OFF 0x10
#define GET_BAT 0x11
#define GET_RUN_TIME  0x12
#define START 0x13
#define STOP  0x14
#define SET_TIME_SYNC 0x15

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */

#define MAX_SERIAL_BUF 1024
#define PACKET_SIZE 12

typedef struct
{
	uint8_t  buf[MAX_SERIAL_BUF];
	uint16_t head; //Ï≤òÎ¶¨?ïòÎ©? head Ï¶ùÍ?.
	uint16_t tail; //?àò?ã†?êòÎ©? tail Ï¶ùÍ?.
} Buffer_Serial;

extern Buffer_Serial SerialTx; //?Ü°?ã†?? head,tail ?Ç¨?ö©?ïà?ïòÍ≥? bufÎß? ?Ç¨?ö©.
extern Buffer_Serial SerialRx; //?àò?ã†?? ÎßÅÎ≤Ñ?çº ?Ç¨?ö©

typedef struct
{
  uint8_t addr;
  uint8_t cmd;
  uint8_t data;
  uint16_t crc;
} BLE_Cmd_Data;

void process(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void uart_recv_int_enable(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
