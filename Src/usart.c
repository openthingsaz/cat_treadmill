/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "ble_cmd.h"
#include "ws2812b.h"
#include "mpu6050_dmp.h"
#include "power.h"
#include "tim.h"
#include <stdbool.h>
int _write(int fd, char *str, int len)
{
  for(int i=0; i<len; i++)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)&str[i], 1, 0xFFFF);
  }
  return len;
}

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = DEBUG_UART1_TX_Pin|DEBUG_UART1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA0-WKUP     ------> USART2_CTS
    PA1     ------> USART2_RTS
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = BLE_UART2_CTS_Pin|BLE_UART2_RTS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BLE_UART2_TX_Pin|BLE_UART2_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, DEBUG_UART1_TX_Pin|DEBUG_UART1_RX_Pin);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA0-WKUP     ------> USART2_CTS
    PA1     ------> USART2_RTS
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, BLE_UART2_CTS_Pin|BLE_UART2_RTS_Pin|BLE_UART2_TX_Pin|BLE_UART2_RX_Pin);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */


Buffer_Serial SerialTx;
Buffer_Serial SerialRx;
uint8_t rx1_data;
uint8_t rx2_data;
//uint8_t rxBuff[MAX_SERIAL_BUF];
uint8_t packet[PACKET_SIZE];
//uint8_t packet[1024];
uint8_t mem_chk = 0;
uint8_t inx = 0;
uint8_t recv_step  = 0;
uint16_t dataLen = 0;
uint16_t dataLenTmp = 0;
uint16_t data_chk = 0;
uint16_t crc_chk = 0;
uint16_t dataLen2 = 0;
uint16_t data_chk2 = 0;

void uart_recv_int_enable(void)
{
  memset(&SerialRx, 0, sizeof(SerialRx));
  memset(&SerialTx, 0, sizeof(SerialTx));
  HAL_UART_Receive_IT(&huart1, &rx1_data, 1);
  HAL_UART_Receive_IT(&huart2, &rx2_data, 1);
}

static const unsigned short crc16tab[256]= {
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};
  
uint16_t crc16_ccitt(const void *buf, int len)
{
	int counter;
	unsigned short crc = 0;
	for( counter = 0; counter < len; counter++)
		crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *(char *)buf++)&0x00FF];
	return crc;
}

void transmit_data(uint8_t cmd, uint8_t* data, uint32_t len)
{
  uint16_t crc = 0;
  uint32_t inx = 0;
  uint8_t* send_data;
  uint32_t cmd_data_len;

  send_data = malloc(len + 20);
  send_data[inx++] = STX;
  cmd_data_len = len + 1;
  memcpy(&send_data[1], &cmd_data_len, sizeof(cmd_data_len));
  memcpy(&send_data[3], &cmd_data_len, sizeof(cmd_data_len));
  inx = inx + 4;
  send_data[inx++] = cmd;
  memcpy(&send_data[6], data, len);
  inx = inx + len;
  crc = crc16_ccitt((void*)&send_data[5], cmd_data_len);
  send_data[inx++] = (crc & 0xFF00) >> 8;
  send_data[inx++] = (crc & 0x00FF);
  send_data[inx++] = ETX;
  HAL_UART_Transmit(&huart2, send_data, inx , 100);
  free(send_data);
}

typedef struct _cat_data {   // 구조체 이름은 _Person
    uint32_t timestamp;
    uint32_t distance;
    uint32_t move_time;
} cat_data; 

cat_data cat_mode_data[7];
void cmd_process(uint8_t cmd, uint32_t data)
{
  //printf("cmd : %02x\r\n", cmd);
  switch (cmd) {
    case GET_STATUS :
      data = (uint32_t)get_status();
    	
      transmit_data(GET_STATUS, (uint8_t*)&data, sizeof(data));
      break;

    case GET_DEGREE :      
      data = (uint32_t)get_degree();
      transmit_data(GET_DEGREE, (uint8_t*)&data, sizeof(data));
      break;
    
    case SET_LED_POS :
      if (get_status() == STAT_SLEEP)
        set_wakeup();
      targetLedPos = (LED_TOTAL / 360.0f) * data;
      set_led_pos((uint8_t)data);
      break;

    case SET_LED_COLOR :
      if (get_status() == STAT_SLEEP)
        set_wakeup();
      dis_rand_led_mode();
      set_led_col(data);
      break;
    
    case SET_RAND_LED_MODE :
      set_rand_led_mode();
      break;

    case GET_BAT :
      data = get_bat_val();
      transmit_data(GET_BAT, (uint8_t*)&data, sizeof(data));
      break;

    //case GET_RUN_TIME :
    //  transmit_data(GET_BAT, &data, sizeof(data));
    //  break;

    case SET_TIME_SYNC :
      timecnt = 0;
      timestamp = data;
      break;

    case GET_MOVE_DATA :
      for (int i=0; i<7; i++) {
        cat_mode_data[i].timestamp = get_now_time() + i;
        cat_mode_data[i].distance = i%200;
        cat_mode_data[i].move_time = i%100;
        //HAL_Delay(1005);
      }
      transmit_data(GET_MOVE_DATA, (uint8_t*)&cat_mode_data, sizeof(cat_mode_data));
      break;

    case GET_POWER_MODE :
      get_power_mode();
      break;

    case SET_POWER_MODE :
      printf("SET_POWER_MODE\r\n");
      set_power_mode((uint8_t)data);
      break;

    default :
      printf("default\r\n"); 
  }
}

void get_move_data(uint32_t time)
{
  
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if(huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, &rx1_data, 1);
    printf("%c\r\n", rx1_data);	
  }
 
  if(huart->Instance == USART2)
	{
    SerialRx.buf[SerialRx.tail] = rx2_data;
    HAL_UART_Receive_IT(&huart2, &rx2_data, 1);
		if (MAX_SERIAL_BUF <= SerialRx.tail + 1)
		{
			SerialRx.tail = 0;
		}
		else
		{
			SerialRx.tail++;
		}
	}
}

void DebugPrint(uint8_t ch){
  uint8_t buff[256];
  memset(buff, 0, sizeof(buff));
}
void process(void)
{
  uint16_t head = 0;
  uint16_t tail = 0;
  uint16_t rxLen = 0;
  uint16_t i = 0;
  uint16_t crc = 0;
  bool recv_end = false;
  
  uint8_t cmd;
  uint32_t data;

  head = SerialRx.head;
  tail = SerialRx.tail;
  if (head != tail) 
  {
    if (head <= tail)
    {
      rxLen = tail - head;
    }
    else
    {
      rxLen = tail + MAX_SERIAL_BUF - head;
    }
    if (rxLen)
    {
      //printf("rxLen : %d\r\n", rxLen);
      for (i=0; i<rxLen; i++) 
      {
        //printf("R : %02x, recv_step : %d, dataLenTmp : %d, dataLen : %d\r\n", SerialRx.buf[SerialRx.head+i], recv_step, dataLenTmp, dataLen);
        if (recv_step == 0) {
          if (inx == 0 && SerialRx.buf[SerialRx.head+i] == STX) {
            recv_step = 1;
            dataLen = 0;
            data_chk = 0;
            inx = 0;
            continue;
          }
        }

        else if (recv_step == 1) {
          if (data_chk == 0) 
          {
            dataLen = SerialRx.buf[SerialRx.head+i];
            ++data_chk;
            continue;
          }
          else {
            dataLen = dataLen | (SerialRx.buf[SerialRx.head+i] << 8);
            dataLenTmp = dataLen;
            memset(packet, 0, sizeof(packet));
            //printf("dataLenTmp : %d\r\n", dataLenTmp);
            data_chk = 0;
            recv_step = 2;
            continue;
          }
        }
        else if (recv_step == 2) {
          if (data_chk2 == 0) 
          {
            dataLen2 = SerialRx.buf[SerialRx.head+i];
            ++data_chk2;
            continue;
          }
          else {
            dataLen2 = dataLen2 | (SerialRx.buf[SerialRx.head+i] << 8);
            //printf("dataLen2 : %d\r\n", dataLen2);
            data_chk2 = 0;
            if (dataLen != dataLen2) {
              recv_step = 0;
              dataLen2 = 0;
              dataLen = 0;
            }
            else recv_step = 3;
            continue;
          }
        }
        else if (recv_step == 3) {
          packet[inx++] = SerialRx.buf[SerialRx.head+i];
          --dataLenTmp;
          if (dataLenTmp == 0) {
            recv_step = 4;
            //printf("i : %d, rxLen : %d\r\n", i, rxLen);
            continue;
          }
        }
        else if (recv_step == 4) {
          packet[inx++] = SerialRx.buf[SerialRx.head+i];
          ++crc_chk;
          if (crc_chk >= 2) 
          {
            crc_chk = 0;
            recv_step = 5;
            continue;
          }
        }
        else if (recv_step == 5) 
        {
          if (SerialRx.buf[SerialRx.head+i] == ETX) {
            recv_end = true;
          }
          else {
            dataLen = 0;
            dataLenTmp = 0;
          }
          recv_step = 0;
          inx = 0;
        }
      }

      while (rxLen--)
      {
        if (MAX_SERIAL_BUF <= SerialRx.head + 1)
        {
          SerialRx.head = 0;
        }
        else
        {
          SerialRx.head++; //?��?�� ?��?��?�� ?��치�?? ?���?.
        }
      }
      //printf("recv_end : %d\r\n", recv_end);
      if (recv_end == true) 
      {
        //printf("recv_end : %d\r\n", recv_end);
        //for(int i=0; i<dataLen+2; i++)
        //  printf("packet : %02x\r\n", packet[i]);
        crc = crc16_ccitt((void*)&packet[0], dataLen + 2); // 
        if (crc == 0) // Crc OK
        {
          cmd = packet[0];
          memcpy(&data, &packet[1], dataLen-1); // -1 is command
          cmd_process(cmd, data);
          //printf("cmd : %02x, data : %08x\r\n", cmd, data);
        }
        else {
          //send NACK
          SerialTx.buf[0] = NCK;
          HAL_UART_Transmit(&huart2, SerialTx.buf, 1, 1);
          printf("NACK\r\n");
        }
        dataLen = 0;
        inx = 0;

      }
    }
  }
  HAL_UART_Receive_IT(&huart2, &rx2_data, 1);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
