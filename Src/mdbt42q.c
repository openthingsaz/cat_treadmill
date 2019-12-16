/**
  ******************************************************************************
  * File Name          : MDBT42Q-AT.c
  * Description        : This file provides code for the configuration
  *                      of the MDBT42Q-AT instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mdbt42q.h"
#include "usart.h"

void ble_gpio_init(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // BLE_RESET High
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // BLE_FLASHED_DEFAULT Low
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // BLE_UART2_PD(POWER DOWN) Low
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // BLE_WAKEUP High

  // POWER Controler
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // PERI_3V3_PWR_nEN
}

void ble_module_init(void)
{
  int8_t data[20];
  int8_t* p;
  memset(data, 0, sizeof(data));
  
  /* dummy command, You need to pass this command once to get At command data. */
  HAL_UART_Transmit(&huart2, "AT?NAME\r\n", sizeof("AT?NAME\r"), 1000);
  HAL_Delay(50);

  HAL_UART_Transmit(&huart2, "AT?FLOWCONTROL\r", sizeof("AT?FLOWCONTROL\r"), 1000);
  HAL_Delay(50);
  HAL_UART_Receive(&huart2 , (uint8_t *)&data, sizeof(data) , 1000);

  p = strstr(data, "en");
  if (p == NULL) 
  {
    HAL_UART_Transmit(&huart2, "AT+FLOWCONTROLEN\r", sizeof("AT+FLOWCONTROLEN\r"), 1000);
    HAL_Delay(500);
    HAL_UART_Transmit(&huart2, "AT+RESET\r", sizeof("AT+RESET\r"), 1000);
    HAL_Delay(500);
  }
}
