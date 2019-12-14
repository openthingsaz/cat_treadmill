/**
  ******************************************************************************
  * File Name          : MDBT42Q-AT.c
  * Description        : This file provides code for the configuration
  *                      of the MDBT42Q-AT instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mdbt42q.h"

void ble_gpio_init(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // BLE_RESET High
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // BLE_FLASHED_DEFAULT Low
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // BLE_UART2_PD(POWER DOWN) Low
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // BLE_WAKEUP High

  // POWER Controler
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // PERI_3V3_PWR_nEN
}

void at_cmd_init(void)
{
  //HAL_UART_Transmit(&huart2, "AT+FLOWCONTROLEN\r", sizeof("AT+FLOWCONTROLEN\r"), 1000);
  //HAL_Delay(1000);
  //HAL_UART_Transmit(&huart2, "AT+RESET\r", sizeof("AT+RESET\r"), 1000);
  //HAL_Delay(1000);
  //HAL_UART_Transmit(&huart2, "AT?FLOWCONTROL\r", sizeof("AT?FLOWCONTROL\r"), 1000);
  //HAL_Delay(1000);

  //HAL_UART_Receive(&huart2 , (uint8_t *)&dummy_mqtt, sizeof(dummy_mqtt) , 1000);
  //printf("11 %s\r\n", dummy_mqtt);
  //printf("2222222222222222222\r\n", dummy_mqtt);
}