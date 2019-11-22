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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // BLE Reset High
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // flash default Low
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // POWER DOWN Low
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // WAKEUP

  // POWER Controler
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // PERI_3V3_PWR_nEN
}
