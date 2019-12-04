/**
  ******************************************************************************
  * File Name          : power.c
  * Description        : This file provides code for the configuration
  *                      of the power instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "power.h"
#include <stdbool.h>

uint8_t bat_val = 0;

/* power Enable */
void power_en(void)
{
  // power Controler
  HAL_GPIO_WritePin(PERI_3V3_PWR_nEN_GPIO_Port, PERI_3V3_PWR_nEN_Pin, GPIO_PIN_RESET); // PERI_3V3_PWR_nEN, Gyroscope, Accelerometer, High Disable, Low Enable, Device : U2
  HAL_GPIO_WritePin(LED_LMIT_EN_GPIO_Port, LED_LMIT_EN_Pin, GPIO_PIN_SET); // LED_LMIT_EN, High Enable, Low Disable, Device : U5
  HAL_GPIO_WritePin(LED_3V3_PWR_nEN_GPIO_Port, LED_3V3_PWR_nEN_Pin, GPIO_PIN_SET); // LED_3V3_PWR_nEN, High Enable, Low Disable, Device : U3
}

/* power Disable */
void power_dis(void)
{
  // power Controler
  // HAL_GPIO_WritePin(PERI_3V3_PWR_nEN_GPIO_Port, PERI_3V3_PWR_nEN_Pin, GPIO_PIN_SET); // PERI_3V3_PWR_nEN, Gyroscope, Bluetooth, High Disable, Low Enable
  HAL_GPIO_WritePin(LED_LMIT_EN_GPIO_Port, LED_LMIT_EN_Pin, GPIO_PIN_RESET); // LED_LMIT_EN, High Enable, Low Disable
  HAL_GPIO_WritePin(LED_3V3_PWR_nEN_GPIO_Port, LED_3V3_PWR_nEN_Pin, GPIO_PIN_RESET); // LED_3V3_PWR_nEN, High Enable, Low Disable
}

uint8_t get_bat_val(void) 
{
  return bat_val;
}


