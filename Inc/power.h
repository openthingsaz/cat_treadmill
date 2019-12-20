/**
  ******************************************************************************
  * File Name          : power.h
  * Description        : This file provides code for the configuration
  *                      of the power instances.
  ******************************************************************************
 */



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __power_H
#define __power_H
#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

void power_en(void);
void power_dis(void);
void led_power_off(void);
void led_power_on(void);
extern uint8_t bat_val;
extern uint8_t power_mode;

#ifdef __cplusplus
}
#endif
#endif /*__ power_H */
