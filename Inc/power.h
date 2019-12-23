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

#define POWR_PERFORMANCE 0x01
#define POWR_SAVE 0x02

void power_en(void);
void power_dis(void);
void led_power_off(void);
void led_power_on(void);
void set_power_mode(uint8_t mode);
uint8_t get_power_mode(void);
void set_wakeup(void);
void set_sleep(void);

extern uint8_t bat_val;
extern uint8_t power_mode;

#ifdef __cplusplus
}
#endif
#endif /*__ power_H */
