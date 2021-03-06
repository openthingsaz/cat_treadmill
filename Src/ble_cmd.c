/**
  ******************************************************************************
  * File Name          : ble_cmd.c
  * Description        : This file provides code for the configuration
  *                      of the ble_cmd instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ble_cmd.h"
#include "ws2812b.h"
#include "power.h"
#include "tim.h"
#include "usart.h"
#include <stdbool.h>


uint32_t run_time = 0;
bool led_rand_mode = false;

uint8_t get_status(void)
{
  return running_mode;
}

uint16_t get_degree(void)
{
  uint16_t degree = (uint16_t)Roll;
  return degree;  
}

uint32_t get_run_time(void)
{
  return run_time;
}

void set_time_sync(uint8_t Hours, uint8_t Minutes, uint8_t Seconds, uint8_t WeekDay, uint8_t Month, uint8_t Date, uint8_t Year) 
{

}
