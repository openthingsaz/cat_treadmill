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
#include <stdbool.h>

uint8_t running_mode = 0;
uint32_t run_time = 0;
bool led_rand_mode = false;

uint8_t get_status(void)
{
  return running_mode;
}

void set_wakeup(void) 
{
  power_en();
}

void set_sleep(void) 
{
  power_dis();
}

uint16_t get_degree(void)
{
  return (uint16_t)Roll;
}



uint8_t get_battery(void)
{
  return get_bat_val();
}



bool set_rand_led_mode(uint8_t mode) 
{
  if (mode == LED_RANDOM) {
    led_rand_mode = true;
  }
  else led_rand_mode = false;
  return led_rand_mode;
}

uint32_t get_run_time(void)
{
  return run_time;
}

void set_time_sync(uint8_t Hours, uint8_t Minutes, uint8_t Seconds, uint8_t WeekDay, uint8_t Month, uint8_t Date, uint8_t Year) 
{

}
