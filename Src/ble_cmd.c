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

void set_wakeup(void) 
{
  power_en();
  HAL_TIM_Base_Start_IT(&htim11); // start the automatic On / Off timer.
  running_mode = STAT_RUNNING;
}

void set_sleep(void) 
{
  power_dis();
  HAL_TIM_Base_Stop_IT(&htim11); // start the automatic On / Off timer.
  running_mode = STAT_SLEEP;
}

void get_degree(void)
{
  uint8_t buf[10];
  uint16_t degree = (uint16_t)Roll;
  uint16_t crc = 0;

  memset(&buf, 0, sizeof(buf));
  buf[0] = STX;
  buf[1] = 0x01;
  buf[2] = GET_DEGREE;
  memcpy(&buf[3], &degree, sizeof(degree));
  crc = crc16_ccitt((void*)&buf[0], 8);
  buf[7] = (crc & 0xFF00) >> 8;
  buf[8] = (crc & 0x00FF);
  buf[9] = ETX;
  HAL_UART_Transmit(&huart1, buf, sizeof(buf), 100);
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
