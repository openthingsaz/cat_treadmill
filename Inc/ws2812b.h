/**
  ******************************************************************************
  * File Name          : WS2812B.h
  * Description        : This file provides code for the configuration
  *                      of the WS2812B instances.
  ******************************************************************************
 */



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ws2812b_H
#define __ws2812b_H
#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "main.h"



#define LED_NO    216

#define LED_BUFFER_LENGTH (LED_NO*12)

void SystemClock_Config(void);
void encode_byte( uint8_t data, int16_t buffer_index );
void generate_ws_buffer( uint8_t RData,uint8_t GData,uint8_t BData, int16_t led_no );
void Send_2812(void);
void setAllPixelColor(uint8_t r, uint8_t g, uint8_t b);
void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
void initLEDMOSI(void);
void test_led_rgb(void);




#ifdef __cplusplus
}
#endif
#endif /*__ ws2812b_H */
