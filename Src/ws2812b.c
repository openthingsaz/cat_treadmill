/**
  ******************************************************************************
  * File Name          : WS2812B.c
  * Description        : This file provides code for the configuration
  *                      of the WS2812B instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ws2812b.h"
#include "tim.h"
#include "power.h"
#include "ble_cmd.h"
#include "mpu6050_dmp.h"
#include "workout.h"

const uint8_t leddata[256*4] = { // size = 256 * 3
  0X44 , 0X44 , 0X44 , 0X44 , // 0
  0X44 , 0X44 , 0X44 , 0X47 , // 1
  0X44 , 0X44 , 0X44 , 0X74 ,
  0X44 , 0X44 , 0X44 , 0X77 ,
  0X44 , 0X44 , 0X47 , 0X44 ,
  0X44 , 0X44 , 0X47 , 0X47 ,
  0X44 , 0X44 , 0X47 , 0X74 ,
  0X44 , 0X44 , 0X47 , 0X77 ,
  0X44 , 0X44 , 0X74 , 0X44 ,
  0X44 , 0X44 , 0X74 , 0X47 ,
  0X44 , 0X44 , 0X74 , 0X74 ,
  0X44 , 0X44 , 0X74 , 0X77 ,
  0X44 , 0X44 , 0X77 , 0X44 ,
  0X44 , 0X44 , 0X77 , 0X47 ,
  0X44 , 0X44 , 0X77 , 0X74 ,
  0X44 , 0X44 , 0X77 , 0X77 ,
  0X44 , 0X47 , 0X44 , 0X44 ,
  0X44 , 0X47 , 0X44 , 0X47 ,
  0X44 , 0X47 , 0X44 , 0X74 ,
  0X44 , 0X47 , 0X44 , 0X77 ,
  0X44 , 0X47 , 0X47 , 0X44 ,
  0X44 , 0X47 , 0X47 , 0X47 ,
  0X44 , 0X47 , 0X47 , 0X74 ,
  0X44 , 0X47 , 0X47 , 0X77 ,
  0X44 , 0X47 , 0X74 , 0X44 ,
  0X44 , 0X47 , 0X74 , 0X47 ,
  0X44 , 0X47 , 0X74 , 0X74 ,
  0X44 , 0X47 , 0X74 , 0X77 ,
  0X44 , 0X47 , 0X77 , 0X44 ,
  0X44 , 0X47 , 0X77 , 0X47 ,
  0X44 , 0X47 , 0X77 , 0X74 ,
  0X44 , 0X47 , 0X77 , 0X77 ,
  0X44 , 0X74 , 0X44 , 0X44 ,
  0X44 , 0X74 , 0X44 , 0X47 ,
  0X44 , 0X74 , 0X44 , 0X74 ,
  0X44 , 0X74 , 0X44 , 0X77 ,
  0X44 , 0X74 , 0X47 , 0X44 ,
  0X44 , 0X74 , 0X47 , 0X47 ,
  0X44 , 0X74 , 0X47 , 0X74 ,
  0X44 , 0X74 , 0X47 , 0X77 ,
  0X44 , 0X74 , 0X74 , 0X44 ,
  0X44 , 0X74 , 0X74 , 0X47 ,
  0X44 , 0X74 , 0X74 , 0X74 ,
  0X44 , 0X74 , 0X74 , 0X77 ,
  0X44 , 0X74 , 0X77 , 0X44 ,
  0X44 , 0X74 , 0X77 , 0X47 ,
  0X44 , 0X74 , 0X77 , 0X74 ,
  0X44 , 0X74 , 0X77 , 0X77 ,
  0X44 , 0X77 , 0X44 , 0X44 ,
  0X44 , 0X77 , 0X44 , 0X47 ,
  0X44 , 0X77 , 0X44 , 0X74 ,
  0X44 , 0X77 , 0X44 , 0X77 ,
  0X44 , 0X77 , 0X47 , 0X44 ,
  0X44 , 0X77 , 0X47 , 0X47 ,
  0X44 , 0X77 , 0X47 , 0X74 ,
  0X44 , 0X77 , 0X47 , 0X77 ,
  0X44 , 0X77 , 0X74 , 0X44 ,
  0X44 , 0X77 , 0X74 , 0X47 ,
  0X44 , 0X77 , 0X74 , 0X74 ,
  0X44 , 0X77 , 0X74 , 0X77 ,
  0X44 , 0X77 , 0X77 , 0X44 ,
  0X44 , 0X77 , 0X77 , 0X47 ,
  0X44 , 0X77 , 0X77 , 0X74 ,
  0X44 , 0X77 , 0X77 , 0X77 ,
  0X47 , 0X44 , 0X44 , 0X44 ,
  0X47 , 0X44 , 0X44 , 0X47 ,
  0X47 , 0X44 , 0X44 , 0X74 ,
  0X47 , 0X44 , 0X44 , 0X77 ,
  0X47 , 0X44 , 0X47 , 0X44 ,
  0X47 , 0X44 , 0X47 , 0X47 ,
  0X47 , 0X44 , 0X47 , 0X74 ,
  0X47 , 0X44 , 0X47 , 0X77 ,
  0X47 , 0X44 , 0X74 , 0X44 ,
  0X47 , 0X44 , 0X74 , 0X47 ,
  0X47 , 0X44 , 0X74 , 0X74 ,
  0X47 , 0X44 , 0X74 , 0X77 ,
  0X47 , 0X44 , 0X77 , 0X44 ,
  0X47 , 0X44 , 0X77 , 0X47 ,
  0X47 , 0X44 , 0X77 , 0X74 ,
  0X47 , 0X44 , 0X77 , 0X77 ,
  0X47 , 0X47 , 0X44 , 0X44 ,
  0X47 , 0X47 , 0X44 , 0X47 ,
  0X47 , 0X47 , 0X44 , 0X74 ,
  0X47 , 0X47 , 0X44 , 0X77 ,
  0X47 , 0X47 , 0X47 , 0X44 ,
  0X47 , 0X47 , 0X47 , 0X47 ,
  0X47 , 0X47 , 0X47 , 0X74 ,
  0X47 , 0X47 , 0X47 , 0X77 ,
  0X47 , 0X47 , 0X74 , 0X44 ,
  0X47 , 0X47 , 0X74 , 0X47 ,
  0X47 , 0X47 , 0X74 , 0X74 ,
  0X47 , 0X47 , 0X74 , 0X77 ,
  0X47 , 0X47 , 0X77 , 0X44 ,
  0X47 , 0X47 , 0X77 , 0X47 ,
  0X47 , 0X47 , 0X77 , 0X74 ,
  0X47 , 0X47 , 0X77 , 0X77 ,
  0X47 , 0X74 , 0X44 , 0X44 ,
  0X47 , 0X74 , 0X44 , 0X47 ,
  0X47 , 0X74 , 0X44 , 0X74 ,
  0X47 , 0X74 , 0X44 , 0X77 ,
  0X47 , 0X74 , 0X47 , 0X44 ,
  0X47 , 0X74 , 0X47 , 0X47 ,
  0X47 , 0X74 , 0X47 , 0X74 ,
  0X47 , 0X74 , 0X47 , 0X77 ,
  0X47 , 0X74 , 0X74 , 0X44 ,
  0X47 , 0X74 , 0X74 , 0X47 ,
  0X47 , 0X74 , 0X74 , 0X74 ,
  0X47 , 0X74 , 0X74 , 0X77 ,
  0X47 , 0X74 , 0X77 , 0X44 ,
  0X47 , 0X74 , 0X77 , 0X47 ,
  0X47 , 0X74 , 0X77 , 0X74 ,
  0X47 , 0X74 , 0X77 , 0X77 ,
  0X47 , 0X77 , 0X44 , 0X44 ,
  0X47 , 0X77 , 0X44 , 0X47 ,
  0X47 , 0X77 , 0X44 , 0X74 ,
  0X47 , 0X77 , 0X44 , 0X77 ,
  0X47 , 0X77 , 0X47 , 0X44 ,
  0X47 , 0X77 , 0X47 , 0X47 ,
  0X47 , 0X77 , 0X47 , 0X74 ,
  0X47 , 0X77 , 0X47 , 0X77 ,
  0X47 , 0X77 , 0X74 , 0X44 ,
  0X47 , 0X77 , 0X74 , 0X47 ,
  0X47 , 0X77 , 0X74 , 0X74 ,
  0X47 , 0X77 , 0X74 , 0X77 ,
  0X47 , 0X77 , 0X77 , 0X44 ,
  0X47 , 0X77 , 0X77 , 0X47 ,
  0X47 , 0X77 , 0X77 , 0X74 ,
  0X47 , 0X77 , 0X77 , 0X77 ,
  0X74 , 0X44 , 0X44 , 0X44 ,
  0X74 , 0X44 , 0X44 , 0X47 ,
  0X74 , 0X44 , 0X44 , 0X74 ,
  0X74 , 0X44 , 0X44 , 0X77 ,
  0X74 , 0X44 , 0X47 , 0X44 ,
  0X74 , 0X44 , 0X47 , 0X47 ,
  0X74 , 0X44 , 0X47 , 0X74 ,
  0X74 , 0X44 , 0X47 , 0X77 ,
  0X74 , 0X44 , 0X74 , 0X44 ,
  0X74 , 0X44 , 0X74 , 0X47 ,
  0X74 , 0X44 , 0X74 , 0X74 ,
  0X74 , 0X44 , 0X74 , 0X77 ,
  0X74 , 0X44 , 0X77 , 0X44 ,
  0X74 , 0X44 , 0X77 , 0X47 ,
  0X74 , 0X44 , 0X77 , 0X74 ,
  0X74 , 0X44 , 0X77 , 0X77 ,
  0X74 , 0X47 , 0X44 , 0X44 ,
  0X74 , 0X47 , 0X44 , 0X47 ,
  0X74 , 0X47 , 0X44 , 0X74 ,
  0X74 , 0X47 , 0X44 , 0X77 ,
  0X74 , 0X47 , 0X47 , 0X44 ,
  0X74 , 0X47 , 0X47 , 0X47 ,
  0X74 , 0X47 , 0X47 , 0X74 ,
  0X74 , 0X47 , 0X47 , 0X77 ,
  0X74 , 0X47 , 0X74 , 0X44 ,
  0X74 , 0X47 , 0X74 , 0X47 ,
  0X74 , 0X47 , 0X74 , 0X74 ,
  0X74 , 0X47 , 0X74 , 0X77 ,
  0X74 , 0X47 , 0X77 , 0X44 ,
  0X74 , 0X47 , 0X77 , 0X47 ,
  0X74 , 0X47 , 0X77 , 0X74 ,
  0X74 , 0X47 , 0X77 , 0X77 ,
  0X74 , 0X74 , 0X44 , 0X44 ,
  0X74 , 0X74 , 0X44 , 0X47 ,
  0X74 , 0X74 , 0X44 , 0X74 ,
  0X74 , 0X74 , 0X44 , 0X77 ,
  0X74 , 0X74 , 0X47 , 0X44 ,
  0X74 , 0X74 , 0X47 , 0X47 ,
  0X74 , 0X74 , 0X47 , 0X74 ,
  0X74 , 0X74 , 0X47 , 0X77 ,
  0X74 , 0X74 , 0X74 , 0X44 ,
  0X74 , 0X74 , 0X74 , 0X47 ,
  0X74 , 0X74 , 0X74 , 0X74 ,
  0X74 , 0X74 , 0X74 , 0X77 ,
  0X74 , 0X74 , 0X77 , 0X44 ,
  0X74 , 0X74 , 0X77 , 0X47 ,
  0X74 , 0X74 , 0X77 , 0X74 ,
  0X74 , 0X74 , 0X77 , 0X77 ,
  0X74 , 0X77 , 0X44 , 0X44 ,
  0X74 , 0X77 , 0X44 , 0X47 ,
  0X74 , 0X77 , 0X44 , 0X74 ,
  0X74 , 0X77 , 0X44 , 0X77 ,
  0X74 , 0X77 , 0X47 , 0X44 ,
  0X74 , 0X77 , 0X47 , 0X47 ,
  0X74 , 0X77 , 0X47 , 0X74 ,
  0X74 , 0X77 , 0X47 , 0X77 ,
  0X74 , 0X77 , 0X74 , 0X44 ,
  0X74 , 0X77 , 0X74 , 0X47 ,
  0X74 , 0X77 , 0X74 , 0X74 ,
  0X74 , 0X77 , 0X74 , 0X77 ,
  0X74 , 0X77 , 0X77 , 0X44 ,
  0X74 , 0X77 , 0X77 , 0X47 ,
  0X74 , 0X77 , 0X77 , 0X74 ,
  0X74 , 0X77 , 0X77 , 0X77 ,
  0X77 , 0X44 , 0X44 , 0X44 ,
  0X77 , 0X44 , 0X44 , 0X47 ,
  0X77 , 0X44 , 0X44 , 0X74 ,
  0X77 , 0X44 , 0X44 , 0X77 ,
  0X77 , 0X44 , 0X47 , 0X44 ,
  0X77 , 0X44 , 0X47 , 0X47 ,
  0X77 , 0X44 , 0X47 , 0X74 ,
  0X77 , 0X44 , 0X47 , 0X77 ,
  0X77 , 0X44 , 0X74 , 0X44 ,
  0X77 , 0X44 , 0X74 , 0X47 ,
  0X77 , 0X44 , 0X74 , 0X74 ,
  0X77 , 0X44 , 0X74 , 0X77 ,
  0X77 , 0X44 , 0X77 , 0X44 ,
  0X77 , 0X44 , 0X77 , 0X47 ,
  0X77 , 0X44 , 0X77 , 0X74 ,
  0X77 , 0X44 , 0X77 , 0X77 ,
  0X77 , 0X47 , 0X44 , 0X44 ,
  0X77 , 0X47 , 0X44 , 0X47 ,
  0X77 , 0X47 , 0X44 , 0X74 ,
  0X77 , 0X47 , 0X44 , 0X77 ,
  0X77 , 0X47 , 0X47 , 0X44 ,
  0X77 , 0X47 , 0X47 , 0X47 ,
  0X77 , 0X47 , 0X47 , 0X74 ,
  0X77 , 0X47 , 0X47 , 0X77 ,
  0X77 , 0X47 , 0X74 , 0X44 ,
  0X77 , 0X47 , 0X74 , 0X47 ,
  0X77 , 0X47 , 0X74 , 0X74 ,
  0X77 , 0X47 , 0X74 , 0X77 ,
  0X77 , 0X47 , 0X77 , 0X44 ,
  0X77 , 0X47 , 0X77 , 0X47 ,
  0X77 , 0X47 , 0X77 , 0X74 ,
  0X77 , 0X47 , 0X77 , 0X77 ,
  0X77 , 0X74 , 0X44 , 0X44 ,
  0X77 , 0X74 , 0X44 , 0X47 ,
  0X77 , 0X74 , 0X44 , 0X74 ,
  0X77 , 0X74 , 0X44 , 0X77 ,
  0X77 , 0X74 , 0X47 , 0X44 ,
  0X77 , 0X74 , 0X47 , 0X47 ,
  0X77 , 0X74 , 0X47 , 0X74 ,
  0X77 , 0X74 , 0X47 , 0X77 ,
  0X77 , 0X74 , 0X74 , 0X44 ,
  0X77 , 0X74 , 0X74 , 0X47 ,
  0X77 , 0X74 , 0X74 , 0X74 ,
  0X77 , 0X74 , 0X74 , 0X77 ,
  0X77 , 0X74 , 0X77 , 0X44 ,
  0X77 , 0X74 , 0X77 , 0X47 ,
  0X77 , 0X74 , 0X77 , 0X74 ,
  0X77 , 0X74 , 0X77 , 0X77 ,
  0X77 , 0X77 , 0X44 , 0X44 ,
  0X77 , 0X77 , 0X44 , 0X47 ,
  0X77 , 0X77 , 0X44 , 0X74 ,
  0X77 , 0X77 , 0X44 , 0X77 ,
  0X77 , 0X77 , 0X47 , 0X44 ,
  0X77 , 0X77 , 0X47 , 0X47 ,
  0X77 , 0X77 , 0X47 , 0X74 ,
  0X77 , 0X77 , 0X47 , 0X77 ,
  0X77 , 0X77 , 0X74 , 0X44 ,
  0X77 , 0X77 , 0X74 , 0X47 ,
  0X77 , 0X77 , 0X74 , 0X74 ,
  0X77 , 0X77 , 0X74 , 0X77 ,
  0X77 , 0X77 , 0X77 , 0X44 ,
  0X77 , 0X77 , 0X77 , 0X47 ,
  0X77 , 0X77 , 0X77 , 0X74 ,
  0X77 , 0X77 , 0X77 , 0X77 ,
};

extern SPI_HandleTypeDef hspi1;

uint8_t ledPos_before = 0;
uint8_t red = 0;
uint8_t green = 50;
uint8_t blue = 0;
uint8_t rand_led_mode = 0;
uint8_t ws_buffer[LED_BUFFER_LENGTH];

void encode_byte( uint8_t data, int16_t buffer_index )
{
  int index = data * 4;
  ws_buffer[buffer_index++ ] = leddata[index++];
  ws_buffer[buffer_index++ ] = leddata[index++];
  ws_buffer[buffer_index++ ] = leddata[index++];
  ws_buffer[buffer_index++ ] = leddata[index++];
}
void generate_ws_buffer( uint8_t RData,uint8_t GData,uint8_t BData, int16_t led_no )
{
  //memset(ws_buffer, 0, sizeof(ws_buffer));
  //ws2812b
  //G--R--B
  //MSB first
  int offset = led_no * 12;
  encode_byte( GData, offset );
  encode_byte( RData, offset+4 );
  encode_byte( BData, offset+8 );
}
void Send_2812(void)
 {
    HAL_SPI_Transmit_DMA( &hspi1, ws_buffer, LED_BUFFER_LENGTH );
    // wait until finished
    while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY ));
 }

void setAllPixelColor(uint8_t r, uint8_t g, uint8_t b)
{
  int i;
  for(i=0;i< LED_NO;i++) {
    generate_ws_buffer( r, g, b, i );
  }
  Send_2812();
  HAL_Delay(1);
}
void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
{
  generate_ws_buffer( r, g, b, n );
  Send_2812();
  HAL_Delay(1);
}
/**
 * initialize MOSI pin to LOW.  Without this, first time transmit for first LED might be wrong.
 *
 */
void initLEDMOSI(void)
{
  uint8_t buffer0[2] = { 0, 0 };
  HAL_SPI_Transmit(&hspi1, buffer0, 1, 100 );
}

void random_led(void)
{
  red = rand() % 256;
  green = rand() % 256;
  blue = rand() % 256;
}

void set_led_update(uint8_t pos)
{
  if (ledPos_before != pos) 
  {
      if (rand_led_mode == 1)
        random_led();

      setAllPixelColor(0, 0, 0);
      setPixelColor( (uint16_t)pos, red, green, blue);
    
      ledPos_before = pos;
      //printf("\r Roll %d, acul : %lu\n", (uint16_t)Roll, exData->get_acumulatedDegree());
      // if If there is motion, wakeup
      if (get_running_mode() == STAT_SLEEP) 
      {
        set_wakeup();
      }
      
    }
}

uint8_t pos_move_check(uint8_t pos)
{
  if (ledPos_before != pos) 
  {  
    ledPos_before = pos;   
    return 1; // led move
  }
  else return 0; // led don't move
}


void set_led_pos(uint8_t pos) 
{
  ledPosUser = pos;
}

void set_led_col(uint32_t data) 
{
  uint8_t r;
  uint8_t g;
  uint8_t b;

  r = (data >> 16) & 0xff;
  g = (data >> 8) & 0xff;
  b = (data >> 0) & 0xff;
  red = r;
  green = g;
  blue = b;

  if (data == 0) {
    led_power_off();
  }
  else {
    led_power_on();  
    HAL_Delay(1);   
  }
  setPixelColor(ledPos, red, green, blue);
}

void set_rand_led_mode(void)
{
  rand_led_mode = 1;
}

void dis_rand_led_mode(void)
{
  rand_led_mode = 0;
}

void test_led_rgb(void) {
  int8_t i;


  // red
  for ( i = 0; i < LED_NO; i++) {
    //setPixelColor( i, 250, 0, 0 );
    setAllPixelColor(0, 0, 0);
    HAL_Delay(400);

    setPixelColor( i, 0, 50, 0 );
    //printf("i : %d\r\n", i);
    HAL_Delay(400);
  }
  /*
  // green
  for ( i = 0; i < LED_NO; i++) {
    //setPixelColor( i, 0, 250, 0 );
    setPixelColor( i, 0, 5, 0 );
    HAL_Delay(200);
  }

  // blue
  for ( i = 0; i < LED_NO; i++) {
    //setPixelColor( i, 0, 250, 0 );
    setPixelColor( i, 0, 0, 5 );
    HAL_Delay(200);
  }
  */
}

/* USER CODE END 0 */
