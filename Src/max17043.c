/**
  ******************************************************************************
  * File Name          : MAX17043.c
  * Description        : This file provides code for the configuration
  *                      of the MAX17043-AT instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "max17043.h"
#include "i2c.h"
#include "usart.h"
typedef unsigned char byte;
int32_t MAX17043_ADDRESS = 0x36 << 1;

uint8_t rx_tx_buf[2];
int16_t regValue = 0;
int32_t readFuntion = 0 ;
float decimal ;

// Function reading register Values
int32_t MAX17043_readRegister (int32_t addrL, int32_t addrH) {
  uint8_t LSB;
  HAL_I2C_Mem_Read(&hi2c1, MAX17043_ADDRESS, addrL, I2C_MEMADD_SIZE_8BIT, &LSB, 1, 100);
  //printf("lower byte is:%02x\n\r ",LSB);    //Testing purposes

  uint8_t MSB;
  HAL_I2C_Mem_Read(&hi2c1, MAX17043_ADDRESS, addrH, I2C_MEMADD_SIZE_8BIT, &MSB, 1, 100);
  //printf("Upper byte is:%02x\n\r ",MSB);     //Testing purposes

  switch (readFuntion) {
  case 1 : // get Vcell
    regValue = (MSB << 4) | (LSB >> 4);
    break;
  case 2 :// get SoC
    //decimal = LSB / 256.0;
    regValue = MSB;
    break;

  case 3 :// get battery Version
    regValue = (MSB << 8) | LSB;
    break;

  case 4 :    // get AlertThreshold
    regValue = 32 - (LSB & 0x1F);
    break;
  case 5 :    // reading the last five bits of the config register .i.e. the Alert threshold in 2s complement form
    regValue = (MSB << 8) | LSB;
    //printf("RegValue is:%04x\n\r ",regValue);
    break;

  case 6 :    // reading the last five bits of the config register .i.e. the Alert threshold in 2s complement form
    regValue = LSB & 0x20;
    //printf("RegValue is:%02x\n\r ",regValue);
    break;
  }
  //return (int16_t)((MSB<<8)+LSB);
  return regValue;
}

// Function writing values to register
int32_t MAX17043_writeRegister(int32_t addr, int16_t data ){
  HAL_I2C_Mem_Write(&hi2c1, MAX17043_ADDRESS, addr, I2C_MEMADD_SIZE_16BIT, &data, sizeof(data), 100);
  return 1;
}

void MAX17043_reset()
{
  MAX17043_writeRegister(COMMAND_REGISTER, 0x5400);
}

void MAX17043_quickStart()
{
  MAX17043_writeRegister(MODE_REGISTER, 0x0040);
}

uint32_t MAX17043_getVCell()
{
  readFuntion = 1 ;
  uint32_t vCell = MAX17043_readRegister(VCELL_REGISTER_LSB, VCELL_REGISTER_MSB);
  return vCell * 1.25; //1.25 : mili voltage level, 0.00125 : voltage level
}

int32_t MAX17043_getSoC()
{
  readFuntion =2 ;
  return MAX17043_readRegister(SOC_REGISTER_LSB, SOC_REGISTER_MSB);
}

int32_t MAX17043_getVersion()
{
  readFuntion =3 ;
  int32_t MAX17043Version = MAX17043_readRegister(VERSION_REGISTER_LSB, VERSION_REGISTER_MSB);
  return MAX17043Version ;
}

int32_t MAX17043_getAlertThreshold()
{
  readFuntion =4 ;
  int32_t alertThreshold = MAX17043_readRegister(CONFIG_REGISTER_LSB, CONFIG_REGISTER_MSB);
  return alertThreshold ;
}

void MAX17043_setAlertThreshold (int32_t alertThresholdValue)
{
  int32_t threshold = alertThresholdValue ;
  if (threshold > 32){
    threshold = 32;
  }
  int32_t threshold_2scomplement = 32 - threshold ; // Alert threshold is in 2s complement .i.e 11111 = 0%
  readFuntion =5 ;
  //printf("Threshold:%d %\n\r ",threshold);
  //printf("threshold_2scomplement:%d %\n\r ",threshold_2scomplement);

  int32_t ConfigReg = MAX17043_readRegister(CONFIG_REGISTER_LSB, CONFIG_REGISTER_MSB);
  //printf("configReg:%02x \n\r ",ConfigReg);
  /**
      Alert Threshold is the last five bits of the config register. So here, the LSB is read first and the required alert threshold
  is inserted into the last five bits.
  **/

  int32_t writeData = (ConfigReg & 0xFFE0) | threshold_2scomplement ;
  int32_t writeData_swapped = (writeData  >>8) | (writeData <<8) ;

  //printf("writeData:%02x %\n\r ",writeData);
  //printf("writeData_swapped:%02x \n\r ",writeData_swapped);
  MAX17043_writeRegister(CONFIG_REGISTER_MSB, writeData_swapped);
  usleep(1000);
}

int32_t MAX17043_getAlertStatus(void)
{
  readFuntion =6 ;
  int32_t alertStatus = MAX17043_readRegister(CONFIG_REGISTER_LSB,CONFIG_REGISTER_MSB);
  return alertStatus ;
}

void max17043_init(void) 
{

  MAX17043_reset ();
  //printf("Battery Status:\t");
  HAL_Delay(1);

  MAX17043_quickStart ();

  uint32_t cellVoltage = MAX17043_getVCell ();
  printf("V: %1.5f\r\n",cellVoltage);

  printf("C: %2d\r\n", MAX17043_getSoC());    



  int MAX17043Version = MAX17043_getVersion ();
  printf("MAX17043 v.%d\r\n",MAX17043Version);

  int alertThreshold = MAX17043_getAlertThreshold ();
  printf("T: %04x\r\n",alertThreshold);

  //MAX17043_setAlertThreshold (20) ;

  //int alertThreshold_after = MAX17043_getAlertThreshold ();
  //printf("New Set Threshold:%d %\n\r ",alertThreshold_after);

  int alertStatus = MAX17043_getAlertStatus ();
  if (alertStatus == 0x20) {
    printf("1\n\r");
  }
  else {
    printf("0\n\r");
  }
}
//void get_bat_volt(void)

int32_t get_bat_val(void) 
{
  MAX17043_reset ();
  HAL_Delay(1);
  MAX17043_quickStart ();
  return MAX17043_getSoC();
}
