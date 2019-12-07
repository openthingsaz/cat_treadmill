/**
  ******************************************************************************
  * File Name          : MAX17043.h
  * Description        : This file provides code for the configuration
  *                      of the MAX17043 instances.
  ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAX17043_H
#define __MAX17043_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define VCELL_REGISTER_MSB		0x02
#define VCELL_REGISTER_LSB		0x03
#define SOC_REGISTER_MSB		0x04
#define SOC_REGISTER_LSB		0x05
#define VERSION_REGISTER_MSB	0x08
#define VERSION_REGISTER_LSB	0x09
#define CONFIG_REGISTER_MSB		0x0C// The power up default value for config register is 97H
#define CONFIG_REGISTER_LSB		0x0D
#define MODE_REGISTER		0x06
#define COMMAND_REGISTER	0xFE

void max17043_init(void);

#ifdef __cplusplus
}
#endif
#endif /*__ MAX17043_H */