/**
  ******************************************************************************
  * File Name          : MDBT42Q.h
  * Description        : This file provides code for the configuration
  *                      of the MDBT42Q instances.
  ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __mdbt42q_H
#define __mdbt42q_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

void ble_module_init(void);
void ble_disable(void);
void ble_enable(void);
#ifdef __cplusplus
}
#endif
#endif /*__ mdbt42q_H */