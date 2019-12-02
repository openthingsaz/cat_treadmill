/****************************************************************************
 **             - pid.h -
 **
 **     This file declares the internal register addresses for ATmega32.
 **
 **     Used with iccAVR and aAVR.
 **
 **     Copyright 2019 OSSTEMIMPLANT. All rights reserved.
 **
 **     File version: $Revision$
 **
 **     by Isaac, 2019024
 **
 **     Imaging System Team, OsstemImplant Corporation.
 ***************************************************************************/
#ifndef _PID_H
#define _PID_H
/*! \brief Section: Included Files
* header files
*/
#include <stdlib.h>
#include "main.h"
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {
  
#endif
  /*! \brief Section: Macro Declarations
  * SCALING
  */
#define SCALING_FACTOR  128 //65536
  
  /*! \brief Maximum values
  *
  * Needed to avoid sign/overflow problems
  */
  // Maximum value of variables
//#define INT16_MAX       0x7fff
//#define INT32_MAX       0x7fffffffL
#define MAX_INT         INT_FAST32_MAX
#define MAX_LONG        INT32_MAX
#define MAX_I_TERM      (MAX_LONG / 2)
  
  // Boolean values
#define FALSE           0
#define TRUE            1
  
  /*! \brief P, I and D parameter values
  *
  * The K_P, K_I and K_D values (P, I and D gains)
  * need to be modified to adapt to the application at hand
  */
  //! \xrefitem todo "Todo" "Todo list"
#define K_P     1.3f //SCALING_FACTOR * 1.3f
  //! \xrefitem todo "Todo" "Todo list"
#define K_I     0.5f //SCALING_FACTOR * 0.5f
  //! \xrefitem todo "Todo" "Todo list"
#define K_D     0.12f //SCALING_FACTOR * 0.12f

#define PID_EMA_Alpha 0.2f ///< initialization of EMA alpha //0.5f
  
  /*! \brief Section: Data Type Definitions
  *
  */
  
  /*! \brief PID Status
  *
  * Setpoints and data used by the PID control algorithm
  */
  typedef struct IPID {
    //! Last process value, used to find derivative of process value.
    float lastProcessValue; //short
    //! The Proportional tuning constant, multiplied with SCALING_FACTOR
    float P_Factor; //short
    //! The Integral tuning constant, multiplied with SCALING_FACTOR
    float I_Factor; //short
    //! The Derivative tuning constant, multiplied with SCALING_FACTOR
    float D_Factor; //short
    //! Summation of time, used for I/D calculations
    unsigned long curTime;
    //! current time
    unsigned long lastTime;
    //! last time
    double Dt; //int
    //! Maximum allowed error, avoid overflow
    float maxError; //short
    //! Summation of errors, used for integrate calculations
    double sumError; //int
    //! Maximum allowed sumerror, avoid overflow
    double maxSumError; //int
  }__attribute__((aligned(1), packed))PidData_t;
  
  /*! \brief Section: Global variables
  *
  */
  
  extern PidData_t *pidData;
  /**
  @Summary
  
  @Description
  
  @Preconditions
  None
  
  @Param
  None
  
  @Returns
  None
  
  @Comment
  */
  void PID_Init( float p_factor, float i_factor, float d_factor, struct IPID *pid );
  void PID_RESET_Integrator( struct IPID *pid_st );
  double PID_Controller( short setPoint, short processValue, struct IPID *pid_st );
  
#ifdef __cplusplus  // Provide C++ Compatibility
  
}

#endif

#endif //_PID_H
