#include "pid.h"
#include "math.h"
#include "time.h"

PidData_t *pidData;

//! Parameters for regulator

/*! \brief Initialisation of PID controller parameters.
 *
 *  Initialise the variables used by the PID algorithm.
 *
 *  \param p_factor  Proportional term.
 *  \param i_factor  Integral term.
 *  \param d_factor  Derivate term.
 *  \param pid  Struct with PID status.
 */
void PID_Init(float p_factor, float i_factor, float d_factor, struct IPID *pid) {
// Set up PID controller parameters

  // Start values for PID controller
  pid->sumError = 0;
  pid->lastProcessValue = 0;
  
  // Tuning constants for PID loop
  pid->P_Factor = p_factor;
  pid->I_Factor = i_factor;
  pid->D_Factor = d_factor;
  
  // Limits to avoid overflow
  pid->maxError = MAX_INT / (pid->P_Factor + 1);
  pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);
}

/*! \brief PID control algorithm.
 *
 *  Calculates output from setpoint, process value and PID status.
 *
 *  \param setPoint  Desired value.
 *  \param processValue  Measured value.
 *  \param pid_st  PID status struct.
 */
#define INVERSE
double PID_Controller(short setPoint, short processValue, struct IPID *pid_st) {
  float error, p_term, d_term;
  double i_term, ret, temp;

//  __disable_interrupt();
  
  pid_st->curTime = time_ms();// Timer : 1mS@16Mhz
  pid_st->Dt = (fabsf(pid_st->curTime - pid_st->lastTime)) / 1000.0; //mS
#ifdef INVERSE
  error = -(setPoint - processValue);
#else
  error = (setPoint - processValue);
#endif
  // Calculate Pterm and limit error overflow
  if (error > pid_st->maxError){
    p_term = MAX_INT;
  }
  else if (error < -pid_st->maxError){
    p_term = -MAX_INT;
  }
  else{
    p_term = pid_st->P_Factor * error;
  }

  // Calculate Iterm and limit integral runaway
  temp = pid_st->sumError + error;
  if(temp > pid_st->maxSumError){
    i_term = MAX_I_TERM;
    pid_st->sumError = pid_st->maxSumError;
  }
  else if(temp < -pid_st->maxSumError){
    i_term = -MAX_I_TERM;
    pid_st->sumError = -pid_st->maxSumError;
  }
  else{
    pid_st->sumError = temp;
    i_term = pid_st->I_Factor * pid_st->sumError * pid_st->Dt;
  }

  // Calculate Dterm
  d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue) / pid_st->Dt;

  pid_st->lastProcessValue = processValue;

  ret = (p_term + i_term + d_term) / SCALING_FACTOR;
  
  if(ret > MAX_INT){
    ret = MAX_INT;
  }
  else if(ret < -MAX_INT){
    ret = -MAX_INT;
  }

  pid_st->lastTime = time_ms();
  
//  __enable_interrupt();
  
  return ret;
}

/*! \brief Resets the integrator.
 *
 *  Calling this function will reset the integrator in the PID regulator.
 */
void PID_RESET_Integrator( struct IPID *pid_st ) {
  pid_st->sumError = 0;
}

/**
  End of File
*/
