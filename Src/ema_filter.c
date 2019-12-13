#include "ema_filter.h"

MovingFilter_t *Cal_Filter;

/*! \brief Initialisation of Double moving filter parameters.
 *
 *  Initialise the variables used by the PID algorithm.
 *
 *  \param gain     Gain(Alpha) term.
 *  \param pFilter  FILTER status struct.
 */
void EMA_FILTER_Init( float gain, MovingFilter_t *pFilter ) {
  pFilter->EMA_S = 0.0f;
  pFilter->EMA_EMA = 0.0f;
  pFilter->DEMA = 0.0f;
  
  pFilter->EMA_Gain = gain;
}

/*! \brief Resets the integrator.
 *
 *  Calling this function will reset the integrator in the filter regulator.
 */
void EMA_FILTER_Reset( MovingFilter_t *pFilter ) {
  pFilter->EMA_S = 0.0f;
  pFilter->EMA_EMA = 0.0f;
  pFilter->DEMA = 0.0f;
}

/*! \brief Moving filter algorithm.
 *
 *  Calculates output from setpoint, process value and PID status.
 *
 *  \param sensorValue  current data value.
 *  \param pFilter      FILTER status struct.
 */
float EMA_Filter( float sensorValue, MovingFilter_t *pFilter ) {
  
 /* Avoid dividing by zero */
  sensorValue = max(1,sensorValue);
  
   /*RUN SENSOR EMA*/
  return pFilter->EMA_S = ( EMA_Alpha * sensorValue ) + ( ( 1 - EMA_Alpha ) * pFilter->EMA_S );    //run the EMA
}

/*! \brief Moving filter algorithm.
 *
 *  Calculates output from setpoint, process value and PID status.
 *
 *  \param alpha       gain.
 *  \param latest      past data value.
 *  \param stored      accumulated data value.
 */
double EMA_Function( float alpha, float latest, float stored) {
  return roundf( alpha * latest )  + roundf( ( 1 - alpha ) * stored);
}

/*! \brief Moving filter algorithm.
 *
 *  Calculates output from setpoint, process value and PID status.
 *
 *  \param sensorValue  current data value.
 *  \param pFilter      FILTER status struct.
 */
float DEMA_Filter( float sensorValue, MovingFilter_t *pFilter ) {
//  __disable_interrupt();
  /* Avoid dividing by zero */
  //sensorValue = max(1,sensorValue);
 
  
  pFilter->EMA_EMA = EMA_Function(EMA_Alpha, EMA_Filter( sensorValue, pFilter ), pFilter->EMA_EMA);
  
  pFilter->DEMA = 2 * pFilter->EMA_S - pFilter->EMA_EMA;
  
//  __enable_interrupt();
  
  return pFilter->DEMA;
  
}
/**
  End of File
*/

//double round( double value ) {
//  double temp;
//  
//  if (value >= 0.0) {
//    temp = floor(value);
//    
//    if ( temp - value <= -0.5 )
//      temp += 1.0;
//    
//    return ( temp );
//  } 
//  else {
//    temp = floor(-value);
//    
//    if ( temp + value <= -0.5 )
//      temp += 1.0;
//    
//    return ( -temp );
//  }
//}
