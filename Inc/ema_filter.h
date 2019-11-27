#ifndef _EMA_FILTER_H
#define _EMA_FILTER_H

/**
Section: Included Files
*/
#include <math.h>
#include "bitopr.h"

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {
  
#endif
  
/*EMA ALPHAS*/
#define EMA_Alpha 0.76f ///< initialization of EMA alpha
  
  /**
  Section: Macro Declarations
  */
#define round( x )        ( ( x ) >= 0 ? (long)( ( x ) + 0.5 ) : (long)( ( x ) - 0.5 ) )
//#define EMA_Function( alpha, latest, stored )     (double)( round( (alpha) * (latest) ) )  + ( round( ( (1) - (alpha) ) * (stored)))
//#define EMA_Filter( sensorValue )  ((EMA_S) = ( EMA_Alpha * (max(1,sensorValue)) ) + ( ( 1 - EMA_Alpha ) * EMA_S ))     
  /**
  Section: Data Type Definitions
  */
    typedef struct IFILTER {
      float EMA_Gain;
      float EMA_S; ///< initialization of EMA Stored
      float EMA_EMA; ///< initialization of EMA_EMA
      float DEMA;
    }MovingFilter_t;
    
  /**
  Section: Global variables
  */
extern MovingFilter_t *Cal_Filter;

  /**
  @Summary
  Initialization routine.
  
  @Description
  
  @Preconditions
  None
  
  @Param
  None
  
  @Returns
  None
  
  @Comment 
  */
    //double round( double value );
    void EMA_FILTER_Reset( MovingFilter_t *pFilter );
    void EMA_FILTER_Init( float gain, MovingFilter_t *pFilter );
    float EMA_Filter( float sensorValue, MovingFilter_t *pFilter );
    double EMA_Function( float alpha, float latest, float stored);
    float DEMA_Filter( float sensorValue, MovingFilter_t *pFilter );
    
#ifdef __cplusplus  // Provide C++ Compatibility
  
}

#endif

#endif //_EMA_FILTER_H
