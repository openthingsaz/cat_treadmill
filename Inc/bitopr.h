/*
**   Converted to IAR version by kenny, 20160919
**   Imaging System Team, OsstemImplant Corporation.    
*/

#ifndef BITOPR_H
#define BITOPR_H

//#define BV(x,y,v)       (x|=(v<<y))
#define SetBit(x,y)     (x|=(1<<y)) 
#define ClrBit(x,y)     (x&=~(1<<y)) 
#define ToggleBit(x,y)  (x^=(1<<y)) 
#define FlipBit(x,y)    (x^=(1<<y)) // Same as ToggleBit.  
#define TestBit(x,y)    (x&(1<<y))
#define ChkBit(x,y)     (x&(1<<y))

#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define TRUE	1
#define FALSE	0

#define ON	1
#define OFF	0


#define HIGH	1
#define LOW	0

#define SET	1
#define CLEAR	0

#define OUTPUT	1
#define INPUT	0

#define MSB 0
#define LSB 1

#define ACK 0
#define NACK 1

#define OK 1
#define DONE 1

  /*! \brief Section: Return data declarations
  * Return data type redefine
  */
  typedef unsigned int IosReturnStatus_t;

#endif

