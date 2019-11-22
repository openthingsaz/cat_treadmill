#ifndef VT100_H
#define VT100_H
/**
Section: Included Files
*/
#include <stdio.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include "intrinsics.h"

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {
  
#endif
  
// constants/macros/typdefs
// text attributes
#define VT100_ATTR_OFF		0
#define VT100_BOLD		1
#define VT100_USCORE		4
#define VT100_BLINK		5
#define VT100_REVERSE		7
#define VT100_BOLD_OFF		21
#define VT100_USCORE_OFF	24
#define VT100_BLINK_OFF		25
#define VT100_REVERSE_OFF	27
  
  /**
  Section: Macro Declarations
  */
//#define vt100Init                       printf("\x1B\x63")
//#define vt100ClearScreen                SerialPutString("\x1B[2J")//printf("\x1B[2J")
//#define vt100SetAttr(attr)              printf("\x1B[%dm", attr)
//#define vt100SetCursorMode(visible)     ((visible) > (0) ) ? ( usart_printf("\x1B[?25h") ) : ( usart_printf("\x1B[?25l") )
//#define vt100SetCursorPos(line,col)     printf("\x1B[%d;%dH",line,col) //{usart_printf"\x1B["); putc(line); putc(';'); putc(col); putc('H');} //
//#define vt100SetCursorReDefine(col)     printf("\x1B[%dD",col) //usart_printf("\x1B["); putc(col); putc('D'); //
    /**
  Section: Data Type Definitions
  */
  
  /**
  Section: Global variables
  */
  
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
// functions

//! vt100Init() initializes terminal and vt100 library
///		Run this init routine once before using any other vt100 function.
void vt100Init(void);

//! vt100ClearScreen() clears the terminal screen
void vt100ClearScreen(void);

//! vt100SetAttr() sets the text attributes like BOLD or REVERSE
///		Text written to the terminal after this function is called will have
///		the desired attribuutes.
void vt100SetAttr(unsigned char attr);

//! vt100SetCursorMode() sets the cursor to visible or invisible
void vt100SetCursorMode(unsigned char visible);

//! vt100SetCursorPos() sets the cursor position
///		All text which is written to the terminal after a SetCursorPos command
///		will begin at the new location of the cursor.
void vt100SetCursorPos( unsigned char line, unsigned char col );

void vt100SetCursorReDefine(unsigned char col);

void vt100ClearLinetoEnd(void);
#ifdef __cplusplus  // Provide C++ Compatibility
  
}

#endif

#endif //VT100_H
