#include "vt100.h"

// Program ROM constants

// Global variables

// Functions
void vt100Init(void)
{
	// initializes terminal to "power-on" settings
	// ESC c
	printf("\x1B\x63");
}

void vt100ClearScreen(void)
{
	// ESC [ 2 J
	printf("\x1B[2J");
}

void vt100ClearLinetoEnd(void)
{
	// ESC [ 2 J
	printf("\x1B[J");
}

void vt100SetAttr(unsigned char attr)
{
	// ESC [ Ps m
	printf("\x1B[%dm",attr);
}

void vt100SetCursorMode(unsigned char visible)
{
	if(visible)
		// ESC [ ? 25 h
		printf("\x1B[?25h");
	else
		// ESC [ ? 25 l
		printf("\x1B[?25l");
}

void vt100SetCursorPos( unsigned char line, unsigned char col )
{
  //__disable_interrupt();
  printf("\x1B[%d;%dH",line,col);
  //__enable_interrupt();
}

void vt100SetCursorReDefine(unsigned char col)
{
  // ESC [ Pl D
  printf("\x1B[%dD",col);
}
