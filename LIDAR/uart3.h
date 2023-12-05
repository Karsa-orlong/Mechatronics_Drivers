// UART0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U3TX (PC7) and U3RX (PC6) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "EdTools.h"

#ifndef UART0_H_
#define UART0_H_

#define MAX_CHARS 80
#define MAX_FIELDS 6
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


void initUart3();
void setUart3BaudRate(uint32_t baudRate, uint32_t fcyc);
void putcUart3(char c);
void putsUart3(char* str);
char getcUart3();
bool kbhitUart3();
char *itoa(uint32_t number, char *string);


#endif
