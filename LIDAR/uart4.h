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
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef UART3_H_
#define UART3_H_

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
