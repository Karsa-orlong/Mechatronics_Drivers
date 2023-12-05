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

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS +1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
}USER_DATA;

void initUart3();
void setUart3BaudRate(uint32_t baudRate, uint32_t fcyc);
void putcUart3(char c);
void putsUart3(char* str);
char getcUart3();
bool kbhitUart3();
void getsUart3(USER_DATA *data);
void parseFields(USER_DATA *data);
char* getFieldString(USER_DATA *data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber);
bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments);
void clearData(USER_DATA *data);


#endif
