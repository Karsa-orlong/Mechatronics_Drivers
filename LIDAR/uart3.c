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

#include "uart0.h"


// PortA masks
#define UART_TX_MASK 128
#define UART_RX_MASK 64

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART3
void initUart3()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R3;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);

    // Configure UART3 pins
    GPIO_PORTC_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTC_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART3 pins
    GPIO_PORTC_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC7_M | GPIO_PCTL_PC6_M); // clear bits 0-7
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC7_U3TX | GPIO_PCTL_PC6_U3RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity
    // Configure UART3 to 115200 baud, 8N1 format
    UART3_CTL_R = 0;                                    // turn-off UART3 to allow safe programming
    UART3_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART3_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART3_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART3_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART3_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart3BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART3_CTL_R = 0;                                    // turn-off UART3 to allow safe programming
    UART3_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART3_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART3_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART3_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART3
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart3(char c)
{
    while (UART3_FR_R & UART_FR_TXFF);               // wait if uart3 tx fifo full
    UART3_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart3(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart3()
{
    while (UART3_FR_R & UART_FR_RXFE);               // wait if uart3 rx fifo empty
    return UART3_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart3()
{
    return !(UART3_FR_R & UART_FR_RXFE);
}

void getsUart3(USER_DATA *data)
{
    char c;
    uint8_t count = 0;

    while(1)
    {
        c = getcUart3();
        if ( ((c == 8) || (c == 127)) && (count > 0) )
        {
            count--;
        }
        else if ( (c == 10) || (c == 13) )
        {
            data->buffer[count] = '\0';
            // NOTE: Here we could go out of our way to
            // set the rest of the data in this buffer
            // to null terminators.
            return;
        }
        else if(c >= 32 )
        {
            if (count == MAX_CHARS)
            {
                data->buffer[count] = '\0';
                return;
            }
            data->buffer[count] = c;
            count++;
        }
    }
}

void parseFields(USER_DATA *data)
{
    // Context:
    // n = numbers =                48-57   ~ [0-9]
    // a = Capital-case Letters =   65-90   ~ [A-Z]
    // a = Lower-case Letters =     97-122  ~ [a-z]
    // s = Special Characters =     45-46   ~ [- and .]
    // d = Delimiter Characters =   else    ~ [else]
    int i = 0;
    data->fieldCount = 0;
    char type ='\0';

    while(data->buffer[i] != '\0')
    {
        type = getCharacterType(data->buffer[i]);
        if(type != 'd')
        {
            data->fieldPosition[data->fieldCount] = i;
            data->fieldType[data->fieldCount] = type;
            data->fieldCount++;
            while(type != 'd')
            {
                i++;
                type = getCharacterType(data->buffer[i]);
            }
        }
        data->buffer[i] = '\0';
        i++;
    }
}
char* getFieldString(USER_DATA *data, uint8_t fieldNumber)
{
    char tempString[MAX_CHARS] = "\0";
    int i,j;
    for(i = data->fieldPosition[fieldNumber], j = 0; data->buffer[i] != '\0' ;i++,j++)
    {
        tempString[j] = data->buffer[i];
    }
    tempString[j+1] = data->buffer[i];
    return tempString;
}
int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber)
{
    int32_t returnValue = strToInt(getFieldString(data, fieldNumber));
    return returnValue;
}

bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    char command[MAX_CHARS] = getFieldString(data, 0);
    bool result = true;
    uint8_t i;

    for( i = 0 ; data->buffer[i] != '\0' ; i++)
    {
        if (data->buffer[i] != strCommand[i] )
        {
            result = false;
        }
    }
    if (minArguments != (data->fieldCount -1 )  )
    {
        result = false;
    }
    return result;
}

void clearData(USER_DATA *data)
{
    int i;
    // CLEAR Buffer array
    for (i=0; i < MAX_CHARS +1; i++)
    {
        data->buffer[i] = '\0';
    }
    // CLEAR fieldType array
    for (i=0; i < MAX_FIELDS; i++)
    {
        data->fieldType[i] = '\0';
    }
    // CLEAR fieldPosition array
    for (i=0; i < MAX_FIELDS; i++)
    {
        data->fieldPosition[i] = 0;
    }
    // CLEAR fieldCount
    data->fieldCount = 0;
}


