// UART1 Library
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

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart4.h"
#include "gpio.h"

// Pins
#define UART3_TX PORTC,7
#define UART3_RX PORTC,6

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart1(void)
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    _delay_cycles(3);
    enablePort(PORTC);

    // Configure UART0 pins
    selectPinPushPullOutput(UART1_TX);
    selectPinDigitalInput(UART1_RX);
    setPinAuxFunction(UART1_TX, GPIO_PCTL_PC5_U1TX);
    setPinAuxFunction(UART1_RX, GPIO_PCTL_PC4_U1RX);

    // Configure UART1 with default baud rate
    UART1_CTL_R = 0;                                    // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (usually 40 MHz)
}

// Set baud rate as function of instruction cycle frequency
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART1_CTL_R = 0;                                    // turn-off UART1 to allow safe programming
    UART1_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART1_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART1
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart1(char c)
{
    while (UART1_FR_R & UART_FR_TXFF);               // wait if UART1 tx fifo full
    UART1_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart1(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart1(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart1(void)
{
    // while (UART1_FR_R & UART_FR_RXFE);               // wait if UART1 rx fifo empty
    return UART1_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart1(void)
{
    return !(UART1_FR_R & UART_FR_RXFE);
}

/**
 *      @brief Function to convert a given number to string
 *      @param string Character pointer to hold converted integer
 *      @param number to covert to string
 *      @return char* pointer for recursive calls
 **/
static char *itoa_helper(char *string, int32_t number)
{
    if (number <= -10)
        string = itoa_helper(string, number / 10);

    *string++ = '0' - number % 10;
    return string;
}

/**
 *      @brief Function to convert integer to string
 *      @param string destination to store converted string
 *      @param number to convert
 *      @return char*
 **/
char *itoa(uint32_t number, char *string)
{
    char *s = string;
    if (number < 0)
        *s++ = '-';
    else
        number = -number; // Append negative sign to start of number

    *itoa_helper(s, number) = '\0'; // Append NULL character to indicate end of string
    return string;
}
