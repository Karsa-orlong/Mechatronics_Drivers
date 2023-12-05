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
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R2;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
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



