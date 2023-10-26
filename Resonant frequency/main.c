

/**
 * main.c
 */

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "nvic.h"

// PortC masks
#define FREQ_IN_MASK 64 // PC6 WT1CCP0

#define GREEN_LED   PORTF,3
#define BLUE_LED    PORTF,2
#define RED_LED     PORTF,1


//Colpitts oscillator
uint32_t frequency =0;
uint32_t last_frequency = 0;
int freq_error = 0;
uint32_t threshold =0;              //TODO To be observed and filled

void enableCounterMode()
{
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 40000000;                       // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)

    // Configure Wide Timer 1 as counter of external events on CCP0 pin
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_POS;           // count positive edges
    WTIMER1_IMR_R = 0;                               // turn-off interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
}

// Frequency counter service publishing latest frequency measurements every second
void timer1Isr()
{

    last_frequency = frequency;                     // Save the last onserved frequency and print out the change in frequency

    frequency = WTIMER1_TAV_R;                   // read counter input
    WTIMER1_TAV_R = 0;                           // reset counter for next period

    freq_error = frequency - last_frequency;        // Calculate error and print it

    togglePinValue(BLUE_LED);

    if(freq_error > threshold){
        putsUart0("Metal plate detected\n");
    }

    char str[200];
    snprintf(str, sizeof(str), "\nf: %d\n", frequency);
    putsUart0(str);

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;           // clear interrupt flag
}

// Initialize Hardware
void init_texas()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    enablePort(PORTC);
    enablePort(PORTF);

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    _delay_cycles(3);

    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);
    selectPinPushPullOutput(RED_LED);

    // Configure SIGNAL_IN for frequency and time measurements
    GPIO_PORTC_AFSEL_R |= FREQ_IN_MASK;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= FREQ_IN_MASK;                // enable bit 6 for digital input

    initUart0();
    setUart0BaudRate(115200, 40e6);
}


int main(void)
{
    init_texas();
    enableCounterMode();


    putsUart0("####################################################################\n");
    putsUart0("Initialzed Hardware\n");

    while(true);

}
