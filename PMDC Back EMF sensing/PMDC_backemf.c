/*
 * PMDC_backemf.c
 *
 *  Created on: Sep 29, 2023
 *      Author: giskusama
 */


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "nvic.h"
#include "gpio.h"
#include "adc0.h"
#include "tm4c123gh6pm.h"


#define PUSH_BUTTON2    PORTF,0
#define RED_LED         PORTF,1
#define BLUE_LED        PORTF,2
#define GREEN_LED       PORTF,3
#define PUSH_BUTTON1    PORTF,4

#define PWM_PIN         PORTB,6
//#define INT_PIN         PORTC,5

#define AIN3            PORTE,0

#define TIMER_LOAD_FREQ 800000 // 50Hz
#define CCP_TIMER_FREQ  40000000 // 1hz
#define FREQ_IN_MASK    64         // WT1CCP0 PC6
#define V_SOURCE        10 // 10 V supply to PMDC motor


int pwm_cmpVal = 512;
uint32_t frequency = 0;

uint16_t raw;
uint16_t adcBuffer1[16] = {0};
uint8_t index = 0;
uint16_t sum = 0;                                           // Total fits in 16b since 12b adc output x 16 samples
uint8_t i;
float alpha = 0.80;
float backemf = 0.0;
float emf_rpm = 0.0;

//################################################################ HARDWARE INIT FUNCTIONS
// Flash green LEDs after successful init of hardware

void flash_hardware_green(){
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(10000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(10000);
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(10000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(10000);
}

/**
 * @brief Initialize the texas evaluation board and set pins as digital outputs or inputs or analog inputs
 * give clocks to timer 1 in CCP or counter mode to measure the rpm of the motor by checking for counts
 * 
 */
void init_texas(){
    initSystemClockTo40Mhz();                               // Initialize clocks and ports and GPIO pins


    // Enable clocks
    enablePort(PORTB);//PWM
    enablePort(PORTC);//INT PIN
    enablePort(PORTE); // Analog input
    enablePort(PORTF);// LEDS & PUSHBUTTONS

    setPinCommitControl(PUSH_BUTTON2);
    // Configure LED and pushbutton pins
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);
    selectPinPushPullOutput(RED_LED);

    // PUSH_BUTTON1 and PUSHBUTTON 2
    selectPinDigitalInput(PUSH_BUTTON1);
    selectPinDigitalInput(PUSH_BUTTON2);
    enablePinPullup(PUSH_BUTTON1);
    enablePinPullup(PUSH_BUTTON2);

    selectPinAnalogInput(AIN3);

//    // Optical switch interrupt configuration


//    selectPinDigitalInput(INT_PIN);
//    selectPinInterruptFallingEdge(INT_PIN);
//    enablePinInterrupt(INT_PIN);
//    enableNvicInterrupt(INT_GPIOC);


    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    _delay_cycles(3);

    // Configure SIGNAL_IN for frequency and time measurements for counter mode
    GPIO_PORTC_AFSEL_R |= FREQ_IN_MASK;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= FREQ_IN_MASK;                // enable bit 6 for digital input

    initUart0();
    setUart0BaudRate(115200, 40e6);                         // Set baud rate for UART
}

/**
 * @brief Initialize PWM on M0PWM0 with 50% duty cycle
 */

initPwm(){
    disablePort(PORTB);                                     // unInit port B and start again
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;                // clock the port B for PWM 0 module
    _delay_cycles(3);

    selectPinPushPullOutput(PWM_PIN);    // Set the pins to be digital outputs
    setPinAuxFunction(PWM_PIN, GPIO_PCTL_PB6_M0PWM0);    // Select auxiliiary functions for the pins

    // M0PWM0  >> motor 1
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                       // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                                     // leave reset state
    PWM0_0_CTL_R = 0;                                       // turn-off PWM0 generator 0

    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO; //gen0, pwm out 0

    PWM0_0_LOAD_R = 1024;                                   // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_0_CMPA_R =512;                                     // 50 % start duty cycle
    PWM0_0_CTL_R = PWM_0_CTL_ENABLE;                        // Turn on PWM generator
    PWM0_ENABLE_R = PWM_ENABLE_PWM0EN;

}

/**
 * @brief Initialize Timer 2 with a frequency of 50hz to trigger the backemfISR()
 * 
 */

void initTimer2(){                                          
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    _delay_cycles(3);

    // Configure Timer 2 for PID controller
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                        // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;                  // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;                 // configure for periodic mode (count down)
    TIMER2_TAILR_R = TIMER_LOAD_FREQ;                       // set load value for interrupt rate of 1000hz
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                        // turn-on interrupts
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);                     // turn-on interrupt 37 (TIMER2A)
//    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

/**
 * @brief Enable Timer 1 in counter mode to 
 * 
 */
void enableCounterMode()
{

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                    // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;              // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;             // configure for periodic mode (count down)
    TIMER1_TAILR_R = CCP_TIMER_FREQ;                 
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                    // turn-on interrupts
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                     // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);                 // turn-on interrupt 37 (TIMER1A)

    // Configure Wide Timer 1 as counter of external events on CCP0 pin
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                   // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                                  // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_POS;              // count positive edges
    WTIMER1_IMR_R = 0;                                  // turn-off interrupts
    WTIMER1_TAV_R = 0;                                  // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                    // turn-on counter
}

//################################################################ SUBROUTINES

/**
 * @brief Set the Pwm Duty Cycle object
 * 
 */
void setPwmDutyCycle(){
    if(!getPinValue(PUSH_BUTTON1)){                          // if button1 is pressed
        pwm_cmpVal += (int)(0.1* 1023);
        if(pwm_cmpVal > 1023){
            pwm_cmpVal = 1023;
        }
        PWM0_0_CMPA_R = pwm_cmpVal;                         // increase the compare value
        waitMicrosecond(1000000);

    }

    if(!getPinValue(PUSH_BUTTON2)){                          // if button2 is pressed
            pwm_cmpVal -= 102;
            if(pwm_cmpVal < 0){
                pwm_cmpVal = 0;
            }
            PWM0_0_CMPA_R = pwm_cmpVal;                     // Decrease the compare value
            waitMicrosecond(1000000);
    }
}


//################################################################ ISR(s)

void timer1Isr()    // Measure frequency
{
    frequency = WTIMER1_TAV_R;                   // read counter input
    float f_temp = (frequency*60/32);            // Motor frequency in rpm
    frequency = (int)f_temp ;
    WTIMER1_TAV_R = 0;                           // reset counter for next period
    togglePinValue(RED_LED);

    char str[100];
    snprintf(str, sizeof(str), "frequency optical: %d  emf_rpm : %f  PWM_compare: %d",  frequency, emf_rpm, pwm_cmpVal);
    putsUart0(str);
    putsUart0("\n\n");

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;           // clear interrupt flag


}

void timer2_baackemfIsr(){  // every 20ms
    PWM0_0_CMPA_R = 0;  // Stop the PWM

    waitMicrosecond(100); // Wait 100us to sample the backemf and to give time for magetic dump to complete
    int i =0;

    for(i =0; i<16; i++){   // Less than 15 us for sampling the ADC 16 times to get the FIR filtered value

        raw = readAdc0Ss3();

        // FIR sliding average filter with circular addressing
        sum -= adcBuffer1[index];
        sum += raw;
        adcBuffer1[index] = raw;
        index = (index + 1) & 15;
    }

    backemf = ((((sum >> 4)+0.5)*0.000806));        // takes a while for the backemf measurement to stabilize   - 15us
    backemf = V_SOURCE - 5.7*backemf;                     // backemf = ((Vref+ - Vref-)*(sum >> 4)/4096 ) + Vref-
                                                            // Scale the backemf according ot the voltage divider connected

    emf_rpm = 165.4*backemf + 11.14;    // calculation of rpm based on backemf based on linear regression model between backemf and optical rpm



    PWM0_0_CMPA_R = pwm_cmpVal; // Start the PWM
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;                                      // Clear thetimer interrupt
    togglePinValue(BLUE_LED);                       // Check the toggle LED to see if the ADC is not being read in a nice fashion

}

//################################################################ Shell for commands from putty below*
#define MAX_CHARS 80
uint8_t count = 0;
char strInput[MAX_CHARS + 1];
char* token;

void processShell() {
    bool end;
    char c;

    if (kbhitUart0()) {
        c = getcUart0();

        end = (c == 13) || (count == MAX_CHARS);
        if (!end) {
            if ((c == 8 || c == 127) && count > 0)
                count--;
            if (c >= ' ' && c < 127)
                strInput[count++] = c;
        } else {
            strInput[count] = '\0';
            count = 0;
            token = strtok(strInput, " ");

            if (strcmp(token, "duty") == 0) {
                    token = strtok(NULL, " ");                                  // Get the next token (angle)
                    if (token != NULL) {
                        int percentage = 0;
                        percentage = atoi(token);
                        float cmpval = (percentage*1023.0/100);
                        pwm_cmpVal = (int)cmpval;
                        if(pwm_cmpVal < 0){
                            pwm_cmpVal =0;
                        }
                        else if(pwm_cmpVal > 1023){
                            pwm_cmpVal = 1023;
                        }
                        PWM0_0_CMPA_R = pwm_cmpVal;
                    } else {
                        putsUart0("Missing integer after 'duty'\n");
                    }
            }

            if(strcmp(token, "show") == 0){

                char str[100];
                snprintf(str, sizeof(str), "frequency optical: %d  backemf : %f  PWM_compare: %d",  frequency, backemf, pwm_cmpVal);
                putsUart0(str);
                putsUart0("\n\n");
            }

            if (strcmp(token, "reboot") == 0) {
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }

            if (strcmp(token, "help") == 0) {
                putsUart0("##############################################################\n");
                putsUart0("Help menu -- commands list\n");
                putsUart0("duty pwmpercent .. Scale the pwm by percentage 10,20,30...100\n");
                putsUart0("show .. Show current backemf, frequency, pwmcmpval\n");
                putsUart0("reboot\n");
                putsUart0("##############################################################\n");
            }
        }
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////



int main(){
    init_texas();
    initPwm();
    initTimer2();
    enableCounterMode();
    initAdc0Ss3();
    flash_hardware_green();

    putsUart0("\n######################################################");
    putsUart0("\nInitialized Hardware\n");

    // Use AIN3 input with N=4 hardware sampling
    setAdc0Ss3Mux(3);
    setAdc0Ss3Log2AverageCount(2);

        TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    while(true){
        processShell();
        setPwmDutyCycle();
    }
}


