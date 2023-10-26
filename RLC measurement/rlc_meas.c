#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "clock.h"
#include "gpio.h"
#include "uart0.h"
#include "nvic.h"

//101 nf - 86 nf
//11.3 nf - 8.10 nf
// 1123 nf - 944 nf

// Pins
//Hardware init flash
#define GREEN_LED           PORTF,3

//Thread indicator LEDS
#define RED_LED             PORTF,1
#define BLUE_LED            PORTF,2

#define DEINTEGRATE         PORTE,0
#define CO_plus             PORTC,6
#define CO_minus            PORTC,7     // Analog comparator negative input
#define FREQ_CCP            PORTD,0

//Capacitance measurement
uint32_t charge_time =0;
double meas_capacitance =0.0;
double other_meassurement = 0.0;
uint32_t fixed_resistance = 995000; // 1Meg resistor
float calib_coeff =1.25;

//Colpitts oscillator
uint32_t frequency =0;
uint32_t last_frequency = 0;
int freq_error = 0;
uint32_t threshold =0;              //TODO To be observed and filled

//Deintegration
uint32_t step_delay_us =60;
bool deint_flag = false;
bool continuos_deint = false;

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
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER2_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER2_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER2_CTL_R = TIMER_CTL_TAEVENT_POS;           // count positive edges
    WTIMER2_IMR_R = 0;                               // turn-off interrupts
    WTIMER2_TAV_R = 0;                               // zero counter for first period
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
}

void initTimer0(){
    // Configure Timer 0 as a free running timer and grab the value of the TAV_R register
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER0_TAMR_R =  TIMER_TAMR_TAMR_1_SHOT | TIMER_TAMR_TACDIR;
//    TIMER0_TAILR_R = 40000000;                       // set load value to 40e6 for 1s which is enough time for any
    // TIMER0_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    TIMER0_TAV_R =0;    // Set the Timer A value to zero
//    TIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
//    NVIC_EN0_R = 1 << (INT_TIMER0A-16);             // turn-on interrupt 37 (TIMER1A)
}

void init_texas(){
    // Enable clocks
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);

        // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R0;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1| SYSCTL_RCGCWTIMER_R2;
    SYSCTL_RCGCACMP_R |= SYSCTL_RCGCACMP_R0; // Give clocks to the analog comparator
    _delay_cycles(3);





    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);
    selectPinPushPullOutput(DEINTEGRATE);

    selectPinDigitalInput(FREQ_CCP);                    // Configure the pin as a CCP pin input
    setPinAuxFunction(FREQ_CCP, GPIO_PCTL_PD0_WT2CCP0);

    selectPinAnalogInput(CO_minus);
    // setPinAuxFunction(CO_plus, GPIO_PCTL_PC6_) // Does it need Aux?

    initTimer0();
    enableCounterMode();

    COMP_ACREFCTL_R |= COMP_ACREFCTL_EN|COMP_ACREFCTL_RNG| COMP_ACREFCTL_VREF_M;    // Vref = 2.469 V, ENabled
    COMP_ACCTL0_R |= COMP_ACCTL0_ASRCP_REF | COMP_ACCTL0_ISEN_RISE | COMP_ACCTL0_CINV; // Internal reference, rising edge, output inverter enabled
//    COMP_ACINTEN_R |= COMP_ACINTEN_IN0;             // Interrupt enabled

//    NVIC_EN0_R = 1 << (INT_COMP0-16);             // turn-on interrupt


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
 * @brief Pulse the DEINTEGRATE pin that privides a path for the capacitor to discharge. The caps will have enough time to discharge in a microsecond
 *
 */
void pulse_deint(uint32_t delay_us){
    setPinValue(DEINTEGRATE, 1);
    waitMicrosecond(delay_us);
    setPinValue(DEINTEGRATE, 0);
}

// Frequency counter service publishing latest frequency measurements every second. Used for LAB Part 2
/*
 * Used for Lab Part 2
 * TODO Hook up the Colpitts oscillator circuit. Note down the "frequency" of the oscillator observed.
 */
void timer1_colpitts_ISR()
{
    last_frequency = frequency;                     // Save the last onserved frequency and print out the change in frequency
    frequency = WTIMER2_TAV_R;                      // read counter input

    freq_error = frequency - last_frequency;        // Calculate error and print it

    WTIMER2_TAV_R = 0;                              // reset counter for next period
    togglePinValue(BLUE_LED);

    if(freq_error > threshold){
        putsUart0("Metal plate detected\n");
    }

    char str[100];
    snprintf(str, sizeof(str), "\nResonant frequency: %d  Frequency Change obs: %d\n", frequency, freq_error);
    putsUart0(str);


    TIMER1_ICR_R = TIMER_ICR_TATOCINT;           // clear interrupt flag
}
/**
 * @brief The Comparator raises an input when the Capacitor charge hits the same as Internal Voltage reference (2.469 V, configurable)
 * Get the value of the charge time and
 *
 */

void comparator0_ISR(){
    /*
        // this is a double check and may not be necessary
        // Since the CINV is true, the output is inverted. So OVAL = 1 when VIN- > VIN+ and vice versa
        // We will see a 1 in OVAL when Capacitor charge just crosses the Internal Voltage reference
        charge_time = TIMER0_TAV_R*25;         // Store the timer A value in nano seconds 1 tick  = 25 nano seconds
        TIMER0_TAV_R = 0;
        meas_capacitance = 0.778975*charge_time/fixed_resistance; // 0.725137947 is for when there is not effect of CSE SAT voltage across the transistor

//        TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                  // turn-on timer


        char str[100];
        snprintf(str, sizeof(str), "\nCharge Time us: %d Measured Capacitance in nf : %lf \n", charge_time, meas_capacitance);
        putsUart0(str);

        // Take more capacitance readings at different liquid levels and form a relationship between the capacitance observed and the water level. It should linearly increase
        COMP_ACINTEN_R &= ~COMP_ACINTEN_IN0;             // Interrupt disabled
        COMP_ACMIS_R |= COMP_ACMIS_IN0; // Clear the interrupt

*/
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

            if(strcmp(token, "deint") == 0){
                deint_flag = true;
                token = strtok(NULL, " ");                                  // Get the next token (angle)
                    if (token != NULL) {
                        step_delay_us = atoi(token);
                }
            }
            if(strcmp(token, "cont") == 0){
                continuos_deint = true;

            }


            if (strcmp(token, "reboot") == 0) {
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }

            if (strcmp(token, "help") == 0) {
                putsUart0("##############################################################\n");
                putsUart0("Help menu -- commands list\n");
                putsUart0("deint step_delay_us.. start a deintegration pulse and swith on the timer\n");
                putsUart0("cont...continous deintegration print out the volume of the water level based on charge time\n");
                putsUart0("reboot\n");
                putsUart0("##############################################################\n");
            }
        }
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * TODO : Take multiple measurements of capacitance with different water levels and form an equation " volume (ml) = k*meas_capacitance + constant"
 * Print the volume in ml along with the capacitance and demo to mark
 *
 */


int main(){
    initSystemClockTo40Mhz();                                                           // Initialize clocks
    init_texas();

//    enableTimerMode();
    initUart0();
    setUart0BaudRate(115200, 40e6);

    bool check_cap = false;
    int count = 0;
    float avg_vol =0;
    int avg_cnt =0;

    putsUart0("####################################################################\n");
    putsUart0("Initialzed Hardware\n");

    while(true){

        if(check_cap){
            if(COMP_ACSTAT0_R == COMP_ACSTAT0_OVAL){
                charge_time = TIMER0_TAV_R*25;                                              // Store the timer A value in nano seconds 1 tick  = 25 nano seconds
                meas_capacitance = 0.778975*charge_time*calib_coeff*1000/fixed_resistance;   // 0.725137947 is for when there is not effect of CSE SAT voltage across the transistor

                TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                                            // turn-off timer
                TIMER0_TAV_R = 0;

                float volume_ml;
//                volume_ml = (meas_capacitance - 109755)/4.363;                              // Calculate and display the volume based on the observed linear increase in capacitance as volume increases
//                volume_ml = 0.0003104*charge_time - 34500;
                  volume_ml = 0.01362*charge_time - 657.4;

//                avg_vol += volume_ml;
//                avg_cnt = (avg_cnt + 1)%16; // count from 0 -15 which is 16 times

//                if(avg_cnt == 15){
//                avg_vol = avg_vol/16;

                char str[100];
                snprintf(str, sizeof(str), "\navg_vol : %f Vol(ml): %f Time ns: %u\n",avg_vol, volume_ml, charge_time );
                putsUart0(str);
//                }
                check_cap = false;
            }
        }

        if(deint_flag){                                                                     // Once deint 100 command is called in the putty , we pulse the deintegration pin, discharge capacitor and start the timer

            pulse_deint(step_delay_us);
//            putsUart0("Capacitor deintegrated\n");
            deint_flag = false;
            TIMER0_TAV_R =0;
//            COMP_ACINTEN_R |= COMP_ACINTEN_IN0;             // Interrupt enabled
//            NVIC_EN0_R = 1 << (INT_COMP0-16);             // turn-on interrupt

            TIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
            check_cap = true;
        }

        if(continuos_deint){
            if(!deint_flag){
                count++;
            }
            if(count == 100000){
                count =0;
                if(!check_cap){
                    deint_flag = true;
                }
            }
        }

        processShell();                                                                 // Used for handling putty commands



    }
}
