/*
 * microstepping.c
 *
 *  Created on: Sep 13, 2023
 *      Author: Kalki
 */


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "clock.h"
#include "gpio.h"
#include "uart0.h"
#include "nvic.h"
#include "ParseData.h"

// Pins
//Hardware init flash
#define GREEN_LED           PORTF,3

//Thread indicator LEDS
#define RED_LED             PORTF,1
#define BLUE_LED            PORTF,2

#define Motor1_EN           PORTB,6                         //  H-bridge pwm control for A-B and C-D coils , M0PWM0 Gen 0
#define Motor2_EN           PORTB,4                         // M0PWM2 Gen 1

#define MOTOR1_DIR          PORTC,4                         // Coils direction controls. Dir pairs of each coil must be the complement of the other
#define MOTOR1_DIR_COMP     PORTC,5
#define MOTOR2_DIR          PORTC,6
#define MOTOR2_DIR_COMP     PORTC,7

#define OPT_ENC             PORTE,3                         // Optical encoder pin for calibration on the motor setup


#define TIMER_LOAD_FREQ     10000                           // 4000 hz interrupt rate
#define STEPS_CALIBRATE     18



//ISR globals
bool mode_isr = false;
double error =0.0;
float electrical_phase =0.0;
float deadband = 0.1;                                                   // deadband of 0.1 degree

// Common globals
float current_theta =0.0;                                               // Input from the Uart will be saved here
float dest_theta =0.0;
float setpointDegrees =0.0;
uint32_t coil1_cmpVal =0;
uint32_t coil2_cmpVal =0;
uint8_t microstepFactor = 1;

bool calibrateStart = false;
bool dir = true;                                                        // determine direction based on the move command in putty


//function mode globals
uint32_t stepdelay_us = 200;
bool mode_func = true;

uint8_t state = 0;
bool runstepperFlag = false;

//LUT arrays to be filled in main()
float sinTable[33];
float cosTable[33];
int pwm_sinA[33];
int pwm_cosB[33];


// Flash green LEDs after succeful init of hardware
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

void init_texas(){
    initSystemClockTo40Mhz();                               // Initialize clocks and ports and GPIO pins

    // Enable clocks
    enablePort(PORTB);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);

    // Configure LED and pushbutton pins
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);
    selectPinPushPullOutput(RED_LED);

    // Set the pins to be digital outputs
    selectPinPushPullOutput(Motor1_EN);
    selectPinPushPullOutput(Motor2_EN);

    // Direction controls for the motors
    selectPinPushPullOutput(MOTOR1_DIR);
    selectPinPushPullOutput(MOTOR1_DIR_COMP);
    selectPinPushPullOutput(MOTOR2_DIR);
    selectPinPushPullOutput(MOTOR2_DIR_COMP);

    // Optical switch interrupt configuration
    selectPinDigitalInput(OPT_ENC);

    initUart0();
    setUart0BaudRate(115200, 40e6);                         // Set baud rate for UART
}

void initMotorPwm(){                                        // Initialize Motor1_EN and Motor2_EN as PWM pins for driving the coils of the motors

    disablePort(PORTB);                                     // unInit port B and start again
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;                // clock the port B for PWM 0 module
    _delay_cycles(3);

    // Set the pins to be digital outputs
    selectPinPushPullOutput(Motor1_EN);
    selectPinPushPullOutput(Motor2_EN);

    // Select auxiliiary functions for the pins
    setPinAuxFunction(Motor1_EN, GPIO_PCTL_PB6_M0PWM0);
    setPinAuxFunction(Motor2_EN, GPIO_PCTL_PB4_M0PWM2);

    // M0PWM0  >> motor 1
    // M0PWM2  >> motor 2
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                       // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                                     // leave reset state
    PWM0_0_CTL_R = 0;                                       // turn-off PWM0 generator 0
    PWM0_1_CTL_R = 0;                                       // turn-off PWM0 generator 1

    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO; //gen0, pwm out 0
    PWM0_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO; // gen1 , pwm out 2

    PWM0_0_LOAD_R = 1024;                                   // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_1_LOAD_R = 1024;

    PWM0_0_CMPA_R =723;                                     // motor 1 cmp vals
    PWM0_1_CMPA_R =723;                                     // motor 2 compare vals

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE;                        // Turn on PWM generator
    PWM0_1_CTL_R = PWM_1_CTL_ENABLE;

    PWM0_ENABLE_R = PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM2EN;

}

void initTimer2(){                                          // Periodic Timer to trigger ISR to microstep in isr mode
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


void initLUT(){

    float angle = 0.0;
    int i;
        for ( i = 0; i < 33; i++) {
            sinTable[i] = sinf(angle * M_PI / 180.0);
            cosTable[i] = cosf(angle * M_PI / 180.0);
            angle += 90.0 / 32;
        }

        sinTable[0] =0.0;                                   // correct the values for boundary cases
        sinTable[32] =1.0;
        cosTable[0] = 1.0;
        cosTable[32] =0.0;

}

// Function to get sine value for an angle in the range [0, 128) . These are micrsteps with 0  = 0 degrees and 128 = 360 degrees
float getSine(uint32_t angle) {
    if (angle <= 32) {
        return sinTable[angle]; // 0 -90 degrees
    } else if (angle <= 64) {
        return sinTable[64 - angle];    // 90 -180 degrees
    } else if (angle <= 96) {
        return -sinTable[angle - 64];   // 180 -270 degrees
    } else {
        return -sinTable[128 - angle];  // 270 -360 degrees
    }
}


// Function to get cosine value for an angle in the range [0, 128)
float getCosine(uint32_t angle) {
    if (angle <= 32) {
        return cosTable[angle]; // 0 -90 degrees
    } else if (angle <= 64) {
        return -cosTable[64 - angle]; // 90 -180 degrees
    } else if (angle <= 96) {
        return -cosTable[angle - 64]; // 180 -270 degrees
    } else {
        return cosTable[128 - angle]; // 270 -360 degrees
    }
}

// Lookup tables for the PWM compare values are initialized here
init_PwmLUT(){
    int i;
    for ( i = 0; i < 33; i++) {
        pwm_sinA[i] = (int)abs(1023*getSine(i));
        pwm_cosB[i] = (int)abs(1023*getCosine(i));
    }
}

//PWM compare values for Coil A based on different phases
getpwmA(uint32_t angle){
    if (angle <= 32) {
            return pwm_sinA[angle]; // 0 -90 degrees
    } else if (angle <= 64) {
        return pwm_sinA[64 - angle];    // 90 -180 degrees
    } else if (angle <= 96) {
        return pwm_sinA[angle - 64];   // 180 -270 degrees
    } else {
        return pwm_sinA[128 - angle];  // 270 -360 degrees
    }
}

//PWM compare values for Coil B based on different phases
getpwmB(uint32_t angle){
    if (angle <= 32) {
            return pwm_cosB[angle]; // 0 -90 degrees
    } else if (angle <= 64) {
        return pwm_cosB[64 - angle]; // 90 -180 degrees
    } else if (angle <= 96) {
        return pwm_cosB[angle - 64]; // 180 -270 degrees
    } else {
        return pwm_cosB[128 - angle]; // 270 -360 degrees
    }
}

//################################################# FULL STEP CONTROl functions below
// These group of functions can control the stepper motor in full steps of 1.8 degrees
// state CW:
// 0 -> 0
// 1 -> 270
// 2 -> 180
// 3 -> 90

void step_cw(uint32_t waitTime_us){

    int val = 1;
    int val_comp =0;

    switch(state)
    {
        case 0:  // quadrant 1

            setPinValue(MOTOR1_DIR,val_comp);
            setPinValue(MOTOR1_DIR_COMP,val);

            setPinValue(MOTOR2_DIR,0);
            setPinValue(MOTOR2_DIR_COMP,0);
            state  = (state + 1)%4;
            waitMicrosecond(waitTime_us);
            break;

        case 1:
            setPinValue(MOTOR1_DIR,0);
            setPinValue(MOTOR1_DIR_COMP,0);

            setPinValue(MOTOR2_DIR,val);
            setPinValue(MOTOR2_DIR_COMP,val_comp);
            state  = (state + 1)%4;
            waitMicrosecond(waitTime_us);
            break;

        case 2:

            setPinValue(MOTOR1_DIR,val);
            setPinValue(MOTOR1_DIR_COMP,val_comp);

            setPinValue(MOTOR2_DIR,0);
            setPinValue(MOTOR2_DIR_COMP,0);
            state  = (state + 1)%4;
            waitMicrosecond(waitTime_us);
            break;

        case 3:
            setPinValue(MOTOR1_DIR,0);
            setPinValue(MOTOR1_DIR_COMP,0);

            setPinValue(MOTOR2_DIR,val_comp);
            setPinValue(MOTOR2_DIR_COMP,val);
            state  = (state + 1)%4;
            waitMicrosecond(waitTime_us);
            break;
    }
}

// state CCW:
// 0 -> 0
// 1 -> 90
// 2 -> 180
// 3 -> 270


void step_ccw(uint32_t waitTime_us){

    int val = 1;
    int val_comp =0;

    switch(state)
    {
        case 0:

            setPinValue(MOTOR1_DIR,val);
            setPinValue(MOTOR1_DIR_COMP,val_comp);

            setPinValue(MOTOR2_DIR,0);
            setPinValue(MOTOR2_DIR_COMP,0);
            state  = (state + 1)%4;
            waitMicrosecond(waitTime_us);
            break;

        case 1:
            setPinValue(MOTOR1_DIR,0);
            setPinValue(MOTOR1_DIR_COMP,0);

            setPinValue(MOTOR2_DIR,val);
            setPinValue(MOTOR2_DIR_COMP,val_comp);
            state  = (state + 1)%4;
            waitMicrosecond(waitTime_us);
            break;

        case 2:

            setPinValue(MOTOR1_DIR,val_comp);
            setPinValue(MOTOR1_DIR_COMP,val);

            setPinValue(MOTOR2_DIR,0);
            setPinValue(MOTOR2_DIR_COMP,0);
            state  = (state + 1)%4;
            waitMicrosecond(waitTime_us);
            break;

        case 3:
            setPinValue(MOTOR1_DIR,0);
            setPinValue(MOTOR1_DIR_COMP,0);

            setPinValue(MOTOR2_DIR,val_comp);
            setPinValue(MOTOR2_DIR_COMP,val);
            state  = (state + 1)%4;
            waitMicrosecond(waitTime_us);
            break;
    }



}


void runStepper( bool direction , uint32_t waitTime_us){


    setPinValue(Motor1_EN, 1);
    setPinValue(Motor2_EN, 1);

    int full_steps = abs(dest_theta) / 1.8;
    int sign =1;    // add angles if im moving anticlockwise
    if(direction){
        sign =-1;   // subtract angles if im moving clockwise
    }

    //Turn for these many full steps
    while(full_steps){
        if(direction){  // direction is true for clockwise
            step_cw(50000);
        }
        else{           // direction is false for anticlockwise
            step_ccw(50000);
        }
        full_steps--;
        current_theta = current_theta +  sign*1.8; //**********????? update current theta as well
    }

    setPinValue(Motor1_EN, 0);                                          // Set the enables to 0 before leaving
    setPinValue(Motor2_EN,0);

    runstepperFlag = false;

}

void calibrate_level(){
    int steps_from_level =0;                                    //  Watch the level of 0 degrees using a protrator, set the motor to be in that position
                                                                //0-180 axis and then count the steps until the pin is hit
    setPinValue(Motor1_EN, 1);
    setPinValue(Motor2_EN, 1);
    while(!getPinValue(OPT_ENC)){                                // value goes low when the optical sensor is hit by the brass teeth
        step_ccw(10000);                                         // Move one full step every time
        steps_from_level++;
    }

    putsUart0("Hit the encoder for calinbration\n");

    int steps_to_level = STEPS_CALIBRATE;                       // Calculated the steps to move back to level once optical encoder is hit

    while(steps_to_level){
        step_cw(10000);
        steps_to_level --;
    }
    putsUart0("Calibrated to 0 degrees\n");

    setpointDegrees = 0; // The set point is now at zero
    current_theta = 0;  // current theta is 0;

    //0-180 axis and then count the steps until the pin is hit
    setPinValue(Motor1_EN, 0);
    setPinValue(Motor2_EN, 0);
}

//################################################# FULL STEP CONTROL FUNCTIONS END

//################################################# ISR MODE for microstepping. Press help in putty to get the command to switch modes between function and isr
// Timer frequency determined by #define TIMER_LOAD_FREQ and initTimer2()
void timerISR(){

                                                            // dir  = true means cw otherwise its ccw
    float sin_elecPhase, cos_elecPhase ;                    // Coil A, Coil B representation
    int physical_angleCoeff = 1;
    int coilA_dirCoeff = 1;
    int coilB_dirCoeff = -1;                                // sin(x), cos(x) coefficients

    error = setpointDegrees - current_theta;

    if(abs(error) > deadband){
                                                            //******lookup next electrical phase and set DIR and EN pins
        if(dir){
            physical_angleCoeff = -1;
            coilA_dirCoeff = -1;
        }

        electrical_phase += 32/microstepFactor;             // microstepFactor allowed values are 1,2,4,8,16,32 . Use the angleIndex for LUT lookup for sine and cosine

        sin_elecPhase = coilA_dirCoeff*getSine(electrical_phase);               // for ccw A,B = sinx, -cosx and for cw A,B = -sinx, -cosx
        cos_elecPhase = coilB_dirCoeff*getCosine(electrical_phase);             // -1 is introdiced because of the signs for excitation of coils in the motor
                                                                                // These phase values determine the directions to move to next and not for the current position

        coil1_cmpVal = getpwmA(electrical_phase);
        coil2_cmpVal = getpwmB(electrical_phase);

        if(sin_elecPhase > 0){
                                                                                // Set DIR for Coil A-B
           setPinValue(MOTOR1_DIR,1);
           setPinValue(MOTOR1_DIR_COMP, 0);
        }
        else if(sin_elecPhase == 0){
            setPinValue(MOTOR1_DIR,0);
            setPinValue(MOTOR1_DIR_COMP, 0);
        }
        else{
            setPinValue(MOTOR1_DIR,0);
            setPinValue(MOTOR1_DIR_COMP, 1);
        }

        if(cos_elecPhase > 0){                                                  //Set DIR for Coil C-D
            setPinValue(MOTOR2_DIR,1);
            setPinValue(MOTOR2_DIR_COMP, 0);
        }
        else if(cos_elecPhase == 0){
            setPinValue(MOTOR2_DIR,0);
            setPinValue(MOTOR2_DIR_COMP, 0);
        }
        else{
            setPinValue(MOTOR2_DIR,0);
            setPinValue(MOTOR2_DIR_COMP, 1);
        }

        current_theta += physical_angleCoeff*(1.8/microstepFactor);

        PWM0_0_CMPA_R = coil1_cmpVal;                                       // Coil 1 EN compare values
        PWM0_1_CMPA_R = coil2_cmpVal;                                       // Coil 2 EN compare values

        if(electrical_phase == 128){                                        // reset phase to 0
            electrical_phase = 0;
        }

        togglePinValue(BLUE_LED);                                           // toggle LED to indicate steps
    }

    else{
                                                                            //Coils are left in full excitation to make sure that the Motor is left in taut state
        PWM0_0_CMPA_R = 1023;                                               // Coil 1 EN compare values to hold the motor in place
        PWM0_1_CMPA_R = 1023;                                               // Coil 2 EN compare values
    }
    togglePinValue(RED_LED);

    TIMER2_ICR_R = TIMER_ICR_TATOCINT;                                      // Clear thetimer interrupt
}


//################################################# Function mode for microstepping. Press help in putty to get the command to switch modes between function and isr

void microstep(bool direction, uint32_t waitTime_us){

    double theta_diff = (setpointDegrees - current_theta)/1.8;
    theta_diff *= microstepFactor;
    int totalSteps = abs((int)theta_diff);                                      // get the total steps to move in which direction
    int angleIndex = 0;

    if(calibrateStart){                                                         // Calibration command is given
        direction = false;                                                      // Turn anticlockwise until you find the optical encoder
        microstepFactor = 32;
        waitTime_us = 200;
        totalSteps = 3200;                                                      // (180/1.8) * 32 // based on the limitations of the motor it wont have to turn this much
    }
    bool calibrateFinished = false;

    float sin_elecPhase, cos_elecPhase ;                                        // Coil A, Coil B
    int physical_angleCoeff = 1;
    int coilA_dirCoeff = 1;
    int coilB_dirCoeff = -1;                                                    // sin(x), cos(x) coefficients

                                                                                //******lookup next electrical phase and set DIR and EN pins
    if(direction){
        physical_angleCoeff = -1;
        coilA_dirCoeff = -1;
    }

    while(totalSteps){                                                          // Loop through the total steps based on the micrstepfactor

        angleIndex += 32/microstepFactor;                                       // microstepFactor allowed values are 1,2,4,8,16,32 . Use the angleIndex for LUT lookup for sine and cosine

        sin_elecPhase = coilA_dirCoeff*getSine(angleIndex);                     // for ccw A,B = sinx, -cosx and for cw A,B = -sinx, -cosx
        cos_elecPhase = coilB_dirCoeff*getCosine(angleIndex);                   // -1 is introdiced because of the signs for excitation of coils in the motor
                                                                                // These phase values determine the directions to move to next and not for the current position

        coil1_cmpVal = getpwmA(angleIndex);
        coil2_cmpVal = getpwmB(angleIndex);

        if(sin_elecPhase > 0){
                                                                                // Set DIR for Coil A-B
           setPinValue(MOTOR1_DIR,1);
           setPinValue(MOTOR1_DIR_COMP, 0);
        }
        else if(sin_elecPhase == 0){
            setPinValue(MOTOR1_DIR,0);
            setPinValue(MOTOR1_DIR_COMP, 0);
        }
        else{
            setPinValue(MOTOR1_DIR,0);
            setPinValue(MOTOR1_DIR_COMP, 1);
        }

        if(cos_elecPhase > 0){                                                  //Set DIR for Coil C-D
            setPinValue(MOTOR2_DIR,1);
            setPinValue(MOTOR2_DIR_COMP, 0);
        }
        else if(cos_elecPhase == 0){
            setPinValue(MOTOR2_DIR,0);
            setPinValue(MOTOR2_DIR_COMP, 0);
        }
        else{
            setPinValue(MOTOR2_DIR,0);
            setPinValue(MOTOR2_DIR_COMP, 1);
        }

        waitMicrosecond(waitTime_us);
        current_theta += physical_angleCoeff*(1.8/microstepFactor);
        totalSteps--;

        PWM0_0_CMPA_R = coil1_cmpVal;                                           // Coil 1 EN compare values
        PWM0_1_CMPA_R = coil2_cmpVal;                                           // Coil 2 EN compare values


        if(angleIndex == 128){                                                  // reset phase to 0
            angleIndex = 0;
        }

        if(calibrateStart){
            if(getPinValue(OPT_ENC)){
                                                                                // If the pin goes high that means that the encoder has been hit and we have to turn clockwise now
                putsUart0("Encoder pin is hit\n");
                direction = true;                                               // turn clokwise now and set the necessary prameters for that below

                physical_angleCoeff = -1;                                       // params for turning clockwise
                coilA_dirCoeff = -1;

                totalSteps = STEPS_CALIBRATE*microstepFactor;                   // turn clockwise by this much to attain 0 degrees
                calibrateStart = false;                                         // No need to check this block after doing it once
                calibrateFinished = true;                                       // Use this flag to set params after while loop is finished for upcoming normal runs
            }
        }

        togglePinValue(BLUE_LED);                                               // toggle LED to indicate steps
    }

    runstepperFlag = false;
    angleIndex = 0;

    PWM0_0_CMPA_R = 1023;                                                       // Coil 1 EN compare values . PWMs left in excited state to leave the motor in a taut state
    PWM0_1_CMPA_R = 1023;                                                       // Coil 2 EN compare values

    if(calibrateFinished){
        putsUart0("Current at 0 degrees\n");
        current_theta = 0.0;
        setpointDegrees = 0.0;
        dest_theta = 0.0;
        calibrateFinished = false;
        microstepFactor = 32;
        stepdelay_us = 200;
        putsUart0("mf : 32 delay 200us . Change it if needed\n");
    }


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

            if (strcmp(token, "move") == 0) {
                token = strtok(NULL, " ");                                      // Get the next token (angle)
                if (token != NULL) {
                    dest_theta = atof(token);
                    setpointDegrees = dest_theta;


                    if(dest_theta > 0){
                        dir = false;                                            // turn anticlockwise
                    }
                    else{
                        dir = true;                                             // turn clockwise
                    }

                } else {
                    putsUart0("Missing angle after 'move'\n");
                }
                runstepperFlag = true;                                          // set the flag to true to call the microstep function

            }

            if (strcmp(token, "mf") == 0) {
                    token = strtok(NULL, " ");                                  // Get the next token (angle)
                    if (token != NULL) {
                        microstepFactor = atoi(token);

                    } else {
                        putsUart0("Missing integer after 'mf'\n");
                    }
            }
            if (strcmp(token, "delay") == 0) {
                    token = strtok(NULL, " ");                                  // Get the next token (angle)
                    if (token != NULL) {
                        stepdelay_us = atoi(token);

                    } else {
                        putsUart0("Missing integer after 'mf'\n");
                    }
            }
            if (strcmp(token, "isr") == 0) {

                mode_func = false;
                mode_isr = true;
                current_theta = 0.0;
                dest_theta = 0.0;
                setpointDegrees = 0.0;
                putsUart0("mode is now isr. Type func to switch to function mode\n");
            }

            if (strcmp(token, "func") == 0) {

                mode_func = true;
                mode_isr = false;
                TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                                // turn-off timer before switching modes
                current_theta = 0.0;
                dest_theta = 0.0;
                setpointDegrees = 0.0;
                putsUart0("mode is now func. Type isr to switch to function mode\n");

            }

            if (strcmp(token, "level") == 0) {                                  // Calibrate command
                calibrateStart = true;
                runstepperFlag = true;
            }

            if (strcmp(token, "reboot") == 0) {
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }

            if (strcmp(token, "help") == 0) {
                putsUart0("##############################################################\n");
                putsUart0("Help menu -- commands list\n");
                putsUart0("reboot\n");
                putsUart0("move +-angle(float) .. \n");
                putsUart0("delay stepDelay_us...motor step delay in us\n");
                putsUart0("mf microstepfactor .. set the microstep factor. Suggested uses of 2,4,8,16,32\n");
                putsUart0("isr...isr run microstepping mode\n");
                putsUart0("func..function run microstepping mode\n");
                putsUart0("level..calibrate to 0. Currently only implemented in function mode.\n Type func to switch to func mode before this command\n");
                putsUart0("##############################################################\n");
            }
        }
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main below
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(){
    init_texas();                               // Initialize the hardware
    initTimer2();
    initMotorPwm();                             // FULL STEP FUNCTIONS WILL NOT WORK IF PWM IS INITIALIZED. COMMENT THIS OUT BEFORE USING IT
    flash_hardware_green();

    initLUT();                                  // Initialize the Lookup arrays
    init_PwmLUT();

    putsUart0("##############################################################\n");
    putsUart0("Initialized Hardware in function mode. Type 'help' for commands list\n");

//    calibrate_level();                        // Calibrate level using FULL STEPS

    putsUart0("Initial calibration started...\n");
    calibrateStart = true;
    runstepperFlag = true;                      // start with calibration using microstepping

    while(true){
        processShell();

        if(runstepperFlag){
//            runStepper(dir, 50000);           // Function for running it in FULL STEPS
            if(mode_func){                      // Function mode is ON
                microstep(dir, stepdelay_us);   // Function for using microstepping
            }
        }

        if(mode_isr){                           // ISR MODE is ON
            TIMER2_CTL_R |= TIMER_CTL_TAEN;                               // turn-on timer to trigger timerISR for microstepping
        }

    }
}
