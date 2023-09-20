/*
 * microstepping.c
 *
 *  Created on: Sep 13, 2023
 *      Author: giskusama
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
#define GREEN_LED       PORTF,3

//Thread indicator LEDS
#define RED_LED         PORTF,1
#define BLUE_LED        PORTF,2

#define Motor1_EN  PORTB,6         //  H-bridge pwm control for A-B and C-D coils , M0PWM0 Gen 0
#define Motor2_EN  PORTB,4          // M0PWM2 Gen 1

#define MOTOR1_DIR          PORTC,4 // Coils direction controls. Dir pairs of each coil must be the complement of the other
#define MOTOR1_DIR_COMP     PORTC,5
#define MOTOR2_DIR          PORTC,6
#define MOTOR2_DIR_COMP     PORTC,7

#define OPT_ENC             PORTD,7

#define ANGLE_PER_FULL_STEP 1.8     // 1.8 degrees per full step
#define NUM_POLE_PAIRS      50

bool dir = true; // global
float setpointDegrees =0;
uint8_t microstepFactor = 1;

uint8_t state = 0;
bool runstepperFlag = false;

float error =0;

float theta =0;         // Input from the Uart will be saved here
float electrical_phase =0;


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
    initSystemClockTo40Mhz();   // Initialize clocks

    // Enable clocks
    enablePort(PORTB);
    enablePort(PORTC);
    enablePort(PORTD);
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
    enablePinPullup(OPT_ENC);//????

    initUart0();
    setUart0BaudRate(115200, 40e6); // Set baud rate
}

void initMotorPwm(){
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // clock the port B for PWM 0 module
    _delay_cycles(3);


    setPinAuxFunction(Motor1_EN, GPIO_PCTL_PB6_M0PWM0);
    setPinAuxFunction(Motor2_EN, GPIO_PCTL_PB4_M0PWM2);

    // M0PWM0  >> motor 1
    // M0PWM2  >> motor 2
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM0_0_CTL_R = 0;                                // turn-off PWM0 generator 0
    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO; //gen0, pwm out 0
    PWM0_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO; // gen1 , pwm out 2

    PWM0_0_LOAD_R = 1024;                           // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_1_LOAD_R = 1024;

    PWM0_0_CMPA_R =0;       // motor 1 cmp vals

    PWM0_1_CMPA_R =0;       // motor 2 compare vals

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE;                // Turn on PWM generator

    PWM0_ENABLE_R = PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM2EN;         // enable outputs in ISR

}


void initTimer2(){
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    _delay_cycles(3);

    // Configure Timer 2 for PID controller
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = 40000;                          // set load value for interrupt rate of 1000hz
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);              // turn-on interrupt 37 (TIMER2A)
//    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}




/*
void microstep(uint8_t num_full_steps, uint8_t microstep_factor, bool dir){

    // dir  = true means cw otherwise its ccw
    int cw_val, cw_val_comp;
    int acw_val, acw_val_comp;
    uint32_t total_steps = steps*microstep_factor;

    if(dir){
        cw_val = 1;        // value
        cw_val_comp = 0;   // value compliment for clockwise direction
    }
    else{
        acw_val =0;         // value and val_compliment pairs for the anti clockwise direction
        acw_val_comp =1;
    }

    while(total_steps > 0){
        // Set directions for both coils based on current angle of the stepper motor
        if(cosf(theta) > 0){
            setPinValue(MOTOR1_DIR,cw_val);
            setPinValue(MOTOR1_DIR_COMP, cw_val_comp);
        }
        else{
            setPinValue(MOTOR1_DIR,acw_val);
            setPinValue(MOTOR1_DIR_COMP, acw_val_comp);
        }

        if(sinf(theta) > 0){
            setPinValue(MOTOR2_DIR,cw_val);
            setPinValue(MOTOR2_DIR_COMP, cw_val_comp);
        }
        else{
            setPinValue(MOTOR2_DIR,acw_val);
            setPinValue(MOTOR2_DIR_COMP, acw_val_comp);
        }

        // Enable PWMs
        PWM0_0_CMPA_R = abs(1024*cosf(theta));       // motor 1 cmp vals

        PWM0_1_CMPA_R =abs(1024*sinf(theta));       // motor 2 compare vals

        PWM0_ENABLE_R = PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN;

        total_steps--;                        // decrement after a step has been taken
        theta += angle_per_step;        // increment current angle

        if(theta == 360){               // reset theta to 0
            theta = 0;
        }

        togglePinValue(BLUE_LED);       // toggle LED to indicate steps
    }

}
*/

void timerISR(){
    uint16_t coil1_cmpVal =0;
    uint16_t coil2_cmpVal =0;

    // dir  = true means cw otherwise its ccw
    int val, val_comp;

    error = setpointDegrees - theta;
    if(error != 0){
        //lookup next electrical phase and set DIR and EN pins

        if(dir){
            val = 1;                                    // value
            val_comp = 0;                               // value compliment for clockwise direction
        }
        else{
            val =0;                                     // value and val_compliment pairs for the anti clockwise direction
            val_comp =1;
        }

        if(cosf(electrical_phase) > 0){                 // Set DIR for Coil A-B
            setPinValue(MOTOR1_DIR,val);
            setPinValue(MOTOR1_DIR_COMP, val_comp);
        }
        else{
            setPinValue(MOTOR1_DIR,val_comp);
            setPinValue(MOTOR1_DIR_COMP, val);
        }

        if(sinf(electrical_phase) > 0){                 //Set DIR for Coil C-D
            setPinValue(MOTOR2_DIR,val);
            setPinValue(MOTOR2_DIR_COMP, val_comp);
        }
        else{
            setPinValue(MOTOR2_DIR,val_comp);
            setPinValue(MOTOR2_DIR_COMP, val);
        }

        coil1_cmpVal = abs(1023*cosf(electrical_phase)); // check if the input is in radians or degrees???
        coil2_cmpVal = abs(1023*sinf(electrical_phase));

        PWM0_0_CMPA_R = coil1_cmpVal;                   // Coil 1 EN compare values
        PWM0_1_CMPA_R = coil2_cmpVal;                   // Coil 2 EN compare values


//        PWM0_ENABLE_R = PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM2EN;         // enable outputs
        electrical_phase += 90/microstepFactor;         // increment current angle in electrical phase. Divide this by 50 to get the mechanical angle
        theta += 1.8/microstepFactor;

        if(electrical_phase == 360){                    // reset phase to 0
            electrical_phase = 0;
        }
        togglePinValue(BLUE_LED);                       // toggle LED to indicate steps
    }

    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}


void shell(USER_DATA* data ){

    char *command = getFieldString(data, 0);            // get the first command
    tolower_string(command);                            // make command into lowercase before comparision. Strcmp also only checks for case insensitive comparision


    if(isCommand(data,command, "help", 0)){
        putsUart0("##############################################################\n");
        putsUart0("Help menu -- commands list\n");
        putsUart0("reboot\n");
        putsUart0("move +/-angle .. move to angle cw or ccw \n");
        putsUart0("mf microstepfactor .. set the microstep factor. Suggested uses of 2,4,8,16,32\n");
        putsUart0("##############################################################\n");
    }

    else if(isCommand(data,command, "reboot", 0)){
        putsUart0("Rebooting...");
        NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
    }

    else if(isCommand(data,command, "move", 1)){
//        char * angle;
//        angle = getFieldString(data, 1);
        int angle = getFieldInteger(data, 1);
//        theta = (float)(atof(angle));
        theta = angle;
//        if(theta > 0){
//            dir = true;
//        }
//        else{
//            dir  = false;
//        }
//        theta = abs(theta);
        runstepperFlag = true;
    }
    else if(isCommand(data,command, "mf", 1)){
        microstepFactor = getFieldInteger(data, 1);             // Microstep factor is used for stepping the motor through each microstep.
                                                                // 90 degrees is divided into multiple microsteps for incrementing teh electrical phase that drives EN PWM signals
    }

}
void calibrateLevel(){
    //rotate ccw until the optical switch is pulled low
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring

    while(getPinValue(OPT_ENC)){

    }

}
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
        case 0:  // quadrant 1

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
void level(){
    int steps_from_level =0;            //  Watch the level of 0 degrees using a protrator, set the motor to be in that position
                                        //0-180 axis and then count the steps until the pin is hit
    while(getPinValue(OPT_ENC)){        // value goes low when the optical sensor is hit by the brass teeth
        step_cw(50000);                 // Move one full step every time
        steps_from_level++;
    }

    while(steps_from_level){
        step_ccw(50000);
    }
}


void runStepper( uint32_t waitTime_us){


    setPinValue(Motor1_EN, 1);
    setPinValue(Motor2_EN, 1);

    int full_steps = abs(theta) / 1.8;

    //Turn for these many full steps
    while(full_steps){
        if(theta >0){
            step_cw(50000);
        }
        else{
            step_ccw(50000);
        }
        full_steps--;
        theta -= 1.8;
    }

    setPinValue(Motor1_EN, 0);          // Set the enables to 0 before leaving
    setPinValue(Motor2_EN,0);

    runstepperFlag = false;

}


int main(){
    init_texas();
   // initMotorPwm();
    initTimer2();
    flash_hardware_green();
    putsUart0("##############################################################\n");
    putsUart0("Initialized Hardware. Type 'help' for commands list\n");

//Test code below
//    theta = 30;
//    runStepper(50000);                                        // cw rotation
//
//    theta = -30;
//    runStepper(50000);                                       // ccw rotation


//    TIMER2_CTL_R |= TIMER_CTL_TAEN;                               // turn-on timer to trigger timerISR

    USER_DATA data;

    while(true){
        if(kbhitUart0()){
            getsUart0(&data);                                       // get the input data from putty
            parseFields(&data);                                     // parse the putty data and check for individual commands below
            shell(&data);
        }

        if(runstepperFlag){
            runStepper(50000);
        }
    }
}
