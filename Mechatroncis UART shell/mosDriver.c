/*
 * mosDriver.c
 *
 *  Created on: Sep 8, 2023
 *      Author: eddia
 */
// Stop Go C Example (Basic)
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
//
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "uart0.h"
#include "wait.h"
//#include "EdTools.h"
#include "ParseData.h"


#define MOS_DRIVER  PORTC,5
#define RED_LED     PORTF,1
#define GREEN_LED   PORTF,3

uint32_t time_energize = 100000;
uint32_t time_falling_emperical =250000;

char msg[100];

void init_mos(){
    initSystemClockTo40Mhz();
    enablePort(PORTC);
    enablePort(PORTF);

    selectPinPushPullOutput(MOS_DRIVER); // digital output
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);

    initUart0();
    setUart0BaudRate(115200, 40e6);

}

void flash_hardware_green(){
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(100000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(100000);
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(100000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(100000);
}
void toggleSwitch(uint32_t time_ener, uint32_t time_falling_emp){

    setPinValue(MOS_DRIVER, 1);
    waitMicrosecond(time_ener);
    setPinValue(MOS_DRIVER, 0);
    waitMicrosecond(time_falling_emp);

//    snprintf(msg, sizeof(msg), "ontime: %d falltime ms: %d", time_ener, time_falling_emp);
//    putsUart0(msg);

}


void shell(USER_DATA* data ){

    char *command = getFieldString(data, 0);    // get the first command
    tolower_string(command);                    // make command into lowercase before comparision. Strcmp also only checks for case insensitive comparision


    if(isCommand(data,command, "help", 0)){
        putsUart0("##############################################################\n");
        putsUart0("Help menu -- commands list\n");
        putsUart0("reboot\n");
        putsUart0("set ontime falltime in ms\n");
        putsUart0("##############################################################\n");
    }

    else if(isCommand(data,command, "reboot", 0)){
        putsUart0("Rebooting...");
        NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
    }
    else if(isCommand(data,command, "set", 2)){

        time_energize = getFieldInteger(data, 1);
        time_energize *= 1000; // convert to us
        time_falling_emperical = getFieldInteger(data, 2);
        time_falling_emperical *= 1000;


    }
    else{
        putsUart0("Please give valid inputs. Check 'help'\n");
    }
    putsUart0("\n");

}

int main(){

    init_mos();
    flash_hardware_green();

    putsUart0("##############################################################\n");
    putsUart0("Initialized Hardware. Type 'help' for commands list\n");

    USER_DATA data;

    while(true){
        if(kbhitUart0()){
            getsUart0(&data); // get the input data from putty
            parseFields(&data); // parse the putty data and check for individual commands below
            shell(&data);
            putsUart0("kbhit\n");
        }
        toggleSwitch(time_energize, time_falling_emperical);

    }

}



