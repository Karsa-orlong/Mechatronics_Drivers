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
#include "i2c0.h"
#include "i2c1.h"


#define GREEN_LED	PORTF,3
#define BLUE_LED	PORTF,2
#define RED_LED		PORTF,1

#define ADS_ADDR_GND 0x48	// 7 bit address that will be left shifted later


void init_texas(){
	initSystemClockTo40Mhz();
	enablePort(PORTF);

    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);

	initUart0();
	setUart0BaudRate(115200, 40e6);

	initI2c0();
	initI2c1();

    setPinValue(GREEN_LED, 1);
    waitMicrosecond(10000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(10000);
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(10000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(10000);
}

void init_ADS(){
	// 0.512V range in config register >> LSB size = 15.625 uV [PGA]
	//Output data rate DR[] / sample rate in cont conversion mode
	//MODE for continous conversion  = 0 >> result of conversion is conversion register

}
void readTMP36(){

}

void readADS(){

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
                // deint_flag = true;
                token = strtok(NULL, " ");                                  // Get the next token (angle)
                    if (token != NULL) {
                        // step_delay_us = atoi(token);
                }
            }
            if(strcmp(token, "cont") == 0){
                // continuos_deint = true;

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


//################################################################



/**
 * main.c
 */
int main(void)
{
    putsUart0("####################################################################\n");
    putsUart0("Initialzed Hardware\n");

	while(true){
		processShell();
	}



}
