
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "uart1.h"
#include "gpio.h"

#define RED_LED         PORTF,1
#define BLUE_LED        PORTF,2
#define GREEN_LED       PORTF,3

#define MOTOR_CTL		PORTE,1

#define OPCODE				0xA5
#define RESP_DESC			0x5A

#define STOP 				0x25
#define RESET				0x40
#define SCAN				0x20
#define GET_INFO			0x50

#define WAIT_FOR_RESPONSE	1500 // 1ms
#define WAIT_FOR_RESET_COMP	2500

uint8_t device_info[20];
uint16_t response_data1,response_data2,response_data3,response_data4,response_data5;
bool printdata = false;
bool accumulated = false;


typedef struct _responsePacket
{
	uint8_t quality;
	uint16_t angle;
	uint16_t distance;
}respPackets;

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
    enablePort(PORTF);// LEDS & PUSHBUTTONS
    enablePort(PORTE);

    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);
    selectPinPushPullOutput(BLUE_LED);
	selectPinPushPullOutput(MOTOR_CTL);

    initUart0();
    setUart0BaudRate(115200, 40e6);                         // Set baud rate for UART

    initUart1();
    setUart1BaudRate(115200, 40e6);                         // Set baud rate for UART


}

void sendCommand(uint8_t command){
	
	// Send a STOP command
	putcUart1(OPCODE);
	putcUart1(command);
}

void ClearLidarBuffer(){
    while(kbhitUart1()){
        getcUart1();                        // Clear the UART buffer
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

            if (strcmp(token, "print") == 0) {
                    token = strtok(NULL, " ");                                  // Get the next token (angle)
                    printdata = true;
            }

            if (strcmp(token, "scan") == 0) {
                token = strtok(NULL, " ");                                  // Get the next token (angle)
                accumulated = false;
                sendCommand(SCAN);
                waitMicrosecond(2000);                                  // Wait 2ms before sending the next request
                uint16_t i = 0;
                uint8_t descriptor[7] = {0};
                // sendCommand(GET_INFO);
                for(i=0; i<7; i++){
                    descriptor[i] = getcUart1();
                }
            }


            if (strcmp(token, "reboot") == 0) {
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }

            if (strcmp(token, "help") == 0) {
                putsUart0("##############################################################\n");
                putsUart0("Help menu -- commands list\n");
                putsUart0("print\n");
                putsUart0("reboot\n");
                putsUart0("##############################################################\n");
            }
        }
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(void)
{
	init_texas();
	respPackets packets[1000];

	setPinValue(MOTOR_CTL,1);

	waitMicrosecond(2000000);

    ClearLidarBuffer();

	sendCommand(STOP);
	waitMicrosecond(1000);									// Wait 1ms before sending the next request


	sendCommand(SCAN);
	waitMicrosecond(2000);									// Wait 2ms before sending the next request
    uint16_t i = 0;
    uint8_t descriptor[7] = {0};
	// sendCommand(GET_INFO);
	for(i=0; i<7; i++){
	    descriptor[i] = getcUart1();
	}


	char dest[20];

	//putsUart0("Quality\tAngle\tDistance\r\n");
	while(true)
	{
	    processShell();

		while(i < 1000 && !accumulated)
		{
//            waitMicrosecond(10);
			response_data1 = getcUart1();
			packets[i].quality = response_data1;


			response_data2 = getcUart1();
			response_data2  = (response_data2 >>1); // remove the check bit

			response_data3 = getcUart1();
			response_data3  = ((response_data3 <<7));

			response_data3 = response_data3 | response_data2;
			response_data3 = response_data3 >> 6;   // Convert the q6 fixed point angle to actual reading

			packets[i].angle = response_data3;


			response_data4 = getcUart1();

			response_data5 = getcUart1();
			response_data5 = (response_data5 << 8);
			response_data5 = response_data5|response_data4;
			response_data5 = response_data5 >> 2; // Convert the distance from fixed point to normal

			packets[i].distance = response_data5;

//			waitMicrosecond(10);

//			if(packets[i].distance > 0){
	            i++;
//			}
			if(i==1000){
			    putsUart0("Accumulated! Yout can print\n");
			    accumulated = true;
	            sendCommand(STOP);
	            waitMicrosecond(2000);
		    }

		}
		if(accumulated & printdata)
		{								// Wait 1ms before sending the next request

			for(i=0; i<1000 ; i++){

                  putsUart0(itoa(packets[i].quality, dest));
                  putsUart0("\t");


                  putsUart0(itoa(packets[i].angle, dest));
                  putsUart0("\t");

                  putsUart0(itoa(packets[i].distance, dest));
                  putsUart0("\t\n");

			}
//			 setPinValue(MOTOR_CTL, 0);
			 printdata = false;
			 i=0;
			// char msg[100];
			// snprintf(msg, sizeof(msg), );
		}
	}

}
