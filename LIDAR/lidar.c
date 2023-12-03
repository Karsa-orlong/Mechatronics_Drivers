
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

#define MOTOR_CTL		PORTE,1;

#define OPCODE				0xA5
#define RESP_DESC			0x5A

#define STOP 				0x25
#define RESET				0x40
#define SCAN				0x20
#define GET_INFO			0x50

#define WAIT_FOR_RESPONSE	1500 // 1ms
#define WAIT_FOR_RESET_COMP	2500

uint8_t device_info[20];
uint8_t response_data1,response_data2,response_data3,response_data4,response_data5;

typedef struct _responsePacket
{
	uint8_t quality;
	uint8_t angle;
	uint8_t distance;
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
	waitMicrosecond(WAIT_FOR_RESPONSE);
}




int main(void)
{
	init_texas();

	setPinValue(MOTOR_CTL,1);

	sendCommand(STOP);
	sendCommand(SCAN);

	respPackets packets[1000];
	bool accumulated = false;

	while(true){
		uint16_t i;
		while(i<1000 && !accumulated){
			response_data1 = getcUart1();
			packets[i].quality = response_data1;

			response_data2 = getcUart1();
			response_data2  = (response_data2 & 0x0F);

			response_data3 = getcUart1();
			response_data3  = ((response_data3 <<8)&0xFF00);

			packets[i].angle = response_data2|response_data3;

			
			response_data4 = getcUart1();
			response_data4  = (response_data4 & 0x0F);

			response_data5 = getcUart1();
			response_data5  = ((response_data5 <<4)&0xF0);

			packets[i].distance = response_data4|response_data5;
			
			i++;
		}
		if(i==1000){
			accumulated = true;
		}
		if(accumulated){
			char msg[100];
			snprintf(msg,sizeof(msg),)
			putsUart0()
		}

	}

}
