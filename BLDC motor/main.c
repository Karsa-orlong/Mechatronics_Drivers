/**
 * @file main.c
 * @author Kalki
 * @brief This project is for commutating a BLDC motor with open loop and Hall effect sensors and back emf commutation techniques.
 * Hardware used is TM4C123GH6PM and BLDC motor with 12 electrical steps
 * @version 0.1
 * @date 2023-10-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */



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


// Pins
//Hardware init flash
#define GREEN_LED           PORTF,3

//Thread indicator LEDS
#define RED_LED             PORTF,1
#define BLUE_LED            PORTF,2


#define MotorEN1  			PORTB,6         // 0,1 Motor Motor1 H-bridge control , M0PWM0,1
#define MotorEN2  			PORTB,7
#define MotorEN3  			PORTB,4         // 4,5 Port B Motor 2 H-bridge control, M0PWM2,3

#define MotorV1				PORTC,4
#define MotorV2				PORTC,5
#define MotorV3				PORTC,6

#define HallIN1				PORTE,0			// Brown pin
#define HallIN2				PORTE,1 		// Blue pin
#define HallIN3				PORTE,2			// Orange--------------------- pin

#define HallMASK1			(1<<0)
#define HallMASK2			(1<<1)
#define HallMASK3			(1<<2)

#define TIMER_LOAD_FREQ     1000000

uint32_t step_delay = 1000000;
uint8_t state =0;
uint8_t nextState = 99; // some random number
bool open_mode = false; // Function mode
bool hall_mode = true; // isr mode



/*
 * State:   Corresponding value:
 *  0     ~     5
 *  1     ~     4
 *  2     ~     6
 *  3     ~     2
 *  4     ~     3
 *  5     ~     1
 */
uint8_t HallBased_nextState[7] = {99,0,4,5,2,1,3 };

uint32_t frequency = 0;
uint32_t motor_hall_rpm =0;
uint32_t num_hall_edges = 0;

/**
 * @brief The function "init_texas" initializes clocks, enables ports, configures LED and pushbutton pins, and
 * initializes UART0 with a baud rate of 115200.
 */
void init_texas(){
    initSystemClockTo40Mhz();   														// Initialize clocks

    // Enable clocks
    enablePort(PORTF);
	enablePort(PORTE);
	enablePort(PORTC);
    enablePort(PORTB);  																// Enable the Port B and the NVIC int for the INT pin of MPU 6050

    // Configure LED and pushbutton pins
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(BLUE_LED);

	selectPinPushPullOutput(MotorEN1);					// Configure the Enables and the voltage levels of the pins as outputs
	selectPinPushPullOutput(MotorEN2);	
	selectPinPushPullOutput(MotorEN3);

	selectPinPushPullOutput(MotorV1);
	selectPinPushPullOutput(MotorV2);
	selectPinPushPullOutput(MotorV3);

	selectPinDigitalInput(HallIN1);						// Configure the hall effect sensors as inputs and make them trigger the GPIO interrupts for Hall effect commutation
	selectPinDigitalInput(HallIN2);
	selectPinDigitalInput(HallIN3);

//	enablePinPullup(HallIN1);
//	enablePinPullup(HallIN2);
//	enablePinPullup(HallIN3);

//	selectPinInterruptFallingEdge(HallIN1);					// Configure interrupts for the Hall effect sensors
//	selectPinInterruptFallingEdge(HallIN2);
//	selectPinInterruptFallingEdge(HallIN3);
//
//	enablePinInterrupt(HallIN1);
//	enablePinInterrupt(HallIN2);
//	enablePinInterrupt(HallIN3);

//	enableNvicInterrupt(INT_GPIOE);

	initUart0();
	setUart0BaudRate(115200, 40e6);

}

/**
 * @brief Configure Timer1 and Timer 2 as interrupts that will control the Hall effect commutation 
 * They are configured as 32 bit narrow timers that are in One shot mode
 * 
 */
void initTimer1(){                                          
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Configure Timer 2 for PID controller
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                        // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;                  // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;                 // configure for one shot mode
    TIMER1_TAILR_R = TIMER_LOAD_FREQ;                       // set load value for interrupt rate of 1000hz
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                        // turn-on interrupts
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);                     // turn-on interrupt 37 (TIMER2A)
//    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer1
}

void initTimer2(){                                          
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    _delay_cycles(3);

    // Configure Timer 2 for PID controller
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                        // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;                  // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;                 // configure for one shot mode
    TIMER2_TAILR_R = TIMER_LOAD_FREQ;                       // set load value for interrupt rate of 1000hz
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                        // turn-on interrupts
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);                     // turn-on interrupt 37 (TIMER2A)
//    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer2
}

void initTimer0(){                                          
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
    _delay_cycles(3);

    // Configure Timer 2 for PID controller
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                        // turn-off timer before reconfiguring
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;                  // configure as 32-bit timer (A+B)
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;                 // configure for one shot mode
    TIMER0_TAILR_R = 40000000;                       // set load value for interrupt rate of 1hz to measure teh rpm of the motor
    TIMER0_IMR_R = TIMER_IMR_TATOIM;                        // turn-on interrupts
    NVIC_EN0_R = 1 << (INT_TIMER0A-16);                     // turn-on interrupt 37 (TIMER2A)
   	TIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer2
}

void commutateClosedLoop(uint8_t hallEff_nextState){
	switch(hallEff_nextState){
		case 0:
			setPinValue(MotorEN1, 1); 	// A -
			setPinValue(MotorV1, 1);

			setPinValue(MotorEN2, 1); // B float
			setPinValue(MotorV1, 0);
			// setPinValue(); 
			
			setPinValue(MotorEN3, 1); // C + 
			setPinValue(MotorV3, 0);
			putsUart0("A - : B f : C + \n");
			break;

		case 1:
		
			setPinValue(MotorEN1, 1); // A - 
			setPinValue(MotorV1, 1);

			setPinValue(MotorEN2, 1); // B +
			setPinValue(MotorV2, 1); 
			
			setPinValue(MotorEN3, 0); // C float
			// setPinValue(MotorV3, 0);
            putsUart0("A - : B + : C f \n");
			break;
		
		case 2:
		
			setPinValue(MotorEN1, 0); // A float
			// setPinValue(MotorV1, 1);

			setPinValue(MotorEN2, 1); // B + 
			setPinValue(MotorV2, 1); 
			
			setPinValue(MotorEN3, 1); // C -
			setPinValue(MotorV3, 0);
            putsUart0("A f : B - : C - \n");
			break;

		case 3:

			setPinValue(MotorEN1, 1); // A +
			setPinValue(MotorV1, 1);

			setPinValue(MotorEN2, 0); // B float 
			// setPinValue(MotorV2, 0); 
			
			setPinValue(MotorEN3, 1); // C -
			setPinValue(MotorV3, 0);
            putsUart0("A + : B f : C - \n");
			break;

		case 4:
		
			setPinValue(MotorEN1, 0); // A +

			setPinValue(MotorEN2, 1); // B -
			setPinValue(MotorV2, 0); 
			
			setPinValue(MotorEN3, 0); // C float
            putsUart0("A + : B - : C f \n");
			break;

		case 5:
		
			setPinValue(MotorEN1, 0); // A float
			// setPinValue(MotorV1, 0);

			setPinValue(MotorEN2, 1); // B -
			setPinValue(MotorV2, 0); 
			
			setPinValue(MotorEN3, 1); // C +
			setPinValue(MotorV3, 1);
            putsUart0("A f : B - : C + \n");

			break;
	}
}

void commutateOpenLoop(uint32_t step_delay_us){
    switch(state){
        case 0:
            setPinValue(MotorEN1, 1);   // A -
            setPinValue(MotorV1, 0);

            setPinValue(MotorEN2, 0); // B float
            // setPinValue();

            setPinValue(MotorEN3, 1); // C +
            setPinValue(MotorV3, 1);

			state = (state + 1)%6;
            waitMicrosecond(step_delay_us);
//          putsUart0("0");
            break;

        case 1:

            setPinValue(MotorEN1, 1); // A -
            setPinValue(MotorV1, 0);

            setPinValue(MotorEN2, 1); // B +
            setPinValue(MotorV2, 1);

            setPinValue(MotorEN3, 0); // C float
            // setPinValue(MotorV3, 0);

			state = (state + 1)%6;
            waitMicrosecond(step_delay_us);
//            putsUart0("1");
            break;

        case 2:

            setPinValue(MotorEN1, 0); // A float
            // setPinValue(MotorV1, 1);

            setPinValue(MotorEN2, 1); // B +
            setPinValue(MotorV2, 1);

            setPinValue(MotorEN3, 1); // C -
            setPinValue(MotorV3, 0);

			state = (state + 1)%6;
            waitMicrosecond(step_delay_us);
//            putsUart0("2");
            break;

        case 3:

            setPinValue(MotorEN1, 1); // A +
            setPinValue(MotorV1, 1);

            setPinValue(MotorEN2, 0); // B float
            // setPinValue(MotorV2, 0);

            setPinValue(MotorEN3, 1); // C -
            setPinValue(MotorV3, 0);

			state = (state + 1)%6;
            waitMicrosecond(step_delay_us);
            break;

        case 4:

            setPinValue(MotorEN1, 1); // A +
            setPinValue(MotorV1, 1);

            setPinValue(MotorEN2, 1); // B -
            setPinValue(MotorV2, 0);

            setPinValue(MotorEN3, 0); // C float
            // setPinValue(MotorV3, 0);

			state = (state + 1)%6;
            waitMicrosecond(step_delay_us);
            break;

        case 5:

            setPinValue(MotorEN1, 0); // A float
            // setPinValue(MotorV1, 0);

            setPinValue(MotorEN2, 1); // B -
            setPinValue(MotorV2, 0);

            setPinValue(MotorEN3, 1); // C +
            setPinValue(MotorV3, 1);

			state = (state + 1)%6;
            waitMicrosecond(step_delay_us);
            break;
    }
}


void HallEffectCommutate(){
	// Figure out which hall effect caused the interrupt
	uint8_t hallBits =0;

    // if(GPIO_PORTE_MIS_R & HallMASK1) {        // Interrupt on the first hall sensor
	// 	hallBits |= 1<<2;
    // }

	hallBits = (getPinValue(HallIN1) << 2) | (getPinValue(HallIN2) << 1) | (getPinValue(HallIN3) << 0);
	nextState = HallBased_nextState[hallBits];

//	if(nextState != state){ // Commutate only when hall effect bits change
	commutateClosedLoop(nextState);
	state = nextState;      // Save the state to compare with in next transition
//	}

	//get the motor phase from the hall effect sensor 
	//Decide on the next commutation step
	//Switch timer ON for the next step
	//Swtch timer OFF where all pins are zero for a while 

	num_hall_edges ++;	// Used to determine the rpm of the motor by the Timer0_measureRpmISR


}

void Timer1_activeISR(){
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;                                      // Clear thetimer1 interrupt
}


void Timer2_sleepISR(){
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;                                      // Clear thetimer2 interrupt
}

void Timer0_measureRpmISR(){

	frequency = (uint32_t)(num_hall_edges/12);	// gives the value of revolutions per second
	motor_hall_rpm = frequency*60;
	num_hall_edges =0;	// Get the num hall edges counted in th eHall effect ISR and measure the frequency every one second

	TIMER0_ICR_R = TIMER_ICR_TATOCINT;                                      // Clear thetimer0 interrup
	togglePinValue(BLUE_LED);

}

//################################################################ Shell for commands from putty below*
#define MAX_CHARS 80
uint8_t count = 0;
char strInput[MAX_CHARS + 1];
char* token;
/**
 * @brief The function "processShell" reads input from the UART and processes different commands based on the
 * input.
 */
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

            if (strcmp(token, "delay") == 0) {
                token = strtok(NULL, " ");                                      // Get the next token (angle)
                if (token != NULL) {
					step_delay = atoi(token);
                } 
            }

            if (strcmp(token, "open") == 0) {
                token = strtok(NULL, " ");                                      // Get the next token (angle)
                open_mode = true;
                hall_mode = false;
            }

            if (strcmp(token, "hall") == 0) {
                token = strtok(NULL, " ");                                      // Get the next token (angle)
                hall_mode = true;
                open_mode = false;

            }

            if (strcmp(token, "reboot") == 0) {
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }

            if (strcmp(token, "help") == 0) {
                putsUart0("##############################################################\n");
                putsUart0("Help menu -- commands list\n");
                putsUart0("reboot\n");
				putsUart0("func .. switch to function mode\n");
				putsUart0("delay step_delay_us....enter the commutation delay for function mode\n");
				putsUart0("isr .. move to ISR commutation with Hall effect sensors\n");
                putsUart0("##############################################################\n");
            }
        }
    }
}



//################################################################
//Main below
//################################################################

int main(){
	init_texas();
	initTimer0();
	initTimer1();
	initTimer2();
//	initMotorPwm();

	putsUart0("//################################################################\n");
	putsUart0("Initialized hardware\n");

	uint8_t  temp1 =50, temp2 =50, temp3  = 50;


	while(true){
		if(kbhitUart0()){
			processShell();				// Parse the inputs from putty as needed
        }

			if(open_mode){
				commutateOpenLoop(step_delay);
			}
			if(hall_mode){
			    HallEffectCommutate();
			}

			temp1 =getPinValue(HallIN1);
			temp2 = getPinValue(HallIN2);
			temp3 = getPinValue(HallIN3);

	}
}
