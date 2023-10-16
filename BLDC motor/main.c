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

#define COMPARATOR_A        PORTE,3
#define COMPARATOR_B        PORTE,4
#define COMPARATOR_C        PORTE,5

#define TIMER_LOAD_FREQ     1000000

uint32_t step_delay = 100000;
uint8_t state =1;
uint8_t nextState = 99; // some random number
bool open_mode = true; // Function mode
bool hall_mode = false; // isr mode
bool backemf_mode  = false;

bool comparatorCheck = false;
uint32_t elapsed_time = 0;



/*
 * State:   Corresponding value:
 *  0     ~     5
 *  1     ~     4
 *  2     ~     6
 *  3     ~     2
 *  4     ~     3
 *  5     ~     1
 * 
state	0	1	2	3	4	5
    H1	1	1	0	0	0	1
    H2	0	0	0	1	1	1
    H3	0	1	1	1	0	0
						
    A	1	1	f	0	0	f
    B	f	0	0	f	1	1
    C	0	f	1	1	f	0

 */
//uint8_t HallBased_nextState[7] = {9,3,5,4,1,2,0}; // pattern 1 slightly worse
//uint8_t HallBased_nextState[7] = {9,3,1,2,5,4,0}; // pattern 2 doesnt work
uint8_t HallBased_nextState[7] = {9,4,0,5,2,3,1}; // pattern 3 working pattern




uint8_t arr[3];
uint8_t index = 0;
uint8_t hallBits =0;

uint32_t frequency = 0;
uint32_t motor_hall_rpm =0;
uint32_t num_hall_edges = 0;
uint32_t hp =100; // hp is hall effect active percentage
uint32_t hallEffect_loadTime = 0;
uint32_t hall_activeTime =0;
uint32_t hall_inactivetime =0;

//################################################################ INIT HARDWARE FUNCTIONS
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

    selectPinDigitalInput(COMPARATOR_A);
    selectPinDigitalInput(COMPARATOR_B);
    selectPinDigitalInput(COMPARATOR_C);

    selectPinInterruptBothEdges(COMPARATOR_A);
    enablePinInterrupt(COMPARATOR_A);
    selectPinInterruptBothEdges(COMPARATOR_B);
    enablePinInterrupt(COMPARATOR_B);
    selectPinInterruptBothEdges(COMPARATOR_C);
    enablePinInterrupt(COMPARATOR_C);

    selectPinInterruptBothEdges(HallIN1);
    enablePinInterrupt(HallIN1);
    selectPinInterruptBothEdges(HallIN2);
    enablePinInterrupt(HallIN2);
    selectPinInterruptBothEdges(HallIN2);
    enablePinInterrupt(HallIN2);

	// enableNvicInterrupt(INT_GPIOE); // Enable in Backemf mode in process shell

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
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);                     // turn-on interrupt 37 (TIMER1A)
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
   	TIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer0
}

void initTimer4(){                                        
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R4;
    _delay_cycles(3);

    // Configure Timer 2 for PID controller
    TIMER4_CTL_R &= ~TIMER_CTL_TAEN;                        // turn-off timer before reconfiguring
    TIMER4_CFG_R = TIMER_CFG_32_BIT_TIMER;                  // configure as 32-bit timer (A+B)
    TIMER4_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;                 // configure for one shot mode
    TIMER4_TAILR_R = 0xFFFFFFFF;                       // set load value large load value 
    TIMER4_IMR_R = TIMER_IMR_TATOIM;                        // turn-on interrupts
    NVIC_EN2_R = 1 << (INT_TIMER4A-16 - 32*2);                     // turn-on interrupt vector 86 (TIMER4A)
//    TIMER4_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer4
}

//################################################################ OPEN LOOP COMMUTATION

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

            putsUart0("A - B f C +\n");
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

            putsUart0("A - B + C f\n");
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

            putsUart0("A f B + C -\n");
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

            putsUart0("A + B f C -\n");
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
            putsUart0("A + B - C f\n");
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
            putsUart0("A f B - C +\n");
            break;
    }
}
//################################################################ HALL EFFECT COMMUTATION

/*
state	0	1	2	3	4	5
    H1	1	1	0	0	0	1
    H2	0	0	0	1	1	1
    H3	0	1	1	1	0	0
						
    A	1	1	f	0	0	f
    B	f	0	0	f	1	1
    C	0	f	1	1	f	0

 */
void commutateClosedLoop(uint8_t case_state){
	switch(case_state){
		case 0:
			setPinValue(MotorEN1, 1); 	// A +
			setPinValue(MotorV1, 1);

			setPinValue(MotorEN2, 0); // B f
			// setPinValue(MotorV1, 1);
			
			setPinValue(MotorEN3, 1); // C -
			setPinValue(MotorV3, 0);

			// putsUart0("H0 ");
//			putsUart0("A + : B f : C - \n");
			break;

		case 1:
		
			setPinValue(MotorEN1, 1); // A +
			setPinValue(MotorV1, 1);

			setPinValue(MotorEN2, 1); // B -
			setPinValue(MotorV2, 0); 
			
			setPinValue(MotorEN3, 0); // C f
			//  setPinValue(MotorV3, 0);

			// putsUart0("H1 ");
//            putsUart0("A + : B - : C f \n");
			break;
		
		case 2:
		
			setPinValue(MotorEN1, 0); // A f
			//  setPinValue(MotorV1, 1);

			setPinValue(MotorEN2, 1); // B -
			setPinValue(MotorV2, 0);
			
			setPinValue(MotorEN3, 1); // C +
			setPinValue(MotorV3, 1);

            // putsUart0("H2 ");
//            putsUart0("A f : B - : C + \n");
			break;

		case 3:

			setPinValue(MotorEN1, 1); // A -
			setPinValue(MotorV1, 0);

			setPinValue(MotorEN2, 0); // B f
			//  setPinValue(MotorV2, 0);
			
			setPinValue(MotorEN3, 1); // C +
			setPinValue(MotorV3, 1);

            // putsUart0("H3 ");
//            putsUart0("A - : B f : C + \n");
			break;

		case 4:
		
			setPinValue(MotorEN1, 1); // A -
			setPinValue(MotorV1, 0);

			setPinValue(MotorEN2, 1); // B +
			setPinValue(MotorV2, 1); 
			
			setPinValue(MotorEN3, 0); // C f
			// setPinValue(MotorV3, 1);

            // putsUart0("H4 ");
//            putsUart0("A - : B + : C f \n");
			break;

		case 5:
		
			setPinValue(MotorEN1, 0); // A f
			//  setPinValue(MotorV1, 0);

			setPinValue(MotorEN2, 1); // B +
			setPinValue(MotorV2, 1);
			
			setPinValue(MotorEN3, 1); // C -
			setPinValue(MotorV3, 0);

            // putsUart0("H5 ");
//            putsUart0("A f : B + : C - \n");

			break;

		default:
		    putsUart0("Error\n");
	}
}



void ClosedLoopISR(){

    char msg[40];

    //waitMicrosecond(200); // wait for the motor and readings to be stabilized

    uint8_t temp_state =0;


    if(hall_mode){

        waitMicrosecond(1300); // Wait before sampling hallBits

        hallBits = (getPinValue(HallIN1) << 2) | (getPinValue(HallIN2) << 1) | (getPinValue(HallIN3) << 0);
        state = HallBased_nextState[hallBits];
    	commutateClosedLoop(state); // Commutate to next step based on case statements that are in state

        num_hall_edges ++;	// Used to determine the rpm of the motor by the Timer0_measureRpmISR
        
        hallEffect_loadTime = (uint32_t)(40e6/(12*frequency));       // Hall effect edges frequency is 12*motor_revs per second
        hall_activeTime = (uint32_t)(hp*hallEffect_loadTime/100);   // Hall effect active percentage
        TIMER1_TAILR_R = hall_activeTime;                       // start the timer to run for 1/freq * percentage amount of time
        TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer1 to make sure that the motor is active for this duration of time
    }

    if(backemf_mode){
            
        elapsed_time = TIMER4_TAV_R;                            // Get the elapsed time value

        TIMER4_CTL_R &= ~TIMER_CTL_TAEN;                        // turn-off timer before reconfiguring
        TIMER4_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;                 // reset the timer to one shot timer with a huge load value
        TIMER4_TAILR_R = elapsed_time;                          // set load value large load value 
        TIMER4_CTL_R |= TIMER_CTL_TAEN;                         // turn-on timer4
        
        state++;                                                // Increment the state and wait for an equivalent elapsed_time before commutating to next state
        comparatorCheck = true;                                 //flag to use in TImer4 ISR
        
    }

    GPIO_PORTE_ICR_R |= 0xFF;                               // Clear all port E interrupts
}


/**
 * @brief This ISR is hit after the active time for the Motor has been crossed. We de-energize the coils and exit from here
 * 
 */
void Timer1_activeISR(){
    
    hall_inactivetime = (uint32_t)((98-hp)*hallEffect_loadTime/100);   // Hall effect inactive percentage
        
    TIMER2_TAILR_R = hall_inactivetime;                         // start the timer2 to run for (T- 1/freq * percentage) amount of time
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                             // turn-on timer2 to make sure that the motor is Inactive for this duration of time

    setPinValue(MotorEN1, 0);           // Motor coils de-energized after this ISR is hit. They were active until this is hit
    setPinValue(MotorEN2, 0);
    setPinValue(MotorEN3, 0);
//    setPinValue(MotorV1, 0);
//    setPinValue(MotorV2, 0);
//    setPinValue(MotorV3, 0);

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;                                      // Clear the timer1 interrupt
}

/**
 * @brief This ISR is hit after the time of inactivity has passed. We restore the state at the end and commutate to the next one 
 * 
 */
void Timer2_sleepISR(){
    commutateClosedLoop(state); // Revert back to the state that was active before all coils were turned off for the Hall effect based commutation

    TIMER2_ICR_R = TIMER_ICR_TATOCINT;                                      // Clear the timer2 interrupt
}

/**
 * @brief Measure the rotations per second or frequency and use that to also calculate the rpm of the motor
 * Use this frequency to also check how much of the time should the Hall effect sensor be active and inactive for between each pattern change that occurs
 */
void Timer0_measureRpmISR(){

	frequency = (uint32_t)(num_hall_edges/12);	// gives the value of revolutions per second
	motor_hall_rpm = frequency*60;
	num_hall_edges =0;	// Get the num hall edges counted in th eHall effect ISR and measure the frequency every one second

	TIMER0_ICR_R = TIMER_ICR_TATOCINT;                                      // Clear the timer0 interrupt
	togglePinValue(BLUE_LED);

}

//################################################################ BACK-EMF COMMUTATION

/**
 * @brief This function follows an excitation sequence and starts a timer right after a given coil is set as a floating coil for the backemf to be measured
 * and using the back-emf comparator circuit the Timer4ISR commutates it to the next state
 * @param case_state Commutate through the motor states by activating coils according to a given excitation sequence
 */

void commutateBackEmf(uint8_t case_state){
    switch(case_state){
        case 0:
            setPinValue(MotorEN1, 1);   // A -
            setPinValue(MotorV1, 0);

            setPinValue(MotorEN2, 0); // B float
            // setPinValue();
            TIMER4_CTL_R |= TIMER_CTL_TAEN;                         // turn-on timer4


            setPinValue(MotorEN3, 1); // C +
            setPinValue(MotorV3, 1);

            break;

        case 1:

            setPinValue(MotorEN1, 1); // A -
            setPinValue(MotorV1, 0);

            setPinValue(MotorEN2, 1); // B +
            setPinValue(MotorV2, 1);

            setPinValue(MotorEN3, 0); // C float
            // setPinValue(MotorV3, 0);
            TIMER4_CTL_R |= TIMER_CTL_TAEN;                         // turn-on timer4

            break;

        case 2:

            setPinValue(MotorEN1, 0); // A float
            // setPinValue(MotorV1, 1);
            TIMER4_CTL_R |= TIMER_CTL_TAEN;                         // turn-on timer4

            setPinValue(MotorEN2, 1); // B +
            setPinValue(MotorV2, 1);

            setPinValue(MotorEN3, 1); // C -
            setPinValue(MotorV3, 0);

            break;

        case 3:

            setPinValue(MotorEN1, 1); // A +
            setPinValue(MotorV1, 1);

            setPinValue(MotorEN2, 0); // B float
            // setPinValue(MotorV2, 0);
            TIMER4_CTL_R |= TIMER_CTL_TAEN;                         // turn-on timer4

            setPinValue(MotorEN3, 1); // C -
            setPinValue(MotorV3, 0);

            break;

        case 4:

            setPinValue(MotorEN1, 1); // A +
            setPinValue(MotorV1, 1);

            setPinValue(MotorEN2, 1); // B -
            setPinValue(MotorV2, 0);

            setPinValue(MotorEN3, 0); // C float
            // setPinValue(MotorV3, 0);
            TIMER4_CTL_R |= TIMER_CTL_TAEN;                         // turn-on timer4

            break;

        case 5:

            setPinValue(MotorEN1, 0); // A float
            // setPinValue(MotorV1, 0);
            TIMER4_CTL_R |= TIMER_CTL_TAEN;                         // turn-on timer4

            setPinValue(MotorEN2, 1); // B -
            setPinValue(MotorV2, 0);

            setPinValue(MotorEN3, 1); // C +
            setPinValue(MotorV3, 1);

            break;
    }
}

/**
 * @brief This ISR is loaded with a huge load value and should not be triggered until the load value is modified by the Comparator ISR
 * This ISR commutates to the next state after comparator signal has been triggered
 */
void Timer4_BackEmfISR(){

    if(comparatorCheck){
        TIMER4_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;                 // reset the timer to one shot timer with a huge load value
        TIMER4_TAILR_R = 0xFFFFFFFF;                            // set load value large load value 
        comparatorCheck = false;
        commutateBackEmf(state);
    }
    TIMER4_ICR_R = TIMER_ICR_TATOCINT;                          // Clear the timer 4 interrupt

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
                token = strtok(NULL, " ");                                      // Get the next token 
                if (token != NULL) {
					step_delay = atoi(token);
                } 
            }

            if (strcmp(token, "open") == 0) {
                token = strtok(NULL, " ");                                      // Get the next token 
                open_mode = true;
                hall_mode = false;
                backemf_mode = false;
                disableNvicInterrupt(INT_GPIOE);
            }

            if (strcmp(token, "hall") == 0) {
                token = strtok(NULL, " ");                                      // Get the next token 
                hall_mode = true;
                open_mode = false;
                backemf_mode = false;

            }
                if (strcmp(token, "hp") == 0) {
                token = strtok(NULL, " ");                                      // Get the next token 
                if(token != NULL){
                    hp = atoi(token);
                }

            }

            if (strcmp(token, "back") == 0) {
            token = strtok(NULL, " ");                                      // Get the next token (
            hall_mode = false;
            open_mode = false;
            backemf_mode = true;
            state =0;

            enableNvicInterrupt(INT_GPIOE);                 // Turn on Interrupts from comparators
            TIMER4_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer4

            }

            if (strcmp(token, "reboot") == 0) {
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }

            if (strcmp(token, "help") == 0) {
                putsUart0("##############################################################\n");
                putsUart0("Help menu -- commands list\n");
                putsUart0("reboot\n");
				putsUart0("open .. switch to function mode\n");
				putsUart0("delay step_delay_us....enter the commutation delay for open mode\n");
				putsUart0("hall .. move to hall commutation with Hall effect sensors\n");
                putsUart0("hp percentage .. hall commutation active percentage\n");
                putsUart0("back .. move to back-emf commutation\n");
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
    initTimer4();

	putsUart0("//################################################################\n");
	putsUart0("Initialized hardware\n");

	char str[40];

	uint8_t  temp1 =50, temp2 =50, temp3  = 50;
	bool rampUpFinished = false;

    hallBits = (getPinValue(HallIN1) << 2) | (getPinValue(HallIN2) << 1) | (getPinValue(HallIN3) << 0);
    state = HallBased_nextState[hallBits];
	while(true){
		if(kbhitUart0()){
			processShell();				// Parse the inputs from putty as needed
        }

        temp1 =getPinValue(HallIN1);    // test hall effects
        temp2 = getPinValue(HallIN2);
        temp3 = getPinValue(HallIN3);
        snprintf(str, sizeof(str), "H1 H2 H3 : %d, %d, %d \n", temp1, temp2, temp3 );
//        putsUart0(str);

			if(open_mode){
				commutateOpenLoop(step_delay);
			}
			if(hall_mode){
			    if(!rampUpFinished){
                    uint32_t i = 500;
                    while(i>0){
                        commutateOpenLoop(5000); // Get the commutation done really fast
                        i--;
                    }
                    i = 500;
                    while(i>0){
                        commutateOpenLoop(3000); // Get the commutation done really fast
                        i--;
                    }
                    setPinValue(MotorEN1, 0);
                    setPinValue(MotorEN2, 0);
                    setPinValue(MotorEN3, 0);

                    waitMicrosecond(10000);
                    enableNvicInterrupt(INT_GPIOE);                 // Turn on Interrupts from comparators
                    rampUpFinished = true;                          // Open loop rampup finished. Dont execute this block in main and only commutate using ISR now
			    }
			}


	}
}
