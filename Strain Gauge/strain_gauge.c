

/**
 * main.c
 */

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
#include "nvic.h"
#include "gpio.h"

#define RED_LED         PORTF,1
#define BLUE_LED        PORTF,2
#define GREEN_LED       PORTF,3

#define PD_SCK          PORTE,1
#define DOUT_HX711      PORTE,2

#define THRESH          8000


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

    selectPinPushPullOutput(PD_SCK);
    selectPinDigitalInput(DOUT_HX711);

    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);
    initUart0();
    setUart0BaudRate(115200, 40e6);                         // Set baud rate for UART

    selectPinPushPullOutput(BLUE_LED);

}


int32_t readData_HX711(){
    int32_t data_in =0;
    int i=0;
    setPinValue(BLUE_LED, 1);
    while(getPinValue(DOUT_HX711)); // Block it until the DATA OUT from HX711 reads low

    for (i =23; i>=0; i--){          // read 24 data bits MSB first
    setPinValue(PD_SCK, 1);
    waitMicrosecond(1);
    data_in |= (getPinValue(DOUT_HX711))<<i;
    setPinValue(PD_SCK,0);
    waitMicrosecond(1);
    }
    setPinValue(PD_SCK,1);
    data_in = data_in^0x800000; // Handle 2s compliment
    waitMicrosecond(1);
    setPinValue(PD_SCK,0);
    setPinValue(BLUE_LED, 0);

    return data_in;

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



            if(strcmp(token, "show") == 0){

                char str[100];
//                snprintf(str, sizeof(str), "frequency optical: %d  backemf : %f  PWM_compare: %d",  frequency, backemf, pwm_cmpVal);
                putsUart0(str);
                putsUart0("\n\n");
            }

            if (strcmp(token, "reboot") == 0) {
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }

            if (strcmp(token, "help") == 0) {
                putsUart0("##############################################################\n");
                putsUart0("Help menu -- commands list\n");
//                putsUart0("show .. Show current backemf, frequency, pwmcmpval\n");
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
    flash_hardware_green();
    putsUart0("\nInitialized Hardware\n");


    char msg[100];

    int64_t sum =0;
    int32_t dataArr[16];
    uint8_t index =0;
    uint8_t i;

    int32_t data =0;
    int32_t data_fir =0;
    int32_t dataLast =0;
    int32_t data_diff =0;

    //Threshold variables
//    int32_t threshold = 10000; // Detect new samples only when the difference between samples has crept up to this threshold value

    int64_t sum_detected =0;
    int32_t dataArr_detected[10];
    uint8_t id_detected =0;
    uint8_t j;

    int32_t data_detected =0;
    int32_t dataLast_detected =0;
    int32_t data_fir_detected =0;
    int32_t data_diff_detected =0;

    float mass;

    bool flag = false;

    for (i=0; i<16; i++){
        dataArr[i] =0;
    }

    for (i=0; i<10; i++){
        dataArr_detected[i] =0;
    }

//waitMicrosecond(5000000); // WAit for the sensor to calibrate

    while(true){
        if(kbhitUart0()){
            processShell();
        }
        data = readData_HX711();    // read the new sample value
        dataLast = data_fir;       // save the old fir sample as the last sample value
        data_diff = data - dataLast;
        data_diff = abs(data_diff);

        sum -= dataArr[index]; // FIR filter
        sum += data;
        dataArr[index] = data;
        index = (index + 1) & 15;
        data_fir = sum >> 4;

        flag = data_diff > THRESH;

        if(flag){
            //save the current data_fir sample in dataLast
            dataLast_detected = dataLast;
            //wait for 1000 us and take 10 new samples and take 10 new samples and average it
            waitMicrosecond(1000);

            for(j=0; j<10; j++){
                data_detected = readData_HX711();    // read the new sample value
                sum_detected -= dataArr_detected[id_detected]; // FIR filter
                sum_detected += data_detected;
                dataArr_detected[id_detected] = data_detected;
                id_detected = (id_detected + 1) %10;
                data_fir_detected = (int32_t)(sum_detected/10);
            }
            //Measure the difference between the avg and saved previous sample
            data_diff_detected = data_fir_detected - dataLast_detected;
            mass = 0.07963*((float)abs(data_diff_detected)) - 700.3;

            putsUart0("\n#####################################\n");
            snprintf(msg, sizeof(msg), "MASS %f ,diff_d: %d", mass, abs(data_diff_detected));
            putsUart0(msg);
            putsUart0("\n#####################################\n\n");

            waitMicrosecond(10);
        }
        snprintf(msg, sizeof(msg), "fir %d ,diff: %d\n", data_fir, data_diff);
        putsUart0(msg);
    }
}
