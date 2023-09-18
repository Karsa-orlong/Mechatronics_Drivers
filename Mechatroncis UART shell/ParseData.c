/*
 * ParseData.c
 *
 *  Created on: Nov 2, 2022
 *      Author: Kalki
 *
 */



//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "ParseData.h"

//-----------------------------------------------------------------------------
//Global Variables included in header file
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Subroutines
/*
 ASCII Space 32
 BACKSPACE 127
 Carriage return  CR 13
 Line feed  LF \n 10

 */


void getsUart0(USER_DATA* data){
    int counter =0;
    char c= 'a';

        while(counter <= MAX_CHARS ){ //CR HANDLING
             c = getcUart0();

        // BACKSPACE HANDLING ASCII == 127
        if(c == 127 ){
            if(counter >0){
                counter--; // backspace entered so decrement the count
            }
        }
        else
        {
            if(counter < MAX_CHARS){
                   data->buffer[counter++] =c;
            }
                // LF HANDLING // when a LF  is entered, set buffer[count] =0 (ASCII 0 is for null terminator that indicates
               // end of string and break out of the program
            if(c == 10 || c==13 || counter == MAX_CHARS){
               data ->buffer[counter-1] = '\0'; // set null pointer
               break; // break out of for loop
           }
            }
        }
}
//-----------------------------------------------------------------------------

void parseFields(USER_DATA* data){
    int counter =0;
   // int fieldcnt =0;
    int isPrev_delimiter = 0;
    int isFirstPass =0;

    do{ //loop through the buffer array until we get a null

                                                                    // parse fields according to a,n,d (Alphabet, numeric, delimiter)
                                                                    //65-90 A-Z ; 97-122 a-z ; 0-9 digits
        if(counter == 0){
            isFirstPass =1;                                         // assume the prev is a delimiter for the start of the string
            data->fieldCount =0;

        }


        if(isPrev_delimiter ==1 || isFirstPass ==1){
            if((data->buffer[counter] >='a' && data->buffer[counter] <='z') || (data->buffer[counter] >='A' && data->buffer[counter] <='Z')){
                data->fieldType[data->fieldCount] = 'a';            // alphabet type
                data->fieldPosition[data->fieldCount] = counter;
                isPrev_delimiter =0;                                // set the flag for the next iteration to indicate that the prev character was not a delimiter
               // data->fieldCount++;                                // field count for the starting command
            }
            else if(data->buffer[counter] >='0' && data->buffer[counter] <='9'){
            data->fieldType[data->fieldCount] = 'n';                // number type
                data->fieldPosition[data->fieldCount]  = counter;   // counter is the position of the field
                isPrev_delimiter =0;
            }
        }
        else if(isPrev_delimiter == 0){                             // previous char was not a delimiter AND the current character is a delimiter
            if(!(data->buffer[counter] >='A' && data->buffer[counter] <='Z') ){
                if(!(data->buffer[counter] >='a' && data->buffer[counter] <='z')){
                    if(!(data->buffer[counter] >='0' && data->buffer[counter] <='9')){

                        isPrev_delimiter =1;                        // set the flag to true so that the next iteration knows that a delimiter was found earlier
                        data->buffer[counter] = '\0';
                        data->fieldCount++;                         // at the transition update the field count
                    }
                }

            }

        }
        counter++;
        isFirstPass =0;                                             // change flag to false isFirstPass after counter is incremented

               }while(data->buffer[counter] != '\0' );
data->fieldCount++;                                                 // count the last field before '/0'
} // end parsefields
//-----------------------------------------------------------------------------


char* getFieldString(USER_DATA* data, uint8_t fieldNumber){

    if(data->fieldType[fieldNumber] == 'a' && fieldNumber <=MAX_FIELDS){
        return &data->buffer[data->fieldPosition[fieldNumber]];      // data->fieldPosition[fieldNumber] returns the position of the field
    }
    else
        return NULLSTRING;
} // end of getFieldString
//-----------------------------------------------------------------------------

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber){
    if(data->fieldType[fieldNumber] == 'n' && fieldNumber <=MAX_FIELDS){
        int32_t result =0 ;
        int32_t digitValue;
        int count = data->fieldPosition[fieldNumber];
        while( data->buffer[count]!= '\0'){ // parse through the
            digitValue = (int)(data->buffer[count] - '0');
            result = result*10 + digitValue;
            count++;
        }
        return result;
    }
    else
        return -1;
} // end of getFieldInteger
//-----------------------------------------------------------------------------


// Function to implement custom strcmp function that is case insensitive
int strcmp_custom(const char *X, const char *Y)
{
    while(*X && *Y){    // run loop while X and Y are not hitting null terminator
                        // check if  X or Y have capital characeters and convert ascii value to lowercase if they are
       char x_lower = (*X >= 'A' && *X <= 'Z') ? (*X + 'a' - 'A') : *X;
       char y_lower = (*Y >= 'A' && *Y <= 'Z') ? (*Y + 'a' - 'A') : *Y;

       if(x_lower !=y_lower){
           break;
       }
       X++;
       Y++;
    }
                        //return the result for the comparision of the first non matching characters
                        // return 0 on a success
    return *(const unsigned char*)X - *(const unsigned char*)Y;
}

//-----------------------------------------------------------------------------


void tolower_string(char*X)
{
    while(*X){
        *X = (*X >= 'A' && *X <= 'Z') ? (*X + 'a' - 'A') : *X;
        X++;
    }
}

//-----------------------------------------------------------------------------

bool isCommand(USER_DATA* data, const char command[], const char strCommand[], uint8_t minArguments)
{
    bool flag ;                                                         // minArguments is Max_fields -1

    int ret = strcmp_custom(strCommand, command);
    if(ret == 0 && (data->fieldCount -1 >= minArguments)){              // if the string commands are the same and the min arguments are
                                                                        // not greater than MAX_FIELDS -1 1 is for the string command itself
        flag = true;
    }
    else{
        flag = false;
    }
    return flag;
}


void printFields(USER_DATA* data){
    int i;
    for (i = 0; i < data->fieldCount; i++)
                    {
                    putcUart0(data->fieldType[i]);
                    putcUart0('\t');
                    putsUart0(&data->buffer[data->fieldPosition[i]]);
                    putcUart0('\n');
                    }
}


void itoa_custom(int num, char* str)
{
    int i =0;
    int base =10;
    bool isneg =false;

    //Handle 0 explicitly
    if(num==0){
        str[i++] = '0';
        str[i] = '\0';
        return;
    }

    //Handle negative numbers
    if(num<0){
        isneg = true;
        num =-num;
    }

    //Process individual digits
    while(num!=0){
        int remainder = num%base;
        str[i++] = remainder + '0'; // Keep adding digits to the string
        num =num/base;
    }

    //Add negative sign to the negative numbers
    if(isneg){
        str[i++] = '-';
    }

    //Null terminator
    str[i] = '\0';

    //reverse the string by swapping the digits
    int start =0;
    int end = i-1;
    while(start < end){
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start ++;
        end--;
    }
}

int32_t atoi_custom(const char str[])
{
    int32_t result =0;
    int32_t digitValue =0;\
    int i=0;
    int base =10;

    while(str[i]!='\0'){
        digitValue = (int)(str[i++] - '\0');
        result = result*base + digitValue;
    }
    return result;

}

