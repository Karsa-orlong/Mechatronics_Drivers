/*
 * ParseData.h
 *
 *  Created on: Nov 2, 2022
 *      Author: Kalki
 *      Methods:
 *      getsUart0 >> get the data from putty and store in a buffer
 *      parseFields >> Parse the data and store the fields, field positions of command, arguments
 *      getFieldInteger
 *      getFieldString >> extract string and integer arguments
 *      strcmp_custom >> string compare custom implentation which is case insensitive
 *      tolower_string >> make the string a lowercase version
 *      isCommand >> check if a command is valid or not
 *      itoa_custom >> Convert integer(base =10) to string
 *      atoi_custom >> Convert a string to a base 10 integer
 */

#ifndef PARSEDATA_H_
#define PARSEDATA_H_

#define MAX_CHARS 80
#define MAX_FIELDS 5

#define NULLSTRING ""

typedef struct _USER_DATA {
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
    } USER_DATA;

//Input and Parse fields functions
void getsUart0(USER_DATA* data);
void parseFields(USER_DATA* data);

//Extract args
char* getFieldString(USER_DATA* data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber);
bool isCommand(USER_DATA* data, const char command[], const char strCommand[], uint8_t minArguments);

//String functions
void tolower_string(char*X);
int strcmp_custom(const char *X, const char *Y);

void itoa_custom(int num, char* str);
int32_t atoi_custom(const char str[]);


//Debug function to print out the buffer
void printFields(USER_DATA* data);

#endif /* PARSEDATA_H_ */
