/*
 * i2c1.h
 *
 *  Created on: Oct 31, 2023
 *      Author: giskusama
 */

#ifndef I2C1_H_
#define I2C1_H_


#include <stdint.h>
#include <stdbool.h>

void initI2c1(void);
// For simple devices with a single internal register
void writeI2c1Data(uint8_t add, uint8_t data);
uint8_t readI2c1Data(uint8_t add);

// For devices with multiple registers
void writeI2c1Register(uint8_t add, uint8_t reg, uint8_t data);
void writeI2c1Registers(uint8_t add, uint8_t reg, const uint8_t data[], uint8_t size);
uint8_t readI2c1Register(uint8_t add, uint8_t reg);
void readI2c1Registers(uint8_t add, uint8_t reg, uint8_t data[], uint8_t size);

// General functions
bool pollI2c1Address(uint8_t add);
bool isI2c1Error(void);


#endif /* I2C1_H_ */
