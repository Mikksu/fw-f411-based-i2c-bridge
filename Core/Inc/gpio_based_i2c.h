#ifndef __GPIO_BASED_I2C_H
#define __GPIO_BASED_I2C_H

#include <stdint.h>
#include "gpio.h"
#include "bitband.h"


int I2C_Master_MemWrite	(uint8_t slaveAddress, uint8_t length, uint8_t *data);
int I2C_Master_MemRead	(uint8_t slaveAddress, uint8_t txLen, uint8_t *txData, uint8_t rxLen, uint8_t *rxData);


#endif

