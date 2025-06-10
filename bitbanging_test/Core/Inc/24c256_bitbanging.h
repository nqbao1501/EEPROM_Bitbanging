/*
 * 24C256.h
 *
 *  Created on: Apr 29, 2025
 *      Author: Admin
 */

#ifndef INC_24C256_H_
#define INC_24C256_H_

#include "stm32f4xx_hal.h"
#include "i2c_bitbanging.h"
#include "uart_bitbanging.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define EEPROM_I2C_ADDR		0xA0
#define EEPROM_PAGE_SIZE	64
#define EEPROM_PAGE_NUM		256
#define UART_RX_BUFFER_SIZE	500

uint8_t EEPROM_random_read(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len);
uint8_t EEPROM_byte_write(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len);
uint8_t EEPROM_write_string(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len);

void UART_Receive_String_Blocking(void);
void process_UART_command();


#endif /* INC_24C256_H_ */
