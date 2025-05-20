/*
 * 24C256.h
 *
 *  Created on: Apr 29, 2025
 *      Author: Admin
 */

#ifndef INC_24C256_H_
#define INC_24C256_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#define EEPROM_I2C_ADDR		0xA0
#define EEPROM_PAGE_SIZE	64
#define EEPROM_PAGE_NUM		256
#define UART_RX_BUFFER_SIZE	500


HAL_StatusTypeDef EEPROM_random_read_hal(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len);
HAL_StatusTypeDef EEPROM_byte_write_hal(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len);
HAL_StatusTypeDef EEPROM_write_string_hal(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len);
void uart_get_string_hal(char* rxBuffer);
void uart_send_string_hal(char* str);
void UART_RxCplt_Callback(uint8_t current_byte);
void process_UART_command();


#endif /* INC_24C256_H_ */
