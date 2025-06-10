/*
 * uart_bitbanging.c
 *
 *  Created on: May 18, 2025
 *      Author: Admin
 */

#include "uart_bitbanging.h"
/** @brief  Sends an amount of data in blocking mode.

  * @note 	For simplicity sake, this will just be 8 bit, no parity UART
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be sent
  * @retval HAL status
 *
 */
void UART_Transmit(uint8_t* pData, uint16_t data_len){
	for (uint16_t i = 0; i < data_len; i++){
		//Send start bit
		TX_CLEAR;
		DWT_Delay_us(104);

		//Send 8 bits
		uint8_t current_byte = pData[i];
		for (uint8_t j = 0; j < 8; j++){
			if (current_byte & 0x01) TX_SET;
			else TX_CLEAR;
			DWT_Delay_us(104);
			current_byte >>=1;
		}

		//Send stop bit
		TX_SET;
		DWT_Delay_us(104);
	}
}

void UART_Receive(uint8_t* pData, uint16_t data_len){
	uint16_t byte_index = 0;
	while (byte_index != data_len){
		while (READ_RX);

		DWT_Delay_us(104*1.5);
		uint8_t received_byte;
		for (int i = 0; i < 8; i++){
			uint8_t received_bit = READ_RX;
			received_byte |= received_bit << i;
			DWT_Delay_us(104);
		}
        DWT_Delay_us(104);

        pData[byte_index++] = received_byte;

	}
}

void UART_Receive_String (uint8_t* pData, uint16_t data_len){
	uint16_t byte_index = 0;
	while (byte_index != data_len - 1){
		uint8_t current_byte;
		UART_Receive(&current_byte, 1);
		pData[byte_index++] = current_byte;
		UART_Transmit(&current_byte, 1);

		if (current_byte == '\r' || current_byte == '\n'){
			char newline[] = "\r\n";
			UART_Transmit((uint8_t*)newline, strlen(newline));
			pData[byte_index] = '\0';
			break;
		}
	}

}

