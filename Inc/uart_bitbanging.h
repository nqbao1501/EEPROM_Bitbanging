/*
 * uart_bitbanging.h
 *
 *  Created on: May 18, 2025
 *      Author: Admin
 */

#ifndef INC_UART_BITBANGING_H_
#define INC_UART_BITBANGING_H_

#include "stm32f4xx.h"
#include "microsecond_delay.h"
#include <string.h>

#define TX_SET		GPIOA->BSRR = GPIO_BSRR_BS2
#define TX_CLEAR	GPIOA->BSRR = GPIO_BSRR_BR2

#define READ_RX 	((GPIOA->IDR & GPIO_IDR_ID3) != 0)

void UART_Transmit(uint8_t* pData, uint16_t data_len);
void UART_Receive(uint8_t* pData, uint16_t data_len);
void UART_Receive_String(uint8_t* pData, uint16_t data_len);

#endif /* INC_UART_BITBANGING_H_ */
