/*
 * i2c_bitbanging.h
 *
 *  Created on: May 19, 2025
 *      Author: Admin
 */

#ifndef INC_I2C_BITBANGING_H_
#define INC_I2C_BITBANGING_H_

#include "stm32f4xx.h"
#include "microsecond_delay.h"

//SDA -> PB11
//SCL -> PB10

#define SDA_SET		GPIOB->BSRR = GPIO_BSRR_BS11
#define SDA_CLEAR	GPIOB->BSRR = GPIO_BSRR_BR11
#define SCL_SET		GPIOB->BSRR = GPIO_BSRR_BS10
#define SCL_CLEAR	GPIOB->BSRR = GPIO_BSRR_BR10


#define SDA_READ	((GPIOB->IDR & GPIO_IDR_ID11) != 0)

void I2C_Start();
void I2C_Stop();
uint8_t I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
#endif /* INC_I2C_BITBANGING_H_ */
