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
#define SDA_MODE_INPUT()    GPIOB->MODER &= ~GPIO_MODER_MODER11; // Clear MODER11 bits to set to Input (00)
#define SDA_MODE_OUTPUT()   GPIOB->MODER |= GPIO_MODER_MODER11_0;
#define SDA_READ	((GPIOB->IDR & GPIO_IDR_ID11) != 0)
#define SCL_READ 	((GPIOB->IDR & GPIO_IDR_ID10) != 0)

#define I2C_STANDARD_MODE
//#define I2C_FAST_MODE

#ifdef I2C_STANDARD_MODE
#define t_LOW		5 		//min 4.7us
#define t_HIGH		4 		//min 4us
#define t_SU_STA	5 		//min 4.7us
#define t_HD_STA	4 		//min 4us
#define t_SU_STO	4 		//min 4us
#define t_BUF		5 		//min 4.7us
#define t_SU_DAT	1 		//min 250ns 	-> dùng DWT_Delay_cycles(1)

#else
#define t_LOW		1		//min 1300ns	-> dùng DWT_Delay_us(1) do nó thực tế là delay 1.3us
#define t_HIGH		60 		//min 600ns 	-> dùng DWT_Delay_cycles(60)
#define t_SU_STA	60 		//min 600ns 	-> dùng DWT_Delay_cycles(60)
#define t_HD_STA	60 		//min 600ns 	-> dùng DWT_Delay_cycles(60)
#define t_SU_STO	60 		//min 600ns 	-> dùng DWT_Delay_cycles(60)
#define t_BUF		1 		//min 1300ns	-> dùng DWT_Delay_us(1) do nó thực tế là delay 1.3us
#define t_SU_DAT	1 		//min 100ns 	-> dùng DWT_Delay_cycles(1)
#endif


void I2C_Start();
void I2C_Stop();
uint8_t I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint8_t write_stop);
uint8_t I2C_Master_Receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
#endif /* INC_I2C_BITBANGING_H_ */
