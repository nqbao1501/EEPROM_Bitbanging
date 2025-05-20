/*
 * i2c_bitbanging.c
 *
 *  Created on: May 19, 2025
 *      Author: Admin
 */

#include "i2c_bitbanging.h"

void I2C_Start(){
	SCL_SET;
	SDA_SET;
	DWT_Delay_us(5);
	SDA_CLEAR;
	DWT_Delay_us(5);
	SCL_CLEAR;
	DWT_Delay_us(5);

}
void I2C_Stop(){
	SDA_CLEAR;
	DWT_Delay_us(5);

	SCL_SET;
	DWT_Delay_us(5);
	SDA_SET;
	DWT_Delay_us(5);
}

static uint8_t I2C_Write_Byte(uint8_t data){
	for (int i = 7; i >= 0; i++){
		if (data & (1<<i))SDA_SET;
		else SDA_CLEAR;
		DWT_Delay_us(1); // t_SU;DAT

		SCL_SET;
		DWT_Delay_us(5);
		SCL_CLEAR;
		DWT_Delay_us(5);
	}

	//poll for ark from slave
	SDA_SET;
	DWT_Delay_us(1);
	SCL_SET;
	DWT_Delay_us(5);
	uint8_t ack = !(SDA_READ);
	SCL_CLEAR;
	DWT_Delay_us(5);
	return ack;

}
uint8_t I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size){
	//Start condition
	I2C_Start();
	//Device address
	if (!I2C_Write_Byte(DevAddress)){
		I2C_Stop();
		return 0;
	}
	//Writing Data
	for (uint16_t i = 0; i < Size; i++){
		if(!I2C_Write_Byte(pData[i])){
			I2C_Stop();
			return 0;
		}
	}
	//Stop condition
	I2C_Stop();
	return 1;
}
