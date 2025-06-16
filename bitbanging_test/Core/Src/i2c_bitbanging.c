/*
 * i2c_bitbanging.c
 *
 *  Created on: May 19, 2025
 *      Author: Admin
 */
#include "i2c_bitbanging.h"

#ifdef I2C_STANDARD_MODE
void I2C_Start(){
	SDA_MODE_OUTPUT();
	SCL_SET;
	//DWT_Delay_us(1);
	SDA_SET;
	DWT_Delay_us(t_SU_STA);
	SDA_CLEAR;
	DWT_Delay_us(t_HD_STA);
	SCL_CLEAR;
	DWT_Delay_us(t_LOW);
}
void I2C_Stop(){
	SDA_MODE_OUTPUT();
	SDA_CLEAR;
	SCL_SET;
	DWT_Delay_us(t_SU_STO);
	SDA_SET;
	DWT_Delay_us(t_BUF);
}

static uint8_t I2C_Write_Byte(uint8_t data){
	//Gửi từng bit trong byte. Thứ tự bit là MSB đến LSB (ngược lại với UART)
	for (int i = 7; i >= 0; i--){
		if (data & (1<<i))SDA_SET;
		else SDA_CLEAR;
		DWT_Delay_cycles(t_SU_DAT);
		SCL_SET;
		DWT_Delay_us(t_HIGH);
		SCL_CLEAR;
		DWT_Delay_us(t_LOW);
	}

	//Chờ ACK từ Slave
	SDA_SET;
	//DWT_Delay_us(1);
	SCL_SET;
	DWT_Delay_us(t_HIGH);
	uint8_t ack = !(SDA_READ);
	SCL_CLEAR;
	DWT_Delay_us(t_LOW);
	return ack;
}

uint8_t I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint8_t write_stop){
	//Start condition
	I2C_Start();

	//Gửi deivce address. Nếu thấy NACK thì gửi STOP
	if (!I2C_Write_Byte(DevAddress)){
		I2C_Stop();
		return 0;
	}
	//Viết từng byte dữ liệu. Nếu thấy NACK thì gửi STOP
	for (uint16_t i = 0; i < Size; i++){
		if(!I2C_Write_Byte(pData[i])){
			I2C_Stop();
			return 0;
		}
	}
	//STOP condition
	if (write_stop) I2C_Stop();
	return 1;
}

static uint8_t I2C_Receive_Byte(uint8_t ack){
	uint8_t received_byte = 0;
	SDA_MODE_INPUT();
	//Data được gửi về MSB đứng trước
	for (uint8_t i = 0; i<8; i++){
		received_byte <<= 1;
		SCL_CLEAR;
		DWT_Delay_us(t_HIGH);
		SCL_SET;
		DWT_Delay_us(t_LOW);
		received_byte |= SDA_READ;
	}
	//Gửi ACK/NACK
	SCL_CLEAR;
	SDA_MODE_OUTPUT();
	if (ack) SDA_CLEAR;
	else SDA_SET;
	//DWT_Delay_us(1);
	SCL_SET;
	DWT_Delay_us(t_HIGH);
	SCL_CLEAR;
	DWT_Delay_us(t_LOW);

	SDA_MODE_INPUT();
	return received_byte;
}

uint8_t I2C_Master_Receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size){
	if (Size == 0) {
	    I2C_Stop();
	    return 0;
	}
	//Start condition
	I2C_Start();
	//Device address
	if (!I2C_Write_Byte(DevAddress + 1)){
		I2C_Stop();
		return 0;
	}
	//Reading data
	for (uint16_t i = 0; i < Size - 1; i++){
		pData[i] = I2C_Receive_Byte(1); // Send ACK
	}
	pData[Size-1] = I2C_Receive_Byte(0); // Send NACK

	//Stop condition
	I2C_Stop();
	return 1;
}

#else
void I2C_Start(){
	SDA_MODE_OUTPUT();
	SCL_SET;
	//DWT_Delay_us(1);
	SDA_SET;
	DWT_Delay_cycles(t_SU_STA);
	SDA_CLEAR;
	DWT_Delay_cycles(t_HD_STA);
	SCL_CLEAR;
	DWT_Delay_us(t_LOW);

}
void I2C_Stop(){
	SDA_MODE_OUTPUT();
	SDA_CLEAR;

	SCL_SET;
	DWT_Delay_cycles(t_SU_STO);
	SDA_SET;
	DWT_Delay_us(t_BUF);
}

static uint8_t I2C_Write_Byte(uint8_t data){
	for (int i = 7; i >= 0; i--){
		if (data & (1<<i))SDA_SET;
		else SDA_CLEAR;
		DWT_Delay_cycles(t_SU_DAT);

		SCL_SET;
		DWT_Delay_cycles(t_HIGH);
		SCL_CLEAR;
		DWT_Delay_us(t_LOW);
	}

	//poll for ark from slave
	SDA_SET;
	//DWT_Delay_us(1);
	SCL_SET;
	DWT_Delay_cycles(t_HIGH);
	uint8_t ack = !(SDA_READ);
	SCL_CLEAR;
	DWT_Delay_us(t_LOW);
	return ack;

}
uint8_t I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint8_t write_stop){
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
	if (write_stop) I2C_Stop();

	return 1;
}

static uint8_t I2C_Receive_Byte(uint8_t ack){
	uint8_t received_byte = 0;
	SDA_MODE_INPUT();
	//Data được gửi về MSB đứng trước
	for (uint8_t i = 0; i<8; i++){
		received_byte <<= 1;
		SCL_CLEAR;
		DWT_Delay_cycles(t_HIGH);
		SCL_SET;
		DWT_Delay_us(t_LOW);
		received_byte |= SDA_READ;
	}
	//send ack/nack
	SCL_CLEAR;
	SDA_MODE_OUTPUT();
	if (ack) SDA_CLEAR;
	else SDA_SET;
	//DWT_Delay_us(1);


	SCL_SET;
	DWT_Delay_cycles(t_HIGH);
	SCL_CLEAR;
	DWT_Delay_us(t_LOW);

	SDA_MODE_INPUT();
	return received_byte;
}

uint8_t I2C_Master_Receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size){
	if (Size == 0) {
	    I2C_Stop();
	    return 0;
	}
	//Start condition
	I2C_Start();
	//Device address
	if (!I2C_Write_Byte(DevAddress + 1)){
		I2C_Stop();
		return 0;
	}
	//Reading data
	for (uint16_t i = 0; i < Size - 1; i++){
		pData[i] = I2C_Receive_Byte(1); // Send ACK
	}
	pData[Size-1] = I2C_Receive_Byte(0); // Send NACK

	//Stop condition
	I2C_Stop();
	return 1;
}
#endif
