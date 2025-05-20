/*
 * 24C256.c
 *
 *  Created on: Apr 29, 2025
 *      Author: Admin
 */

#include "24C256.h"

extern char uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern volatile uint16_t uart_rx_buffer_index ; //Số thứ tự của buffer
extern volatile uint8_t string_ready_flag ;
extern uint8_t external_msg[20];
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

HAL_StatusTypeDef EEPROM_random_read_hal(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len){
	HAL_StatusTypeDef returnValue;
	// Chuyen MemAddress thanh 2 bit rieng biet
	uint8_t addr[2];
	addr[0] = (uint8_t) ((MemAddress & 0xFF00) >> 8);
	addr[1] = (uint8_t) (MemAddress & 0xFF);

	// Gui 2 bit dia chi qua i2c
	returnValue = HAL_I2C_Master_Transmit(hi2c, DevAddress, addr, 2, HAL_MAX_DELAY);
	if (returnValue != HAL_OK)
	{
		return returnValue;
	}

	// Nhan lai data tu slave bang cach bat dau 1 chu ki i2c nua
	returnValue = HAL_I2C_Master_Receive(hi2c, DevAddress, pData, len, HAL_MAX_DELAY);
	return returnValue;
}

HAL_StatusTypeDef EEPROM_byte_write_hal(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len){
	HAL_StatusTypeDef returnValue;

	uint8_t *buffer;
	buffer = (uint8_t *)malloc(len + 2);

    if (buffer == NULL) {
        return HAL_ERROR; // Return an error status if allocation failed
    }

	buffer[0] = (uint8_t) ((MemAddress & 0xFF00) >> 8);
	buffer[1] = (uint8_t) (MemAddress & 0xFF);
	memcpy(buffer + 2, pData, len);

	returnValue = HAL_I2C_Master_Transmit(hi2c, DevAddress, buffer, len+2, HAL_MAX_DELAY);
	free(buffer);
	if (returnValue != HAL_OK) return returnValue;

	//while (HAL_I2C_Master_Transmit(hi2c, DevAddress, 0, 0, HAL_MAX_DELAY) != HAL_OK);
	HAL_Delay(5);
	return HAL_OK;
}
 HAL_StatusTypeDef EEPROM_write_string_hal(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len){
	uint16_t len_quotient = len / 64;
	uint16_t len_remainder = len % 64;
	uint16_t addr_quotient = MemAddress / 64;
	uint16_t addr_remainder = MemAddress % 64;
	HAL_StatusTypeDef returnValue;

	//Nếu địa chỉ viết bắt đầu chia hết cho 64 byte -> cứ viết từng chunk 64 byte đến khi nào hết thì thôi.
	if (addr_remainder == 0)
	{
		for (int i = 0; i < len_quotient; i++){
			returnValue = EEPROM_byte_write_hal(hi2c, DevAddress, MemAddress, &pData[i*64], 64);
			if (returnValue != HAL_OK) return returnValue;
			MemAddress += 64;

		}
		if (len_remainder > 0){
			returnValue = EEPROM_byte_write_hal(hi2c, DevAddress, MemAddress, &pData[len_quotient*64], len_remainder);
			if (returnValue != HAL_OK) return returnValue;
		}
		return HAL_OK;
	}
	//Nếu địa chỉ viết bắt đầu không chia hết cho 64 byte -> viết tới chỗ chia hết 64 byte, viết theo chuck 64 byte, rồi viết đống còn lại
	else{
		uint16_t next_page_break = 64*(addr_quotient + 1);
		uint16_t chunk1_len = next_page_break - MemAddress;
		if (chunk1_len > len){
			chunk1_len = len;
		}

		//Viet toi cho chia het 64 byte
		returnValue = EEPROM_byte_write_hal(hi2c, DevAddress, MemAddress, pData, chunk1_len);
		if (returnValue != HAL_OK) return returnValue;
		MemAddress += chunk1_len;

		//Viet theo nhung chunk 64 byte
		uint16_t remaining_len_quotient = (len - chunk1_len) / 64;
		uint16_t remaining_len_remainder = (len - chunk1_len) % 64;
		for (int i = 0; i < remaining_len_quotient; i++){
			returnValue = EEPROM_byte_write_hal(hi2c, DevAddress, MemAddress, &pData[i*64 + chunk1_len], 64);
			if (returnValue != HAL_OK) return returnValue;
			MemAddress += 64;
		}

		//Neu con thua lai phan khong phai 64 byte, viet tiep
		if (remaining_len_remainder > 0){
			returnValue = EEPROM_byte_write_hal(hi2c, DevAddress, MemAddress, &pData[remaining_len_quotient*64 + chunk1_len], remaining_len_remainder);
			if (returnValue != HAL_OK) return returnValue;
		}
		return HAL_OK;

	}



 }


void UART_RxCplt_Callback(uint8_t current_byte){
	HAL_UART_Transmit(&huart2, &current_byte, 1, HAL_MAX_DELAY);

	//String đã kết thúc chưa?
	if (current_byte == '\r' || current_byte == '\n'){
		char newline[] = "\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)newline, strlen(newline), HAL_MAX_DELAY);
		uart_rx_buffer[uart_rx_buffer_index] = '\0';
		string_ready_flag = 1;
		uart_rx_buffer_index = 0;

	}
	else if (uart_rx_buffer_index < 500-1){
		uart_rx_buffer[uart_rx_buffer_index++] = current_byte;
	}
}

void process_UART_command(){
	if (!string_ready_flag) return;
	string_ready_flag = 0;
	/*
	 * Những thành phần của 1 lệnh bao gồm
	 * - Tên lệnh : WRITE, READ
	 * - Địa chỉ của EEPROM: 0xA0, 0xA1, ...
	 * - Địa chỉ bắt đầu đọc/ viết trong EEPROM: 0x1234
	 * - Nội dung viết: "lmasdsads"
	 * hoặc
	 * - Số byte đọc: 100
	 * -> với tất cả mọi lệnh, đều có thể tách thành 4 phần tử (vd: WRITE 0xA0 0x1234 "day la viet thu" hoặc READ 0xA0 0x1234 8
	 */
	char command_name[10];
	char dev_address[10];
	char mem_address[10];
	char fourth_part[400];
	char command_string[UART_RX_BUFFER_SIZE];
	strncpy(command_string, (char*) uart_rx_buffer, UART_RX_BUFFER_SIZE);

	int initial_parsing = sscanf(command_string, "%s %s %s", command_name, dev_address, mem_address);
	if (initial_parsing == 3){
		uint16_t dev_address_int;
		uint16_t mem_address_int;

		dev_address_int = (uint16_t) strtoul(dev_address, NULL, 0);
		mem_address_int = (uint16_t) strtoul(mem_address, NULL, 0);
		if (strcmp("WRITE", command_name) == 0){
			char temp_cmd[10], temp_dev[10], temp_mem[10];
			int complete_parse = sscanf(command_string, "%s %s %s \"%399[^\"]\"", temp_cmd, temp_dev, temp_mem, fourth_part);

			if (complete_parse == 4){
				uint8_t* msg = (uint8_t*) fourth_part;
				uint16_t msg_len = strlen(fourth_part) + 1;

				HAL_StatusTypeDef write_status = EEPROM_write_string_hal(&hi2c1, dev_address_int, mem_address_int, msg, msg_len);

				if (write_status != HAL_OK) return;
			}
			else{
				//Write command thieu parameter
				return;
			}
		}
		else if (strcmp("READ", command_name) == 0){
			char temp_cmd[10], temp_dev[10], temp_mem[10];
			int complete_parse = sscanf(command_string, "%s %s %s %s", temp_cmd, temp_dev, temp_mem, fourth_part);
			if (complete_parse == 4){

				uint16_t num_bytes_to_read = (uint16_t)strtoul(fourth_part, NULL, 0);
				uint8_t msg[500];
				HAL_StatusTypeDef write_status = EEPROM_random_read_hal(&hi2c1, dev_address_int, mem_address_int, msg, num_bytes_to_read);
				HAL_UART_Transmit(&huart2, msg, num_bytes_to_read, HAL_MAX_DELAY);
			}
			else{
				//Write command thieu parameter
				return;
			}

		}
		else{
			//Lỗi do lệnh sai
			return;
		}
	}
	else{
		//Lỗi do không đủ phần tử trong lệnh
		return;
	}



}
/*
void I2C_init(void){
	//Enable I2C clock and GPIO clock
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;		//Enable i2c periphral clock on the apb1 bus
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;	//Enable the GPIO B peripheral clock on the AHB1 bus

	//Config PB7 and PB6 for i2c communication
	GPIOB->MODER |= ((0b10 << 12) | (0b10 << 14)); 	//Set PB6 and PB7 as alternate function mode
	GPIOB->OTYPER |= ((0b1 << 6) | (0b1 << 7)); 	//Set open drain output
	GPIOB->OSPEEDR |= ((0b01 << 12) | (0b01 << 14));
	GPIOB->AFR[0] |= ((0b0100 << 24) | (0b0100 << 28)); //Select i2c in AFR

	//Reset the I2C
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;

	//Config the FREQ bits with the APB clock frequency value (I2C peripheral) connected to APB).
	I2C1->CR2 |= (6 << 0);

	//Config I2C clock control register (CCR) . Standard mode so T_high = T_low = CCR*T_pclk1, T_high = T_low = 5.10^(-6), T_pclk1 = 1/6.25.10^6 -> CCR = 31.25. Lm tron thanh 31 do yeu cau cua Standard mode la up to 100kbit
	I2C1->CCR |= (31<<0); //RM0090n pg 873

	//Config TRISE register (1000ns/1/6.25.10^6 + 1= 7.25. Lay 8 cho do day la TRISE
	I2C1->TRISE |= (8<<0);

	//Enable i2c1
	I2C1->CR1 |= I2C_CR1_PE;

}

void I2C_start(void){
	//Send start bit
	I2C1->CR1 |= I2C_CR1_START;
	// Wait for SB bit in I2C->SR1 to set (RM0090 pg 871)
	while (!(I2C1->SR1 & (1<<0))){

	}
}

void I2C_write(uint8_t data){
	//Wait for TXE bit in I2C->SR1 to set. this indicates that DR is empty
}
*/
