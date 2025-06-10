/*
 * 24C256_bitbanging.c
 *
 *  Created on: May 26, 2025
 *      Author: Admin
 */
#include "24c256_bitbanging.h"

extern char uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern volatile uint16_t uart_rx_buffer_index ; //Số thứ tự của buffer
extern volatile uint8_t string_ready_flag ;
extern uint8_t external_msg[20];

uint8_t EEPROM_byte_write(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len){
	uint8_t returnValue;

	uint8_t *buffer;
	buffer = (uint8_t *)malloc(len + 2);

    if (buffer == NULL) {
        return 2; // Return an error status if allocation failed
    }

	buffer[0] = (uint8_t) ((MemAddress & 0xFF00) >> 8);
	buffer[1] = (uint8_t) (MemAddress & 0xFF);
	memcpy(buffer + 2, pData, len);

	returnValue = I2C_Master_Transmit(DevAddress, buffer, len+2, 1);
	free(buffer);
	if (returnValue != 1) return returnValue;

	//while (HAL_I2C_Master_Transmit(hi2c, DevAddress, 0, 0, HAL_MAX_DELAY) != HAL_OK);
	DWT_Delay_us(5000);
	return 1;
}

uint8_t EEPROM_random_read(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len){
	uint8_t returnValue;
	// Chuyen MemAddress thanh 2 bit rieng biet
	uint8_t addr[2];
	addr[0] = (uint8_t) ((MemAddress & 0xFF00) >> 8);
	addr[1] = (uint8_t) (MemAddress & 0xFF);

	// Gui 2 bit dia chi qua i2c
	returnValue = I2C_Master_Transmit(DevAddress, addr, 2, 0);
	if (returnValue != 1)
	{
		return returnValue;
	}

	// Nhan lai data tu slave bang cach bat dau 1 chu ki i2c nua
	returnValue = I2C_Master_Receive(DevAddress, pData, len);
	return returnValue;
}

uint8_t EEPROM_write_string(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len){
	uint16_t len_quotient = len / 64;
	uint16_t len_remainder = len % 64;
	uint16_t addr_quotient = MemAddress / 64;
	uint16_t addr_remainder = MemAddress % 64;
	uint8_t returnValue;

	//Nếu địa chỉ viết bắt đầu chia hết cho 64 byte -> cứ viết từng chunk 64 byte đến khi nào hết thì thôi.
	if (addr_remainder == 0)
	{
		for (int i = 0; i < len_quotient; i++){
			returnValue = EEPROM_byte_write(DevAddress, MemAddress, &pData[i*64], 64);
			if (returnValue != 1) return returnValue;
			MemAddress += 64;

		}
		if (len_remainder > 0){
			returnValue = EEPROM_byte_write(DevAddress, MemAddress, &pData[len_quotient*64], len_remainder);
			if (returnValue !=1) return returnValue;
		}
		return 1;
	}
	//Nếu địa chỉ viết bắt đầu không chia hết cho 64 byte -> viết tới chỗ chia hết 64 byte, viết theo chuck 64 byte, rồi viết đống còn lại
	else{
		uint16_t next_page_break = 64*(addr_quotient + 1);
		uint16_t chunk1_len = next_page_break - MemAddress;
		if (chunk1_len > len){
			chunk1_len = len;
		}

		//Viet toi cho chia het 64 byte
		returnValue = EEPROM_byte_write(DevAddress, MemAddress, pData, chunk1_len);
		if (returnValue != 1) return returnValue;
		MemAddress += chunk1_len;

		//Viet theo nhung chunk 64 byte
		uint16_t remaining_len_quotient = (len - chunk1_len) / 64;
		uint16_t remaining_len_remainder = (len - chunk1_len) % 64;
		for (int i = 0; i < remaining_len_quotient; i++){
			returnValue = EEPROM_byte_write( DevAddress, MemAddress, &pData[i*64 + chunk1_len], 64);
			if (returnValue != 1) return returnValue;
			MemAddress += 64;
		}

		//Neu con thua lai phan khong phai 64 byte, viet tiep
		if (remaining_len_remainder > 0){
			returnValue = EEPROM_byte_write(DevAddress, MemAddress, &pData[remaining_len_quotient*64 + chunk1_len], remaining_len_remainder);
			if (returnValue != 1) return returnValue;
		}
		return 1;

	}



}

void UART_Receive_String_Blocking(void){
	uart_rx_buffer_index = 0;
	string_ready_flag = 0;
	memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
	while(1){
		uint8_t current_byte;
		UART_Receive(&current_byte, 1);
		UART_Transmit(&current_byte, 1);
		if (current_byte == '\r' || current_byte == '\n'){
			char newline[] = "\r\n";
			UART_Transmit((uint8_t*)newline, strlen(newline));
			uart_rx_buffer[uart_rx_buffer_index] = '\0';
			string_ready_flag = 1;
			uart_rx_buffer_index = 0;
			break;
		}

		else if (current_byte == '\b' || current_byte == 127){
			if (uart_rx_buffer_index > 0){
				uart_rx_buffer_index--;
				uart_rx_buffer[uart_rx_buffer_index] = '\0';
				char back_space[] = "\b \b";
				UART_Transmit((uint8_t*) back_space, strlen(back_space));
			}
			continue;
		}

		else if (uart_rx_buffer_index < UART_RX_BUFFER_SIZE-1){
			uart_rx_buffer[uart_rx_buffer_index++] = current_byte;

		}

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

				uint8_t write_status = EEPROM_write_string(dev_address_int, mem_address_int, msg, msg_len);

				if (write_status != 1) return;
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
				uint8_t write_status = EEPROM_random_read(dev_address_int, mem_address_int, msg, num_bytes_to_read);
				UART_Transmit(msg, num_bytes_to_read);
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
