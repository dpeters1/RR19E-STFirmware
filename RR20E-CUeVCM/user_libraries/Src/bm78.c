/*
 * bm78.c
 *
 *  Created on: Mar 17, 2020
 *      Author: Dominic
 */

#include "bm78.h"
#include "main.h"
#include <string.h>

static BM78_handler_t bm78;

static uint8_t rx_buffer[200];
static uint8_t rx_buffer_index = 0;

void BT_init(UART_HandleTypeDef * uart_bt){

	bm78.huart = uart_bt;

	BT_power_on(false);

	HAL_UART_Receive_IT(bm78.huart, rx_buffer, 1);
}

void BT_power_on(bool on)
{
	HAL_GPIO_WritePin(BT_SW_BTN_GPIO_Port, BT_SW_BTN_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
	// Set configuration pins for normal operation
	HAL_GPIO_WritePin(BT_CFG1_GPIO_Port, BT_CFG1_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BT_CFG2_GPIO_Port, BT_CFG2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BT_EAN_GPIO_Port, BT_EAN_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BT_RST_GPIO_Port, BT_RST_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);

	bm78.state = on ? STATE_POWER_ON : STATE_POWER_OFF;
}


void BT_transmit(uint8_t * buffer, uint16_t len)
{
	HAL_UART_Transmit_IT(bm78.huart, buffer, len);
}

//static uint8_t calculate_chksum_byte(uint8_t * buffer, uint16_t len)
//{
//	uint8_t checksum = 0;
//
//	for(uint8_t i=0; i<len; i++){
//		checksum += buffer[i];
//	}
//
//	return 0xFF - checksum + 1;
//}
//
//void BT_send_command(uint8_t command, uint8_t * params, uint16_t params_len)
//{
//	static uint8_t command_buffer[BM78_COMMAND_BUFFER_SIZE];
//	uint16_t encoded_len = 0;
//
//	memset(command_buffer, 0, sizeof(command_buffer));
//
//	command_buffer[encoded_len++] = BM78_UART_SYNC_WORD;
//
//	uint16_t message_len = params_len + sizeof(command);
//	command_buffer[encoded_len++] = (message_len & 0xFF00) >> 8;
//	command_buffer[encoded_len++] = message_len & 0xFF;
//
//	command_buffer[encoded_len++] = command;
//
//	memcpy(&command_buffer[encoded_len], params, params_len);
//	encoded_len += params_len;
//
//	command_buffer[encoded_len++] = calculate_chksum_byte(&command_buffer[1], message_len + sizeof(message_len));
//
//	HAL_UART_Transmit_IT(bm78.huart, command_buffer, encoded_len);
//
//	uint32_t res = HAL_UART_Receive_IT(bm78.huart, rx_buffer, 1);
//	if(!res) printf("send success\n");
//}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == bm78.huart){

		if(rx_buffer[rx_buffer_index] == '\n'){
			printf("%s\n", rx_buffer);

			memset(rx_buffer, 0, rx_buffer_index+1);
			rx_buffer_index = 0;
		}
		else rx_buffer_index++;

		HAL_UART_Receive_IT(bm78.huart, &rx_buffer[rx_buffer_index], 1);
	}
}
