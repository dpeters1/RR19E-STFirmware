/*
 * bm78.h
 *
 *  Created on: Mar 17, 2020
 *      Author: Dominic
 */

#ifndef INC_BM78_H_
#define INC_BM78_H_

#include <stdbool.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"


#define BM78_COMMAND_BUFFER_SIZE	200
#define BM78_UART_SYNC_WORD			0xAA
#define BM78_UART_CMD_HEADER_SIZE	3
#define BM78_CONFIG_MODE_TIMEOUT_MS		3000
#define BM78_MAX_ADVERTISING_NAME_SIZE	20


/* Define group BM78 Commands */
#define BM78_CMD_READ_LOCAL_INFORMATION	0x01
#define BM78_CMD_READ_DEVICE_NAME		0x07
#define BM78_CMD_WRITE_DEVICE_NAME		0x08
#define BM78_CMD_ERASE_PAIR_INFO		0x09
#define BM78_CMD_READ_PAIR_MODE			0x0A
#define BM78_CMD_WRITE_PAIR_MODE		0x0B
#define BM78_CMD_READ_PAIR_DEV_INFO		0x0C
#define BM78_CMD_DEL_PAIR_DEV			0x0D
#define BM78_CMD_READ_PIN_CODE			0x50
#define BM78_CMD_WRITE_PIN_CODE			0x51
#define BM78_CMD_LEAVE_CFG_MODE			0x52

#define BM78_EVT_CONFIG_STATUS			0x8F

typedef enum {
	STATE_POWER_OFF,
	STATE_POWER_ON,
	STATE_CONFIGURATION,
	STATE_ACCESS_READY,
	STATE_CONNECTED
} BM78_state_t;

typedef enum{
	MODE_NORMAL,
	MODE_EEPROM
} BM78_mode_t;

typedef enum{
	PAIR_METHOD_PIN,
	PAIR_METHOD_JUST_WORKS
} BM78_pair_method_t;

typedef struct {
	UART_HandleTypeDef * huart;
	volatile BM78_state_t state;
	BM78_mode_t mode;
} BM78_handler_t;
/* Public function prototypes */
void BT_init(UART_HandleTypeDef * uart_bt);
void BT_power_on(bool turn_on, BM78_mode_t mode);
void BT_transmit(uint8_t * buffer, uint16_t len);
void BT_send_command(uint8_t command, uint8_t * params, uint16_t params_len);
BM78_state_t BT_get_state();
void BT_set_device_name(char * name);
void BT_set_pairing_method(BM78_pair_method_t pair_method);
void BT_set_pin(char * pin_code);
//void BT_advertise();
//void BT_stream(uint8_t * buffer, uint16_t len);

#endif /* INC_BM78_H_ */
