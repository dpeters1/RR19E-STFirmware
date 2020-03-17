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


/* Define group BM78 Commands */
#define BM78_CMD_READ_LOCAL_INFORMATION	0x01
#define BM78_CMD_READ_DEVICE_NAME		0x07
#define BM78_CMD_WRITE_DEVICE_NAME		0x08
#define BM78_CMD_ERASE_PAIR_INF			0x09
#define BM78_CMD_READ_PAIR_MODE			0x0A
#define BM78_CMD_WRITE_PAIR_MODE		0x0B
#define BM78_CMD_READ_PAIR_DEV_INFO		0x0C
#define BM78_CMD_DEL_PAIR_DEV			0x0D

typedef enum {
	STATE_POWER_OFF,
	STATE_POWER_ON,
	STATE_STANDBY,
	STATE_LINK_BACK,
	STATE_SPP_CONNECTED,
	STATE_BLE_CONNECTED,
	STATE_IDLE,
	STATE_POWER_DOWN_S2
} BM78_state_t;

typedef struct {
	UART_HandleTypeDef * huart;
	BM78_state_t state;
} BM78_handler_t;
/* Public function prototypes */
void BT_init(UART_HandleTypeDef * uart_bt);
void BT_power_on(bool on);
void BT_transmit(uint8_t * buffer, uint16_t len);
//void BT_advertise();
//void BT_stream(uint8_t * buffer, uint16_t len);

#endif /* INC_BM78_H_ */
