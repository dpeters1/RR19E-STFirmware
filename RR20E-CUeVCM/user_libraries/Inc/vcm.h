/*
 * vcm.h
 *
 *  Created on: Mar 18, 2020
 *      Author: Dominic
 */

#ifndef INC_VCM_H_
#define INC_VCM_H_

#include "main.h"

#define LOAD_CH_MC		0
#define LOAD_CH_BMS		1
#define LOAD_CH_IMD		2

#define ANLG_IN_CH_THROTTLE1	0
#define ANLG_IN_CH_THROTTLE2	1
#define ANLG_IN_CH_BRAKE		2

typedef enum {
	ID_INPUT_THROTTLE1,
	ID_INPUT_THROTTLE2,
	ID_INPUT_BRAKE,
	ID_OUTPUT_THROTTLE,
	ID_OUTPUT_BRAKE,
	ID_LOAD_CURRENT_MC,
	ID_LOAD_CURRENT_IMD,
	ID_LOAD_CURRENT_BMS,
	ID_LOAD_CURRENT_PUMP,
	ID_LV_VOLTAGE_BATTERY,
	ID_STATUS_FAULT,
	ID_STATUS_HV_LIVE,
	ID_STATUS_READY_TO_DRIVE,
	ID_HV_CURRENT_MOTOR,
	ID_HV_VOLTAGE_BATTERY,
	ID_TEMP_MOTOR,
	ID_TEMP_MC,
	ID_TEMP_BATTERY1,
	ID_TEMP_BATTERY2,
	ID_MOTOR_RPM,
	ID_MOTOR_TORQUE
} VCM_data_id_t;

typedef enum{
	FAULT_ESTOP = 	1<<0,
	FAULT_BOT = 	1<<1,
	FAULT_INERTIA = 1<<2,
	FAULT_BMS = 	1<<3,
	FAULT_IMD = 	1<<4,
	FAULT_MC =		1<<5,
	FAULT_VCM = 	1<<6
} VCM_fault_t;

typedef struct {
	uint8_t input_throttle1;
	uint8_t input_throttle2;
	uint8_t input_brake;
	uint8_t output_throttle;
	uint8_t output_brake;
	uint16_t load_current_mc;
	uint16_t load_current_bms;
	uint16_t load_current_imd;
	uint16_t load_current_pump;
	uint16_t lv_voltage_battery;
	uint8_t status_fault;
	uint8_t status_hv_live;
	uint8_t status_ready_to_drive;
	uint16_t hv_current_motor;
	uint16_t hv_voltage_battery;
	uint16_t temp_motor;
	uint16_t temp_mc;
	uint16_t temp_battery1;
	uint16_t temp_battery2;
	uint16_t motor_rpm;
	uint16_t motor_torque;
} VCM_data_t;

typedef struct {
	uint8_t id;
	uint8_t	len;
	void * p_value;
} VCM_message_block_t;


/* Public function prototypes */
void VCM_init();


#endif /* INC_VCM_H_ */
