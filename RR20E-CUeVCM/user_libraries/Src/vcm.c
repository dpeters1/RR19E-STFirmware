/*
 * vcm.c
 *
 *  Created on: Mar 18, 2020
 *      Author: Dominic
 */

#include "vcm.h"
#include "bsp.h"
#include "scheduler.h"
#include "bm78.h"

static VCM_data_t vcm_data;

static void VCM_transmit_BT_data(void * handle)
{
	static const VCM_message_block_t messages[] = {
			{.id = ID_INPUT_THROTTLE1,
			.len = sizeof(vcm_data.input_throttle1),
			.p_value = &vcm_data.input_throttle1},

			{.id =ID_INPUT_THROTTLE2,
			.len = sizeof(vcm_data.input_throttle1),
			.p_value = &vcm_data.input_throttle2},

			{.id = ID_INPUT_BRAKE,
			.len = sizeof(vcm_data.input_brake),
			.p_value = &vcm_data.input_brake},

			{.id = ID_LOAD_CURRENT_MC,
			.len = sizeof(vcm_data.load_current_mc),
			.p_value = &vcm_data.load_current_mc},

			{.id = ID_LOAD_CURRENT_BMS,
			.len = sizeof(vcm_data.load_current_bms),
			.p_value = &vcm_data.load_current_bms},

			{.id = ID_LOAD_CURRENT_IMD,
			.len = sizeof(vcm_data.load_current_imd),
			.p_value = &vcm_data.load_current_imd},

			{.id = ID_LOAD_CURRENT_PUMP,
			.len = sizeof(vcm_data.load_current_pump),
			.p_value = &vcm_data.load_current_pump},

			{.id = ID_LV_VOLTAGE_BATTERY,
			.len = sizeof(vcm_data.lv_voltage_battery),
			.p_value = &vcm_data.lv_voltage_battery},

			{.id = ID_STATUS_FAULT,
			.len = sizeof(vcm_data.status_fault),
			.p_value = &vcm_data.status_fault},

			{.id = ID_STATUS_HV_LIVE,
			.len = sizeof(vcm_data.status_hv_live),
			.p_value = &vcm_data.status_hv_live},

			{.id = ID_STATUS_READY_TO_DRIVE,
			.len = sizeof(vcm_data.status_ready_to_drive),
			.p_value = &vcm_data.status_ready_to_drive},

			{.id = ID_HV_VOLTAGE_BATTERY,
			.len = sizeof(vcm_data.hv_voltage_battery),
			.p_value = &vcm_data.hv_voltage_battery},

			{.id = ID_HV_CURRENT_MOTOR,
			.len = sizeof(vcm_data.hv_current_motor),
			.p_value = &vcm_data.hv_current_motor},

			{.id = ID_TEMP_MOTOR,
			.len = sizeof(vcm_data.temp_motor),
			.p_value = &vcm_data.temp_motor},

			{.id = ID_TEMP_MC,
			.len = sizeof(vcm_data.temp_mc),
			.p_value = &vcm_data.temp_mc},

			{.id = ID_TEMP_BATTERY1,
			.len = sizeof(vcm_data.temp_battery1),
			.p_value = &vcm_data.temp_battery1},

			{.id = ID_TEMP_BATTERY2,
			.len = sizeof(vcm_data.temp_battery2),
			.p_value = &vcm_data.temp_battery2},

			{.id = ID_MOTOR_RPM,
			.len = sizeof(vcm_data.motor_rpm),
			.p_value = &vcm_data.motor_rpm},

			{.id = ID_MOTOR_TORQUE,
			.len = sizeof(vcm_data.motor_torque),
			.p_value = &vcm_data.motor_torque},
	};

	static uint8_t serial_buffer[255] = {0};
	static uint16_t encoded_len = 0;

	// Reset the buffer each time
	memset(serial_buffer, 0, encoded_len);
	encoded_len = 0;

	for(uint8_t i = 0; i < sizeof(messages)/sizeof(VCM_message_block_t); i++) {

		serial_buffer[encoded_len++] = messages[i].id;
		serial_buffer[encoded_len++] = messages[i].len;
		memcpy(&serial_buffer[encoded_len], messages[i].p_value, messages[i].len);
		encoded_len += messages[i].len;
	}

	BT_transmit(serial_buffer, encoded_len);
}


static void VCM_loop(void * handle)
{
	// Update the adc readings
	vcm_data.input_throttle1 = (uint8_t)(BSP_get_analog_input(ANLG_IN_CH_THROTTLE1) * 100 / 4096);
	vcm_data.input_throttle2 = (uint8_t)(BSP_get_analog_input(ANLG_IN_CH_THROTTLE2) * 100 / 4096);
	vcm_data.input_brake = (uint8_t)(BSP_get_analog_input(ANLG_IN_CH_BRAKE) * 100 / 4096);

	vcm_data.load_current_mc = BSP_get_load_current(LOAD_CH_MC);
	vcm_data.load_current_bms = BSP_get_load_current(LOAD_CH_BMS);
	vcm_data.load_current_imd = BSP_get_load_current(LOAD_CH_IMD);
}

void VCM_init()
{
	scheduler_add(VCM_transmit_BT_data, NULL, 500, 500);

	// Run the main loop at 100 Hz
	scheduler_add(VCM_loop, NULL, 0, 10);

	// Some initial values for testing
	vcm_data.output_throttle = 52;
	vcm_data.status_hv_live = 1;
	vcm_data.lv_voltage_battery = 14800;
	vcm_data.hv_current_motor = 1021;
	vcm_data.hv_voltage_battery = 29700;

	vcm_data.motor_rpm = 14223;
	vcm_data.motor_torque = 542;

	vcm_data.temp_motor = 95;
	vcm_data.temp_mc = 67;
	vcm_data.temp_battery1 = 40;
	vcm_data.temp_battery2 = 42;
}
