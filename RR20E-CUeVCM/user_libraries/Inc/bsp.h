/*
 * bsp.h
 *
 *  Created on: Mar 12, 2020
 *      Author: Dominic
 */

#ifndef INC_BSP_H_
#define INC_BSP_H_

#include "main.h"

#define BSP_INSTANCE_DEF(bsp_inst_name) static bsp_handler_t bsp_inst_name = {0};

#define OUTPUT_BANK_SIZE		4
#define NUM_OUTPUT_CHANNELS		12
#define NUM_INPUT_CHANNELS		8
#define NUM_ANLG_INPUT_CHANNELS 4
#define NUM_ANLG_OUT_CHANNELS   2

typedef enum {
	LOW,
	HIGH
} BSP_bool_t;

typedef enum {
	MTOR_DIR_FORWARD,
	MTOR_DIR_BACKWARD
} bsp_mtor_direction_t;

typedef enum{
	BUZZER_OFF,
	BUZZER_PITCH_LOW,
	BUZZER_PITCH_MED,
	BUZZER_PITCH_HIGH
} bsp_buzzer_noise_t;

typedef enum {
	INTERRUPT_NONE,
	INTERRUPT_HI_TO_LO,
	INTERRUPT_LO_TO_HI,
	INTERRUPT_TOGGLE
} bsp_pin_irq_evt_t;

typedef enum{
	BSP_SUCCESS,
	BSP_ERROR
} bsp_ret_code_t;

typedef struct {
	const gpio_t gpio;
	bsp_pin_irq_evt_t irq_evt;
	uint8_t state;
} bsp_input_irq_t;

typedef void (*irq_handler)(uint8_t channel, bsp_pin_irq_evt_t event);

typedef struct{
	TIM_HandleTypeDef * tim_buzzer;
	TIM_HandleTypeDef * tim_motor;
	ADC_HandleTypeDef * adc_csense;
	ADC_HandleTypeDef * adc_anlg_in;
	ADC_HandleTypeDef * adc_exp_hdr;
	DAC_HandleTypeDef * dac_anlg_out;
	CAN_HandleTypeDef * can;
	UART_HandleTypeDef * uart_bt;
	irq_handler app_pin_irq_handler;
//	void * (canbus_irq_handler)(uint8_t * rx_buffer, uint16_t len);
} bsp_handler_t;


typedef struct {
	uint32_t analog_input[NUM_ANLG_INPUT_CHANNELS];
	uint32_t load_current[NUM_OUTPUT_CHANNELS];
	uint32_t motor_current;
} bsp_adc_data_t;

/* Public function prototypes */
void BSP_init(bsp_handler_t * p_bsp_handler);
void BSP_set_fault_led(BSP_bool_t on);
void BSP_set_buzzer(bsp_buzzer_noise_t noise);
void BSP_set_load_output(uint8_t channel, BSP_bool_t on);
uint16_t BSP_get_load_current(uint8_t channel);
void BSP_enable_interrupt(uint8_t channel, bsp_pin_irq_evt_t evt);
void BSP_disable_interrupt(uint8_t channel);
void BSP_set_analog_output(uint8_t channel, uint16_t value_12b);
uint16_t BSP_get_analog_input(uint8_t channel);
void BSP_set_motor_output(bsp_mtor_direction_t dir, uint8_t duty_cycle);
uint16_t BSP_get_motor_current();


#endif /* INC_BSP_H_ */
