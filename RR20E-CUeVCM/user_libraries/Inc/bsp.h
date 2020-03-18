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
	MTOR_SIDE_HALF_A,
	MTOR_SIDE_HALF_B,
	MTOR_SIDE_FULL
} bsp_mtor_side_t;

typedef enum{
	BUZZER_PITCH_LOW,
	BUZZER_PITCH_MED,
	BUZZER_PITCH_HIGH
} bsp_buzzer_pitch_t;

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
void BSP_fault_led_on(bool on);
void BSP_buzzer_on(bool on, bsp_buzzer_pitch_t pitch);
void BSP_load_channel_on(uint8_t channel, bool on);
uint16_t BSP_load_channel_get_current(uint8_t channel);
void BSP_pin_interrupt_enable(uint8_t channel, bsp_pin_irq_evt_t evt);
void BSP_pin_interrupt_disable(uint8_t channel);
void BSP_analog_out_set(uint8_t channel, uint8_t value_percent);
bsp_adc_data_t * BSP_get_adc_readings();
uint16_t BSP_analog_in_read(uint8_t channel);


#endif /* INC_BSP_H_ */
