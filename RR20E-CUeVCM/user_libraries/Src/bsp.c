/*
 * bsp.c
 *
 *  Created on: Mar 12, 2020
 *      Author: Dominic
 */

#include "bsp.h"
#include "scheduler.h"

static bsp_handler_t * bsp;
static bsp_adc_data_t adc_readings;

static const gpio_t output_channel_map[NUM_OUTPUT_CHANNELS] =
	{{LOAD_OUT1_1_Pin, LOAD_OUT1_1_GPIO_Port}, {LOAD_OUT1_2_Pin, LOAD_OUT1_2_GPIO_Port},
	{LOAD_OUT1_3_Pin, LOAD_OUT1_3_GPIO_Port}, {LOAD_OUT1_4_Pin, LOAD_OUT1_4_GPIO_Port},
	{LOAD_OUT2_1_Pin, LOAD_OUT2_1_GPIO_Port}, {LOAD_OUT2_2_Pin, LOAD_OUT2_2_GPIO_Port},
	{LOAD_OUT2_3_Pin, LOAD_OUT2_3_GPIO_Port}, {LOAD_OUT2_4_Pin, LOAD_OUT2_4_GPIO_Port},
	{LOAD_OUT3_1_Pin, LOAD_OUT3_1_GPIO_Port}, {LOAD_OUT3_2_Pin, LOAD_OUT3_2_GPIO_Port},
	{LOAD_OUT3_3_Pin, LOAD_OUT3_3_GPIO_Port}, {LOAD_OUT3_4_Pin, LOAD_OUT3_4_GPIO_Port}};

static bsp_input_irq_t input_channel_map[NUM_INPUT_CHANNELS] =
	{{{EXTI8_INPUT_Pin, EXTI8_INPUT_GPIO_Port}, 0, 0}, {{EXTI9_INPUT_Pin, EXTI9_INPUT_GPIO_Port}, 0, 0},
	{{EXTI14_INPUT_Pin, EXTI14_INPUT_GPIO_Port}, 0, 0}, {{EXTI15_INPUT_Pin, EXTI15_INPUT_GPIO_Port}, 0, 0},
	{{EXTI10_INPUT_Pin, EXTI10_INPUT_GPIO_Port}, 0, 0}, {{EXTI11_INPUT_Pin, EXTI11_INPUT_GPIO_Port}, 0, 0},
	{{EXTI12_INPUT_Pin, EXTI12_INPUT_GPIO_Port}, 0, 0}, {{EXTI13_INPUT_Pin, EXTI13_INPUT_GPIO_Port}, 0, 0}};


void BSP_init(bsp_handler_t * p_bsp_handler)
{
	bsp = p_bsp_handler;

	if(bsp->adc_anlg_in != NULL){
		HAL_ADC_Start_DMA(bsp->adc_anlg_in, adc_readings.analog_inputs, NUM_ANLG_INPUT_CHANNELS);
	}

	if(bsp->adc_csense != NULL) HAL_ADC_Start_DMA(bsp->adc_csense, adc_readings.current_sense_mv, OUTPUT_BANK_SIZE+1);

	HAL_GPIO_WritePin(CSENSE_P1_SEL_GPIO_Port, CSENSE_P1_SEL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CSENSE_P2_SEL_GPIO_Port, CSENSE_P2_SEL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CSENSE_P3_SEL_GPIO_Port, CSENSE_P3_SEL_Pin, GPIO_PIN_SET);
}

void BSP_fault_led_on(bool on)
{
	HAL_GPIO_WritePin(OUT_LED_FAULT_GPIO_Port, OUT_LED_FAULT_Pin, on ? GPIO_PIN_SET:GPIO_PIN_RESET);
}


void BSP_buzzer_on(bool on, bsp_buzzer_pitch_t pitch)
{
	if(on){
		HAL_TIM_Base_DeInit(bsp->tim_buzzer);

		switch(pitch){
		case BUZZER_PITCH_LOW:
			bsp->tim_buzzer->Init.Prescaler = 500-1;
			break;
		case BUZZER_PITCH_MED:
				bsp->tim_buzzer->Init.Prescaler = 450-1;
				break;
		case BUZZER_PITCH_HIGH:
				bsp->tim_buzzer->Init.Prescaler = 400-1;
				break;
		}

		HAL_TIM_Base_Init(bsp->tim_buzzer);
	}

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = on ? 50:0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(bsp->tim_buzzer, &sConfigOC, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(bsp->tim_buzzer, TIM_CHANNEL_1);
}


void BSP_output_channel_on(uint8_t channel, bool on)
{
	if(channel > sizeof(output_channel_map)/sizeof(gpio_t)) return;

	HAL_GPIO_WritePin(output_channel_map[channel].port, output_channel_map[channel].pin, on ? GPIO_PIN_SET:GPIO_PIN_RESET);
}


uint16_t BSP_output_channel_get_current(uint8_t channel)
{
	return 0;
}


void BSP_motor_control(bsp_mtor_side_t side, int8_t duty_cycle)
{

}


uint16_t BSP_motor_get_current(bsp_mtor_side_t side)
{
	return 0;
}


uint16_t BSP_analog_in_read(uint8_t channel)
{

	return 0;
}


void BSP_analog_out_set(uint8_t channel, uint8_t value_percent)
{

}


void BSP_pin_interrupt_enable(uint8_t channel, bsp_pin_irq_evt_t evt)
{
	if(channel > sizeof(input_channel_map)/sizeof(bsp_input_irq_t)) return;

	// Get the initial pin state
	input_channel_map[channel].state = HAL_GPIO_ReadPin(input_channel_map[channel].gpio.port, input_channel_map[channel].gpio.pin);

	input_channel_map[channel].irq_evt = evt;
}


void BSP_pin_interrupt_disable(uint8_t channel)
{
	if(channel > sizeof(input_channel_map)/sizeof(bsp_input_irq_t)) return;

	input_channel_map[channel].irq_evt = INTERRUPT_NONE;
}


bsp_adc_data_t * BSP_get_adc_readings()
{
	return &adc_readings;
}


void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
	bsp_input_irq_t * p_irq = NULL;
	uint8_t channel;

	for(channel = 0; channel<sizeof(input_channel_map)/sizeof(bsp_input_irq_t); channel++){
		if(input_channel_map[channel].gpio.pin == gpio_pin){
			p_irq = &(input_channel_map[channel]);
			break;
		}
	}

	if(p_irq != NULL){
		uint8_t new_state = HAL_GPIO_ReadPin(p_irq->gpio.port, p_irq->gpio.pin);

		bsp_pin_irq_evt_t evt = INTERRUPT_NONE;
		if(new_state == 1 && p_irq->state == 0) evt = INTERRUPT_LO_TO_HI;
		if(new_state == 0 && p_irq->state == 1) evt = INTERRUPT_HI_TO_LO;

		// Call the application IRQ handler
		if(evt & p_irq->irq_evt) bsp->app_pin_irq_handler(channel, evt);

		p_irq->state = new_state;
	}
}


static void scheduler_restart_adc(void * handle)
{
	HAL_ADC_Start_DMA(bsp->adc_csense, adc_readings.current_sense_mv, OUTPUT_BANK_SIZE+1);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == bsp->adc_csense){
		// Active bank is the currently active CS_sel pin of the multiplexed output current pins
//		static uint8_t active_bank = 0;
//
//		if(active_bank == 2) active_bank = 0;
//		else active_bank++;
//
//		// Select the correct drive chip current pin
//		HAL_GPIO_WritePin(CSENSE_P1_SEL_GPIO_Port, CSENSE_P1_SEL_Pin, active_bank == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
//		HAL_GPIO_WritePin(CSENSE_P2_SEL_GPIO_Port, CSENSE_P2_SEL_Pin, active_bank == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
//		HAL_GPIO_WritePin(CSENSE_P3_SEL_GPIO_Port, CSENSE_P3_SEL_Pin, active_bank == 2 ? GPIO_PIN_RESET : GPIO_PIN_SET);


		// Schedule next current measurement in 1ms to allow reading to settle
		SCH_add(scheduler_restart_adc, bsp->adc_csense, 1, 0);
	}
}


