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
		HAL_ADC_Start_DMA(bsp->adc_anlg_in, adc_readings.analog_input, NUM_ANLG_INPUT_CHANNELS);
	}

	if(bsp->adc_csense != NULL){
		HAL_ADC_Start_DMA(bsp->adc_csense, adc_readings.load_current, OUTPUT_BANK_SIZE+1);
	}

	// Initialize the  analog outputs to zero
	BSP_set_analog_output(0, 0);
	BSP_set_analog_output(1, 0);

	// Turn off the buzzer
	BSP_set_buzzer(BUZZER_OFF);

	// Set the motor output to zero
	BSP_set_motor_output(MTOR_DIR_FORWARD, 0);
}

void BSP_set_fault_led(BSP_bool_t on)
{
	HAL_GPIO_WritePin(OUT_LED_FAULT_GPIO_Port, OUT_LED_FAULT_Pin, on ? GPIO_PIN_SET:GPIO_PIN_RESET);
}


void BSP_set_buzzer(bsp_buzzer_noise_t noise)
{
	if(bsp->tim_buzzer == NULL) return;

	switch(noise){
	case BUZZER_OFF:
		break;
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

	HAL_TIM_Base_DeInit(bsp->tim_buzzer);
	HAL_TIM_Base_Init(bsp->tim_buzzer);

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = noise == BUZZER_OFF ? 0 : 50;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(bsp->tim_buzzer, &sConfigOC, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(bsp->tim_buzzer, TIM_CHANNEL_1);
}


void BSP_set_load_output(uint8_t channel, BSP_bool_t on)
{
	if(channel > sizeof(output_channel_map)/sizeof(gpio_t)) return;

	HAL_GPIO_WritePin(output_channel_map[channel].port, output_channel_map[channel].pin, on ? GPIO_PIN_SET:GPIO_PIN_RESET);
}


uint16_t BSP_get_load_current(uint8_t channel)
{
	if(channel >=  NUM_OUTPUT_CHANNELS) return 0xFFFF;
	// 3300: 3.3V ADC reference voltage
	// 4096: 2^12 bits of resolution
	// 500: Load driver current sense ratio (see datasheet)
	// 330: Current sense resistor
	return adc_readings.load_current[channel] * 3300 / 4096 * 500 / 330;
}


void BSP_set_motor_output(bsp_mtor_direction_t dir, uint8_t duty_cycle)
{
	if(bsp->tim_motor == NULL) return;
	if(duty_cycle > 100) return;

	HAL_GPIO_WritePin(MTOR_CSENSE_SEL_GPIO_Port, MTOR_CSENSE_SEL_Pin, dir ? GPIO_PIN_RESET : GPIO_PIN_SET);

	HAL_GPIO_WritePin(MOTOR_CTRL_A_GPIO_Port, MOTOR_CTRL_A_Pin, dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_CTRL_B_GPIO_Port, MOTOR_CTRL_B_Pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = duty_cycle;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(bsp->tim_motor, &sConfigOC, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(bsp->tim_motor, TIM_CHANNEL_1);
}


uint16_t BSP_get_motor_current()
{
	// 3300: 3.3V ADC reference voltage
	// 4096: 2^12 bits of resolution
	// 1540: Load driver current sense ratio (see datasheet)
	// 680: Current sense resistor
	return adc_readings.motor_current * 3300 / 4096 * 1540 / 680;
}


uint16_t BSP_get_analog_input(uint8_t channel)
{

	return adc_readings.analog_input[channel];
}

/*
 * Note: The output impedance can be changed by enabling/disabling the output buffer in cubeMX
 * With unbuffered output, you have a voltage swing nearly between the rails, and an output impedance of 1 Mega-Ohm.
 * With buffered output, you have 15k output impedance, but reduced swing of up to 200mV at both rails (+200mV .. VRef-200mV
 */
void BSP_set_analog_output(uint8_t channel, uint16_t value_12b)
{
	if(value_12b > 4095) return;
	if(channel >= NUM_ANLG_OUT_CHANNELS) return;
	if(bsp->dac_anlg_out == NULL) return;

	uint32_t dac_channel = channel ? DAC_CHANNEL_2 : DAC_CHANNEL_1;
	uint32_t dac_value = value_12b;

	HAL_DAC_SetValue(bsp->dac_anlg_out, dac_channel, DAC_ALIGN_12B_R, dac_value);
	HAL_DAC_Start(bsp->dac_anlg_out, dac_channel);
}


uint8_t BSP_get_digital_input(uint8_t channel)
{
	if(channel > sizeof(input_channel_map)/sizeof(bsp_input_irq_t)) return 0;

	input_channel_map[channel].state = HAL_GPIO_ReadPin(input_channel_map[channel].gpio.port, input_channel_map[channel].gpio.pin);

	return input_channel_map[channel].state;
}


void BSP_enable_interrupt(uint8_t channel, bsp_pin_irq_evt_t evt)
{
	if(channel > sizeof(input_channel_map)/sizeof(bsp_input_irq_t)) return;

	// Get the initial pin state
	input_channel_map[channel].state = HAL_GPIO_ReadPin(input_channel_map[channel].gpio.port, input_channel_map[channel].gpio.pin);

	input_channel_map[channel].irq_evt = evt;
}


void BSP_disable_interrupt(uint8_t channel)
{
	if(channel > sizeof(input_channel_map)/sizeof(bsp_input_irq_t)) return;

	input_channel_map[channel].irq_evt = INTERRUPT_NONE;
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


static void scheduler_restart_adc(void * p_active_bank)
{
	uint8_t active_bank = *(uint8_t *)p_active_bank;
	uint32_t num_measurements = OUTPUT_BANK_SIZE;
	if(active_bank == 2) num_measurements++; // Also sample the motor driver current
	HAL_ADC_Start_DMA(bsp->adc_csense, &(adc_readings.load_current[active_bank*OUTPUT_BANK_SIZE]), num_measurements);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == bsp->adc_csense){
		// Active bank is the currently active CS_sel pin of the multiplexed output current pins
		static uint8_t active_bank = 0;

		if(active_bank == 2) active_bank = 0;
		else active_bank++;

		// Select the correct drive chip current pin
		HAL_GPIO_WritePin(CSENSE_P1_SEL_GPIO_Port, CSENSE_P1_SEL_Pin, active_bank == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(CSENSE_P2_SEL_GPIO_Port, CSENSE_P2_SEL_Pin, active_bank == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(CSENSE_P3_SEL_GPIO_Port, CSENSE_P3_SEL_Pin, active_bank == 2 ? GPIO_PIN_RESET : GPIO_PIN_SET);

		HAL_ADC_Stop_DMA(hadc);
		// Schedule next current measurement in 1ms to allow reading to settle
		scheduler_add(scheduler_restart_adc, &active_bank, 1, 0);
	}
}


