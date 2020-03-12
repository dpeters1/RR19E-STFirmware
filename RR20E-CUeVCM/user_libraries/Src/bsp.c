/*
 * bsp.c
 *
 *  Created on: Mar 12, 2020
 *      Author: Dominic
 */

#include "bsp.h"

static bsp_handler_t bsp;


void BSP_init(TIM_HandleTypeDef * p_buzzer_tim)
{
	bsp.tim_buzzer = p_buzzer_tim;
}

void BSP_fault_led_on(bool on)
{
	HAL_GPIO_WritePin(OUT_LED_FAULT_GPIO_Port, OUT_LED_FAULT_Pin, on ? GPIO_PIN_SET:GPIO_PIN_RESET);
}


void BSP_buzzer_on(bool on)
{
	HAL_StatusTypeDef ret_code;

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = on ? 50:0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	// All LEDs off
	ret_code = HAL_TIM_PWM_ConfigChannel(bsp.tim_buzzer, &sConfigOC, TIM_CHANNEL_1);

	if(!ret_code)printd("pwm success\n");
}
