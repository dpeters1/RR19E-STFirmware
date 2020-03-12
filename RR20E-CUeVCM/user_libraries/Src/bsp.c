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


void BSP_buzzer_on(bool on, bsp_buzzer_pitch_t pitch)
{
	if(on){
		HAL_TIM_Base_DeInit(bsp.tim_buzzer);

		switch(pitch){
		case BUZZER_PITCH_LOW:
			bsp.tim_buzzer->Init.Prescaler = 500-1;
			break;
		case BUZZER_PITCH_MED:
				bsp.tim_buzzer->Init.Prescaler = 450-1;
				break;
		case BUZZER_PITCH_HIGH:
				bsp.tim_buzzer->Init.Prescaler = 400-1;
				break;
		}

		HAL_TIM_Base_Init(bsp.tim_buzzer);
	}

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = on ? 50:0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(bsp.tim_buzzer, &sConfigOC, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(bsp.tim_buzzer, TIM_CHANNEL_1);
}
