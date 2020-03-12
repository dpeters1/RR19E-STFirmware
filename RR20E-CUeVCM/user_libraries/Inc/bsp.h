/*
 * bsp.h
 *
 *  Created on: Mar 12, 2020
 *      Author: Dominic
 */

#ifndef INC_BSP_H_
#define INC_BSP_H_

#include "main.h"

typedef struct{
	TIM_HandleTypeDef * tim_buzzer;
} bsp_handler_t;

/* Public function prototypes */
void BSP_init(TIM_HandleTypeDef * p_buzzer_tim);
void BSP_fault_led_on(bool on);
void BSP_buzzer_on(bool on);

#endif /* INC_BSP_H_ */
