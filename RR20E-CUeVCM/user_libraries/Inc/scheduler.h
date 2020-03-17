/*
 * scheduler.h
 *
 *  Created on: Mar 12, 2020
 *      Author: Dominic
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_


#include <stdint.h>
#include "stm32f4xx_hal.h"

#define MAX_NUM_OF_TASKS 8


typedef struct {
  uint32_t delay;
  uint32_t period;
  uint8_t run;
  void (*exec)(void *handle);
  void *handle;
} tTask;


void SCH_init(TIM_HandleTypeDef * tim_handler);
void SCH_add(void (*exec)(void *), void *handle, uint32_t delay, uint32_t period);
void SCH_del(void (*exec)(void *), void *handle);
void SCH_exec();
void SCH_update();


#endif
