/*
 * scheduler.c
 *
 *	Created on: 29.08.2016
 *      Author: wn
 *
 *  Modified on: Mar 12, 2020
 *      Author: Dominic
 *
 *      Original source at https://gitlab.com/wolutator/TeaThermoTimer/-/blob/master/src/PontCoopScheduler.c
 *      Made some modifications for use in the CUeVCM firmware
 */


#include <stdlib.h>

#include "scheduler.h"

static tTask tasks[MAX_NUM_OF_TASKS];
static TIM_HandleTypeDef * scheduler_timer;


void SCH_init(TIM_HandleTypeDef * tim_handler) {
  for (uint8_t i = 0; i < MAX_NUM_OF_TASKS; i++) {
    tasks[i].delay = 0;
    tasks[i].period = 0;
    tasks[i].run = 0;
    tasks[i].exec = NULL;
    tasks[i].handle = NULL;
  }

  scheduler_timer = tim_handler;
  HAL_TIM_Base_Start_IT(scheduler_timer);
}

void SCH_add(void (*exec)(void *), void *handle, uint32_t delay, uint32_t period) {
  for (uint8_t i = 0; i < MAX_NUM_OF_TASKS; i++) {
    if (tasks[i].exec == NULL) {
      tasks[i].delay = delay;
      tasks[i].period = period;
      tasks[i].run = 0;
      tasks[i].exec = exec;
      tasks[i].handle = handle;
      break;
    }
  }
}

void SCH_del(void (*exec)(void *), void *handle) {
  for (uint8_t i = 0; i < MAX_NUM_OF_TASKS; i++) {
    if ((tasks[i].exec == exec) && (tasks[i].handle == handle)) {
      tasks[i].exec = NULL;
      break;
    }
  }
}

void SCH_exec() {
  for (uint8_t i = 0; i < MAX_NUM_OF_TASKS; i++) {
    if (tasks[i].exec != NULL && tasks[i].run > 0) {
      tasks[i].run--;
      tasks[i].exec(tasks[i].handle);
      if (tasks[i].period == 0) {
        tasks[i].exec = NULL;
      }
    }
  }
}



void SCH_update() {
  for (uint8_t i = 0; i < MAX_NUM_OF_TASKS; i++) {
    if (tasks[i].exec != NULL) {
      if (tasks[i].delay == 0) {
        tasks[i].delay = tasks[i].period;
        tasks[i].run++;
      } else {
        tasks[i].delay--;
      }
    }
  }
}
