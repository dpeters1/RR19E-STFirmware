/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* Uncomment when debugging for printf support */
#define DEBUG

/* Code snippet to insert in the mx_gpio configuration after re-generating with cubeMX */
//#ifndef DEBUG
//  GPIO_InitStruct.Pin |= DBG_LED_Pin;
//#endif
/***********************************************/

#ifdef DEBUG
#define printd(x) printf(x)
#else
#define printd(x) do {} while (0)
#endif
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_CTRL_A_Pin GPIO_PIN_2
#define MOTOR_CTRL_A_GPIO_Port GPIOE
#define MTOR_CSENSE_SEL_Pin GPIO_PIN_3
#define MTOR_CSENSE_SEL_GPIO_Port GPIOE
#define PWM_MOTOR_Pin GPIO_PIN_5
#define PWM_MOTOR_GPIO_Port GPIOE
#define LOAD_OUT2_1_Pin GPIO_PIN_6
#define LOAD_OUT2_1_GPIO_Port GPIOE
#define CSENSE_P1_SEL_Pin GPIO_PIN_13
#define CSENSE_P1_SEL_GPIO_Port GPIOC
#define LOAD_OUT1_1_Pin GPIO_PIN_14
#define LOAD_OUT1_1_GPIO_Port GPIOC
#define LOAD_OUT1_2_Pin GPIO_PIN_15
#define LOAD_OUT1_2_GPIO_Port GPIOC
#define ADC1_HDR_Pin GPIO_PIN_0
#define ADC1_HDR_GPIO_Port GPIOC
#define ADC2_HDR_Pin GPIO_PIN_1
#define ADC2_HDR_GPIO_Port GPIOC
#define ADC_MTOR_CSENSE_Pin GPIO_PIN_2
#define ADC_MTOR_CSENSE_GPIO_Port GPIOC
#define ADC_PWR_CSENSE4_Pin GPIO_PIN_3
#define ADC_PWR_CSENSE4_GPIO_Port GPIOC
#define ADC_PWR_CSENSE3_Pin GPIO_PIN_0
#define ADC_PWR_CSENSE3_GPIO_Port GPIOA
#define ADC_PWR_CSENSE2_Pin GPIO_PIN_1
#define ADC_PWR_CSENSE2_GPIO_Port GPIOA
#define ADC_PWR_CSENSE1_Pin GPIO_PIN_2
#define ADC_PWR_CSENSE1_GPIO_Port GPIOA
#define LOAD_OUT1_3_Pin GPIO_PIN_3
#define LOAD_OUT1_3_GPIO_Port GPIOA
#define DAC_OUT1_Pin GPIO_PIN_4
#define DAC_OUT1_GPIO_Port GPIOA
#define DAC_OUT2_Pin GPIO_PIN_5
#define DAC_OUT2_GPIO_Port GPIOA
#define LOAD_OUT2_4_Pin GPIO_PIN_6
#define LOAD_OUT2_4_GPIO_Port GPIOA
#define LOAD_OUT1_4_Pin GPIO_PIN_7
#define LOAD_OUT1_4_GPIO_Port GPIOA
#define ANLG_INPUT3_Pin GPIO_PIN_4
#define ANLG_INPUT3_GPIO_Port GPIOC
#define ANLG_INPUT1_Pin GPIO_PIN_5
#define ANLG_INPUT1_GPIO_Port GPIOC
#define ANLG_INPUT4_Pin GPIO_PIN_0
#define ANLG_INPUT4_GPIO_Port GPIOB
#define ANLG_INPUT2_Pin GPIO_PIN_1
#define ANLG_INPUT2_GPIO_Port GPIOB
#define LOAD_OUT2_3_Pin GPIO_PIN_10
#define LOAD_OUT2_3_GPIO_Port GPIOE
#define LOAD_OUT_2_2_Pin GPIO_PIN_11
#define LOAD_OUT_2_2_GPIO_Port GPIOE
#define CSENSE_P2_SEL_Pin GPIO_PIN_12
#define CSENSE_P2_SEL_GPIO_Port GPIOE
#define LOOAD_OUT3_1_Pin GPIO_PIN_13
#define LOOAD_OUT3_1_GPIO_Port GPIOE
#define LOAD_OUT3_2_Pin GPIO_PIN_14
#define LOAD_OUT3_2_GPIO_Port GPIOE
#define LOAD_OUT3_3_Pin GPIO_PIN_15
#define LOAD_OUT3_3_GPIO_Port GPIOE
#define LOAD_OUT3_4_Pin GPIO_PIN_10
#define LOAD_OUT3_4_GPIO_Port GPIOB
#define CSENSE_P3_SEL_Pin GPIO_PIN_11
#define CSENSE_P3_SEL_GPIO_Port GPIOB
#define CANBUS_RX_Pin GPIO_PIN_12
#define CANBUS_RX_GPIO_Port GPIOB
#define CANBUS_TX_Pin GPIO_PIN_13
#define CANBUS_TX_GPIO_Port GPIOB
#define EXTI8_INPUT_Pin GPIO_PIN_8
#define EXTI8_INPUT_GPIO_Port GPIOD
#define EXTI9_INPUT_Pin GPIO_PIN_9
#define EXTI9_INPUT_GPIO_Port GPIOD
#define EXTI10_INPUT_Pin GPIO_PIN_10
#define EXTI10_INPUT_GPIO_Port GPIOD
#define EXTI11_INPUT_Pin GPIO_PIN_11
#define EXTI11_INPUT_GPIO_Port GPIOD
#define EXTI12_INPUT_Pin GPIO_PIN_12
#define EXTI12_INPUT_GPIO_Port GPIOD
#define EXTI13_INPUT_Pin GPIO_PIN_13
#define EXTI13_INPUT_GPIO_Port GPIOD
#define EXTI14_INPUT_Pin GPIO_PIN_14
#define EXTI14_INPUT_GPIO_Port GPIOD
#define EXTI15_INPUT_Pin GPIO_PIN_15
#define EXTI15_INPUT_GPIO_Port GPIOD
#define EXTI6_HDR_Pin GPIO_PIN_6
#define EXTI6_HDR_GPIO_Port GPIOC
#define EXTI7_HDR_Pin GPIO_PIN_7
#define EXTI7_HDR_GPIO_Port GPIOC
#define HDR_SPI_CS_Pin GPIO_PIN_8
#define HDR_SPI_CS_GPIO_Port GPIOC
#define I2C_SDA_AUX_Pin GPIO_PIN_9
#define I2C_SDA_AUX_GPIO_Port GPIOC
#define I2C_SCL_AUX_Pin GPIO_PIN_8
#define I2C_SCL_AUX_GPIO_Port GPIOA
#define UART_TX_BT_Pin GPIO_PIN_9
#define UART_TX_BT_GPIO_Port GPIOA
#define UART_RX_BT_Pin GPIO_PIN_10
#define UART_RX_BT_GPIO_Port GPIOA
#define UART_CTS_BT_Pin GPIO_PIN_11
#define UART_CTS_BT_GPIO_Port GPIOA
#define UART_RTS_BT_Pin GPIO_PIN_12
#define UART_RTS_BT_GPIO_Port GPIOA
#define PWM_BUZZER_Pin GPIO_PIN_15
#define PWM_BUZZER_GPIO_Port GPIOA
#define SPI_SCK_AUX_Pin GPIO_PIN_10
#define SPI_SCK_AUX_GPIO_Port GPIOC
#define SPI_MISO_AUX_Pin GPIO_PIN_11
#define SPI_MISO_AUX_GPIO_Port GPIOC
#define SPI_MOSI_AUX_Pin GPIO_PIN_12
#define SPI_MOSI_AUX_GPIO_Port GPIOC
#define SPI_CS_AUX_Pin GPIO_PIN_0
#define SPI_CS_AUX_GPIO_Port GPIOD
#define BT_RST_Pin GPIO_PIN_1
#define BT_RST_GPIO_Port GPIOD
#define BT_EAN_Pin GPIO_PIN_2
#define BT_EAN_GPIO_Port GPIOD
#define BT_CFG1_Pin GPIO_PIN_3
#define BT_CFG1_GPIO_Port GPIOD
#define BT_CFG2_Pin GPIO_PIN_4
#define BT_CFG2_GPIO_Port GPIOD
#define BT_SW_BTN_Pin GPIO_PIN_5
#define BT_SW_BTN_GPIO_Port GPIOD
#define SPI_SCK_HDR_Pin GPIO_PIN_3
#define SPI_SCK_HDR_GPIO_Port GPIOB
#define SPI_MISO_HDR_Pin GPIO_PIN_4
#define SPI_MISO_HDR_GPIO_Port GPIOB
#define SPI_MOSI_HDR_Pin GPIO_PIN_5
#define SPI_MOSI_HDR_GPIO_Port GPIOB
#define OUT_LED_FAULT_Pin GPIO_PIN_7
#define OUT_LED_FAULT_GPIO_Port GPIOB
#define MOTOR_CTRL_B_Pin GPIO_PIN_1
#define MOTOR_CTRL_B_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define DBG_LED_GPIO_Port GPIOA
#define DBG_LED_Pin GPIO_PIN_13
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
