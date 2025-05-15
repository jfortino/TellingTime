/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
	MENU_IDLE,
	MENU_SET_TIME,
	MENU_SET_DATE,
	MENU_FORMAT_TIME,
	MENU_FORMAT_DATE,
	MENU_SET_VIB_LEN,
	MENU_SET_HOUR,
	MENU_SET_MINUTE,
	MENU_SET_FORMAT_TIME,
	MENU_SET_FORMAT_DATE,
	MENU_CHANGE_VIB_LENGTH,
	MENU_SET_DAY,
	MENU_SET_MONTH,
	MENU_SET_YEAR
} EMenuFSMState;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UP_BTN_Pin GPIO_PIN_13
#define UP_BTN_GPIO_Port GPIOC
#define UP_BTN_EXTI_IRQn EXTI4_15_IRQn
#define DATE_BTN_Pin GPIO_PIN_15
#define DATE_BTN_GPIO_Port GPIOC
#define DATE_BTN_EXTI_IRQn EXTI4_15_IRQn
#define VIB_MOTOR_Pin GPIO_PIN_0
#define VIB_MOTOR_GPIO_Port GPIOH
#define FLASH_CS_Pin GPIO_PIN_1
#define FLASH_CS_GPIO_Port GPIOH
#define TIME_BTN_Pin GPIO_PIN_0
#define TIME_BTN_GPIO_Port GPIOA
#define TIME_BTN_EXTI_IRQn EXTI0_1_IRQn
#define SEL_BTN_Pin GPIO_PIN_1
#define SEL_BTN_GPIO_Port GPIOA
#define SEL_BTN_EXTI_IRQn EXTI0_1_IRQn
#define FLASH_RST_Pin GPIO_PIN_2
#define FLASH_RST_GPIO_Port GPIOA
#define DOWN_BTN_Pin GPIO_PIN_3
#define DOWN_BTN_GPIO_Port GPIOA
#define DOWN_BTN_EXTI_IRQn EXTI2_3_IRQn
#define PO_BTN_Pin GPIO_PIN_4
#define PO_BTN_GPIO_Port GPIOA
#define PO_BTN_EXTI_IRQn EXTI4_15_IRQn
#define VBATT_READ_Pin GPIO_PIN_0
#define VBATT_READ_GPIO_Port GPIOB
#define VBATT_CTRL_Pin GPIO_PIN_1
#define VBATT_CTRL_GPIO_Port GPIOB
#define MCP_STAT_Pin GPIO_PIN_2
#define MCP_STAT_GPIO_Port GPIOB
#define MCP_STAT_EXTI_IRQn EXTI2_3_IRQn
#define FLASH_WP_Pin GPIO_PIN_10
#define FLASH_WP_GPIO_Port GPIOB
#define HR_BTN_Pin GPIO_PIN_11
#define HR_BTN_GPIO_Port GPIOB
#define HR_BTN_EXTI_IRQn EXTI4_15_IRQn
#define VOL_UP_BTN_Pin GPIO_PIN_14
#define VOL_UP_BTN_GPIO_Port GPIOB
#define VOL_UP_BTN_EXTI_IRQn EXTI4_15_IRQn
#define VOL_DOWN_BTN_Pin GPIO_PIN_8
#define VOL_DOWN_BTN_GPIO_Port GPIOA
#define VOL_DOWN_BTN_EXTI_IRQn EXTI4_15_IRQn
#define SD_MODE_Pin GPIO_PIN_15
#define SD_MODE_GPIO_Port GPIOA
#define GAIN_12DB_Pin GPIO_PIN_3
#define GAIN_12DB_GPIO_Port GPIOB
#define GAIN_15DB_Pin GPIO_PIN_4
#define GAIN_15DB_GPIO_Port GPIOB
#define GAIN_9DB_Pin GPIO_PIN_5
#define GAIN_9DB_GPIO_Port GPIOB
#define GAIN_3DB_Pin GPIO_PIN_8
#define GAIN_3DB_GPIO_Port GPIOB
#define SPO2_INT_Pin GPIO_PIN_9
#define SPO2_INT_GPIO_Port GPIOB
#define SPO2_INT_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
