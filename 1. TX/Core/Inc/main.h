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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define ON 1
#define OFF 0

#define NUM_OF_TX 1				// Number from 1 to 6
#define SLEEP_MODE ON			// ON or OFF sleep mode

// Turn OFF for better power consumption
#define UART_LOG OFF
#define LED OFF

// Available transmit period
#define SLEEP_TIME_10_SEC 1
#define SLEEP_TIME_30_SEC 5
#define SLEEP_TIME_1_MIN 12

#define SLEEP_TIME SLEEP_TIME_1_MIN		// Set transmit period

#define RED_LED_ON HAL_GPIO_WritePin(GPIOB, LED_Red_Pin, GPIO_PIN_SET)
#define RED_LED_OFF HAL_GPIO_WritePin(GPIOB, LED_Red_Pin, GPIO_PIN_RESET)
#define RED_LED_TOGLE HAL_GPIO_TogglePin(GPIOB, LED_Red_Pin)

#define GREEN_LED_ON HAL_GPIO_WritePin(GPIOB, LED_Green_Pin, GPIO_PIN_SET)
#define GREEN_LED_OFF HAL_GPIO_WritePin(GPIOB, LED_Green_Pin, GPIO_PIN_RESET)
#define GREEN_LED_TOGLE HAL_GPIO_TogglePin(GPIOB, LED_Green_Pin)

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
#define nrf_IRQ_Pin GPIO_PIN_2
#define nrf_IRQ_GPIO_Port GPIOA
#define nrf_IRQ_EXTI_IRQn EXTI2_IRQn
#define nrf_CE_Pin GPIO_PIN_3
#define nrf_CE_GPIO_Port GPIOA
#define nrf_CS_Pin GPIO_PIN_4
#define nrf_CS_GPIO_Port GPIOA
#define ACTION_Pin GPIO_PIN_12
#define ACTION_GPIO_Port GPIOA
#define LED_Red_Pin GPIO_PIN_8
#define LED_Red_GPIO_Port GPIOB
#define LED_Green_Pin GPIO_PIN_9
#define LED_Green_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
