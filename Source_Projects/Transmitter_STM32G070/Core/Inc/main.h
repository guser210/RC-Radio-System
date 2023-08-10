/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
void maincpp();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GIMBAL_CH3_Pin GPIO_PIN_0
#define GIMBAL_CH3_GPIO_Port GPIOA
#define GIMBAL_CH4_Pin GPIO_PIN_1
#define GIMBAL_CH4_GPIO_Port GPIOA
#define SWITCH_1_Pin GPIO_PIN_2
#define SWITCH_1_GPIO_Port GPIOA
#define SWITCH_2_Pin GPIO_PIN_3
#define SWITCH_2_GPIO_Port GPIOA
#define SWITCH_3_Pin GPIO_PIN_4
#define SWITCH_3_GPIO_Port GPIOA
#define SWITCH_4_Pin GPIO_PIN_5
#define SWITCH_4_GPIO_Port GPIOA
#define SWITCH_5_Pin GPIO_PIN_6
#define SWITCH_5_GPIO_Port GPIOA
#define SWITCH_6_Pin GPIO_PIN_7
#define SWITCH_6_GPIO_Port GPIOA
#define GIMBAL_CH1_Pin GPIO_PIN_0
#define GIMBAL_CH1_GPIO_Port GPIOB
#define GIMBAL_CH2_Pin GPIO_PIN_1
#define GIMBAL_CH2_GPIO_Port GPIOB
#define BATTERY_READ_Pin GPIO_PIN_10
#define BATTERY_READ_GPIO_Port GPIOB
#define LORA_RX_EN_Pin GPIO_PIN_12
#define LORA_RX_EN_GPIO_Port GPIOB
#define LORA_SPI_SCK_Pin GPIO_PIN_13
#define LORA_SPI_SCK_GPIO_Port GPIOB
#define LORA_SPI_MISO_Pin GPIO_PIN_14
#define LORA_SPI_MISO_GPIO_Port GPIOB
#define LORA_SPI_MOSI_Pin GPIO_PIN_15
#define LORA_SPI_MOSI_GPIO_Port GPIOB
#define LORA_SPI_NSS_Pin GPIO_PIN_8
#define LORA_SPI_NSS_GPIO_Port GPIOA
#define LORA_TX_EN_Pin GPIO_PIN_9
#define LORA_TX_EN_GPIO_Port GPIOA
#define LORA_RESET_Pin GPIO_PIN_6
#define LORA_RESET_GPIO_Port GPIOC
#define LORA_BUSY_Pin GPIO_PIN_7
#define LORA_BUSY_GPIO_Port GPIOC
#define LORA_DIO1_Pin GPIO_PIN_10
#define LORA_DIO1_GPIO_Port GPIOA
#define LORA_DIO1_EXTI_IRQn EXTI4_15_IRQn
#define LORA_DIO2_Pin GPIO_PIN_11
#define LORA_DIO2_GPIO_Port GPIOA
#define LORA_DIO2_EXTI_IRQn EXTI4_15_IRQn
#define LORA_DIO3_Pin GPIO_PIN_12
#define LORA_DIO3_GPIO_Port GPIOA
#define LORA_DIO3_EXTI_IRQn EXTI4_15_IRQn
#define LED_1_Pin GPIO_PIN_15
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_0
#define LED_2_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_1
#define LED_3_GPIO_Port GPIOD
#define LED_4_Pin GPIO_PIN_2
#define LED_4_GPIO_Port GPIOD
#define TRIM_6_Pin GPIO_PIN_4
#define TRIM_6_GPIO_Port GPIOB
#define TRIM_6_EXTI_IRQn EXTI4_15_IRQn
#define TRIM_5_Pin GPIO_PIN_5
#define TRIM_5_GPIO_Port GPIOB
#define TRIM_5_EXTI_IRQn EXTI4_15_IRQn
#define TRIM_4_Pin GPIO_PIN_6
#define TRIM_4_GPIO_Port GPIOB
#define TRIM_4_EXTI_IRQn EXTI4_15_IRQn
#define TRIM_3_Pin GPIO_PIN_7
#define TRIM_3_GPIO_Port GPIOB
#define TRIM_3_EXTI_IRQn EXTI4_15_IRQn
#define TRIM_2_Pin GPIO_PIN_8
#define TRIM_2_GPIO_Port GPIOB
#define TRIM_2_EXTI_IRQn EXTI4_15_IRQn
#define TRIM_1_Pin GPIO_PIN_9
#define TRIM_1_GPIO_Port GPIOB
#define TRIM_1_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
