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
#include "stm32f1xx_hal.h"

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
void RxAnswerFunction(void);
char MotorControl (void);
void SpeedColibration(void);
uint8_t CalobrationDistance (void);
void LimSwChack (void);
uint8_t CanTxMsg(uint64_t  Data, uint16_t Addres, uint8_t size);
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
	#define    DWT_CYCCNT    *(volatile uint32_t*)0xE0001004
  #define    DWT_CONTROL   *(volatile uint32_t*)0xE0001000
  #define    SCB_DEMCR     *(volatile uint32_t*)0xE000EDFC
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Tim2Ch1_Pin GPIO_PIN_0
#define Tim2Ch1_GPIO_Port GPIOA
#define Tim2Ch2_Pin GPIO_PIN_1
#define Tim2Ch2_GPIO_Port GPIOA
#define Tim2Ch3_Pin GPIO_PIN_2
#define Tim2Ch3_GPIO_Port GPIOA
#define PowerMon_Pin GPIO_PIN_15
#define PowerMon_GPIO_Port GPIOB
#define PowerMon_EXTI_IRQn EXTI15_10_IRQn
#define LimSwLOW_Pin GPIO_PIN_8
#define LimSwLOW_GPIO_Port GPIOA
#define LimSwHIGH_Pin GPIO_PIN_9
#define LimSwHIGH_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_10
#define ENABLE_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_8
#define DIR_GPIO_Port GPIOB
#define STEP_Pin GPIO_PIN_9
#define STEP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
