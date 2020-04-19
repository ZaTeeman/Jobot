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
char MotorControl (void);
void ShedulerOfRoad(void);
void ShedulerOfLoad(void);
void ShedulerOfMove(void);
uint8_t CanTxMsg(uint64_t  Data, uint16_t Addres, uint8_t size);
uint8_t UARTTxMsg(uint32_t  Data, uint8_t size);
uint8_t CalobrationDistance (void);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define ZOMMER_Pin GPIO_PIN_14
#define ZOMMER_GPIO_Port GPIOC
#define DIR_Pin GPIO_PIN_0
#define DIR_GPIO_Port GPIOA
#define STEP_Pin GPIO_PIN_1
#define STEP_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_2
#define ENABLE_GPIO_Port GPIOA
#define LimSwHIGH_Pin GPIO_PIN_12
#define LimSwHIGH_GPIO_Port GPIOB
#define LimSwLOW_Pin GPIO_PIN_13
#define LimSwLOW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
