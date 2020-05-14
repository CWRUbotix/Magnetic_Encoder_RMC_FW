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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC_ABS_Pin GPIO_PIN_1
#define ENC_ABS_GPIO_Port GPIOA
#define CAN_ID_128_Pin GPIO_PIN_3
#define CAN_ID_128_GPIO_Port GPIOA
#define CAN_ID_64_Pin GPIO_PIN_4
#define CAN_ID_64_GPIO_Port GPIOA
#define CAN_ID_32_Pin GPIO_PIN_0
#define CAN_ID_32_GPIO_Port GPIOB
#define CAN_ID_16_Pin GPIO_PIN_1
#define CAN_ID_16_GPIO_Port GPIOB
#define CAN_ID_8_Pin GPIO_PIN_2
#define CAN_ID_8_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_10
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_11
#define I2C_SDA_GPIO_Port GPIOB
#define CAN_ID_4_Pin GPIO_PIN_12
#define CAN_ID_4_GPIO_Port GPIOB
#define CAN_ID_2_Pin GPIO_PIN_13
#define CAN_ID_2_GPIO_Port GPIOB
#define CAN_ID_1_Pin GPIO_PIN_14
#define CAN_ID_1_GPIO_Port GPIOB
#define POWER_SENSE_Pin GPIO_PIN_15
#define POWER_SENSE_GPIO_Port GPIOB
#define ENC_A_Pin GPIO_PIN_8
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_9
#define ENC_B_GPIO_Port GPIOA
#define ENC_I_Pin GPIO_PIN_10
#define ENC_I_GPIO_Port GPIOA
#define ENC_I_EXTI_IRQn EXTI15_10_IRQn
#define STATUS_3_Pin GPIO_PIN_3
#define STATUS_3_GPIO_Port GPIOB
#define STATUS_2_Pin GPIO_PIN_4
#define STATUS_2_GPIO_Port GPIOB
#define STATUS_1_Pin GPIO_PIN_5
#define STATUS_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
