/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define SW_TopLeft_Pin GPIO_PIN_0
#define SW_TopLeft_GPIO_Port GPIOF
#define SW_TopRight_Pin GPIO_PIN_1
#define SW_TopRight_GPIO_Port GPIOF
#define SPI_CS_ACC_Pin GPIO_PIN_4
#define SPI_CS_ACC_GPIO_Port GPIOA
#define SPI_IMU_SCK_Pin GPIO_PIN_5
#define SPI_IMU_SCK_GPIO_Port GPIOA
#define SPI_IMU_MISO_Pin GPIO_PIN_6
#define SPI_IMU_MISO_GPIO_Port GPIOA
#define SPI_IMU_MOSI_Pin GPIO_PIN_7
#define SPI_IMU_MOSI_GPIO_Port GPIOA
#define SPI_CS_GYRO_Pin GPIO_PIN_0
#define SPI_CS_GYRO_GPIO_Port GPIOB
#define IMU_INT_ACC_Pin GPIO_PIN_11
#define IMU_INT_ACC_GPIO_Port GPIOA
#define IMU_INT_GYRO_Pin GPIO_PIN_12
#define IMU_INT_GYRO_GPIO_Port GPIOA
#define I2C_INA_SCL_Pin GPIO_PIN_15
#define I2C_INA_SCL_GPIO_Port GPIOA
#define SW_BotLeft_Pin GPIO_PIN_4
#define SW_BotLeft_GPIO_Port GPIOB
#define SW_BotRight_Pin GPIO_PIN_5
#define SW_BotRight_GPIO_Port GPIOB
#define RGB_LED_Pin GPIO_PIN_6
#define RGB_LED_GPIO_Port GPIOB
#define I2C_INA_SDA_Pin GPIO_PIN_7
#define I2C_INA_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
