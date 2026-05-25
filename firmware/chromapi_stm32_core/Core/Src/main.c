/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "sts3215_hal.h"
#include "sts3215_protocol.h"
#include "ina226.h"
#include "sk6812.h"
#include "bridge.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INA226_I2C_ADDR    0x40
#define SHUNT_OHMS         0.002
#define MAX_CURRENT_A      8.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
AutoFox_INA226 gINA226;
STS3215_HAL_Handle_t hservo;

volatile uint8_t  g_reply_received  = 0;
volatile uint8_t  g_reply_id        = 0;
volatile uint8_t  g_reply_error     = 0;
volatile uint8_t  g_reply_data_len  = 0;
volatile uint8_t  g_reply_data[STS3215_REPLY_DATA_MAX] = {0};
volatile STS3215_Status_t g_reply_status = STS3215_OK;
uint32_t last_led_update = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void on_reply(const STS3215_Reply_t *reply, uint8_t idx, STS3215_Status_t status, void *ctx)
{
	(void)idx;
	(void)ctx;

	g_reply_id       = reply->id;
	g_reply_error    = reply->error;
	g_reply_status   = status;
	g_reply_data_len = reply->data_len;

	uint8_t copy_len = (reply->data_len <= STS3215_REPLY_DATA_MAX) ? reply->data_len : STS3215_REPLY_DATA_MAX;
	for (uint8_t i = 0; i < copy_len; i++) {
		g_reply_data[i] = reply->data[i];
	}

	g_reply_received = 1;
}

static void on_error(STS3215_HAL_Error_t err, void *ctx)
{
	(void)ctx;
}
__attribute__((used))
int _write(int file, char *ptr, int len)
{
	(void)file;
	for (int i = 0; i < len; i++) {
		if ((ITM->TCR & ITM_TCR_ITMENA_Msk) &&
				(ITM->TER & (1UL << 0U))) {
			ITM_SendChar((uint32_t)(uint8_t)*ptr);
		}
		ptr++;
	}
	return len;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		Bridge_TxCpltCallback();
	}
	else if (huart->Instance == USART2) {
		STS3215_HAL_TxCpltCallback(&hservo);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1) {
		Bridge_RxEventCallback(Size);
	}
	else if (huart->Instance == USART2) {
		STS3215_HAL_RxEventCallback(&hservo, Size);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		Bridge_ErrorCallback();
	}
	else if (huart->Instance == USART2) {
		STS3215_HAL_ErrorCallback(&hservo);
	}
}

void Chromapi_SystemInit(void) {
	STS3215_HAL_Init(&hservo, &huart2, 50U, on_reply, on_error, NULL);
	AutoFox_INA226_Constructor(&gINA226);
	AutoFox_INA226_Init(&gINA226, INA226_I2C_ADDR, SHUNT_OHMS, MAX_CURRENT_A);
	Bridge_Init(&huart1);
	HAL_Delay(500U);
	printf("[SYS] System Initialized\r\n");
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
 int main(void)
 {

	 /* USER CODE BEGIN 1 */

	 /* USER CODE END 1 */

	 /* MCU Configuration--------------------------------------------------------*/

	 /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	 HAL_Init();

	 /* USER CODE BEGIN Init */

	 /* USER CODE END Init */

	 /* Configure the system clock */
	 SystemClock_Config();

	 /* USER CODE BEGIN SysInit */

	 /* USER CODE END SysInit */

	 /* Initialize all configured peripherals */
	 MX_GPIO_Init();
	 MX_DMA_Init();
	 MX_USART2_UART_Init();
	 MX_I2C1_Init();
	 MX_TIM4_Init();
	 MX_USART1_UART_Init();
	 MX_SPI1_Init();
	 /* USER CODE BEGIN 2 */
	 Chromapi_SystemInit();

	 /* USER CODE END 2 */

	 /* Infinite loop */
	 /* USER CODE BEGIN WHILE */
	 while (1)
	 {

		 STS3215_HAL_Process(&hservo);
		 Bridge_Process();

		 if ((HAL_GetTick() - last_led_update) >= 500U) {
			 last_led_update = HAL_GetTick();
			 Bridge_UpdateBatteryLed();
		 }
		 /* USER CODE END WHILE */

		 /* USER CODE BEGIN 3 */
	 }
	 /* USER CODE END 3 */
 }

 /**
  * @brief System Clock Configuration
  * @retval None
  */
 void SystemClock_Config(void)
 {
	 RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	 RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	 /** Configure the main internal regulator output voltage
	  */
	 HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	 /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	 RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	 RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	 if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	 {
		 Error_Handler();
	 }

	 /** Initializes the CPU, AHB and APB buses clocks
	  */
	 RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			 |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	 RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	 RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	 {
		 Error_Handler();
	 }
 }

 /**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
 static void MX_I2C1_Init(void)
 {

	 /* USER CODE BEGIN I2C1_Init 0 */

	 /* USER CODE END I2C1_Init 0 */

	 /* USER CODE BEGIN I2C1_Init 1 */

	 /* USER CODE END I2C1_Init 1 */
	 hi2c1.Instance = I2C1;
	 hi2c1.Init.Timing = 0x00300619;
	 hi2c1.Init.OwnAddress1 = 0;
	 hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	 hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	 hi2c1.Init.OwnAddress2 = 0;
	 hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	 hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	 hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	 if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	 {
		 Error_Handler();
	 }

	 /** Configure Analogue filter
	  */
	 if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	 {
		 Error_Handler();
	 }

	 /** Configure Digital filter
	  */
	 if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 /* USER CODE BEGIN I2C1_Init 2 */

	 /* USER CODE END I2C1_Init 2 */

 }

 /**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
 static void MX_SPI1_Init(void)
 {

	 /* USER CODE BEGIN SPI1_Init 0 */

	 /* USER CODE END SPI1_Init 0 */

	 /* USER CODE BEGIN SPI1_Init 1 */

	 /* USER CODE END SPI1_Init 1 */
	 /* SPI1 parameter configuration*/
	 hspi1.Instance = SPI1;
	 hspi1.Init.Mode = SPI_MODE_MASTER;
	 hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	 hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	 hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	 hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	 hspi1.Init.NSS = SPI_NSS_SOFT;
	 hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	 hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	 hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	 hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	 hspi1.Init.CRCPolynomial = 7;
	 hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	 hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	 if (HAL_SPI_Init(&hspi1) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 /* USER CODE BEGIN SPI1_Init 2 */

	 /* USER CODE END SPI1_Init 2 */

 }

 /**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
 static void MX_TIM4_Init(void)
 {

	 /* USER CODE BEGIN TIM4_Init 0 */

	 /* USER CODE END TIM4_Init 0 */

	 TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	 TIM_MasterConfigTypeDef sMasterConfig = {0};
	 TIM_OC_InitTypeDef sConfigOC = {0};

	 /* USER CODE BEGIN TIM4_Init 1 */

	 /* USER CODE END TIM4_Init 1 */
	 htim4.Instance = TIM4;
	 htim4.Init.Prescaler = 0;
	 htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	 htim4.Init.Period = 19;
	 htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	 htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	 if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	 if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	 if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 sConfigOC.OCMode = TIM_OCMODE_PWM1;
	 sConfigOC.Pulse = 0;
	 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	 if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 /* USER CODE BEGIN TIM4_Init 2 */

	 /* USER CODE END TIM4_Init 2 */
	 HAL_TIM_MspPostInit(&htim4);

 }

 /**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
 static void MX_USART1_UART_Init(void)
 {

	 /* USER CODE BEGIN USART1_Init 0 */

	 /* USER CODE END USART1_Init 0 */

	 /* USER CODE BEGIN USART1_Init 1 */

	 /* USER CODE END USART1_Init 1 */
	 huart1.Instance = USART1;
	 huart1.Init.BaudRate = 115200;
	 huart1.Init.WordLength = UART_WORDLENGTH_8B;
	 huart1.Init.StopBits = UART_STOPBITS_1;
	 huart1.Init.Parity = UART_PARITY_NONE;
	 huart1.Init.Mode = UART_MODE_TX_RX;
	 huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	 huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	 huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	 huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	 huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	 if (HAL_UART_Init(&huart1) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 /* USER CODE BEGIN USART1_Init 2 */

	 /* USER CODE END USART1_Init 2 */

 }

 /**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
 static void MX_USART2_UART_Init(void)
 {

	 /* USER CODE BEGIN USART2_Init 0 */

	 /* USER CODE END USART2_Init 0 */

	 /* USER CODE BEGIN USART2_Init 1 */

	 /* USER CODE END USART2_Init 1 */
	 huart2.Instance = USART2;
	 huart2.Init.BaudRate = 1000000;
	 huart2.Init.WordLength = UART_WORDLENGTH_8B;
	 huart2.Init.StopBits = UART_STOPBITS_1;
	 huart2.Init.Parity = UART_PARITY_NONE;
	 huart2.Init.Mode = UART_MODE_TX_RX;
	 huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	 huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	 huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	 huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	 huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	 if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 8, 8) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 /* USER CODE BEGIN USART2_Init 2 */

	 /* USER CODE END USART2_Init 2 */

 }

 /**
  * Enable DMA controller clock
  */
 static void MX_DMA_Init(void)
 {

	 /* DMA controller clock enable */
	 __HAL_RCC_DMAMUX1_CLK_ENABLE();
	 __HAL_RCC_DMA1_CLK_ENABLE();

	 /* DMA interrupt init */
	 /* DMA1_Channel1_IRQn interrupt configuration */
	 HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
	 HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	 /* DMA1_Channel2_IRQn interrupt configuration */
	 HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
	 HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	 /* DMA1_Channel3_IRQn interrupt configuration */
	 HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 7, 0);
	 HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	 /* DMA1_Channel4_IRQn interrupt configuration */
	 HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
	 HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	 /* DMA1_Channel5_IRQn interrupt configuration */
	 HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	 HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

 }

 /**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
 static void MX_GPIO_Init(void)
 {
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	 /* USER CODE BEGIN MX_GPIO_Init_1 */

	 /* USER CODE END MX_GPIO_Init_1 */

	 /* GPIO Ports Clock Enable */
	 __HAL_RCC_GPIOF_CLK_ENABLE();
	 __HAL_RCC_GPIOA_CLK_ENABLE();
	 __HAL_RCC_GPIOB_CLK_ENABLE();

	 /*Configure GPIO pin Output Level */
	 HAL_GPIO_WritePin(SPI_CS_ACC_GPIO_Port, SPI_CS_ACC_Pin, GPIO_PIN_RESET);

	 /*Configure GPIO pin Output Level */
	 HAL_GPIO_WritePin(SPI_CS_GYRO_GPIO_Port, SPI_CS_GYRO_Pin, GPIO_PIN_RESET);

	 /*Configure GPIO pins : SW_TopLeft_Pin SW_TopRight_Pin */
	 GPIO_InitStruct.Pin = SW_TopLeft_Pin|SW_TopRight_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	 /*Configure GPIO pin : SPI_CS_ACC_Pin */
	 GPIO_InitStruct.Pin = SPI_CS_ACC_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	 HAL_GPIO_Init(SPI_CS_ACC_GPIO_Port, &GPIO_InitStruct);

	 /*Configure GPIO pin : SPI_CS_GYRO_Pin */
	 GPIO_InitStruct.Pin = SPI_CS_GYRO_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	 HAL_GPIO_Init(SPI_CS_GYRO_GPIO_Port, &GPIO_InitStruct);

	 /*Configure GPIO pins : IMU_INT_ACC_Pin IMU_INT_GYRO_Pin */
	 GPIO_InitStruct.Pin = IMU_INT_ACC_Pin|IMU_INT_GYRO_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	 /*Configure GPIO pins : SW_BotLeft_Pin SW_BotRight_Pin */
	 GPIO_InitStruct.Pin = SW_BotLeft_Pin|SW_BotRight_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	 /* USER CODE BEGIN MX_GPIO_Init_2 */

	 /* USER CODE END MX_GPIO_Init_2 */
 }

 /* USER CODE BEGIN 4 */

 /* USER CODE END 4 */

 /**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
 void Error_Handler(void)
 {
	 /* USER CODE BEGIN Error_Handler_Debug */
	 /* User can add his own implementation to report the HAL error return state */
	 __disable_irq();
	 while (1)
	 {
	 }
	 /* USER CODE END Error_Handler_Debug */
 }
#ifdef USE_FULL_ASSERT
 /**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
 void assert_failed(uint8_t *file, uint32_t line)
 {
	 /* USER CODE BEGIN 6 */
	 /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	 /* USER CODE END 6 */
 }
#endif /* USE_FULL_ASSERT */
