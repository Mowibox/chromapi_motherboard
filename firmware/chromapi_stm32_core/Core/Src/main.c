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
#include <ws2812b.h>

#include "servo_bus.h"
#include "ina226.h"
#include "bmi088.h"
#include "bridge.h"
#include "mahony.h"
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
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_tim4_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
static AutoFox_INA226 gINA226;
static BMI088 gIMU;
static MahonyFilter_t g_mahony;
static PowerReading_t g_power;
static uint32_t last_led_update = 0;

static volatile bool g_imu_task_due = false;
static volatile bool g_power_task_due = false;

#define BUS_CYCLE_DIV 2U  // bus cycle = 400 Hz / 2 = 200 Hz (~3.8 ms of bus time per 5 ms cycle)
static uint8_t g_tim6_div = 0;
static uint8_t g_tim6_power_div = 0; // INA226 refresh ODR = 10 Hz (400 Hz / 40)
static uint32_t g_imu_last_us = 0;
static bool g_imu_ok = false;

#define VBAT_MAX_UV      8400000L   // 8.4V
#define VBAT_GREEN_UV    7800000L   // 7.8V
#define VBAT_YELLOW_UV   7400000L   // 7.4V
#define VBAT_ORANGE_UV   7000000L   // 7.0V
#define VBAT_RED_UV      6800000L   // 6.8V
#define VBAT_CRIT_UV     6600000L   // < 6.6V
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
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		Bridge_TxCpltCallback();
	}
	else if (huart->Instance == USART2) {
		ServoBus_UartTxCplt();
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1) {
		Bridge_RxEventCallback(Size);
	}
	else if (huart->Instance == USART2) {
		ServoBus_UartRxEvent(Size);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		Bridge_ErrorCallback();
	}
	else if (huart->Instance == USART2) {
		ServoBus_UartError();
	}
}

// µs clock built on the DWT cycle counter (HAL_GetTick's 1 ms is too coarse for a 2.5 ms
// IMU period). Main-loop context only: not reentrant. Must be called at least every
// 2^32 / SystemCoreClock (~26 s at 160 MHz), which the main loop does.
static uint32_t micros(void) {
	static uint32_t last_cyc = 0, us = 0, rem = 0;
	const uint32_t cyc_per_us = SystemCoreClock / 1000000U;
	uint32_t now = DWT->CYCCNT;
	rem += now - last_cyc;
	last_cyc = now;
	us  += rem / cyc_per_us;
	rem %= cyc_per_us;
	return us;
}

static void Imu_Task(void) {
	uint32_t now = micros();
	float dt = (now - g_imu_last_us) * 1e-6f;
	g_imu_last_us = now;
	if (dt <= 0.0f || dt > 0.5f) dt = 0.0025f;

	if (!BMI088_ReadAccelerometer(&gIMU) || !BMI088_ReadGyroscope(&gIMU)) return;
	Mahony_Update(&g_mahony, gIMU.gyr_rps[0], gIMU.gyr_rps[1], gIMU.gyr_rps[2],
			gIMU.acc_mps2[0], gIMU.acc_mps2[1], gIMU.acc_mps2[2], dt);
}

static void Power_Task(void) {
	g_power.bus_uV     = AutoFox_INA226_GetBusVoltage_uV(&gINA226);
	g_power.current_uA = AutoFox_INA226_GetCurrent_uA(&gINA226);
	g_power.power_uW   = AutoFox_INA226_GetPower_uW(&gINA226);
}

static void Battery_Led_Update(void)
{
	int32_t uV = g_power.bus_uV;  // refreshed at 10 Hz

	uint8_t r, g, b;

	if (uV >= VBAT_GREEN_UV) {
		int32_t ratio = ((uV - VBAT_GREEN_UV) * 50L) / (VBAT_MAX_UV - VBAT_GREEN_UV);
		if (ratio > 50) ratio = 50;
		r = 0;
		g = 50;
		b = (uint8_t)ratio;
	}
	else if (uV >= VBAT_YELLOW_UV) {
		int32_t ratio = ((uV - VBAT_YELLOW_UV) * 50L) / (VBAT_GREEN_UV - VBAT_YELLOW_UV);
		if (ratio > 50) ratio = 50;
		r = (uint8_t)(50L - ratio);
		g = 50;
		b = 0;
	}
	else if (uV >= VBAT_ORANGE_UV) {
		int32_t ratio = ((uV - VBAT_ORANGE_UV) * 50L) / (VBAT_YELLOW_UV - VBAT_ORANGE_UV);
		if (ratio > 50) ratio = 50;
		r = 50;
		g = (uint8_t)(15L + ratio * 35L / 50L);
		b = 0;
	}
	else if (uV >= VBAT_RED_UV) {
		int32_t ratio = ((uV - VBAT_RED_UV) * 50L) / (VBAT_ORANGE_UV - VBAT_RED_UV);
		if (ratio > 50) ratio = 50;
		r = 50;
		g = (uint8_t)(ratio * 15L / 50L);
		b = 0;
	}
	else if (uV >= VBAT_CRIT_UV) {
		r = 50; g = 0; b = 0;
	}
	else {
		uint8_t blink = (HAL_GetTick() % 200U) < 100U ? 50 : 0;
		r = blink; g = 0; b = 0;
	}

	led_set_RGB(LED_STATUS_START_INDEX, r, g, b);
	led_render();
}

// IWDG on LSI (~32 kHz) / 64, reload 250: ~500 ms. Covers the longest legit main-loop stall
// (SET_SERVO_ID, ~225 ms). Once started it cannot be stopped; frozen while the core is halted.
static void iwdg_start(void)
{
	DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;
	IWDG->KR  = 0xCCCCU;  // start
	IWDG->KR  = 0x5555U;  // unlock PR/RLR
	IWDG->PR  = IWDG_PR_PR_2;  // /64
	IWDG->RLR = 250U - 1U;
	while (IWDG->SR != 0U) {}
	IWDG->KR  = 0xAAAAU;  // refresh
}

void Chromapi_SystemInit(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // DWT cycle counter for micros()
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	ServoBus_Init(&huart2);
	AutoFox_INA226_Constructor(&gINA226);
	(void)AutoFox_INA226_Init(&gINA226, INA226_I2C_ADDR, SHUNT_OHMS, MAX_CURRENT_A);
	Bridge_Init(&huart1);
	g_imu_ok = BMI088_Init(&gIMU, &hspi1,
			SPI_CS_ACC_GPIO_Port,  SPI_CS_ACC_Pin,
			SPI_CS_GYRO_GPIO_Port, SPI_CS_GYRO_Pin);

	Mahony_Init(&g_mahony);
	if (g_imu_ok) {
		Mahony_Calibrate(&g_mahony, &gIMU);
	}

	Power_Task();

	ServoBus_ScanPresent();
	HAL_Delay(500U);
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
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */
	Chromapi_SystemInit();
	if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
		Error_Handler();
	}
	iwdg_start();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		IWDG->KR = 0xAAAAU;  // refresh

		uint32_t now_us = micros();
		uint16_t cycle_us;
		if (ServoBus_Process(now_us, &cycle_us)) {  // end of a bus cycle: stream the fresh state
			Bridge_SendStateSnapshot(now_us, cycle_us, &gIMU, &g_mahony, &g_power);
		}

		Bridge_Process();
		Bridge_CheckLink();

		if (g_imu_task_due) {
			g_imu_task_due = false;
			if (g_imu_ok) Imu_Task();
		}

		if (g_power_task_due) {
			g_power_task_due = false;
			Power_Task();
		}

		if ((HAL_GetTick() - last_led_update) >= 500U) {
			last_led_update = HAL_GetTick();
			Battery_Led_Update();
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
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 20;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
	hi2c1.Init.Timing = 0x10F02A89;
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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
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
	htim4.Init.Period = 199;
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
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 160-1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 2499;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

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
	huart1.Init.BaudRate = 2000000;
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
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
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
	HAL_GPIO_WritePin(SPI_CS_GYRO_GPIO_Port, SPI_CS_GYRO_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI_CS_ACC_GPIO_Port, SPI_CS_ACC_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : SW_TopLeft_Pin SW_TopRight_Pin */
	GPIO_InitStruct.Pin = SW_TopLeft_Pin|SW_TopRight_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI_CS_GYRO_Pin */
	GPIO_InitStruct.Pin = SPI_CS_GYRO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI_CS_GYRO_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI_CS_ACC_Pin */
	GPIO_InitStruct.Pin = SPI_CS_ACC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI_CS_ACC_GPIO_Port, &GPIO_InitStruct);

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		g_imu_task_due = true;

		if (++g_tim6_div >= BUS_CYCLE_DIV) {
			g_tim6_div = 0;
			ServoBus_TickCycle();
		}
		if (++g_tim6_power_div >= 40) {  // 400 Hz / 40
			g_tim6_power_div = 0;
			g_power_task_due = true;
		}
	}
}
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
