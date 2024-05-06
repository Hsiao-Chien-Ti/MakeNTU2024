/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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
#include "app_threadx.h"
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#if defined ( __GNUC__) && !defined(__clang__)
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct GPIOPin {
	GPIO_TypeDef *GPIO_Port;
	uint16_t GPIO_Pin;
} GPIOPin;
typedef struct Motor {
	GPIOPin IN1;
	GPIOPin IN2;
	GPIOPin SW;
	uint32_t counter;
	uint32_t swState;
} Motor;
Motor motors[14];
//uint32_t resetCounter;
void Success_Handler(void) {
	BSP_LED_Off(LED_RED);
	while (1) {
		BSP_LED_Toggle(LED_GREEN);
		HAL_Delay(200);
	}
//  HAL_GPIO_WritePin (LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
//  while(1)
//  {
//    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
//    tx_thread_sleep(50);
//  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	Motor m[] = { { { Motor1_IN1_GPIO_Port, Motor1_IN1_Pin }, {
			Motor1_IN2_GPIO_Port, Motor1_IN2_Pin }, { Motor1_SW_GPIO_Port,
			Motor1_SW_Pin }, 0, 1 }, { { Motor2_IN1_GPIO_Port, Motor2_IN1_Pin },
			{ Motor2_IN2_GPIO_Port, Motor2_IN2_Pin }, { Motor2_SW_GPIO_Port,
					Motor2_SW_Pin }, 0, 1 }, { { Motor3_IN1_GPIO_Port,
			Motor3_IN1_Pin }, { Motor3_IN2_GPIO_Port, Motor3_IN2_Pin }, {
			Motor3_SW_GPIO_Port, Motor3_SW_Pin }, 0, 1 }, { {
			Motor4_IN1_GPIO_Port, Motor4_IN1_Pin }, { Motor4_IN2_GPIO_Port,
			Motor4_IN2_Pin }, { Motor4_SW_GPIO_Port, Motor4_SW_Pin }, 0, 1 }, {
			{ Motor5_IN1_GPIO_Port, Motor5_IN1_Pin }, { Motor5_IN2_GPIO_Port,
					Motor5_IN2_Pin }, { Motor5_SW_GPIO_Port, Motor5_SW_Pin }, 0,
			1 }, { { Motor6_IN1_GPIO_Port, Motor6_IN1_Pin }, {
			Motor6_IN2_GPIO_Port, Motor6_IN2_Pin }, { Motor6_SW_GPIO_Port,
			Motor6_SW_Pin }, 0, 1 }, { { Motor7_IN1_GPIO_Port, Motor7_IN1_Pin },
			{ Motor7_IN2_GPIO_Port, Motor7_IN2_Pin }, { Motor7_SW_GPIO_Port,
					Motor7_SW_Pin }, 0, 1 }, { { Motor8_IN1_GPIO_Port,
			Motor8_IN1_Pin }, { Motor8_IN2_GPIO_Port, Motor8_IN2_Pin }, {
			Motor8_SW_GPIO_Port, Motor8_SW_Pin }, 0, 1 }, { {
			Motor9_IN1_GPIO_Port, Motor9_IN1_Pin }, { Motor9_IN2_GPIO_Port,
			Motor9_IN2_Pin }, { Motor9_SW_GPIO_Port, Motor9_SW_Pin }, 0, 1 }, {
			{ Motor10_IN1_GPIO_Port, Motor10_IN1_Pin }, { Motor10_IN2_GPIO_Port,
					Motor10_IN2_Pin }, { Motor10_SW_GPIO_Port, Motor10_SW_Pin },
			0, 1 }, { { Motor11_IN1_GPIO_Port, Motor11_IN1_Pin }, {
			Motor11_IN2_GPIO_Port, Motor11_IN2_Pin }, { Motor11_SW_GPIO_Port,
			Motor11_SW_Pin }, 0, 1 }, {
			{ Motor12_IN1_GPIO_Port, Motor12_IN1_Pin }, { Motor12_IN2_GPIO_Port,
					Motor12_IN2_Pin }, { Motor12_SW_GPIO_Port, Motor12_SW_Pin },
			0, 1 }, { { Motor13_IN1_GPIO_Port, Motor13_IN1_Pin }, {
			Motor13_IN2_GPIO_Port, Motor13_IN2_Pin }, { Motor13_SW_GPIO_Port,
			Motor13_SW_Pin }, 0, 1 }, {
			{ Motor14_IN1_GPIO_Port, Motor14_IN1_Pin }, { Motor14_IN2_GPIO_Port,
					Motor14_IN2_Pin }, { Motor14_SW_GPIO_Port, Motor14_SW_Pin },
			0, 1 } };
	for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
		motors[i] = m[i];
	}
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	BSP_LED_Init(LED_GREEN);
	BSP_LED_Init(LED_RED);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(&htim2);
	// stop all motors
	for (int i = 0; i < (sizeof(motors) / sizeof(motors[0])); i++) {
		stopMotor(i);
		motors[i].swState = 1;
	}
//	resetCounter = 0;
  /* USER CODE END 2 */

  MX_ThreadX_Init();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x11;
  MACAddr[2] = 0x83;
  MACAddr[3] = 0x45;
  MACAddr[4] = 0x26;
  MACAddr[5] = 0x20;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1536;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Motor4_IN2_Pin|Motor13_IN1_Pin|Motor14_IN1_Pin|Motor14_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Motor5_IN2_Pin|Motor5_IN1_Pin|Motor2_IN1_Pin|Motor13_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Motor1_IN2_Pin|Motor6_IN1_Pin|Motor8_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Motor10_IN1_Pin|Motor10_IN2_Pin|Motor8_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Motor1_IN1_Pin|Motor6_IN2_Pin|Motor2_IN2_Pin|Motor9_IN1_Pin
                          |Motor9_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Motor4_IN1_Pin|Motor3_IN2_Pin|Motor3_IN1_Pin|Motor7_IN1_Pin
                          |Motor7_IN2_Pin|Motor12_IN2_Pin|Motor12_IN1_Pin|Motor11_IN2_Pin
                          |Motor11_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Motor4_IN2_Pin Motor13_IN1_Pin Motor14_IN1_Pin Motor14_IN2_Pin */
  GPIO_InitStruct.Pin = Motor4_IN2_Pin|Motor13_IN1_Pin|Motor14_IN1_Pin|Motor14_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor5_IN2_Pin Motor5_IN1_Pin Motor2_IN1_Pin Motor13_IN2_Pin */
  GPIO_InitStruct.Pin = Motor5_IN2_Pin|Motor5_IN1_Pin|Motor2_IN1_Pin|Motor13_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor1_IN2_Pin Motor6_IN1_Pin Motor8_IN2_Pin */
  GPIO_InitStruct.Pin = Motor1_IN2_Pin|Motor6_IN1_Pin|Motor8_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor10_IN1_Pin Motor10_IN2_Pin Motor8_IN1_Pin */
  GPIO_InitStruct.Pin = Motor10_IN1_Pin|Motor10_IN2_Pin|Motor8_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor1_IN1_Pin Motor6_IN2_Pin Motor2_IN2_Pin Motor9_IN1_Pin
                           Motor9_IN2_Pin */
  GPIO_InitStruct.Pin = Motor1_IN1_Pin|Motor6_IN2_Pin|Motor2_IN2_Pin|Motor9_IN1_Pin
                          |Motor9_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor1_SW_Pin Motor3_SW_Pin Motor5_SW_Pin */
  GPIO_InitStruct.Pin = Motor1_SW_Pin|Motor3_SW_Pin|Motor5_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor9_SW_Pin Motor8_SW_Pin Motor10_SW_Pin Motor2_SW_Pin
                           Motor11_SW_Pin Motor4_SW_Pin Motor12_SW_Pin Motor13_SW_Pin */
  GPIO_InitStruct.Pin = Motor9_SW_Pin|Motor8_SW_Pin|Motor10_SW_Pin|Motor2_SW_Pin
                          |Motor11_SW_Pin|Motor4_SW_Pin|Motor12_SW_Pin|Motor13_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor14_SW_Pin */
  GPIO_InitStruct.Pin = Motor14_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Motor14_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor4_IN1_Pin Motor3_IN2_Pin Motor3_IN1_Pin Motor7_IN1_Pin
                           Motor7_IN2_Pin Motor12_IN2_Pin Motor12_IN1_Pin Motor11_IN2_Pin
                           Motor11_IN1_Pin */
  GPIO_InitStruct.Pin = Motor4_IN1_Pin|Motor3_IN2_Pin|Motor3_IN1_Pin|Motor7_IN1_Pin
                          |Motor7_IN2_Pin|Motor12_IN2_Pin|Motor12_IN1_Pin|Motor11_IN2_Pin
                          |Motor11_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor7_SW_Pin Motor6_SW_Pin */
  GPIO_InitStruct.Pin = Motor7_SW_Pin|Motor6_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}

/**
 * @brief  This function is executed in case of success.
 * @retval None
 */
//void Success_Handler(void)
//{
//  /* USER CODE BEGIN Success_Handler_Debug */
//  BSP_LED_Off(LED_RED);
//  while(1)
//  {
//    BSP_LED_Toggle(LED_GREEN);
//    tx_thread_sleep(50);
//  }
//  /* USER CODE END Success_Handler_Debug */
//}
void stopMotor(int motor_idx) {
//	printf("%d\n", motor_idx);
	HAL_GPIO_WritePin(motors[motor_idx].IN1.GPIO_Port,
			motors[motor_idx].IN1.GPIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motors[motor_idx].IN2.GPIO_Port,
			motors[motor_idx].IN2.GPIO_Pin, GPIO_PIN_RESET);
	motors[motor_idx].counter = 0;
	motors[motor_idx].swState = 1;
}
void controlSpeed(float speed) {
	uint32_t sp = (int) (speed * htim1.Init.Period);
	TIM1->CCR1 = sp;
	TIM3->CCR3 = sp;
}
void controlMotor(int motor_idx, uint32_t duration, char direction) {
	if (duration == 0) {
		return;
	}
	GPIO_PinState in1_state = direction == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET;
	GPIO_PinState in2_state = direction == 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
	HAL_GPIO_WritePin(motors[motor_idx].IN1.GPIO_Port,
			motors[motor_idx].IN1.GPIO_Pin, in1_state);
	HAL_GPIO_WritePin(motors[motor_idx].IN2.GPIO_Port,
			motors[motor_idx].IN2.GPIO_Pin, in2_state);
//	printf("start motor %d\n", motor_idx);
	motors[motor_idx].counter = duration;
}
//void updateReset(uint32_t value)
//{
//	if(resetCounter<value+1000)
//	{
//		resetCounter = value+1000;
//	}
//}
//void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
//{
//	printf("%d\n", GPIO_Pin);
//	for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
//		if(motors[i].SW.GPIO_Pin==GPIO_Pin && motors[i].counter)
//		{
//			printf("stop motor %d\n", i);
//			stopMotor(i);
//		}
//	}
//}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

	if (htim->Instance == TIM2) {

		for (int i = 0; i < MOTOR_COUNT; i++) {
			GPIO_PinState sw = HAL_GPIO_ReadPin(motors[i].SW.GPIO_Port,
					motors[i].SW.GPIO_Pin);
			if (sw == GPIO_PIN_RESET && sw != motors[i].swState
					&& motors[i].counter) {
//				printf("stop motor %d\n", i);
				stopMotor(i);
			} else {
				if (motors[i].counter == 1) {
//					printf("stop motor %d\n", i);
					stopMotor(i);
				}
				if (motors[i].counter) {
					motors[i].counter--;
				}
			}
			motors[i].swState = sw;
		}
//		if(resetCounter==1)
//		{
//			for(int i=0;i<MOTOR_COUNT;i++)
//			{
//				controlMotor(i, 3000, 0);
//			}
//		}
//		if(resetCounter)
//		{
//			resetCounter--;
//		}

//	printf("%d\n", result);
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	BSP_LED_Off(LED_GREEN);
	while (1) {
		BSP_LED_Toggle(LED_RED);
		HAL_Delay(200);
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* Infinite loop */
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
