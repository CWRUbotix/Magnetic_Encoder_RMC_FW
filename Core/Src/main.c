/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>

#include "utils.h"
#include "memory.h"
#include "encoder.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define HAL_CHECK(func) if((func) != HAL_OK) { Error_Handler(); }

int16_t CAN_id;
bool power_ok;

uint64_t i2c_buffer;

Encoder_t encoder;
EncoderSettings_t encoder_settings;
EncoderDiagnostics_t encoder_diagnostics;

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

  __HAL_RCC_I2C2_CLK_ENABLE();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_CAN_Start(&hcan)) {
      Error_Handler();
  }

  CAN_id = 0b00000000;
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_1_GPIO_Port, CAN_ID_1_Pin) << 0u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_2_GPIO_Port, CAN_ID_2_Pin) << 1u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_4_GPIO_Port, CAN_ID_4_Pin) << 2u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_8_GPIO_Port, CAN_ID_8_Pin) << 3u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_16_GPIO_Port, CAN_ID_16_Pin) << 4u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_32_GPIO_Port, CAN_ID_32_Pin) << 5u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_64_GPIO_Port, CAN_ID_64_Pin) << 6u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_128_GPIO_Port, CAN_ID_128_Pin) << 7u);

  // initialize encoder
  encoder_init(&encoder);
  // encoder settings
  encoder_settings.zero_offset = 0;
  encoder_settings.error_en.error_low = 1;
  encoder_settings.error_en.error_high = 1;
  encoder_settings.settings1.iwidth = 0;
  encoder_settings.settings1.noiseset = 0;
  encoder_settings.settings1.dir = 0;
  encoder_settings.settings1.daecdis = 0;
  encoder_settings.settings1.dataselect = 0;
  encoder_settings.settings1.pwmon = 1;
  encoder_settings.settings2 = 0x0;

    HAL_CHECK(encoder_config_all_settings(&hspi1, &encoder, &encoder_settings));

  if(encoder_get_absolute_position(&hspi1, &encoder) != HAL_OK) {
      Error_Handler();
  }

  //eeprom

//    memset(i2c_buffer, 0, 128);
    #ifdef RESET_EEPROM_MEMORY
        HAL_CHECK(eeprom_clear_all(&hi2c2));
    #endif

    HAL_CHECK(eeprom_read_page(&hi2c2, ENCODER_TICKS_LOCATION, &i2c_buffer));
    encoder.count = (int32_t) i2c_buffer;

    HAL_CHECK(eeprom_read_page(&hi2c2, ENCODER_POLARITY_LOCATION, &i2c_buffer));

    HAL_CHECK(eeprom_read_page(&hi2c2, ENCODER_ABSOLUTE_OFFSET_LOCATION, &i2c_buffer));

    HAL_CHECK(eeprom_read_page(&hi2c2, FEEDBACK_PERIOD_LOCATION, &i2c_buffer));


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    #ifdef HAS_5V5
        power_ok = HAL_GPIO_ReadPin(POWER_SENSE_GPIO_Port, POWER_SENSE_Pin);
    #else
        power_ok = true;
    #endif
    // check for panic (ie: the power is lo
    if(!power_ok) {
        // panic
        __disable_irq();
        HAL_GPIO_WritePin(STATUS_1_GPIO_Port, STATUS_1_Pin, 0);
        HAL_GPIO_WritePin(STATUS_2_GPIO_Port, STATUS_2_Pin, 0);
        HAL_GPIO_WritePin(STATUS_3_GPIO_Port, STATUS_3_Pin, 0);

        i2c_buffer = encoder.count;
        HAL_CHECK(eeprom_write_page(&hi2c2, ENCODER_TICKS_LOCATION, &i2c_buffer));
        i2c_buffer = encoder.inverted;
        HAL_CHECK(eeprom_write_page(&hi2c2, ENCODER_POLARITY_LOCATION, &i2c_buffer));
        i2c_buffer = encoder.absolute_offset;
        HAL_CHECK(eeprom_write_page(&hi2c2, ENCODER_ABSOLUTE_OFFSET_LOCATION, &i2c_buffer));
        i2c_buffer = 20; // CHANGE THIS LATER
        HAL_CHECK(eeprom_write_page(&hi2c2, FEEDBACK_PERIOD_LOCATION, &i2c_buffer));

        // wait until power is back
        while(!HAL_GPIO_ReadPin(POWER_SENSE_GPIO_Port, POWER_SENSE_Pin)) {}
        __enable_irq();
        continue;
    }

    uint32_t can_frame_available = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
    if(can_frame_available > 0) {
        CAN_RxHeaderTypeDef hddr;
        uint8_t data[8];
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &hddr, data);
    }

      encoder_get_diagnostics(&hspi1, &encoder, &encoder_diagnostics);
      encoder_get_errors(&hspi1, &encoder, &encoder_diagnostics);
      encoder_get_absolute_position(&hspi1, &encoder);

      // turn off status1 led if magnet placement is good
      if(!encoder_diagnostics.mag_low && !encoder_diagnostics.mag_high) {
          HAL_GPIO_WritePin(STATUS_2_GPIO_Port, STATUS_2_Pin, 0);
      } else {
          HAL_GPIO_WritePin(STATUS_2_GPIO_Port, STATUS_2_Pin, 1);
      }
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
    CAN_FilterTypeDef can_filter = {};

    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterBank = 0;
    can_filter.FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(&hcan, &can_filter);
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */
  // https://tutel.me/c/electronics/questions/272427/stm32+busy+flag+is+set+after+i2c+initialization
  // 1


  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STATUS_3_Pin|STATUS_2_Pin|STATUS_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CAN_ID_128_Pin CAN_ID_64_Pin */
  GPIO_InitStruct.Pin = CAN_ID_128_Pin|CAN_ID_64_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_ID_32_Pin CAN_ID_16_Pin CAN_ID_8_Pin CAN_ID_4_Pin 
                           CAN_ID_2_Pin CAN_ID_1_Pin */
  GPIO_InitStruct.Pin = CAN_ID_32_Pin|CAN_ID_16_Pin|CAN_ID_8_Pin|CAN_ID_4_Pin 
                          |CAN_ID_2_Pin|CAN_ID_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_SENSE_Pin */
  GPIO_InitStruct.Pin = POWER_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(POWER_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_A_Pin ENC_B_Pin */
  GPIO_InitStruct.Pin = ENC_A_Pin|ENC_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_I_Pin */
  GPIO_InitStruct.Pin = ENC_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ENC_I_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_3_Pin STATUS_2_Pin STATUS_1_Pin */
  GPIO_InitStruct.Pin = STATUS_3_Pin|STATUS_2_Pin|STATUS_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    if(pin == ENC_A_Pin || pin == ENC_B_Pin) {
        bool a_high, b_high;
        a_high = HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin);
        b_high = HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin);
        encoder_update_count(&encoder, a_high, b_high);
        HAL_GPIO_TogglePin(STATUS_3_GPIO_Port, STATUS_3_Pin);
    } else if(pin == ENC_I_Pin) {
        // what do we want to do with this?
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
  HAL_GPIO_WritePin(STATUS_1_GPIO_Port, STATUS_1_Pin, 0);
  HAL_GPIO_WritePin(STATUS_2_GPIO_Port, STATUS_2_Pin, 0);
  HAL_GPIO_WritePin(STATUS_3_GPIO_Port, STATUS_3_Pin, 0);
  for(;;) {
      HAL_Delay(250);
      HAL_GPIO_TogglePin(STATUS_1_GPIO_Port, STATUS_1_Pin);
      HAL_GPIO_TogglePin(STATUS_2_GPIO_Port, STATUS_2_Pin);
      HAL_GPIO_TogglePin(STATUS_3_GPIO_Port, STATUS_3_Pin);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
