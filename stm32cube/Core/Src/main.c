/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "lwip/sockets.h"          <----
//#include <string.h>
#include "tcpServerRAW.h"
//#include "tcpClientRAW.h"
#include "interface.h"
//#include <stdio.h>
//#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define PORT	5015          <----
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//int sock, g;						<----
//struct sockaddr_in address;      <---
//err_t err = -1;					<----

char spi_buf_set[50], spi_buf_lem[50], spi_buf_mos[50];
double v_ref = 4.0;
const double LSB_ADC = (2 * 4.0)/pow(2, 24);
const uint32_t HALF_CODE = pow(2,24)/2;
const double LSB_DAC = 4.0/pow(2, 20);
double lem_v = 0;
double lem_A = 0;
double set_v = 0;
double in_set_v = 0;
double last_in_set_v = 0;
double set_A = 0;
double last_set_A = 0;
double mos_v = 0;
double err = 0;
double acc_err = 0;
double pid_out = 0;
double I = 0;

double get_adc_lem();
double get_adc_set();
double get_set_V();
double get_lem_A();
int need_change_sign = 0;
void set_dac_mos(double dac);
void send_single_adc_cnv();
void send_adc_cnvs(int n);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern struct netif gnetif;
extern parameters par;
int32_t raw;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_USART3_UART_Init();
  MX_LWIP_Init();
  MX_TIM7_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */

  tcp_server_init();
  //tcp_client_init();
  tcp_server_init();
  initInterface();

  HAL_GPIO_WritePin(LEM_RDL_GPIO_Port, LEM_RDL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ADC_CNV_GPIO_Port, ADC_CNV_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ethernetif_input(&gnetif);
	  sys_check_timeouts();

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 0x0;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi5.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 80;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 300;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOS_CS_GPIO_Port, MOS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L2_RIGHT_GPIO_Port, L2_RIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|SET_RDL_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LEM_RDL_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOS_LDAC_GPIO_Port, MOS_LDAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ADC_CNV_Pin|L2_LEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SET_BUSY_Pin */
  GPIO_InitStruct.Pin = SET_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SET_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOS_CS_Pin */
  GPIO_InitStruct.Pin = MOS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOS_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L2_RIGHT_Pin */
  GPIO_InitStruct.Pin = L2_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L2_RIGHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LEM_RDL_Pin */
  GPIO_InitStruct.Pin = LEM_RDL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEM_RDL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SET_RDL_Pin */
  GPIO_InitStruct.Pin = SET_RDL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SET_RDL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEM_OK_Pin LEM_BUSY_Pin */
  GPIO_InitStruct.Pin = LEM_OK_Pin|LEM_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MOS_LDAC_Pin */
  GPIO_InitStruct.Pin = MOS_LDAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOS_LDAC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_CNV_Pin L2_LEFT_Pin */
  GPIO_InitStruct.Pin = ADC_CNV_Pin|L2_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void send_single_adc_cnv(){
	HAL_GPIO_WritePin(ADC_CNV_GPIO_Port, ADC_CNV_Pin, GPIO_PIN_SET);
	__NOP();
	HAL_GPIO_WritePin(ADC_CNV_GPIO_Port, ADC_CNV_Pin, GPIO_PIN_RESET);
//	for (int i = 0; i < 6; i++) {
//		__NOP();
//	}
}

void send_adc_cnvs(int n){
	for (int i =0; i<n; i++){
		send_single_adc_cnv();
	}
}


double get_adc_lem(){

	uint32_t code = 0x000000;
	double adc_val;


//	HAL_GPIO_WritePin(ADC_CNV_GPIO_Port, ADC_CNV_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(LEM_RDL_GPIO_Port, LEM_RDL_Pin, GPIO_PIN_RESET);

	 for (int i = 0; i < 10; i++) {
	        __NOP();
	    }

	 while (HAL_GPIO_ReadPin(LEM_BUSY_GPIO_Port, LEM_BUSY_Pin) == GPIO_PIN_SET) {
	         // Czekaj na zakończenie konwersji
	     }
//	 HAL_GPIO_WritePin(ADC_CNV_GPIO_Port, ADC_CNV_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(LEM_RDL_GPIO_Port, LEM_RDL_Pin, GPIO_PIN_RESET);

	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf_lem, 3, 200);
	HAL_GPIO_WritePin(LEM_RDL_GPIO_Port, LEM_RDL_Pin, GPIO_PIN_SET);


	((uint8_t *)&code)[2] = (unsigned int)spi_buf_lem[0];
	((uint8_t *)&code)[1] = (unsigned int)spi_buf_lem[1];
	((uint8_t *)&code)[0] = (unsigned int)spi_buf_lem[2];

	adc_val = code* LSB_ADC;
	if(code >= HALF_CODE){
		adc_val -= 2*v_ref;
	}
//	adc_val = adc_val/0.377 + 0.04;
//	adc_val = (adc_val/0.377)*1000;

//	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	return adc_val;

}

double get_adc_set(){

	char msg[100];
	int adc_val_int=0;
//	sprintf(msg, "get_adc_set start\r\n");
//	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	uint32_t code = 0x000000;
	double adc_val;

//	HAL_GPIO_WritePin(LEM_RDL_GPIO_Port, LEM_RDL_Pin, GPIO_PIN_RESET);

//	for (int i = 0; i < 2; i++) {
//	        __NOP();
//	    }

	while (HAL_GPIO_ReadPin(SET_BUSY_GPIO_Port, SET_BUSY_Pin) == GPIO_PIN_SET) {
	    }

	HAL_GPIO_WritePin(SET_RDL_GPIO_Port, SET_RDL_Pin, GPIO_PIN_RESET);

	HAL_SPI_Receive(&hspi2, (uint8_t*)spi_buf_set, 3, 100);
	HAL_GPIO_WritePin(SET_RDL_GPIO_Port, SET_RDL_Pin, GPIO_PIN_SET);

	((uint8_t *)&code)[2] = (unsigned int)spi_buf_set[0];
	((uint8_t *)&code)[1] = (unsigned int)spi_buf_set[1];
	((uint8_t *)&code)[0] = (unsigned int)spi_buf_set[2];

	adc_val = code* LSB_ADC;
	if(code >= HALF_CODE){
		adc_val -= 2*v_ref;
	}
	return adc_val;
}

double get_set_V(){
	set_v = get_adc_set();
	return (set_v - 0.007) * 3.316;
}

double get_lem_A(){
	lem_v = get_adc_lem();
//	return lem_v*30.77-0.15;
	return (lem_v - 0.00478 )*41.363;
}

void set_dac_mos(double dac){
	uint32_t code;

	HAL_GPIO_WritePin(MOS_LDAC_GPIO_Port, MOS_LDAC_Pin, GPIO_PIN_SET);
	dac = dac/2;
	if(dac > v_ref){
		dac = v_ref;
	}else if(dac < 0.0){
		dac = 0.0;
	}
	code = round(dac/LSB_DAC);
	code = code << 4;		// 4 bits shift to left to have 24 bits

	spi_buf_mos[0] = ((uint8_t*)&code)[2];
	spi_buf_mos[1] = ((uint8_t*)&code)[1];
	spi_buf_mos[2] = ((uint8_t*)&code)[0];

	HAL_GPIO_WritePin(MOS_CS_GPIO_Port, MOS_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi5, (uint8_t*)&spi_buf_mos, 3, 100);
	HAL_GPIO_WritePin(MOS_CS_GPIO_Port, MOS_CS_Pin, GPIO_PIN_SET);

	// update output
	for(int i=0;i<8;i++);	//to make at least 20 ns delay
	HAL_GPIO_WritePin(MOS_LDAC_GPIO_Port, MOS_LDAC_Pin, GPIO_PIN_RESET);
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
  if (htim->Instance == TIM7) {
	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, SET);

	  if (par.mode.val == 0) { // switch off current and reset pi values
		  set_dac_mos(0);
		  err = 0;
		  acc_err = 0;
		  HAL_GPIO_WritePin(L2_LEFT_GPIO_Port, L2_LEFT_Pin, RESET);
		  HAL_GPIO_WritePin(L2_RIGHT_GPIO_Port, L2_RIGHT_Pin, RESET);
	  }
	  else if (par.mode.val == 1 || par.mode.val == 2) {
		  send_adc_cnvs(100);
		  lem_A = get_lem_A();
		  last_set_A = set_A;
		  if (par.mode.val == 1){
			  in_set_v = get_set_V()*10;
			  set_A = in_set_v * 10; // 1A_lem = 0.1V_set
		  }
		  if (par.mode.val == 2){
			  set_A = par.cur.val;
		  }
		  par.adc.ch1.volt.val = in_set_v;
		  par.adc.ch2.volt.val = lem_A;

		  //increase gain I if lower current (because of gate characteristics of transistor)
		  if (lem_A < 1 && lem_A > -1){
			  I = par.I.val *1.6;
		  } else if (lem_A < 3 && lem_A > -3){
			  I = par.I.val *1.3;
		  } else {
			  I = par.I.val;
		  }

		  if ( signbit(set_A) != signbit(lem_A) ){
			  if (fabs(lem_A) > 0.01){
				  set_A = 0;
				  need_change_sign = 1;
			  }else{
				  if (need_change_sign){
					  if (set_A <0){
						  HAL_GPIO_WritePin(L2_LEFT_GPIO_Port, L2_LEFT_Pin, RESET);
						  HAL_GPIO_WritePin(L2_RIGHT_GPIO_Port, L2_RIGHT_Pin, SET);
					  }else{
						  HAL_GPIO_WritePin(L2_LEFT_GPIO_Port, L2_LEFT_Pin, SET);
						  HAL_GPIO_WritePin(L2_RIGHT_GPIO_Port, L2_RIGHT_Pin, RESET);
					  }
					  need_change_sign = 0;
				  }
			  }
		  }

		  err = lem_A - set_A;

		  // error limit for smooth current changes
		  if (err > par.ermax.val){
			  err = par.ermax.val;
		  }
		  if (err < -par.ermax.val){
			  err = -par.ermax.val;
		  }

		  acc_err = acc_err + err;

		  // error limit for smooth current changes
		  if (acc_err > par.aermax.val){acc_err = par.aermax.val;}
		  if (acc_err < -par.aermax.val){acc_err = -par.aermax.val;}

		  pid_out = par.goff.val + acc_err*(I);
		  if (pid_out > 9 ){
			  pid_out = 9;
		  }
		  if (pid_out < 0 ){
			  pid_out = 0;
		  }

		  set_dac_mos(pid_out);
	//	  set_dac_mos(par.dac.ch1.volt.val);

	//	  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

		  if (par.dir.val>0.5){
			  HAL_GPIO_WritePin(L2_LEFT_GPIO_Port, L2_LEFT_Pin, RESET);
			  HAL_GPIO_WritePin(L2_RIGHT_GPIO_Port, L2_RIGHT_Pin, SET);
		  } else if (par.dir.val<-0.5){
			  HAL_GPIO_WritePin(L2_LEFT_GPIO_Port, L2_LEFT_Pin, SET);
			  HAL_GPIO_WritePin(L2_RIGHT_GPIO_Port, L2_RIGHT_Pin, RESET);
		  } else{
			  HAL_GPIO_WritePin(L2_LEFT_GPIO_Port, L2_LEFT_Pin, RESET);
			  HAL_GPIO_WritePin(L2_RIGHT_GPIO_Port, L2_RIGHT_Pin, RESET);
		  }
	  }

	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, RESET);
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
  __disable_irq();
  while (1)
  {
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

