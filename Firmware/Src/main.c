/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "EEPROM.h"
#include "mpu6050.h"
#include "string.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
MPU6050_t MPU6050;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_ADDR 0xA0
#define MPU6050_ADDR 0xD0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c2_tx;
DMA_HandleTypeDef hdma_i2c2_rx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init();
void UintToStr3(uint16_t u, uint8_t *y);
void Cal_uart_size(uint8_t* As_data, uint16_t* Bi_data,uint8_t n);
void AsciiToBinary(uint8_t *Ascii, uint8_t *Binary);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//i2c
//uint8_t dataRead[100];
//uint8_t dataWrite[100] = {0x41, 0x30, 0x46,0x46, 00, 00,0xC0,84, 75};/
uint8_t tk = 0;
uint8_t i2c_write_flag = 0;
uint8_t i2c_read_flag = 0;
uint8_t i2c2_read_flag = 0;
//uart
uint8_t Rx_Data[100] = "";
uint8_t Tx_Data[100] = {'A','0','F','F', '0'};//A0 FF 46 00 XXXX
uint8_t uart_tx_flag = 0;
uint8_t uart_rx_flag = 0;
uint16_t Uart_Rx_Size = 0;
uint16_t page = 0;
uint8_t tim2_it_flag = 0;
char buffer[50];
//mpu6050
//double Ax;
//double Ay;
//double Az;
//double Gx;
//double Gy;
//double Gz;
double roll;
double pitch;

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//  for (int i = 0; i < 100; i++){
//	  dataWrite[i] = i+32;
//  }
	MPU6050_Init();
	HAL_TIM_Base_Start_IT(&htim2);
	//HAL_UART_Receive_DMA(&huart3, Rx_Data, 6);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rx_Data, sizeof Rx_Data);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	//sscanf(&tk1,"%x",&tk2);
	//HAL_UART_Transmit_DMA(&huart3, Tx_Data, 20);
//  HAL_I2C_Mem_Write(&hi2c1, 0xA0, (3<<6), 2, dataWrite, 70, 1000);
//  HAL_Delay(5);
//  HAL_I2C_Mem_Write(&hi2c1, 0xA0, (2<<6), 2, dataWrite, 64, 1000);
//  HAL_Delay(1000);
//  HAL_I2C_Mem_Read(&hi2c1, 0xA0, (2<<6), 2, dataRead, 64, 1000);
//  HAL_Delay(1000);
//  HAL_UART_Transmit_DMA(&huart3, Tx_Data, sizeof(Tx_Data));
//  HAL_Delay(1000);
//	for(int i=0; i<511; i++){
//		EEPROM_PageErase(i);
//	}
	//HAL_I2C_Mem_Write_DMA(&hi2c1, 0xA0, (0<<6), 2, dataWrite, 64);
	//HAL_I2C_Mem_Read_DMA(&hi2c1, 0xA0, (0 << 6), 2, dataRead, 100);
	//HAL_UART_Receive_DMA(&huart3, Rx_Data, sizeof(Rx_Data));
	//HAL_I2C_Mem_Read(&hi2c1, 0xA0, (3<<6), 2, dataRead, 64, 1000);
	//EEPROM_Write(3, 0, dataWrite, 70);
	//EEPROM_Read(3, 0, dataRead, 70);
	//HAL_UART_Transmit_DMA(&huart3, dataRead, sizeof(dataRead));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (HAL_I2C_IsDeviceReady(&hi2c2, MPU6050_ADDR, 1, 100) == HAL_OK) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);

			//tk = 193;
		}
		else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
		}

		if (HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_ADDR, 1, 100) == HAL_OK) {
			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,SET);
			//tk = 252;
		}
		else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
		}
//		//Test
//		if (page < 511)
//			page++;
//		else
//			page = 0;
//		HAL_I2C_Mem_Read_DMA(&hi2c1, 0xA0, (page << 6), 2, Rx_Data, 64);
//		HAL_Delay(1000);
//		//End Test

		//Code BTL
		MPU6050_Read_All(&MPU6050);
		//HAL_UART_Transmit_DMA(&huart3, dataWrite, sizeof dataWrite);
//		HAL_UART_Receive_DMA(&huart3, Rx_Data, sizeof Rx_Data);
//		HAL_Delay(1000);
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		//HAL_Delay(1000);
		if(tim2_it_flag){


			tim2_it_flag = 0;
			roll = MPU6050.KalmanAngleX;
			pitch = MPU6050.KalmanAngleY;
			if (i2c2_read_flag){
			i2c2_read_flag = 0;

			//sprintf(buffer, "%.2f\r\n",roll);
			snprintf(buffer, sizeof(buffer),"%06.2f, %06.2f\r\n",roll,pitch);//abc.de,xyz.mn\r\n
			HAL_UART_Transmit_DMA(&huart3,(uint8_t*) buffer, sizeof(buffer));
			while (uart_tx_flag == 0);
			uart_tx_flag = 0;
		}
//		if (tim2_it_flag) {
//			tim2_it_flag = 0;
//		MPU6050_Read_Gyro(&MPU6050);
//		MPU6050_Read_Accel(&MPU6050);
			//MPU6050_Read_All(&MPU6050);
		}


//		Ax = MPU6050.Ax;
//		Ay = MPU6050.Ay;
//		Az = MPU6050.Az;
//		Gx = MPU6050.Gx;
//		Gy = MPU6050.Gy;
//		Gz = MPU6050.Gz;
//		roll = MPU6050.KalmanAngleX;
//		pitch = MPU6050.KalmanAngleY;
		//HAL_I2C_Mem_Read_DMA(&hi2c1, 0xA0, (1<<6), 2, dataRead, 64);
		if (uart_rx_flag) {
			uart_rx_flag = 0;

			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//			uint8_t mode;
//			uint8_t tk2[4];
//			for(int i=0; i<4; i++){
//				tk2[i] = Rx_Data[8+i];
//			}
//			sscanf(&tk2,"%x",&page);
//			Cal_uart_size(&Rx_Data[6],&mode,2);

			switch (Rx_Data[3]) {
			case 0:		//Read page
				page = (Rx_Data[4] << 8) | ((Rx_Data[5] & 0xC0));

				//memset(dataRead, 0, sizeof(dataRead));
//				uint8_t Tx_Data_2[4] = {'A','0','F','F'};
//				for(int i=0; i<4; i++){
//					Tx_Data[i] = Tx_Data_2[i];
//				}
				Tx_Data[5] = '0';
				HAL_I2C_Mem_Read_DMA(&hi2c1, EEPROM_ADDR, page, 2, &Tx_Data[6], 64);
//				Tx_Data[2] = 0x46;
//				for(int i=3; i<=5; i++){
//					Tx_Data[i] = Rx_Data[i];
//				}
				break;
			case 1:		//Read address
				//memset(dataRead, 0, sizeof(dataRead));
				Tx_Data[5] = '1';
				HAL_I2C_Mem_Read_DMA(&hi2c1, EEPROM_ADDR, (Rx_Data[4] << 8) | (Rx_Data[5]), 2, &Tx_Data[6], 1);
//				Tx_Data[2] = 0x06;
//				for(int i=3; i<=5; i++){
//					Tx_Data[i] = Rx_Data[i];
//				}
				break;
			case 2:		//Write page
				page = (Rx_Data[4] << 8) | ((Rx_Data[5] & 0xC0));
				HAL_I2C_Mem_Write_DMA(&hi2c1, EEPROM_ADDR, page, 2, &Rx_Data[6], Uart_Rx_Size - 6);
				break;
			case 3:		//Write address
				HAL_I2C_Mem_Write_DMA(&hi2c1, EEPROM_ADDR, (Rx_Data[4] << 8) | (Rx_Data[5]), 2, &Rx_Data[6], 1);
				break;

			default:
				// default statements
				;
			}
//			if (Rx_Data[3] == 0) {
//				page = (Rx_Data[4] << 8) | ((Rx_Data[5] & 0xC0));
//				memset(dataRead, 0, sizeof(dataRead));
//				HAL_I2C_Mem_Read_DMA(&hi2c1, EEPROM_ADDR, page, 2, dataRead, 64);
//
//			} else if (Rx_Data[3] == 1) {		//Read
//				memset(dataRead, 0, sizeof(dataRead));
//				HAL_I2C_Mem_Read_DMA(&hi2c1, EEPROM_ADDR,
//						(Rx_Data[4] << 8) | (Rx_Data[5]), 2, dataRead, 1);
//			}

			//HAL_UART_Receive_DMA(&huart3, Rx_Data, 6);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rx_Data, sizeof Rx_Data);
			__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
		}
		if (i2c_write_flag) {
			i2c_write_flag = 0;
//			HAL_UART_Transmit_DMA(&huart3, dataWrite, sizeof(dataWrite));
//			while (uart_tx_flag == 0)
//				;
			//uart_tx_flag = 0;
			//HAL_I2C_Mem_Read_DMA(&hi2c1, 0xA0, (page << 6), 2, dataRead, 64);
//			if (page < 511)
//				page++;
//			else
//				page = 0;
//			UintToStr3(page, &dataWrite[17]);

			//HAL_I2C_Mem_Write_DMA(&hi2c1, 0xA0, (page << 6), 2, dataWrite, 64);
		}
		if (i2c_read_flag) {
			i2c_read_flag = 0;
//			for(int i=0; i<64; i++){
//				Tx_Data[i]
//			}
			switch (Tx_Data[5]) {
			case '0':		//Read page
				Tx_Data[70] = '!';
				HAL_UART_Transmit_DMA(&huart3, Tx_Data, 6+64+1);
				break;
			case '1':		//Read address
				Tx_Data[7] = '!';
				HAL_UART_Transmit_DMA(&huart3, Tx_Data, 6+1+1);
				break;
			default:
				;
			}
			//HAL_UART_Transmit_DMA(&huart3, Tx_Data, 4+64);
			while (uart_tx_flag == 0)
				;
			uart_tx_flag = 0;

			//End Code BTL
//			if (page < 511)
			//page++;
//			else
//				page = 0;
			//HAL_I2C_Mem_Write_DMA(&hi2c1, 0xA0, (page << 6), 2, dataWrite, 64);
//		}
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 400000;
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

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Prescaler = 8400 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000 - 1;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		tk++;
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		i2c_write_flag = 1;

//		if(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY){
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//		}
		//HAL_I2C_Mem_Read_DMA(&hi2c1, 0xA0, (1<<6), 2, dataRead, 64);
	}

}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		tk--;
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		i2c_read_flag = 1;
		//HAL_UART_Transmit_DMA(&huart3, dataRead, sizeof(dataRead));
	}
	if (hi2c == &hi2c2) {
		i2c2_read_flag = 1;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	uart_tx_flag = 1;
	tk = 100;


	//HAL_I2C_Mem_Write_DMA(&hi2c1, 0xA0, (1<<6), 2, dataWrite, 64);

}
//Normal DMA
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	uart_rx_flag = 1;
//	tk = 100;
//	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
}
// IDLE Line DMA
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &huart3) {
		//Cal_uart_size(&Rx_Data[4],&Uart_Rx_Size,2);
		//;
		Uart_Rx_Size = Size/2;
		uint8_t Rx_Data_temp[100];
		for(int i=0; i<Uart_Rx_Size; i++){
			AsciiToBinary(&Rx_Data[2*i], &Rx_Data_temp[i]);
//			0 0
//			2 1
//			4 2
		}
		memset(Rx_Data, 0, sizeof(Rx_Data));
		for(int i=0; i<Uart_Rx_Size; i++){
			Rx_Data[i] = Rx_Data_temp[i];
		}

		//AsciiToBinary(Rx_Data, Rx_Data);
//		sscanf(&tk1,"%x",&tk2);
//		uint8_t tk1[20];
//		uint16_t tk2 = 0xA0FF;
//		for(int i=0; i<4; i++){
//			tk1[i] = Rx_Data[i];
//		}
//		sscanf(&tk1,"%x",&tk2);
		if ((Rx_Data[0] == 0xA0) && (Rx_Data[1] == 0xFF)
				&& (Rx_Data[2] == Uart_Rx_Size)){
//		if((tk2 == 0xA0FF) && (Uart_Rx_Size == Size)){
			uart_rx_flag = 1; //Frame is correct
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		}
		else { //Frame is incorrect
			HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rx_Data, sizeof Rx_Data);
			__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
		}


	}
}
//Timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	tim2_it_flag = 1;
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}

void UintToStr3(uint16_t u, uint8_t *y) {

	//y[5] = '\r';
	y[2] = u % 10 + 0x30;
	u = u / 10;
	y[1] = u % 10 + 0x30;
	u = u / 10;
	y[0] = u % 10 + 0x30;
}

void Cal_uart_size(uint8_t* As_data, uint16_t* Bi_data, uint8_t n){
	*Bi_data = *As_data - 0x30;
	for(int i=0; i<n-1; i++){
		*Bi_data*=10;
		*Bi_data += (*(As_data+i+1) - 0x30);
	}
}

void AsciiToBinary(uint8_t *Ascii, uint8_t *Binary){
	uint8_t Data[2];
	Data[0] = *Ascii;
	Data[1] = *(Ascii+1);
	sscanf(Data,"%x",Binary);
}
//void hexToASCII(const char *hex, char *ascii, size_t maxAsciiLength)
//{
//    // Loop through the hex string
//    for (size_t i = 0; hex[i] != '\0' && i < maxAsciiLength * 2; i += 2)
//    {
//        // Extract two characters from hex string
//        char part[3] = {hex[i], hex[i + 1], '\0'};
//
//        // Convert it into base 16 and typecast as a character
//        char ch = strtol(part, NULL, 16);
//
//        // Add this character to the final ASCII string
//        *ascii++ = ch;
//    }
//    // Null-terminate the ASCII string
//    *ascii = '\0';
//}
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
	while (1) {
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
