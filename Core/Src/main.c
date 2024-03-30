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
#include "I2C_LCD.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include "modbus_crc.h"
#include<string.h>
#define SLAVE_ADDRESS_LCD 0x39
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart5, (uint8_t *)ptr, len, HAL_MAX_DELAY);
	return len;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t RxData[64];
uint8_t TxData[8];
uint16_t Data[16];
char buffer[40];

void sendData (uint8_t *data)
{
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);  // enable the transmission
	HAL_UART_Transmit(&huart4, data, 8, 1000);
	HAL_GPIO_WritePin(TX_EN_GPIO_Port,TX_EN_Pin , GPIO_PIN_RESET);  // stop the transmission

}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	Data[0] = RxData[3]<<8 | RxData[4];
	Data[1] = RxData[5]<<8 | RxData[6];
	Data[2] = RxData[7]<<8 | RxData[8];
	Data[3] = RxData[9]<<8 | RxData[10];
	Data[4] = RxData[11]<<8 | RxData[12];
	Data[5] = RxData[13]<<8 | RxData[14];
	Data[6] = RxData[15]<<8 | RxData[16];
	Data[7] = RxData[17]<<8 | RxData[18];
	Data[8] = RxData[19]<<8 | RxData[20];
	Data[9] = RxData[21]<<8 | RxData[22];
	Data[10] = RxData[23]<<8 | RxData[24];
	Data[11] = RxData[25]<<8 | RxData[26];
	Data[12] = RxData[27]<<8 | RxData[28];
	Data[13] = RxData[29]<<8 | RxData[30];
	Data[14] = RxData[31]<<8 | RxData[32];
	Data[15] = RxData[33]<<8 | RxData[34];
//	Data[16] = RxData[35]<<8 | RxData[36];
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET);

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
  MX_USART4_UART_Init();
  MX_USART5_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxData, 32);

  TxData[0] = 0x01;  // slave address
  TxData[1] = 0x03;  // Function code for Read Holding Registers
//  TxData[1] = 0x04;  // Function code for Read Input Registers

  /*
   * The function code 0x03 means we are reading Holding Registers
   * The Register address ranges from 40001 - 50000
   * The register address we input can range from 0-9999 (0x00-0x270F)
   * Here 0 corresponds to the Address 40001 and 9999 corresponds to 50000
   * Although we can only read 125 registers sequentially at once
   */
  TxData[2] = 0x00;  //00
  TxData[3] = 0x61; // voltage current and power values done
  //The Register address will be 00000000 00000100 = 4 + 40001 = 40005

//  TxData[2] = 0;
//  TxData[3] = 0x01;
  //The Register address will be 00000000 00000001 = 1 +30001 = 30002

  TxData[4] = 0x00;
  TxData[5] = 0x0A; //meter will start fetching data from 0061H address to total 1f consecutive data from the next addresses.
  // no of registers to read will be 00000000 00000101 = 5 Registers = 10 Bytes

  uint16_t crc = crc16(TxData, 6);
  TxData[6] = crc&0xFF;   // CRC LOW
  TxData[7] = (crc>>8)&0xFF;  // CRC HIGH


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	sendData(TxData);
	HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxData, 32);

	printf("***** Voltage Values *****\n");
	uint16_t voltage1 = Data[0];  // for frequency 75 to 9 more data
	int Value1 = (int)voltage1;   // Convert hexadecimal to integer
	float floatValue1 = Value1 / 10.0;  // Divide by 100 to get float value
//	sprintf(buffer, "R_Phase Voltage: %.2f\r\n", floatValue1);
	sprintf(buffer, "R_Voltage: %.2f\r\n", floatValue1);
	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);  // Send the string over UART
//	HAL_I2C_Master_Transmit(&hi2c1, 0x39<<1, (uint8_t *)buffer, strlen(buffer), 100);
	sprintf(buffer, "R_Voltage: %.2f", floatValue1);
	lcd_init();
	lcd_send_cmd(0x80|0x00);
	HAL_Delay(10);
	lcd_send_string(buffer);
	HAL_Delay(100);

	uint16_t voltage2 = Data[1];
	int Value2 = (int)voltage2;
	float floatValue2 = Value2 / 10.0;
//	sprintf(buffer, "Y_Phase Voltage: %.2f\r\n", floatValue2);
	sprintf(buffer, "Y_Voltage: %.2f\r\n", floatValue2);
	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);
//	HAL_I2C_Master_Transmit(&hi2c1, 0x39<<1, (uint8_t *)buffer, strlen(buffer), 100);
	sprintf(buffer, "Y_Voltage: %.2f", floatValue2);
	lcd_send_cmd(0x80|0x40);
	HAL_Delay(10);
	lcd_send_string(buffer);
	HAL_Delay(100);

	uint16_t voltage3 = Data[2];
	int Value3 = (int)voltage3;
	float floatValue3 = Value3 / 10.0;
//	sprintf(buffer, "B_Phase Voltage: %.2f\r\n", floatValue3);
	sprintf(buffer, "B_Voltage: %.2f\r\n", floatValue3);
	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);
//	HAL_I2C_Master_Transmit(&hi2c1, 0x39<<1, (uint8_t *)buffer, strlen(buffer), 100);
	sprintf(buffer, "B_Voltage: %.2f", floatValue3);
	lcd_send_cmd(0x80|0x14);
	HAL_Delay(10);
	lcd_send_string(buffer);
	HAL_Delay(100);

	printf("\n");
	printf("***** Current Values *****\n");

	uint16_t current_1 = Data[3];
	int Value4 = (int)current_1;
	float floatValue4 = Value4 / 100.0;
//	sprintf(buffer, "R_Phase Current: %.2f\r\n", floatValue4);
	sprintf(buffer, "R_Current: %.2f\r\n", floatValue4);
	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);
//	HAL_I2C_Master_Transmit(&hi2c1, 0x39<<1, (uint8_t *)buffer, strlen(buffer), 100);
	sprintf(buffer, "R_Current: %.2f", floatValue4);
	lcd_send_cmd(0x80|0x54);
	HAL_Delay(10);
	lcd_send_string(buffer);
	HAL_Delay(100);


	uint16_t current_2 = Data[4];
	int Value5 = (int)current_2;
	float floatValue5 = Value5 / 100.0;
	sprintf(buffer, "Y_Phase Current: %.2f\r\n", floatValue5);
	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);
//	HAL_I2C_Master_Transmit(&hi2c1, 0x39<<1, (uint8_t *)buffer, strlen(buffer), 100);

	uint16_t current_3 = Data[5];
	int Value6 = (int)current_3;
	float floatValue6 = Value6 / 100.0;
	sprintf(buffer, "B_Phase Current: %.2f\r\n", floatValue6);
	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);
//	HAL_I2C_Master_Transmit(&hi2c1, 0x39<<1, (uint8_t *)buffer, strlen(buffer), 100);

	printf("\n");
	printf("***** Power Values *****\n");

	uint16_t Active_power_1 = Data[6];
	int Value7 = (int)Active_power_1;
	float floatValue7 = Value7 / 1000.0;
	sprintf(buffer, "R_Phase Power: %.2f\r\n", floatValue7);
	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);
//	HAL_I2C_Master_Transmit(&hi2c1, 0x39<<1, (uint8_t *)buffer, strlen(buffer), 100);

	uint16_t Active_power_2 = Data[7];
	int Value8 = (int)Active_power_2;
	float floatValue8 = Value8 / 1000.0;
	sprintf(buffer, "Y_Phase Power: %.2f\r\n", floatValue8);
	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);
//	HAL_I2C_Master_Transmit(&hi2c1, 0x39<<1, (uint8_t *)buffer, strlen(buffer), 100);

	uint16_t Active_power_3 = Data[8];
	int Value9 = (int)Active_power_3;
	float floatValue9 = Value9 / 1000.0;
	sprintf(buffer, "B_Phase Power: %.2f\r\n", floatValue9);
	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);
//	HAL_I2C_Master_Transmit(&hi2c1, 0x39<<1, (uint8_t *)buffer, strlen(buffer), 100);

	printf("\n");
	printf("***** Total Power Values *****\n");

	uint16_t Total_power = Data[9];
	int Value10 = (int)Total_power;
	float floatValue10 = Value10 / 1000.0;
	sprintf(buffer, "Total_Active Power: %.2f\r\n", floatValue10);
	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);
//	HAL_I2C_Master_Transmit(&hi2c1, 0x39<<1, (uint8_t *)buffer, strlen(buffer), 100);

	printf("\n\n\n");

//	uint16_t frequency_h = Data[2];  // for frequency 75 to 9 more data
//	int Value = (int)frequency_h;   // Convert hexadecimal to integer
//	float floatValue = Value / 100.0;  // Divide by 100 to get float value
//	sprintf(buffer, "Frequency Value: %.2f\r\n", floatValue);
//	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);  // Send the string over UART
//
//	uint16_t LR_LY_Voltage = Data[3];
//	int Value1 = (int)LR_LY_Voltage;
//	float floatValue1 = Value1 / 10.0;
//	sprintf(buffer, "LR-LY_Voltage Value: %.2f\r\n", floatValue1);
//	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);
//
//	uint16_t LY_LB_Voltage = Data[4];
//	int Value2 = (int)LY_LB_Voltage;
//	float floatValue2 = Value2 / 10.0;
//	sprintf(buffer, "LY-LB_Voltage Value: %.2f\r\n", floatValue2);
//	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);
//
//	uint16_t LB_LR_Voltage = Data[5];
//	int Value3 = (int)LB_LR_Voltage;
//	float floatValue3 = Value3 / 10.0;
//	sprintf(buffer, "LB-LR_Voltage Value: %.2f\r\n", floatValue3);
//	HAL_UART_Transmit(&huart5, (uint8_t *)buffer, strlen(buffer), 100);
//	printf("\n");

	HAL_Delay(200); // Add a delay for demonstration purposes
	lcd_clear();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00707CBB;
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
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart4, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
