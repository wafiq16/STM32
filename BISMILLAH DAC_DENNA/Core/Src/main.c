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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lcd_character.h"
#include "stdbool.h"
#include "math.h"
#include "string.h"
#include "MY_DHT22.h"

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
 DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char buffer[200];
float TempC, Humidity;
char uartData[50];
char statusBaju[50];
int status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float valVolt = 3; //Range 0 - 3 volts
	uint8_t valByte;
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
  MX_DAC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	//--------------------------IDENTITAS----------------------- 
	lcd_gotoxy(0,0);
	lcd_puts("PENGERING PAKAIAN");
	lcd_gotoxy(0,1);
	lcd_puts("DENNA YUNAN OKTARA");
	lcd_gotoxy(0,2);
	lcd_puts("3 D3 ELIN B");
	lcd_gotoxy(0,3);
	lcd_puts("1303191053");
	HAL_Delay(5000);
	lcd_clear();
	
	//-----------------------------DAC--------------------------
	valByte = (uint8_t)((valVolt/3.0)*255);
	
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, valByte);
	
	DHT22_Init(GPIOA, GPIO_PIN_15);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
	//---------------------------COBA RELAY------------------------------
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);
//	lcd_gotoxy(0,0);
//	lcd_puts("Blower ON");
//	HAL_Delay(2000);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET);
//	lcd_gotoxy(0,1);
//	lcd_puts("Blower OFF");
//	HAL_Delay(2000);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		int strSize = sprintf((char*)buffer, "mosok rakenek");
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strSize, 100);
		
		// ====================== Jika DHT22 BISA DIAKSES ==================================
		if(DHT22_GetTemp_Humidity(&TempC, &Humidity) == 1)
		{
			
			// kondisi suhu dibawah 50 
			if(TempC < 50.0 ){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);
				//lcd_gotoxy(0,0);
				//lcd_puts("Blower ON");
				//HAL_Delay(2000);
				
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
				//lcd_gotoxy(0,0);
				//lcd_puts("Exhaust OFF");
				//HAL_Delay(2000);
				//int strSize = sprintf((char*)buffer, "dibawah 50");
				//HAL_UART_Transmit(&huart3, buffer, strSize, 100);
			}
			
			// kondisi suhu diatas 50
			else{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET);
				//lcd_gotoxy(0,0);
				//lcd_puts("Blower OFF");
				//HAL_Delay(2000);
				
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
				//lcd_gotoxy(0,0);
				//lcd_puts("Exhaust ON");
				//HAL_Delay(2000);
			}
			
			// kondisi kelembapan diatas 20
			if(Humidity > 20.0){
				//lcd_gotoxy(0,0);
				//lcd_puts("Belum Kering");
				//HAL_Delay(2000);
				sprintf(statusBaju, "Belum Kering");
				status = 0;
			}
			
			// kondisi kelembapan dibawah 20
			else{
				//lcd_gotoxy(0,0);
				//lcd_puts("Sudah Kering");
				//HAL_Delay(2000);
				sprintf(statusBaju, "Sudah Kering");
				status = 1;
			}
			
			int digitT = sprintf(buffer, "%0.2f", TempC);
			int digitH = sprintf(buffer, "%0.2f", Humidity);
			int digitD = sprintf(buffer, "%d", status);
			//sprintf(uartData, "%0.2f%0.2f%s", TempC, Humidity, statusBaju);
			sprintf(uartData, "AT%d%0.2fH%d%0.2fD%d%dB", digitT, TempC, digitH, Humidity, digitD, status);
			HAL_UART_Transmit(&huart3, (uint8_t *)uartData, strlen(uartData), 100);
		}
		
		// JIKA DHT22 TDK BISA DIAKSES
		else
		{
			sprintf(uartData, "AT%d%sH%d%sD%d%sB", strlen("null"), "null", strlen("null"), "null", strlen("null"), "null");
			HAL_UART_Transmit(&huart3, (uint8_t *)uartData, strlen(uartData), 100);
		}
		
		HAL_Delay(1000);
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
