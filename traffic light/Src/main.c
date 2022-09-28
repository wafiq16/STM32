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
#include "lcd_character.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

int range = 0;
int max = 4;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
//terima
uint8_t rxBuffer[20];
int arusTerima = 0;
bool USART3StatusRx = false;
bool flagJalan = 0;

//kirim
uint8_t txBuffer[20];
uint16_t txSize;
//int arusKirim = 4;
int dayaKirim = 0;
int statusKirim = 0;
// safe for 0 <<-->> thread for 1

bool USART3StatusTx = false;
uint8_t strSize;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void reverse(char* str, int len) 
{ 
    int i = 0, j = len - 1, temp; 
    while (i < j) { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; 
        j--; 
    } 
} 
  
// Converts a given integer x to string str[].  
// d is the number of digits required in the output.  
// If d is more than the number of digits in x,  
// then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) { 
        str[i++] = (x % 10) + '0'; 
        x = x / 10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
  
// Converts a floating-point/double number to a string. 
void ftoa(float n, char* res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) { 
        res[i] = '.'; // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter  
        // is needed to handle cases like 233.007 
        fpart = fpart * pow(10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int adcpol;
float adc1,adc2,adc3,adc4,adc5,adc6,V1,V2,I1,I2,Ps,Pb,pf,ARUS;
char lcdbuff1[15];
char lcdbuff2[15];
char lcdbuff3[15];
char lcdbuff4[15];
char lcdbuff5[15];
char lcdbuff6[15];
char lcdbuff7[15];
bool jalan = 0;
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
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();
	HAL_ADC_Init(&hadc1);
	HAL_UART_Receive_DMA(&huart3, rxBuffer, 3);
	
	lcd_init();
	lcd_gotoxy(0,0);
	lcd_puts("Motring & Protec");
	lcd_gotoxy(0,1);
	lcd_puts("    1 Phase   ");
	HAL_Delay(2000);

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_UART_Receive_DMA(&huart3, rxBuffer, 3);
    /* USER CODE END WHILE */
		strSize = sprintf(txBuffer, "A%.2f%.1f%dS", I2, Pb, statusKirim);
		HAL_UART_Transmit(&huart3,(uint8_t *)txBuffer, strSize, HAL_MAX_DELAY);
		HAL_Delay(10);
    /* USER CODE BEGIN 3 */

//	lcd_gotoxy(0,0);
//	lcd_puts("Motring & Protec");
//	lcd_gotoxy(0,1);
//	lcd_puts("    1 Phase   ");
//	
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1,50)== HAL_OK)
		adc1= HAL_ADC_GetValue(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1,50)== HAL_OK)
		adc2= HAL_ADC_GetValue(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1,50)== HAL_OK)
		adc3= HAL_ADC_GetValue(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1,50)== HAL_OK)
		adc4= HAL_ADC_GetValue(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1,50)== HAL_OK)
		adc5= HAL_ADC_GetValue(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1,50)== HAL_OK) // hal ok juika ada masalah maka tidak jalan
		adc6= HAL_ADC_GetValue(&hadc1);

		
if(HAL_GPIO_ReadPin(pb1_GPIO_Port,pb1_Pin)==0){

	lcd_clear();	
	lcd_gotoxy(0,0);
	lcd_puts("   MONITORING");
		lcd_gotoxy(0,1);
	lcd_puts("=================");
		HAL_Delay(2000);
	lcd_clear();	
	
  	V1 = (adc1/4095* 311);
//		V1 = adc1 * 0.05379 ;
		intToStr (V1,lcdbuff1,1);
		lcd_gotoxy(4,0);
		lcd_puts(lcdbuff1);
		lcd_gotoxy(0,0);
		lcd_puts("Vs=");
		lcd_gotoxy(7,0);
		lcd_puts("V");


  	V2 = (adc2/4095* 311);		
//		V2 = adc2 * 0.05379;
		intToStr(V2,lcdbuff2,1);
		lcd_gotoxy(4,1);
		lcd_puts(lcdbuff2);
		lcd_gotoxy(0,1);
		lcd_puts("Vb=");
		lcd_gotoxy(7,1);
		lcd_puts("V");
		
		I1 = ((adc3 *0.0185)- 37.823);
		
//		I1 = (adc3);
		intToStr(I1,lcdbuff3,1);
		lcd_gotoxy(12,0);
		lcd_puts(lcdbuff3);
		lcd_gotoxy(9,0);
		lcd_puts("Is=");
		lcd_gotoxy(15,0);
		lcd_puts("A");
		
		I2 = ((adc4 *0.0185)- 37.823);
		ARUS = I2;
		intToStr(I2,lcdbuff4,1);
		lcd_gotoxy(12,1);
		lcd_puts(lcdbuff4);
		lcd_gotoxy(9,1);
		lcd_puts("Ib=");
		lcd_gotoxy(15,1);
		lcd_puts("A");
	
	HAL_Delay(3000);
	lcd_clear();	

		Ps = (V1)*(I1) ;
		ftoa (Ps,lcdbuff5,1);
		lcd_gotoxy(4,0);
		lcd_puts(lcdbuff5);
		lcd_gotoxy(0,0);
		lcd_puts("Ps=");
		lcd_gotoxy(10,0);
		lcd_puts("W");
	
	
		Pb = (V2)*(I2) ;
		ftoa (Pb,lcdbuff6,1);
		lcd_gotoxy(4,1);
		lcd_puts(lcdbuff6);
		lcd_gotoxy(0,1);
		lcd_puts("Pb=");
		lcd_gotoxy(10,1);
		lcd_puts("W");

	HAL_Delay(3000);
}


if(HAL_GPIO_ReadPin(pb2_GPIO_Port,pb2_Pin)==0 || (arusTerima == 2))
	{
	lcd_clear();
	lcd_gotoxy(0,0);
	lcd_puts("SETTING");
	lcd_gotoxy(0,1);
	lcd_puts("Limit Arus= 2A");
	HAL_Delay(2000);
		
	lcd_clear();
	lcd_gotoxy(0,0);
	lcd_puts("SET");
	lcd_gotoxy(0,1);
	lcd_puts("OK...");	
	HAL_Delay(1000);
	
		I2 = ((adc4 *0.0185)- 37.823);
		ARUS = I2;
		intToStr(I2,lcdbuff4,1);
		lcd_gotoxy(4,0);
		lcd_puts(lcdbuff4);
		lcd_gotoxy(0,0);
		lcd_puts("Ib=");
		lcd_gotoxy(7,0);
		lcd_puts("A");
	
		
		Pb = (V2)*(I2) ;
		ftoa (Pb,lcdbuff6,1);
		lcd_gotoxy(12,0);
		lcd_puts(lcdbuff6);
		lcd_gotoxy(9,0);
		lcd_puts("Pb=");
		lcd_gotoxy(15,0);
		lcd_puts("W");
		
	
	if(ARUS > 2) // LEBIH dari 2 Ampere
	{
	HAL_GPIO_WritePin(Relay1_GPIO_Port,Relay1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Relay2_GPIO_Port,Relay2_Pin,GPIO_PIN_SET);
	lcd_gotoxy(0,1);
	lcd_puts("Rangkaian Trip");
	statusKirim = 1;
	}
	if(ARUS < 2)  // KURANG dari 2 Ampere
	{
	HAL_GPIO_WritePin(Relay1_GPIO_Port,Relay1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Relay2_GPIO_Port,Relay2_Pin,GPIO_PIN_RESET);
	lcd_gotoxy(0,1);
	lcd_puts("Rangkaian Normal");
	statusKirim = 0;
	}
	arusTerima = 0;
}


if(HAL_GPIO_ReadPin(pb3_GPIO_Port,pb3_Pin)==0 || (arusTerima == 4))
	{
	lcd_clear();
	lcd_gotoxy(0,0);
	lcd_puts("SETTING");
	lcd_gotoxy(0,1);
	lcd_puts("Limit Arus= 4A");
	HAL_Delay(2000);
		
	lcd_clear();
	lcd_gotoxy(0,0);
	lcd_puts("SET");
	lcd_gotoxy(0,1);
	lcd_puts("OK...");	
	HAL_Delay(1000);
	
		
		I2 = ((adc4 *0.0185)- 37.823);
		ARUS = I2;
		intToStr(I2,lcdbuff4,1);
		lcd_gotoxy(4,0);
		lcd_puts(lcdbuff4);
		lcd_gotoxy(0,0);
		lcd_puts("Ib=");
		lcd_gotoxy(7,0);
		lcd_puts("A");
	
		
		Pb = (V2)*(I2) ;
		ftoa (Pb,lcdbuff6,1);
		lcd_gotoxy(12,0);
		lcd_puts(lcdbuff6);
		lcd_gotoxy(9,0);
		lcd_puts("Pb=");
		lcd_gotoxy(15,0);
		lcd_puts("W");
		jalan = false;
		
	
	if(ARUS > 4)  // LEBIH dari 4 Ampere
	{
	HAL_GPIO_WritePin(Relay1_GPIO_Port,Relay1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Relay2_GPIO_Port,Relay2_Pin,GPIO_PIN_SET);
	lcd_gotoxy(0,1);
	lcd_puts("Rangkaian Trip");
	statusKirim = 1;
	}
	if(ARUS < 4)  // KURANG dari 4 Ampere
	{
	HAL_GPIO_WritePin(Relay1_GPIO_Port,Relay1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Relay2_GPIO_Port,Relay2_Pin,GPIO_PIN_RESET);
	lcd_gotoxy(0,1);
	lcd_puts("Rangkaian Normal");
	statusKirim = 0;
	} 
	arusTerima = 0;
	
	}
	HAL_ADC_Stop(&hadc1);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Relay1_Pin|Relay2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : Relay1_Pin Relay2_Pin */
  GPIO_InitStruct.Pin = Relay1_Pin|Relay2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : pb1_Pin pb2_Pin pb3_Pin */
  GPIO_InitStruct.Pin = pb1_Pin|pb2_Pin|pb3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3){
		if(rxBuffer[0] == 'B' && rxBuffer[2] == 'S') {
			arusTerima = rxBuffer[1] - '0';
			ARUS = arusTerima;
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
