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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;
SPI_HandleTypeDef hspi2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Drive_left GPIO_PIN_4
#define Drive_right GPIO_PIN_5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC_Value[2];
char rxBuffer[5];
char txBuffer[5];
uint8_t enkoder_yon;
int enkoderSayma=0;
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */
uint8_t aRxBuffer[] = "\r\n**hareketli** \r\n";


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_DMA_Init(void);
static void MX_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);
static void Timeout_Error_Handler(void);
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
  /* USER CODE BEGIN 2 */
  MX_ADC_Init();
  MX_DMA_Init();
  MX_UART_Init();
  MX_TIM2_Init();
  /* USER CODE END 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, 2);
 // HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  if(HAL_UART_Transmit(&huart1, (uint8_t*)"uart basladi", 22, 5000)!= HAL_OK)
  		  					  {
  			  				    Error_Handler();
  		 	 				  }

    for(volatile uint8_t i=0;i<16;i++)
    {
  	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  	  HAL_Delay(100);
    }
    __HAL_SPI_ENABLE(&hspi2);

     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

     switch(HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)aRxBuffer, (uint8_t *)rxBuffer, 5, 5000))
     {
       case HAL_OK:
       	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
       	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
         break;

       case HAL_TIMEOUT:
       	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
       	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
       case HAL_ERROR:
       	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
       	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
         break;
       default:
         break;
     }
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(HAL_UART_Transmit(&huart1, (uint8_t*)"uart basladi", 22, 5000)!= HAL_OK)
	  		  					  {
	  			  				    Error_Handler();
	  		 	 				  }

	  if(HAL_UART_Receive(&huart1, (uint8_t*)rxBuffer, 5, 100) != HAL_OK)
	 		  				 		{
	 		  				 			Error_Handler();
	 		  				 		}
	  if(strncmp (rxBuffer, "Start", 5)==0)
	  {
	  if(ADC_Value[1] < 1000)
		  		{
		  				HAL_GPIO_WritePin(GPIOA, Drive_left, SET);

		  				HAL_GPIO_WritePin(GPIOA, Drive_right, RESET);
		  				HAL_Delay(200);
		  				if(HAL_UART_Transmit(&huart1, (uint8_t*)"engel tespit edildi", 22, 5000)!= HAL_OK)
		  					  {
			  				    Error_Handler();
		 	 				  }
		  		}
		  			else
		  			{
		  				HAL_GPIO_WritePin(GPIOA, Drive_left, SET);
		  				HAL_GPIO_WritePin(GPIOA, Drive_right, SET);
		  				HAL_Delay(200);
		  				while(HAL_SPI_GetState(&hspi2)!= HAL_SPI_STATE_READY)
		  						 	 {
		  							   switch(HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)aRxBuffer, (uint8_t *)txBuffer, 43, 5000))
		  						 	  {
		  						  			case HAL_OK:
		  							    	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
		  					 		    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		  					 		    	  while(HAL_UART_GetState(&huart1)!= HAL_UART_STATE_READY)
		  						   		      if(HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, 43, 5000)!= HAL_OK)
		  						  				  {
		  						  				    Error_Handler();
		  						  				  }
		  						  	 		      break;
		  						  	 		   case HAL_TIMEOUT:
		  						 	 		    	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
		  									    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		  									   case HAL_ERROR:
		  						   	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
		  						   	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		  						      break;
		  								default:
		  				 		      break;
		  						 	  }
		  					 	 }
		  			 HAL_Delay(500);
		  			}
	  }

	  else if(strncmp (rxBuffer, "Stop", 4)==0)
	  {
		  HAL_GPIO_WritePin(GPIOA, Drive_left, RESET);
		  HAL_GPIO_WritePin(GPIOA, Drive_right, RESET);
 if(HAL_UART_Transmit(&huart1, (uint8_t*)"DURDU", 12, 1000) != HAL_OK)
		  			{
		  		 		Error_Handler();
		  		  	}
		  		  		HAL_Delay(500);

	  }
	  enkoderSayma = __HAL_TIM_GET_COUNTER(&htim2);
	 	  enkoder_yon = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
	 	  HAL_Delay(100);




  }
  }
  /* USER CODE END 3 */


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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef Output_Pin = {0};
  HAL_GPIO_WritePin(GPIOA, Drive_left | Drive_right , GPIO_PIN_RESET);
  Output_Pin.Mode = GPIO_MODE_OUTPUT_PP;
  Output_Pin.Pin = Drive_left | Drive_right;
  Output_Pin.Pull = GPIO_NOPULL;
  Output_Pin.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &Output_Pin);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    /*Configure GPIO pin : PC13 */
   Output_Pin.Pin = GPIO_PIN_13;
   Output_Pin.Mode = GPIO_MODE_OUTPUT_PP;
   Output_Pin.Pull = GPIO_NOPULL;
   Output_Pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &Output_Pin);

    /*Configure GPIO pin : PB0 */
   Output_Pin.Pin = GPIO_PIN_0;
   Output_Pin.Mode = GPIO_MODE_OUTPUT_PP;
   Output_Pin.Pull = GPIO_NOPULL;
   Output_Pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &Output_Pin);


}

/* USER CODE BEGIN 4 */
static void MX_ADC_Init(void)
{
	RCC_PeriphCLKInitTypeDef ADC_Clock = {0};
		ADC_Clock.PeriphClockSelection = RCC_PERIPHCLK_ADC;
		ADC_Clock.AdcClockSelection = RCC_ADCPCLK2_DIV6;
		if(HAL_RCCEx_PeriphCLKConfig(&ADC_Clock) != HAL_OK)
		{
			Error_Handler();
		}

		ADC_ChannelConfTypeDef KanalConfig = {0};

		hadc1.Instance = ADC1;
		hadc1.Init.ContinuousConvMode = ENABLE;
		hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc1.Init.DiscontinuousConvMode = DISABLE;
		hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		hadc1.Init.NbrOfConversion = 2;
		hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
		if(HAL_ADC_Init(&hadc1) != HAL_OK)
		{
			Error_Handler();
		}

		KanalConfig.Channel = ADC_CHANNEL_0;
		KanalConfig.Rank = ADC_REGULAR_RANK_1;
		KanalConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
		if(HAL_ADC_ConfigChannel(&hadc1, &KanalConfig) != HAL_OK)
			{
				Error_Handler();
			}

		KanalConfig.Channel = ADC_CHANNEL_1;
		KanalConfig.Rank = ADC_REGULAR_RANK_2;
		KanalConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
		if(HAL_ADC_ConfigChannel(&hadc1, &KanalConfig) != HAL_OK)
			{
				Error_Handler();
			}

}

static void MX_DMA_Init(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn,1,0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

}

static void MX_UART_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	if(HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}

}

static void MX_TIM2_Init(void)
{
	TIM_MasterConfigTypeDef masterAyarla = {0};
	TIM_Encoder_InitTypeDef sConfig = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 2000-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65356-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
		sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
		sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
		sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
		sConfig.IC1Filter = 0;
		sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
		sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
		sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
		sConfig.IC2Filter = 0;
		if(HAL_TIM_Encoder_Init(&htim2,  &sConfig)!=HAL_OK){
			Error_Handler();
		}

	/*clockAyarla.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if(HAL_TIM_ConfigClockSource(&htim2, &clockAyarla) != HAL_OK)
	{
		Error_Handler();
	}*/
	masterAyarla.MasterOutputTrigger = TIM_TRGO_ENABLE;
	masterAyarla.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if(HAL_TIMEx_MasterConfigSynchronization(&htim2, &masterAyarla) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Timeout_Error_Handler(void)
{
  while(1)
  {
	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	    HAL_Delay(1000);
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
	 while(1)
		  {
		    /* Toogle LED2 for error */
		    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		    HAL_Delay(250);
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
