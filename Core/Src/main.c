/* USER CODE BEGIN Header */
/** sterowanie silnikami
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int wypelnienie_uart=49;
int wypelnienie_uart2=49;
int tim4_period = 99;

int tim6_period = 99;
///////////////////////////////// receve
#define LINE_MAX_LENGTH	80

static char line_buffer[LINE_MAX_LENGTH + 1];
static uint32_t line_length;

void line_append(uint8_t value)
{
	if (value == '\r' || value == '\n') {
		// odebraliśmy znak końca linii
		if (line_length > 0) {
			// jeśli bufor nie jest pusty to dodajemy 0 na końcu linii
			line_buffer[line_length] = '\0';
			// przetwarzamy dane
			printf("Otrzymano: %s\n", line_buffer);

			//kody: prawy/lewy/oba | przód/tyl | wypelnienie
			//          P/L/O      |    P/T    |    0-99
			// stop = SP

			if (strcmp(line_buffer, "wlacz") == 0) {
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			}
			else if (strcmp(line_buffer, "wylacz") == 0) {
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			}

			else if (line_buffer[0]=='O'&&line_buffer[1]=='P'){ //OP
				HAL_GPIO_WritePin(DIR_minus_GPIO_Port, DIR_minus_Pin, 1);  //direction1 1 = przod
				HAL_GPIO_WritePin(DIR2_minus_GPIO_Port, DIR2_minus_Pin, 0); //direction2 0 = przod
				HAL_TIM_Base_Start_IT(&htim4);
				HAL_TIM_Base_Start_IT(&htim6);
			}
			else if (line_buffer[0]=='O'&&line_buffer[1]=='T'){
				HAL_GPIO_WritePin(DIR_minus_GPIO_Port, DIR_minus_Pin, 0);  //direction1 1 = przod
				HAL_GPIO_WritePin(DIR2_minus_GPIO_Port, DIR2_minus_Pin, 1); //direction2 0 = przod
				HAL_TIM_Base_Start_IT(&htim4);
				HAL_TIM_Base_Start_IT(&htim6);
			}
			else if (line_buffer[0]=='S'&&line_buffer[1]=='P'){
				HAL_TIM_Base_Stop_IT(&htim4);
				HAL_TIM_Base_Stop_IT(&htim6);
			}
			else if (line_buffer[0]=='P'&&line_buffer[1]=='P'){
				HAL_TIM_Base_Start_IT(&htim4);
				HAL_GPIO_WritePin(DIR2_minus_GPIO_Port, DIR2_minus_Pin, 1); //direction2 0 = przod
				HAL_TIM_Base_Start_IT(&htim6);
				HAL_GPIO_WritePin(DIR_minus_GPIO_Port, DIR_minus_Pin, 1);  //direction1 0 = tyl
			}
			else if (line_buffer[0]=='L'&&line_buffer[1]=='P'){
				HAL_TIM_Base_Start_IT(&htim4);
				HAL_GPIO_WritePin(DIR2_minus_GPIO_Port, DIR2_minus_Pin, 0); //direction2 0 = przod
				HAL_TIM_Base_Start_IT(&htim6);
				HAL_GPIO_WritePin(DIR_minus_GPIO_Port, DIR_minus_Pin, 0);  //direction1 0 = tyl
			}
			else if (line_buffer[0]=='D'&&line_buffer[1]=='1'&&line_buffer[2]=='T'){ //Led 1 On
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
			}
			else if (line_buffer[0]=='D'&&line_buffer[1]=='1'&&line_buffer[2]=='F'){ //Led 1 On
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);

			} //*/
			else {
				printf("Nieznane polecenie: %s\n", line_buffer);
			}

			wypelnienie_uart = (int)(line_buffer[2] - '0')*10 + (int)(line_buffer[3] - '0');


			// zaczynamy zbieranie danych od nowa
			line_length = 0;

		}
	}
	else {
		if (line_length >= LINE_MAX_LENGTH) {
			// za dużo danych, usuwamy wszystko co odebraliśmy dotychczas
			line_length = 0;
		}
		// dopisujemy wartość do bufora
		line_buffer[line_length++] = value;
	}
}


/////////////////////////////////////////////////  send
int __io_putchar(int ch)
{
	if (ch == '\n'){
		uint8_t ch2 = '\r';
		HAL_UART_Transmit(&huart2, &ch2, 1, HAL_MAX_DELAY);
	}

	  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	  return 1;
}
/////////////////////////

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
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  int i;

  for (i = 0; i < 10; i++) {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(100);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int a=0;
  int b=0;
  printf("Hello World!\n");
  while (1)
  {

	  uint8_t test;
	  uint8_t value;
	  if (HAL_UART_Receive(&huart2, &value, 1, 0) == HAL_OK) {
		  line_append(value);
		  test = value;
		  //printf("otrzymano: ");
		  printf(value);
	  }
	  if (HAL_GPIO_ReadPin(User_button_GPIO_Port, User_button_Pin)==0){
		  if (a==1){
			  printf("b1\n");
			  b++;
		  }
		  a=0;
	  }
	  if (HAL_GPIO_ReadPin(User_button_GPIO_Port, User_button_Pin)==1){  //detecting edge
		  a=1;
	  }


	  //HAL_GPIO_WritePin(DIR_minus_GPIO_Port, DIR_minus_Pin, 1);  //direction1 1 = przod
	  //HAL_GPIO_WritePin(DIR2_minus_GPIO_Port, DIR2_minus_Pin, 0); //direction2 0 = przod

	  /*
	  if (HAL_GPIO_ReadPin(User_button_GPIO_Port, User_button_Pin)==0)
	  {
		  HAL_TIM_Base_Start_IT(&htim4);
		  HAL_TIM_Base_Start_IT(&htim6);
	  }else{
		  HAL_TIM_Base_Stop_IT(&htim4);//
		  HAL_TIM_Base_Stop_IT(&htim6);
	  }
	  */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim6.Init.Prescaler = 199;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DIR_minus_Pin|PUL_minus_Pin|PUL2_minus_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR2_minus_GPIO_Port, DIR2_minus_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_button_Pin */
  GPIO_InitStruct.Pin = User_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DIR_minus_Pin PUL_minus_Pin PUL2_minus_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DIR_minus_Pin|PUL_minus_Pin|PUL2_minus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_L_Pin */
  GPIO_InitStruct.Pin = Button_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR2_minus_Pin */
  GPIO_InitStruct.Pin = DIR2_minus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR2_minus_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//if (GPIO_Pin == Button_L_Pin){
	//	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	//}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim == &htim6){
		HAL_GPIO_TogglePin(PUL_minus_GPIO_Port, PUL_minus_Pin);
	}

	if (htim == &htim4){
		HAL_GPIO_TogglePin(PUL2_minus_GPIO_Port, PUL2_minus_Pin);
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
