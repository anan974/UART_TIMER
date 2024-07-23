/* USER CODE BEGIN Header */
/**
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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// again
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t counter;
uint32_t arr;
int choice = 0;
int prev_choice;
uint8_t on_led_mode[] = "On led mode\n";
uint8_t off_led_mode[] = "Off led mode\n";
uint8_t waiting_20s_mode[] = "Waiting 20s mode\n";
uint8_t waiting_10s_mode[] = "Waiting 10s mode\n";
uint8_t stop_waiting_mode[] = "Stop waiting mode\n";
uint8_t read_sensor_mode[]="Read sensor mode\n";
uint8_t blinking_mode[20] = "\nBlinking Mode\n";
uint8_t modulation_mode[20] = "\nModulation Mode\n";
uint8_t stop_modulation_mode[42] = "\nStop Modulation Mode\n";
uint8_t stop_blinking_mode[42] = "\nStop Blinking Mode\n";
uint8_t done[40] = "\nDone! Enter 9 to stop\n";
uint8_t be_on_operation[100] = "\nSelected Mode Is On Operation\n";
uint8_t interrupt_the_mode[100] = "Stop running current mode\n";
uint8_t logg[200] ="\n1. On/off mode\n2.Waiting 10s mode\n3. Waiting 20s mode\n4. Reading sensor mode\n5.Blinking Mode\n6. Modulation Mode\n";
///
uint32_t waiting;
uint32_t previous_tim = 0;
uint32_t current_tim = 0;
uint8_t msg[1];
uint8_t led[5] ={LED1_Pin,LED2_Pin,LED3_Pin,LED4_Pin,LED5_Pin};
//flag

bool choice_flag = true;
bool stop_flag;
bool on_led_flag = false,
	waiting_flag,
	 on_operation_flag;
//
//sensor
uint8_t RHI, RHD, TCI, TCD, SUM;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;
uint8_t TFI = 0;
uint8_t TFD = 0;
char strCopy[15];
//
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < delay);
}

uint8_t DHT11_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT11_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
  HAL_Delay(20);   // wait for 20ms
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as input
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT11_Read (void)
{
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
}

void Read_RH(void) {
	char cmd[30];
	sprintf(cmd,"Rh: %2.f",RH);
	HAL_UART_Transmit(&huart1,(uint8_t*)cmd,(uint8_t)strlen(cmd), 100);
	uint8_t pct[]="%\n";
	HAL_UART_Transmit(&huart1,(uint8_t*)pct,(uint8_t)sizeof(pct), 100);
}

void Read_Temp(void) {
	char cmd[30];
	sprintf(cmd,"Temp: %2.f C",tCelsius);
	HAL_UART_Transmit(&huart1,(uint8_t*)cmd,(uint8_t)strlen(cmd), 100);
}


void On_led(void) {
	HAL_UART_Transmit_IT(&huart1, on_led_mode, sizeof(on_led_mode));
	for (int i = 0; i < 5; i++) {
		HAL_GPIO_WritePin(LED2_GPIO_Port, led[i], 1);
	}
	uint8_t on[] = "\nLEDs TURNED ON";
	HAL_UART_Transmit_IT(&huart1, on, sizeof(on));
	on_led_flag = true;
}

void Off_led(void) {
	HAL_UART_Transmit_IT(&huart1, off_led_mode, sizeof(off_led_mode));
	for (int i = 0; i < 5; i++) {
		HAL_GPIO_WritePin(LED2_GPIO_Port, led[i], 0);
		uint8_t off[] = "\nLEDs TURNED OFF";
		HAL_UART_Transmit_IT(&huart1, off, sizeof(off));
	}
	on_led_flag=false;
}


void Stop_Waiting_Mode(void) {
	HAL_TIM_Base_Stop_IT(&htim1);
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	waiting_flag = false;
}

void Waiting_Mode_Set(uint32_t time, int x)  {
	current_tim = HAL_GetTick();
	waiting_flag= true;
	//counter = __HAL_TIM_GET_COUNTER(&htim1);
	//arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
	__HAL_TIM_SET_AUTORELOAD(&htim1, time - 1);
	HAL_TIM_Base_Start_IT(&htim1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	previous_tim = current_tim;
	current_tim = HAL_GetTick();
	waiting=(current_tim-previous_tim)/1000;
	if (htim->Instance == htim1.Instance) {
		char onled[60];
		uint8_t offled[] = "LED OFF\n";
		sprintf(onled,"LED ON. CYCLE: %lu s\n",waiting);
		HAL_UART_Transmit(&huart1, (uint8_t*)onled, (uint8_t)strlen(onled),100);
		if (__HAL_TIM_GET_AUTORELOAD(&htim1) == 24000 - 1) {
			for (int i = 0; i < 5; i++) {
				HAL_GPIO_WritePin(LED2_GPIO_Port, led[i], 1);
				for (int i = 0; i < 2000; i++) {
					for (int j = 0; j < 500; j++) {
					}
				}
			}
		} else if (__HAL_TIM_GET_AUTORELOAD(&htim1) == 12000 - 1){
			for (int i = 4; i >= 0; i--) {
				HAL_GPIO_WritePin(LED2_GPIO_Port, led[i], 1);
				for (int i = 0; i < 2000; i++) {
					for (int j = 0; j < 500; j++) {
					}
				}
			}
		}

		for (int i = 0; i < 5; i++) {
			HAL_GPIO_TogglePin(LED2_GPIO_Port, led[i]);
		}
		HAL_UART_Transmit(&huart1, offled, sizeof(offled),100);
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance==huart1.Instance)
  {
	  choice = msg[0]-'0';
	  if(choice == 9) stop_flag = true;
	  if( (waiting_flag && choice == prev_choice)) {
		  HAL_UART_Transmit_IT(&huart1, be_on_operation, sizeof(be_on_operation));
	  } else if (on_operation_flag && choice != prev_choice) {
		  HAL_UART_Transmit_IT(&huart1,interrupt_the_mode, sizeof(interrupt_the_mode));
		  stop_flag=true;
	  }
	  //else if(choice == 7 && !modulation_flag && !blinking_flag)
	 /* {
		  HAL_UART_Transmit(&huart1, blinking_mode, sizeof(blinking_mode), HAL_MAX_DELAY);
		  HAL_UART_Receive(&huart1, counter_from_x, sizeof(counter_from_x), HAL_MAX_DELAY);
		  counter = a2i(counter_from_x);
		  StartBlinkingMode(counter);
		  choice_flag = false;
	  } */
	  prev_choice = choice;
	  choice_flag = true;
	  HAL_UART_Receive_IT(&huart1, msg, sizeof(msg));
  }
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, msg, sizeof(msg));
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (DHT11_Start()) {
			RHI = DHT11_Read(); // Relative humidity integral
			RHD = DHT11_Read(); // Relative humidity decimal
			TCI = DHT11_Read(); // Celsius integral
			TCD = DHT11_Read(); // Celsius decimal
			SUM = DHT11_Read(); // Check sum
			if (RHI + RHD + TCI + TCD == SUM) {
				// Can use RHI and TCI for any purposes if whole number only needed
				tCelsius = (float) TCI + (float) (TCD / 10.0);
				tFahrenheit = tCelsius * 9 / 5 + 32;
				RH = (float) RHI + (float) (RHD / 10.0);
			}
		}
		if (stop_flag) {
			/*if (modulation_flag)
			 StopModulationMode();
			 else if (blinking_flag)
			 StopBlinkingMode();
			 */
			if (waiting_flag)
				Stop_Waiting_Mode();
			else if (on_led_flag)
				Off_led();
			stop_flag = false;
		}

		if (choice_flag) {
			switch (choice) {
			case 0:
				HAL_UART_Transmit_IT(&huart1, logg, sizeof(logg));
				choice = -1;
				break;
			case 1:
				if (!on_led_flag) {
					On_led();
				} else {
					Off_led();
				}
				break;
			case 2:
				HAL_UART_Transmit_IT(&huart1, waiting_20s_mode, sizeof(waiting_20s_mode));
				HAL_Delay(200);
				Waiting_Mode_Set(24000, 20);
				waiting_flag = true;
				break;
			case 3:
				HAL_UART_Transmit_IT(&huart1, waiting_10s_mode, sizeof(waiting_10s_mode));
				HAL_Delay(200);
				Waiting_Mode_Set(12000, 10);
				waiting_flag = true;
				break;
			case 4:
				Read_RH();
				HAL_Delay(200);
				Read_Temp();
				break;
			case 9:
				break;
			default:
				if (choice != -1) {
					uint8_t nhaplai[] = "Error\n";
					HAL_UART_Transmit_IT(&huart1, nhaplai, sizeof(nhaplai));
					choice = 0;
				}
			}
			choice_flag = false;
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_Delay(1000);
		/* USER CODE END 3 */
	}
	}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 60000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin
                           LED5_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

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
