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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct rand_value {
  float mean;//variable for mean
  float sq_mean;//variable for square mean
};
//structure for mean and square mean of 3 tasks.
struct rand_value memory[3];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId blink01Handle;
osThreadId Task1Handle;
osThreadId Task2Handle;
osThreadId Task3Handle;
osThreadId TaskUARTHandle;
osSemaphoreId semaphor1Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void StartBlink03(void const * argument);
void StartTask1(void const * argument);
void StartTask2(void const * argument);
void StartTask3(void const * argument);
void StartTaskUART(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float random_float(const float min, const float max) //function for choosing random value of float
{
    if (max == min) return min;
    else if (min < max) return (max - min) * ((float)rand() / RAND_MAX) + min;

    // return 0 if min > max
    return 0;
}

int _write(int file, char * ptr, int len)

{   int DataIdx;

    for(DataIdx = 0; DataIdx < len; DataIdx++)

    { ITM_SendChar(*ptr++); }

     return len;

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	srand(time(NULL)); //needed for pseudo random numbers to work properly (to be random :) )
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of semaphor1 */
  osSemaphoreDef(semaphor1);
  semaphor1Handle = osSemaphoreCreate(osSemaphore(semaphor1), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of blink01 */
  osThreadDef(blink01, StartBlink03, osPriorityNormal, 0, 128);
  blink01Handle = osThreadCreate(osThread(blink01), NULL);

  /* definition and creation of Task1 */
  osThreadDef(Task1, StartTask1, osPriorityIdle, 0, 256);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  /* definition and creation of Task2 */
  osThreadDef(Task2, StartTask2, osPriorityLow, 0, 256);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  /* definition and creation of Task3 */
  osThreadDef(Task3, StartTask3, osPriorityBelowNormal, 0, 256);
  Task3Handle = osThreadCreate(osThread(Task3), NULL);

  /* definition and creation of TaskUART */
  osThreadDef(TaskUART, StartTaskUART, osPriorityNormal, 0, 512);
  TaskUARTHandle = osThreadCreate(osThread(TaskUART), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit_DMA(&huart2, ch, sizeof(char));
    //UART_PutChar(ch);
	HAL_Delay(10);
   return ch;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlink03 */
/**
  * @brief  Function implementing the blink01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlink03 */
void StartBlink03(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask1 */
/**
* @brief Function implementing the Task1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask1 */
void StartTask1(void const * argument)
{
  /* USER CODE BEGIN StartTask1 */
  /* Infinite loop */
  for(;;)
  {
	  float sum_val=0;
	  float temp_val = 0;
	  float  sum_square=0;
	  for(int i=0; i<1000;++i)
	  {
		  temp_val = random_float(-1,+1);
		  sum_val += temp_val;
		  sum_square += (temp_val*temp_val);

	  }
	  sum_val=sum_val/1000;
	  sum_square /= 1000;
	  float  sq_mean = sqrt(sum_square);
	  osSemaphoreWait(semaphor1Handle, osWaitForever);
	  memory[0].mean = sum_val;
	  memory[0].sq_mean = sq_mean;
	  osSemaphoreRelease(semaphor1Handle);

    osDelay(100);
  }
  /* USER CODE END StartTask1 */
}

/* USER CODE BEGIN Header_StartTask2 */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask2 */
void StartTask2(void const * argument)
{
  /* USER CODE BEGIN StartTask2 */
  /* Infinite loop */
  for(;;)
  {
	  float sum_val=0;
	  float temp_val = 0;
	  float  sum_square=0;
	  for(int i=0; i<1000;++i)
	  {
		  temp_val = random_float(-1,+1);
		  sum_val += temp_val;
		  sum_square += (temp_val*temp_val);

	  }
	  sum_val=sum_val/1000;
	  sum_square /= 1000;
	  float  sq_mean = sqrt(sum_square);
	  osSemaphoreWait(semaphor1Handle, osWaitForever);
	  memory[1].mean = sum_val;
	  memory[1].sq_mean = sq_mean;
	  osSemaphoreRelease(semaphor1Handle);

    osDelay(100);
  }
  /* USER CODE END StartTask2 */
}

/* USER CODE BEGIN Header_StartTask3 */
/**
* @brief Function implementing the Task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask3 */
void StartTask3(void const * argument)
{
  /* USER CODE BEGIN StartTask3 */
  /* Infinite loop */
  for(;;)
  {
	  float sum_val=0;
	  float temp_val = 0;
	  float  sum_square=0;
	  for(int i=0; i<1000;++i)
	  {
		  temp_val = random_float(-1,+1);
		  sum_val += temp_val;
		  sum_square += (temp_val*temp_val);

	  }
	  sum_val=sum_val/1000;
	  sum_square /= 1000;
	  float  sq_mean = sqrt(sum_square);
	  osSemaphoreWait(semaphor1Handle, osWaitForever);
	  memory[2].mean = sum_val;
	  memory[2].sq_mean = sq_mean;
	  osSemaphoreRelease(semaphor1Handle);

    osDelay(100);
  }
  /* USER CODE END StartTask3 */
}

/* USER CODE BEGIN Header_StartTaskUART */
/**
* @brief Function implementing the TaskUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskUART */
void StartTaskUART(void const * argument)
{
  /* USER CODE BEGIN StartTaskUART */
  /* Infinite loop */
  for(;;)
  {
	 HAL_UART_Abort(&huart2);
	  	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  	  {
	  	    Error_Handler();
	  	  }
	  osSemaphoreWait(semaphor1Handle, osWaitForever);
	  float mean = (memory[0].mean + memory[1].mean + memory[2].mean)/3;
	  float sq_mean = sqrt((memory[0].sq_mean*memory[0].sq_mean + memory[1].sq_mean*memory[1].sq_mean+ memory[2].sq_mean*memory[2].sq_mean)/3);
	  uint8_t data[60]; // Tablica przechowujaca wysylana wiadomosc.
	 // uint8_t data2[50]; // Tablica przechowujaca wysylana wiadomosc.

	  u_int16_t size = sprintf(data, "Srednia z wartosci: %f.\n\r Srednia kwadratowa: %f. \n\r" , mean, sq_mean);
	  HAL_UART_Transmit_DMA(&huart2, data, size);
	   //size = sprintf(data2, "Srednia sq wartosci: %f.\n\r", sq_mean);
printf("Srednia z wartosci: %f.\n\r Srednia kwadratowa: %f. \n\r" , mean, sq_mean);

	   //printf("%f \n\r", memory[1].mean);
	   //printf("HEJHEJ  \n\r");
	   //fflush(stdout);
	  //memory[1].sq_mean = sq_mean;
	  osSemaphoreRelease(semaphor1Handle);


	  osDelay(1000);
  }
  /* USER CODE END StartTaskUART */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
