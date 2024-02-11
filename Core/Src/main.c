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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "lcd_i2c.h"
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
//I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for task_serialComm */
osThreadId_t task_serialCommHandle;
const osThreadAttr_t task_serialComm_attributes = {
  .name = "task_serialComm",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_AMS5812_re */
osThreadId_t task_AMS5812_reHandle;
const osThreadAttr_t task_AMS5812_re_attributes = {
  .name = "task_AMS5812_re",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_i2cLCD */
osThreadId_t task_i2cLCDHandle;
const osThreadAttr_t task_i2cLCD_attributes = {
  .name = "task_i2cLCD",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for queue_AMS5812_readings */
osMessageQueueId_t queue_AMS5812_readingsHandle;
const osMessageQueueAttr_t queue_AMS5812_readings_attributes = {
  .name = "queue_AMS5812_readings"
};
/* Definitions for mutex_AMS5812_reading */
osMutexId_t mutex_AMS5812_readingHandle;
const osMutexAttr_t mutex_AMS5812_reading_attributes = {
  .name = "mutex_AMS5812_reading"
};
/* USER CODE BEGIN PV */

// LCDPARAMETERS
#define LCD_I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit

#define LCD_ROWS 2 // Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD

//AMS5812 I2C Address
#define AMS5812_I2C_ADDR 0x78

typedef struct AMS5812_reading{
	float temperature;
	float pressure;
}AMS5812_reading;

AMS5812_reading AMS5812_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void Start_SerialComms_task(void *argument);
void Start_AMS5812Read_task(void *argument);
void Start_i2cLCD_task(void *argument);

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of mutex_AMS5812_reading */
  mutex_AMS5812_readingHandle = osMutexNew(&mutex_AMS5812_reading_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of queue_AMS5812_readings */
  queue_AMS5812_readingsHandle = osMessageQueueNew (5, sizeof(AMS5812_reading), &queue_AMS5812_readings_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of task_serialComm */
  task_serialCommHandle = osThreadNew(Start_SerialComms_task, NULL, &task_serialComm_attributes);

  /* creation of task_AMS5812_re */
  task_AMS5812_reHandle = osThreadNew(Start_AMS5812Read_task, NULL, &task_AMS5812_re_attributes);

  /* creation of task_i2cLCD */
  task_i2cLCDHandle = osThreadNew(Start_i2cLCD_task, NULL, &task_i2cLCD_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hi2c1.Init.ClockSpeed = 100000;
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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_SerialComms_task */
/**
  * @brief  Function implementing the task_serialComm thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_SerialComms_task */
void Start_SerialComms_task(void *argument)
{
  /* USER CODE BEGIN 5 */
	AMS5812_reading AMS5812_data_read;
	uint32_t transmit_timeout = 50;
	char msgInit[17]="Transmit Init:\n\r";
	HAL_UART_Transmit(&huart2, (uint8_t*)msgInit, 17, transmit_timeout);
  /* Infinite loop */
  for(;;)
  {
	  if(osMessageQueueGet(queue_AMS5812_readingsHandle, &AMS5812_data_read, NULL, 100) == osOK){
		  uint8_t out_str[30];
		  sprintf((char*)out_str, "Temp: %05.2f ; Press: %.2f\r\n", AMS5812_data_read.temperature, AMS5812_data_read.pressure);
		  HAL_UART_Transmit(&huart2, (uint8_t*)out_str, strlen((char*)out_str), HAL_MAX_DELAY);
		  // %0: Pad the number with zeros if it has fewer than the specified width.
		  // 5: Total width of the number, including the decimal point and decimal digits.
		  // .2: Two decimal places.
		  // f: Indicates that the argument is a floating-point number.

	  }else{
		  HAL_UART_Transmit(&huart2, (uint8_t*)"Queue Get Fail\n\r", 16, transmit_timeout);
	  }
      osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_AMS5812Read_task */
/**
* @brief Function implementing the task_AMS5812_re thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_AMS5812Read_task */
void Start_AMS5812Read_task(void *argument)
{
  /* USER CODE BEGIN Start_AMS5812Read_task */

	// Device has a 7-bit address
	// Last bit is reserved for read/write command (1:read)
	//static const uint8_t AMS5812_ADDR = AMS5812_I2C_ADDR << 1;
	// model AMS5812-0050-D ; Pressure Range: 0-5 psi (0-344.7 mbar)
	const int pmin = 0;
	const int pmax = 5;

	HAL_StatusTypeDef ret; // Returns a status code
	uint8_t buff[32];

	  // Extracting pressure MSB and LSB
	  uint8_t pressureMSB;
	  uint8_t pressureLSB;

	  // Extracting temperature MSB and LSB
	  uint8_t temperatureMSB;
	  uint8_t temperatureLSB;

  /* Infinite loop */
  for(;;)
  {
	  //osStatus_t result = osMutexAcquire(mutex_AMS5812_readingHandle, osWaitForever);
	  if(osMutexAcquire(mutex_AMS5812_readingHandle, 100) == osOK){
		  // Read 4 bytes for the pressure and temperature registers
		  ret = HAL_I2C_Master_Receive(&hi2c1, (AMS5812_I2C_ADDR << 1), buff, 4, 100);
		  if(ret != HAL_OK){
			  strcpy((char*)buff, "Error Rx\r\n");
			  //HAL_UART_Transmit(&huart2, buff, strlen((char*)buff), HAL_MAX_DELAY);
		  }else{
			  // Extracting pressure MSB and LSB
			  pressureMSB = buff[0];
			  pressureLSB = buff[1];

			  // Extracting temperature MSB and LSB
			  temperatureMSB = buff[2];
			  temperatureLSB = buff[3];

			  //convert the pressure data into pressure in mbar, see AMS 5812 datasheet
			  AMS5812_data.pressure = ((256*pressureMSB+pressureLSB)-3277.0)*(pmax-pmin)/26214+pmin;

			  //convert the temperature data into temperature in degree celsius
			  AMS5812_data.temperature = (((256*temperatureMSB+temperatureLSB)-3277.0)/238.309)-25;
		  }
		  osMutexRelease(mutex_AMS5812_readingHandle);

		  osMessageQueuePut(queue_AMS5812_readingsHandle, &AMS5812_data, 0, 200);
	  }

      osDelay(1000);
  }
  /* USER CODE END Start_AMS5812Read_task */
}

/* USER CODE BEGIN Header_Start_i2cLCD_task */
/**
* @brief Function implementing the task_i2cLCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_i2cLCD_task */
void Start_i2cLCD_task(void *argument)
{
  /* USER CODE BEGIN Start_i2cLCD_task */
	  lcd_init();
	  lcd_clear();
	  char *text_press = "Press:";
	  char *text_temp = "Temp:";
	  lcd_set_cursor(0, 0);
	  lcd_write_string(text_press);
	  lcd_set_cursor(1, 0);
	  lcd_write_string(text_temp);
	  //lcd_backlight(1); // Turn on backlight
  /* Infinite loop */
  for(;;)
  {
	  if(osMutexAcquire(mutex_AMS5812_readingHandle, 100) == osOK){
		  char temp_str[30];
		  char press_str[30];
		  sprintf(press_str, "%05.2f", AMS5812_data.pressure);
		  sprintf(temp_str, "%05.2f", AMS5812_data.temperature);
		  lcd_set_cursor(0, 7);
		  lcd_write_string(press_str);
		  lcd_set_cursor(1, 7);
		  lcd_write_string(temp_str);
		  osMutexRelease(mutex_AMS5812_readingHandle);
	  }
	  osDelay(1000);
  }
  /* USER CODE END Start_i2cLCD_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9) {
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
