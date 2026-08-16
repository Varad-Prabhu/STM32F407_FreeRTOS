/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint8_t source;
    uint32_t value;
} DataMessage_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_READY_BIT    (1U << 0U)
#define DATA_READY_BIT      (1U << 1U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
TaskHandle_t sensorTaskHandle;
TaskHandle_t dataTaskHandle;
TaskHandle_t processingTaskHandle;
TaskHandle_t monitorTaskHandle;
TaskHandle_t eventTaskHandle;

QueueHandle_t dataQueue;

SemaphoreHandle_t uartMutex;

EventGroupHandle_t eventGroup;

TimerHandle_t systemTimer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void UART_Print(const char *msg);

static void SystemTimerCallback(TimerHandle_t xTimer);

static void SensorTask(void *argument);
static void DataTask(void *argument);
static void ProcessingTask(void *argument);
static void SystemMonitorTask(void *argument);
static void EventMonitorTask(void *argument);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  dataQueue = xQueueCreate(10, sizeof(DataMessage_t));

  if (dataQueue == NULL)
  {
      Error_Handler();
  }

  uartMutex = xSemaphoreCreateMutex();

  if (uartMutex == NULL)
  {
      Error_Handler();
  }

  eventGroup = xEventGroupCreate();

  if (eventGroup == NULL)
  {
      Error_Handler();
  }

  systemTimer = xTimerCreate( "SystemTimer", pdMS_TO_TICKS(1000), pdTRUE, NULL, SystemTimerCallback);

  if (systemTimer == NULL)
  {
      Error_Handler();
  }

  if (xTimerStart(systemTimer, 0) != pdPASS)
  {
      Error_Handler();
  }

  xTaskCreate(SensorTask, "SensorTask", 256, NULL, 2, &sensorTaskHandle);

  xTaskCreate(DataTask, "DataTask", 256, NULL, 2, &dataTaskHandle);

  xTaskCreate(ProcessingTask, "ProcessingTask", 256, NULL, 2, &processingTaskHandle);

  xTaskCreate(SystemMonitorTask, "MonitorTask", 512, NULL, 1, &monitorTaskHandle);

  xTaskCreate(EventMonitorTask, "EventTask", 256, NULL, 1, &eventTaskHandle);

  vTaskStartScheduler();

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void UART_Print(const char *msg)
{
    xSemaphoreTake(uartMutex, portMAX_DELAY);

    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

    xSemaphoreGive(uartMutex);
}

static void SystemTimerCallback(TimerHandle_t xTimer)
{
    UART_Print("System timer: 1 second\r\n");
}

static void SensorTask(void *argument)
{
    DataMessage_t msg;

    uint32_t value = 100;

    for (;;)
    {
        msg.source = 1;
        msg.value = value++;

        xQueueSend(dataQueue, &msg, portMAX_DELAY);

        UART_Print("Sensor data sent\r\n");

        xEventGroupSetBits(eventGroup, SENSOR_READY_BIT);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void DataTask(void *argument)
{
    DataMessage_t msg;

    uint32_t value = 200;

    for (;;)
    {
        msg.source = 2;
        msg.value = value++;

        xQueueSend(dataQueue, &msg, portMAX_DELAY);

        UART_Print("Data sent\r\n");

        xEventGroupSetBits(eventGroup, DATA_READY_BIT);

        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

static void ProcessingTask(void *argument)
{
    DataMessage_t msg;

    char buffer[80];

    for (;;)
    {
        /* Wait for data from queue */
        if (xQueueReceive(dataQueue, &msg, pdMS_TO_TICKS(100)) == pdPASS)
        {
            snprintf(buffer, sizeof(buffer), "Processing source=%d value=%lu\r\n", msg.source, (unsigned long)msg.value);

            UART_Print(buffer);
        }

        /* Check whether button generated notification */
        if (ulTaskNotifyTake(pdTRUE, 0))
        {
            UART_Print("Button notification received\r\n");
            HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == B1_Pin)
    {
        vTaskNotifyGiveFromISR(processingTaskHandle, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void SystemMonitorTask(void *argument)
{
    char buffer[400];

    UBaseType_t sensorStack;
    UBaseType_t dataStack;
    UBaseType_t processingStack;
    UBaseType_t monitorStack;
    UBaseType_t eventStack;
    UBaseType_t queueMessages;
    UBaseType_t queueSpaces;

    size_t freeHeap;
    size_t minFreeHeap;

    for (;;)
    {
        sensorStack = uxTaskGetStackHighWaterMark(sensorTaskHandle);

        dataStack = uxTaskGetStackHighWaterMark(dataTaskHandle);

        processingStack = uxTaskGetStackHighWaterMark(processingTaskHandle);

        monitorStack = uxTaskGetStackHighWaterMark(monitorTaskHandle);

        eventStack = uxTaskGetStackHighWaterMark(eventTaskHandle);

        freeHeap = xPortGetFreeHeapSize();

        minFreeHeap = xPortGetMinimumEverFreeHeapSize();

        queueMessages = uxQueueMessagesWaiting(dataQueue);
        queueSpaces = uxQueueSpacesAvailable(dataQueue);

        snprintf(buffer,
                 sizeof(buffer),
                 "\r\n"
                 "========== RTOS MONITOR ==========\r\n"
                 "SensorTask      Stack Free: %lu\r\n"
                 "DataTask        Stack Free: %lu\r\n"
                 "ProcessingTask  Stack Free: %lu\r\n"
                 "MonitorTask     Stack Free: %lu\r\n"
                 "EventTask       Stack Free: %lu\r\n"
                 "Free Heap       : %lu\r\n"
                 "Minimum Heap    : %lu\r\n"
                 "Queue Messages  : %lu\r\n"
                 "Queue Free Slots: %lu\r\n"
                 "===================================\r\n",
                 (unsigned long)sensorStack,
                 (unsigned long)dataStack,
                 (unsigned long)processingStack,
                 (unsigned long)monitorStack,
                 (unsigned long)eventStack,
                 (unsigned long)freeHeap,
                 (unsigned long)minFreeHeap,
                 (unsigned long)queueMessages,
                 (unsigned long)queueSpaces);

        UART_Print(buffer);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void EventMonitorTask(void *argument)
{
    EventBits_t bits;

    for (;;)
    {
        bits = xEventGroupWaitBits(eventGroup, SENSOR_READY_BIT | DATA_READY_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

        if ((bits & (SENSOR_READY_BIT | DATA_READY_BIT)) == (SENSOR_READY_BIT | DATA_READY_BIT))
        {
            UART_Print("Both SENSOR and DATA are READY\r\n");
        }
    }
}
/* USER CODE END 4 */

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
  if (htim->Instance == TIM6)
  {
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
#ifdef USE_FULL_ASSERT
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
