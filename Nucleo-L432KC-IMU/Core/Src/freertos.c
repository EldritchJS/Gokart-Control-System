/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "appLSM9DS1.h"
#include "appI2C.h"
#include "appUSART.h"
#include "console.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for consoleTask */
osThreadId_t consoleTaskHandle;
uint32_t consoleTaskBuffer[ 512 ];
osStaticThreadDef_t consoleTaskControlBlock;
const osThreadAttr_t consoleTask_attributes = {
  .name = "consoleTask",
  .cb_mem = &consoleTaskControlBlock,
  .cb_size = sizeof(consoleTaskControlBlock),
  .stack_mem = &consoleTaskBuffer[0],
  .stack_size = sizeof(consoleTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for memsTask */
osThreadId_t memsTaskHandle;
uint32_t memsTaskBuffer[ 1024 ];
osStaticThreadDef_t memsTaskControlBlock;
const osThreadAttr_t memsTask_attributes = {
  .name = "memsTask",
  .cb_mem = &memsTaskControlBlock,
  .cb_size = sizeof(memsTaskControlBlock),
  .stack_mem = &memsTaskBuffer[0],
  .stack_size = sizeof(memsTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for I2C1RXEventQueue */
osMessageQueueId_t I2C1RXEventQueueHandle;
uint8_t I2C1RXEventQueueBuffer[ 1 * sizeof( uint8_t ) ];
osStaticMessageQDef_t I2C1RXEventQueueControlBlock;
const osMessageQueueAttr_t I2C1RXEventQueue_attributes = {
  .name = "I2C1RXEventQueue",
  .cb_mem = &I2C1RXEventQueueControlBlock,
  .cb_size = sizeof(I2C1RXEventQueueControlBlock),
  .mq_mem = &I2C1RXEventQueueBuffer,
  .mq_size = sizeof(I2C1RXEventQueueBuffer)
};
/* Definitions for I2C1TXEventQueue */
osMessageQueueId_t I2C1TXEventQueueHandle;
uint8_t I2C1TXEventQueueBuffer[ 1 * sizeof( uint8_t ) ];
osStaticMessageQDef_t I2C1TXEventQueueControlBlock;
const osMessageQueueAttr_t I2C1TXEventQueue_attributes = {
  .name = "I2C1TXEventQueue",
  .cb_mem = &I2C1TXEventQueueControlBlock,
  .cb_size = sizeof(I2C1TXEventQueueControlBlock),
  .mq_mem = &I2C1TXEventQueueBuffer,
  .mq_size = sizeof(I2C1TXEventQueueBuffer)
};
/* Definitions for USART1RXEventQueue */
osMessageQueueId_t USART1RXEventQueueHandle;
uint8_t USART1RXEventQueueBuffer[ 1 * sizeof( uint8_t ) ];
osStaticMessageQDef_t USART1RXEventQueueControlBlock;
const osMessageQueueAttr_t USART1RXEventQueue_attributes = {
  .name = "USART1RXEventQueue",
  .cb_mem = &USART1RXEventQueueControlBlock,
  .cb_size = sizeof(USART1RXEventQueueControlBlock),
  .mq_mem = &USART1RXEventQueueBuffer,
  .mq_size = sizeof(USART1RXEventQueueBuffer)
};
/* Definitions for taskTimer */
osTimerId_t taskTimerHandle;
osStaticTimerDef_t taskTimerControlBlock;
const osTimerAttr_t taskTimer_attributes = {
  .name = "taskTimer",
  .cb_mem = &taskTimerControlBlock,
  .cb_size = sizeof(taskTimerControlBlock),
};
/* Definitions for USART1RXMutex */
osMutexId_t USART1RXMutexHandle;
osStaticMutexDef_t USART1RXMutexControlBlock;
const osMutexAttr_t USART1RXMutex_attributes = {
  .name = "USART1RXMutex",
  .cb_mem = &USART1RXMutexControlBlock,
  .cb_size = sizeof(USART1RXMutexControlBlock),
};
/* Definitions for I2C1Mutex */
osMutexId_t I2C1MutexHandle;
osStaticMutexDef_t I2C1MutexControlBlock;
const osMutexAttr_t I2C1Mutex_attributes = {
  .name = "I2C1Mutex",
  .cb_mem = &I2C1MutexControlBlock,
  .cb_size = sizeof(I2C1MutexControlBlock),
};
/* Definitions for USART1TXMutexHandle */
osMutexId_t USART1TXMutexHandle;
osStaticMutexDef_t USART1TXMutexControlBlock;
const osMutexAttr_t USART1TXMutex_attributes = {
  .name = "USART1TXMutes",
  .cb_mem = &USART1TXMutexControlBlock,
  .cb_size = sizeof(USART1TXMutexControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ConsoleTask(void *argument);
void MEMSTask(void *argument);
void taskTimerCb(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of USART1RXMutex */
  USART1RXMutexHandle = osMutexNew(&USART1RXMutex_attributes);

  /* creation of I2C1Mutex */
  I2C1MutexHandle = osMutexNew(&I2C1Mutex_attributes);

  /* creation of myMutex03 */
  USART1TXMutexHandle = osMutexNew(&USART1TXMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of taskTimer */
  taskTimerHandle = osTimerNew(taskTimerCb, osTimerPeriodic, NULL, &taskTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of I2C1RXEventQueue */
  I2C1RXEventQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &I2C1RXEventQueue_attributes);

  /* creation of I2C1TXEventQueue */
  I2C1TXEventQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &I2C1TXEventQueue_attributes);

  /* creation of USART1RXEventQueue */
  USART1RXEventQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &USART1RXEventQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  USART1Init();
  I2C1Init();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of consoleTask */
  consoleTaskHandle = osThreadNew(ConsoleTask, NULL, &consoleTask_attributes);

  /* creation of memsTask */
  memsTaskHandle = osThreadNew(MEMSTask, NULL, &memsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ConsoleTask */
/**
  * @brief  Function implementing the consoleTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ConsoleTask */
__weak void ConsoleTask(void *argument)
{
  /* USER CODE BEGIN ConsoleTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ConsoleTask */
}

/* USER CODE BEGIN Header_MEMSTask */
/**
* @brief Function implementing the memsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MEMSTask */
__weak void MEMSTask(void *argument)
{
  /* USER CODE BEGIN MEMSTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MEMSTask */
}

/* taskTimerCb function */
__weak void taskTimerCb(void *argument)
{
  /* USER CODE BEGIN taskTimerCb */
	osThreadResume(memsTaskHandle);
  /* USER CODE END taskTimerCb */
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

