/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos.c
  * @brief   Base code for FreeRTOS applcation. Declaration of tasks, global
  *          variables, kernel objects, typedefs and project libraries.
  ******************************************************************************
  * @author  Gabriel Toffanetto França da Rocha 
  *          Laboratory of Autonomous Vehicles (LMA) - FEM/Unicamp
  * @date    Created:  October 9, 2024
  *          Modified: 
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Tasks includes -- START
#include "taskControle.h"
#include "microAutoware.h"

// Tasks includes -- END

// Libraries includes -- START
#include "light_printf.h"
#include "utils.h"

// Libraries includes -- END

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for TaskControle */
osThreadId_t TaskControleHandle;
const osThreadAttr_t TaskControle_attributes = {
  .name = "TaskControle",
  .stack_size = 1500 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TaskMicroAutowa */
osThreadId_t TaskMicroAutowaHandle;
const osThreadAttr_t TaskMicroAutowa_attributes = {
  .name = "TaskMicroAutowa",
  .stack_size = 4500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MutexControlSignal */
osMutexId_t MutexControlSignalHandle;
const osMutexAttr_t MutexControlSignal_attributes = {
  .name = "MutexControlSignal"
};
/* Definitions for MutexControlAction */
osMutexId_t MutexControlActionHandle;
const osMutexAttr_t MutexControlAction_attributes = {
  .name = "MutexControlAction"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */



/* USER CODE END FunctionPrototypes */

void StartTaskControle(void *argument);
extern void StartMicroAutoware(void *argument);

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
  /* creation of MutexControlSignal */
  MutexControlSignalHandle = osMutexNew(&MutexControlSignal_attributes);

  /* creation of MutexControlAction */
  MutexControlActionHandle = osMutexNew(&MutexControlAction_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of TaskControle */
  TaskControleHandle = osThreadNew(StartTaskControle, NULL, &TaskControle_attributes);

  /* creation of TaskMicroAutowa */
  TaskMicroAutowaHandle = osThreadNew(StartMicroAutoware, NULL, &TaskMicroAutowa_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTaskControle */
/**
  * @brief  __weak function for TaskControle, overrided by
  *         taskControle.h & taskControle.c.
  * @param  argument: Not used
  */
/* USER CODE END Header_StartTaskControle */
__weak void StartTaskControle(void *argument)
{
  /* USER CODE BEGIN StartTaskControle */

  /* USER CODE END StartTaskControle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* USER CODE END Application */

