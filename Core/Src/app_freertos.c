/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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
/* Definitions for BPKA_Thread */
osThreadId_t BPKA_ThreadHandle;
const osThreadAttr_t BPKA_Thread_attributes = {
  .name = "BPKA_Thread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for HW_RNG_Thread */
osThreadId_t HW_RNG_ThreadHandle;
const osThreadAttr_t HW_RNG_Thread_attributes = {
  .name = "HW_RNG_Thread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for BLE_HOST_Thread */
osThreadId_t BLE_HOST_ThreadHandle;
const osThreadAttr_t BLE_HOST_Thread_attributes = {
  .name = "BLE_HOST_Thread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for HCI_ASYNCH_EVT_Thread */
osThreadId_t HCI_ASYNCH_EVT_ThreadHandle;
const osThreadAttr_t HCI_ASYNCH_EVT_Thread_attributes = {
  .name = "HCI_ASYNCH_EVT_Thread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for LINK_LAYER_Thread */
osThreadId_t LINK_LAYER_ThreadHandle;
const osThreadAttr_t LINK_LAYER_Thread_attributes = {
  .name = "LINK_LAYER_Thread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for AMM_BCKGND_Thread */
osThreadId_t AMM_BCKGND_ThreadHandle;
const osThreadAttr_t AMM_BCKGND_Thread_attributes = {
  .name = "AMM_BCKGND_Thread",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 4
};
/* Definitions for FLASH_MANAGER_BCKGND_Thread */
osThreadId_t FLASH_MANAGER_BCKGND_ThreadHandle;
const osThreadAttr_t FLASH_MANAGER_BCKGND_Thread_attributes = {
  .name = "FLASH_MANAGER_BCKGND_Thread",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 4
};
/* Definitions for LINK_LAYER_TEMP_MEAS_Thread */
osThreadId_t LINK_LAYER_TEMP_MEAS_ThreadHandle;
const osThreadAttr_t LINK_LAYER_TEMP_MEAS_Thread_attributes = {
  .name = "LINK_LAYER_TEMP_MEAS_Thread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for ADV_LP_REQ_Thread */
osThreadId_t ADV_LP_REQ_ThreadHandle;
const osThreadAttr_t ADV_LP_REQ_Thread_attributes = {
  .name = "ADV_LP_REQ_Thread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for MEAS_REQ_Thread */
osThreadId_t MEAS_REQ_ThreadHandle;
const osThreadAttr_t MEAS_REQ_Thread_attributes = {
  .name = "MEAS_REQ_Thread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for LINK_LAYER_Thread_Mutex */
osMutexId_t LINK_LAYER_Thread_MutexHandle;
const osMutexAttr_t LINK_LAYER_Thread_Mutex_attributes = {
  .name = "LINK_LAYER_Thread_Mutex"
};
/* Definitions for BPKA_Thread_Sem */
osSemaphoreId_t BPKA_Thread_SemHandle;
const osSemaphoreAttr_t BPKA_Thread_Sem_attributes = {
  .name = "BPKA_Thread_Sem"
};
/* Definitions for HW_RNG_Thread_Sem */
osSemaphoreId_t HW_RNG_Thread_SemHandle;
const osSemaphoreAttr_t HW_RNG_Thread_Sem_attributes = {
  .name = "HW_RNG_Thread_Sem"
};
/* Definitions for BLE_HOST_Thread_Sem */
osSemaphoreId_t BLE_HOST_Thread_SemHandle;
const osSemaphoreAttr_t BLE_HOST_Thread_Sem_attributes = {
  .name = "BLE_HOST_Thread_Sem"
};
/* Definitions for HCI_ASYNCH_EVT_Thread_Sem */
osSemaphoreId_t HCI_ASYNCH_EVT_Thread_SemHandle;
const osSemaphoreAttr_t HCI_ASYNCH_EVT_Thread_Sem_attributes = {
  .name = "HCI_ASYNCH_EVT_Thread_Sem"
};
/* Definitions for LINK_LAYER_Thread_Sem */
osSemaphoreId_t LINK_LAYER_Thread_SemHandle;
const osSemaphoreAttr_t LINK_LAYER_Thread_Sem_attributes = {
  .name = "LINK_LAYER_Thread_Sem"
};
/* Definitions for AMM_BCKGND_Thread_Sem */
osSemaphoreId_t AMM_BCKGND_Thread_SemHandle;
const osSemaphoreAttr_t AMM_BCKGND_Thread_Sem_attributes = {
  .name = "AMM_BCKGND_Thread_Sem"
};
/* Definitions for FLASH_MANAGER_BCKGND_Thread_Sem */
osSemaphoreId_t FLASH_MANAGER_BCKGND_Thread_SemHandle;
const osSemaphoreAttr_t FLASH_MANAGER_BCKGND_Thread_Sem_attributes = {
  .name = "FLASH_MANAGER_BCKGND_Thread_Sem"
};
/* Definitions for LINK_LAYER_TEMP_MEAS_Thread_Sem */
osSemaphoreId_t LINK_LAYER_TEMP_MEAS_Thread_SemHandle;
const osSemaphoreAttr_t LINK_LAYER_TEMP_MEAS_Thread_Sem_attributes = {
  .name = "LINK_LAYER_TEMP_MEAS_Thread_Sem"
};
/* Definitions for ADV_LP_REQ_Thread_Sem */
osSemaphoreId_t ADV_LP_REQ_Thread_SemHandle;
const osSemaphoreAttr_t ADV_LP_REQ_Thread_Sem_attributes = {
  .name = "ADV_LP_REQ_Thread_Sem"
};
/* Definitions for IDLEEVT_PROC_GAP_COMPLETE_Sem */
osSemaphoreId_t IDLEEVT_PROC_GAP_COMPLETE_SemHandle;
const osSemaphoreAttr_t IDLEEVT_PROC_GAP_COMPLETE_Sem_attributes = {
  .name = "IDLEEVT_PROC_GAP_COMPLETE_Sem"
};
/* Definitions for MEAS_REQ_Sem */
osSemaphoreId_t MEAS_REQ_Thread_SemHandle;
const osSemaphoreAttr_t MEAS_REQ_Thread_Sem_attributes = {
  .name = "MEAS_REQ_Thread_Sem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {



}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

