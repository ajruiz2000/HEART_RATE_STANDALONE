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
/* Definitions for PB1_BUTTON_PUSHED_Thread */
osThreadId_t PB1_BUTTON_PUSHED_ThreadHandle;
const osThreadAttr_t PB1_BUTTON_PUSHED_Thread_attributes = {
  .name = "PB1_BUTTON_PUSHED_Thread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for PB2_BUTTON_PUSHED_Thread */
osThreadId_t PB2_BUTTON_PUSHED_ThreadHandle;
const osThreadAttr_t PB2_BUTTON_PUSHED_Thread_attributes = {
  .name = "PB2_BUTTON_PUSHED_Thread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for PB3_BUTTON_PUSHED_Thread */
osThreadId_t PB3_BUTTON_PUSHED_ThreadHandle;
const osThreadAttr_t PB3_BUTTON_PUSHED_Thread_attributes = {
  .name = "PB3_BUTTON_PUSHED_Thread",
  .priority = (osPriority_t) osPriorityNormal,
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
/* Definitions for PB1_BUTTON_PUSHED_Thread_Sem */
osSemaphoreId_t PB1_BUTTON_PUSHED_Thread_SemHandle;
const osSemaphoreAttr_t PB1_BUTTON_PUSHED_Thread_Sem_attributes = {
  .name = "PB1_BUTTON_PUSHED_Thread_Sem"
};
/* Definitions for PB2_BUTTON_PUSHED_Thread_Sem */
osSemaphoreId_t PB2_BUTTON_PUSHED_Thread_SemHandle;
const osSemaphoreAttr_t PB2_BUTTON_PUSHED_Thread_Sem_attributes = {
  .name = "PB2_BUTTON_PUSHED_Thread_Sem"
};
/* Definitions for PB3_BUTTON_PUSHED_Thread_Sem */
osSemaphoreId_t PB3_BUTTON_PUSHED_Thread_SemHandle;
const osSemaphoreAttr_t PB3_BUTTON_PUSHED_Thread_Sem_attributes = {
  .name = "PB3_BUTTON_PUSHED_Thread_Sem"
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void BPKA_BG_Process_Entry(void *argument);
void HW_RNG_Process_Entry(void *argument);
void BleStack_Process_BG_Entry(void *argument);
void Ble_UserEvtRx_Entry(void *argument);
void ll_sys_bg_process_Entry(void *argument);
void AMM_BackgroundProcess_Entry(void *argument);
void FM_BackgroundProcess_Entry(void *argument);
void APP_BLE_Key_Button1_Action_Entry(void *argument);
void APP_BLE_Key_Button2_Action_Entry(void *argument);
void APP_BLE_Key_Button3_Action_Entry(void *argument);
void request_temperature_measurement_Entry(void *argument);
void APP_BLE_AdvLowPower_Entry(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* creation of LINK_LAYER_Thread_Mutex */
  LINK_LAYER_Thread_MutexHandle = osMutexNew(&LINK_LAYER_Thread_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  /* creation of BPKA_Thread_Sem */
  BPKA_Thread_SemHandle = osSemaphoreNew(1, 1, &BPKA_Thread_Sem_attributes);

  /* creation of HW_RNG_Thread_Sem */
  HW_RNG_Thread_SemHandle = osSemaphoreNew(1, 1, &HW_RNG_Thread_Sem_attributes);

  /* creation of BLE_HOST_Thread_Sem */
  BLE_HOST_Thread_SemHandle = osSemaphoreNew(1, 1, &BLE_HOST_Thread_Sem_attributes);

  /* creation of HCI_ASYNCH_EVT_Thread_Sem */
  HCI_ASYNCH_EVT_Thread_SemHandle = osSemaphoreNew(1, 1, &HCI_ASYNCH_EVT_Thread_Sem_attributes);

  /* creation of LINK_LAYER_Thread_Sem */
  LINK_LAYER_Thread_SemHandle = osSemaphoreNew(1, 1, &LINK_LAYER_Thread_Sem_attributes);

  /* creation of AMM_BCKGND_Thread_Sem */
  AMM_BCKGND_Thread_SemHandle = osSemaphoreNew(1, 1, &AMM_BCKGND_Thread_Sem_attributes);

  /* creation of FLASH_MANAGER_BCKGND_Thread_Sem */
  FLASH_MANAGER_BCKGND_Thread_SemHandle = osSemaphoreNew(1, 1, &FLASH_MANAGER_BCKGND_Thread_Sem_attributes);

  /* creation of PB1_BUTTON_PUSHED_Thread_Sem */
  PB1_BUTTON_PUSHED_Thread_SemHandle = osSemaphoreNew(1, 1, &PB1_BUTTON_PUSHED_Thread_Sem_attributes);

  /* creation of PB2_BUTTON_PUSHED_Thread_Sem */
  PB2_BUTTON_PUSHED_Thread_SemHandle = osSemaphoreNew(1, 1, &PB2_BUTTON_PUSHED_Thread_Sem_attributes);

  /* creation of PB3_BUTTON_PUSHED_Thread_Sem */
  PB3_BUTTON_PUSHED_Thread_SemHandle = osSemaphoreNew(1, 1, &PB3_BUTTON_PUSHED_Thread_Sem_attributes);

  /* creation of LINK_LAYER_TEMP_MEAS_Thread_Sem */
  LINK_LAYER_TEMP_MEAS_Thread_SemHandle = osSemaphoreNew(1, 1, &LINK_LAYER_TEMP_MEAS_Thread_Sem_attributes);

  /* creation of ADV_LP_REQ_Thread_Sem */
  ADV_LP_REQ_Thread_SemHandle = osSemaphoreNew(1, 1, &ADV_LP_REQ_Thread_Sem_attributes);

  /* creation of IDLEEVT_PROC_GAP_COMPLETE_Sem */
  IDLEEVT_PROC_GAP_COMPLETE_SemHandle = osSemaphoreNew(1, 1, &IDLEEVT_PROC_GAP_COMPLETE_Sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of BPKA_Thread */
  BPKA_ThreadHandle = osThreadNew(BPKA_BG_Process_Entry, thread_input, &BPKA_Thread_attributes);

  /* creation of HW_RNG_Thread */
  HW_RNG_ThreadHandle = osThreadNew(HW_RNG_Process_Entry, thread_input, &HW_RNG_Thread_attributes);

  /* creation of BLE_HOST_Thread */
  BLE_HOST_ThreadHandle = osThreadNew(BleStack_Process_BG_Entry, thread_input, &BLE_HOST_Thread_attributes);

  /* creation of HCI_ASYNCH_EVT_Thread */
  HCI_ASYNCH_EVT_ThreadHandle = osThreadNew(Ble_UserEvtRx_Entry, thread_input, &HCI_ASYNCH_EVT_Thread_attributes);

  /* creation of LINK_LAYER_Thread */
  LINK_LAYER_ThreadHandle = osThreadNew(ll_sys_bg_process_Entry, thread_input, &LINK_LAYER_Thread_attributes);

  /* creation of AMM_BCKGND_Thread */
  AMM_BCKGND_ThreadHandle = osThreadNew(AMM_BackgroundProcess_Entry, thread_input, &AMM_BCKGND_Thread_attributes);

  /* creation of FLASH_MANAGER_BCKGND_Thread */
  FLASH_MANAGER_BCKGND_ThreadHandle = osThreadNew(FM_BackgroundProcess_Entry, thread_input, &FLASH_MANAGER_BCKGND_Thread_attributes);

  /* creation of PB1_BUTTON_PUSHED_Thread */
  PB1_BUTTON_PUSHED_ThreadHandle = osThreadNew(APP_BLE_Key_Button1_Action_Entry, thread_input, &PB1_BUTTON_PUSHED_Thread_attributes);

  /* creation of PB2_BUTTON_PUSHED_Thread */
  PB2_BUTTON_PUSHED_ThreadHandle = osThreadNew(APP_BLE_Key_Button2_Action_Entry, thread_input, &PB2_BUTTON_PUSHED_Thread_attributes);

  /* creation of PB3_BUTTON_PUSHED_Thread */
  PB3_BUTTON_PUSHED_ThreadHandle = osThreadNew(APP_BLE_Key_Button3_Action_Entry, thread_input, &PB3_BUTTON_PUSHED_Thread_attributes);

  /* creation of LINK_LAYER_TEMP_MEAS_Thread */
  LINK_LAYER_TEMP_MEAS_ThreadHandle = osThreadNew(request_temperature_measurement_Entry, thread_input, &LINK_LAYER_TEMP_MEAS_Thread_attributes);

  /* creation of ADV_LP_REQ_Thread */
  ADV_LP_REQ_ThreadHandle = osThreadNew(APP_BLE_AdvLowPower_Entry, thread_input, &ADV_LP_REQ_Thread_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_BPKA_BG_Process_Entry */
/**
* @brief Function implementing the BPKA_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BPKA_BG_Process_Entry */
void BPKA_BG_Process_Entry(void *argument)
{
  /* USER CODE BEGIN BPKA_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BPKA_Thread */
}

/* USER CODE BEGIN Header_HW_RNG_Process_Entry */
/**
* @brief Function implementing the HW_RNG_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HW_RNG_Process_Entry */
void HW_RNG_Process_Entry(void *argument)
{
  /* USER CODE BEGIN HW_RNG_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HW_RNG_Thread */
}

/* USER CODE BEGIN Header_BleStack_Process_BG_Entry */
/**
* @brief Function implementing the BLE_HOST_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BleStack_Process_BG_Entry */
void BleStack_Process_BG_Entry(void *argument)
{
  /* USER CODE BEGIN BLE_HOST_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BLE_HOST_Thread */
}

/* USER CODE BEGIN Header_Ble_UserEvtRx_Entry */
/**
* @brief Function implementing the HCI_ASYNCH_EVT_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ble_UserEvtRx_Entry */
void Ble_UserEvtRx_Entry(void *argument)
{
  /* USER CODE BEGIN HCI_ASYNCH_EVT_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HCI_ASYNCH_EVT_Thread */
}

/* USER CODE BEGIN Header_ll_sys_bg_process_Entry */
/**
* @brief Function implementing the LINK_LAYER_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ll_sys_bg_process_Entry */
void ll_sys_bg_process_Entry(void *argument)
{
  /* USER CODE BEGIN LINK_LAYER_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LINK_LAYER_Thread */
}

/* USER CODE BEGIN Header_AMM_BackgroundProcess_Entry */
/**
* @brief Function implementing the AMM_BCKGND_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AMM_BackgroundProcess_Entry */
void AMM_BackgroundProcess_Entry(void *argument)
{
  /* USER CODE BEGIN AMM_BCKGND_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AMM_BCKGND_Thread */
}

/* USER CODE BEGIN Header_FM_BackgroundProcess_Entry */
/**
* @brief Function implementing the FLASH_MANAGER_BCKGND_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FM_BackgroundProcess_Entry */
void FM_BackgroundProcess_Entry(void *argument)
{
  /* USER CODE BEGIN FLASH_MANAGER_BCKGND_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FLASH_MANAGER_BCKGND_Thread */
}

/* USER CODE BEGIN Header_APP_BLE_Key_Button1_Action_Entry */
/**
* @brief Function implementing the PB1_BUTTON_PUSHED_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_APP_BLE_Key_Button1_Action_Entry */
void APP_BLE_Key_Button1_Action_Entry(void *argument)
{
  /* USER CODE BEGIN PB1_BUTTON_PUSHED_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PB1_BUTTON_PUSHED_Thread */
}

/* USER CODE BEGIN Header_APP_BLE_Key_Button2_Action_Entry */
/**
* @brief Function implementing the PB2_BUTTON_PUSHED_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_APP_BLE_Key_Button2_Action_Entry */
void APP_BLE_Key_Button2_Action_Entry(void *argument)
{
  /* USER CODE BEGIN PB2_BUTTON_PUSHED_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PB2_BUTTON_PUSHED_Thread */
}

/* USER CODE BEGIN Header_APP_BLE_Key_Button3_Action_Entry */
/**
* @brief Function implementing the PB3_BUTTON_PUSHED_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_APP_BLE_Key_Button3_Action_Entry */
void APP_BLE_Key_Button3_Action_Entry(void *argument)
{
  /* USER CODE BEGIN PB3_BUTTON_PUSHED_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PB3_BUTTON_PUSHED_Thread */
}

/* USER CODE BEGIN Header_request_temperature_measurement_Entry */
/**
* @brief Function implementing the LINK_LAYER_TEMP_MEAS_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_request_temperature_measurement_Entry */
void request_temperature_measurement_Entry(void *argument)
{
  /* USER CODE BEGIN LINK_LAYER_TEMP_MEAS_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LINK_LAYER_TEMP_MEAS_Thread */
}

/* USER CODE BEGIN Header_APP_BLE_AdvLowPower_Entry */
/**
* @brief Function implementing the ADV_LP_REQ_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_APP_BLE_AdvLowPower_Entry */
void APP_BLE_AdvLowPower_Entry(void *argument)
{
  /* USER CODE BEGIN ADV_LP_REQ_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ADV_LP_REQ_Thread */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

