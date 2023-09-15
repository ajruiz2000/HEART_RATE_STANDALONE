/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.h
  * @author  MCD Application Team
  * @brief   FreeRTOS applicative header file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_FREERTOS_H
#define __APP_FREERTOS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os2.h"
#include "app_entry.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* Declaration of all FreeRTOS global resources used by the application */

/* BPKA_TASK related resources */
extern osThreadId_t BPKA_ThreadHandle;
extern const osThreadAttr_t BPKA_Thread_attributes;
extern osSemaphoreId_t  BPKA_Thread_SemHandle;
extern const osSemaphoreAttr_t BPKA_Thread_Sem_attributes;

/* HW_RNG_TASK related resources  */
extern osThreadId_t HW_RNG_ThreadHandle;
extern const osThreadAttr_t HW_RNG_Thread_attributes;
extern osSemaphoreId_t  HW_RNG_Thread_SemHandle;
extern const osSemaphoreAttr_t HW_RNG_Thread_Sem_attributes;

/* BLE_STACK_TASK related resources */
extern osThreadId_t BLE_HOST_ThreadHandle;
extern const osThreadAttr_t BLE_HOST_Thread_attributes;
extern osSemaphoreId_t BLE_HOST_Thread_SemHandle;
extern const osSemaphoreAttr_t BLE_HOST_Thread_Sem_attributes;

/* HCI_ASYNCH_EVT_TASK related resources */
extern osThreadId_t HCI_ASYNCH_EVT_ThreadHandle;
extern const osThreadAttr_t HCI_ASYNCH_EVT_Thread_attributes;
extern osSemaphoreId_t HCI_ASYNCH_EVT_Thread_SemHandle;
extern const osSemaphoreAttr_t HCI_ASYNCH_EVT_Thread_Sem_attributes;

/* LINK_LAYER_TASK related resources */
extern osThreadId_t LINK_LAYER_ThreadHandle;
extern const osThreadAttr_t LINK_LAYER_Thread_attributes;
extern osSemaphoreId_t LINK_LAYER_Thread_SemHandle;
extern const osSemaphoreAttr_t LINK_LAYER_Thread_Sem_attributes;
extern osMutexId_t LINK_LAYER_Thread_MutexHandle;
extern const osMutexAttr_t LINK_LAYER_Thread_Mutex_attributes;

/* AMM_BCKGND_TASK related resources */
extern osThreadId_t AMM_BCKGND_ThreadHandle;
extern const osThreadAttr_t AMM_BCKGND_Thread_attributes;
extern osSemaphoreId_t AMM_BCKGND_Thread_SemHandle;
extern const osSemaphoreAttr_t AMM_BCKGND_Thread_Sem_attributes;

/* FLASH_MANAGER_BCKGND_TASK related resources */
extern osThreadId_t FLASH_MANAGER_BCKGND_ThreadHandle;
extern const osThreadAttr_t FLASH_MANAGER_BCKGND_Thread_attributes;
extern osSemaphoreId_t FLASH_MANAGER_BCKGND_Thread_SemHandle;
extern const osSemaphoreAttr_t FLASH_MANAGER_BCKGND_Thread_Sem_attributes;

/* LINK_LAYER_TEMP_MEAS_TASK related resources */
extern osThreadId_t LINK_LAYER_TEMP_MEAS_ThreadHandle;
extern const osThreadAttr_t LINK_LAYER_TEMP_MEAS_Thread_attributes;
extern osSemaphoreId_t LINK_LAYER_TEMP_MEAS_Thread_SemHandle;
extern const osSemaphoreAttr_t LINK_LAYER_TEMP_MEAS_Thread_Sem_attributes;

/* ADV_LP_REQ_TASK related resources */
extern osThreadId_t ADV_LP_REQ_ThreadHandle;
extern const osThreadAttr_t ADV_LP_REQ_Thread_attributes;
extern osSemaphoreId_t ADV_LP_REQ_Thread_SemHandle;
extern const osSemaphoreAttr_t ADV_LP_REQ_Thread_Sem_attributes;

/* MEAS_REQ_TASK related resources */
extern osThreadId_t MEAS_REQ_ThreadHandle;
extern const osThreadAttr_t MEAS_REQ_Thread_attributes;
extern osSemaphoreId_t MEAS_REQ_Thread_SemHandle;
extern const osSemaphoreAttr_t MEAS_REQ_Thread_Sem_attributes;

/* gap_cmd_resp sync mechanism */
extern osSemaphoreId_t IDLEEVT_PROC_GAP_COMPLETE_SemHandle;
extern const osSemaphoreAttr_t IDLEEVT_PROC_GAP_COMPLETE_Sem_attributes;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_FREERTOS_H__ */
