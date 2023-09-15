/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ll_sys_if.c
  * @author  MCD Application Team
  * @brief   Source file for initiating system
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

#include "app_common.h"
#include "main.h"
#include "ll_intf.h"
#include "ll_sys.h"
#include "adc_ctrl.h"
#include "linklayer_plat.h"

/* FREERTOS_MARK */
#include "app_freertos.h"
/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Global variables ----------------------------------------------------------*/

/* USER CODE BEGIN GV */

/* USER CODE END GV */

#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION == 1)
void ll_sys_bg_temperature_measurement(void);
static void ll_sys_bg_temperature_measurement_init(void);
static void request_temperature_measurement(void);
/* FREERTOS_MARK */
static void request_temperature_measurement_Entry(void* thread_input);
#endif /* USE_TEMPERATURE_BASED_RADIO_CALIBRATION */

/* FREERTOS_MARK */
static void ll_sys_bg_process_Entry(void* thread_input);

/**
  * @brief  Link Layer background process initialization
  * @param  None
  * @retval None
  */
void ll_sys_bg_process_init(void)
{
  /* Tasks creation */
	  LINK_LAYER_Thread_SemHandle = osSemaphoreNew(1, 0, &LINK_LAYER_Thread_Sem_attributes);
	  LINK_LAYER_Thread_MutexHandle = osMutexNew(&LINK_LAYER_Thread_Mutex_attributes);
	  LINK_LAYER_ThreadHandle = osThreadNew(ll_sys_bg_process_Entry, NULL, &LINK_LAYER_Thread_attributes);
}

/**
  * @brief  Link Layer background process next iteration scheduling
  * @param  None
  * @retval None
  */
void ll_sys_schedule_bg_process(void)
{
	  /* FREERTOS_MARK */
	  osSemaphoreRelease(LINK_LAYER_Thread_SemHandle);
}

/**
  * @brief  Link Layer configuration phase before application startup.
  * @param  None
  * @retval None
  */
void ll_sys_config_params(void)
{
  /* Configure link layer behavior for low ISR use and next event scheduling method:
   * - SW low ISR is used.
   * - Next event is scheduled from ISR.
   */
  ll_intf_config_ll_ctx_params(USE_RADIO_LOW_ISR, NEXT_EVENT_SCHEDULING_FROM_ISR);
#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION == 1)
  /* Initialize link layer temperature measurement background task */
  ll_sys_bg_temperature_measurement_init();

  /* Link layer IP uses temperature based calibration instead of periodic one */
  ll_intf_set_temperature_sensor_state();
#endif /* USE_TEMPERATURE_BASED_RADIO_CALIBRATION */
}

#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION == 1)
/**
  * @brief  Link Layer temperature request background process initialization
  * @param  None
  * @retval None
  */
void ll_sys_bg_temperature_measurement_init(void)
{
  /* Tasks creation */
	  /* FREERTOS_MARK */
	  LINK_LAYER_TEMP_MEAS_Thread_SemHandle = osSemaphoreNew(1, 0, &LINK_LAYER_TEMP_MEAS_Thread_Sem_attributes);
	  LINK_LAYER_TEMP_MEAS_ThreadHandle = osThreadNew(request_temperature_measurement_Entry, NULL, &LINK_LAYER_TEMP_MEAS_Thread_attributes);
}

/**
  * @brief  Request backroud task processing for temperature measurement
  * @param  None
  * @retval None
  */
void ll_sys_bg_temperature_measurement(void)
{
	  /* FREERTOS_MARK */
	  osSemaphoreRelease(LINK_LAYER_TEMP_MEAS_Thread_SemHandle);
}

/**
  * @brief  Request temperature measurement
  * @param  None
  * @retval None
  */
void request_temperature_measurement(void)
{
  int16_t temperature_value = 0;

  /* Enter limited critical section : disable all the interrupts with priority higher than RCC one
   * Concerns link layer interrupts (high and SW low) or any other high priority user system interrupt
   */
  UTILS_ENTER_LIMITED_CRITICAL_SECTION(RCC_INTR_PRIO<<4);

  /* Request ADC IP activation */
  adc_ctrl_req(SYS_ADC_LL_EVT, ADC_ON);

  /* Get temperature from ADC dedicated channel */
  temperature_value = adc_ctrl_request_temperature();

  /* Request ADC IP deactivation */
  adc_ctrl_req(SYS_ADC_LL_EVT, ADC_OFF);

  /* Give the temperature information to the link layer */
  ll_intf_set_temperature_value(temperature_value);

  /* Exit limited critical section */
  UTILS_EXIT_LIMITED_CRITICAL_SECTION();
}

/* FREERTOS_MARK */
static void request_temperature_measurement_Entry(void* thread_input)
{
  (void)(thread_input);

  while(1)
  {
    osSemaphoreAcquire(LINK_LAYER_TEMP_MEAS_Thread_SemHandle, osWaitForever);
    osMutexAcquire(LINK_LAYER_Thread_MutexHandle, osWaitForever);
    request_temperature_measurement();
    osMutexRelease(LINK_LAYER_Thread_MutexHandle);
  }
}

#endif /* USE_TEMPERATURE_BASED_RADIO_CALIBRATION */

/* FREERTOS_MARK */
static void ll_sys_bg_process_Entry(void* thread_input)
{
  (void)(thread_input);

  while(1)
  {
    osSemaphoreAcquire(LINK_LAYER_Thread_SemHandle, osWaitForever);
    osMutexAcquire(LINK_LAYER_Thread_MutexHandle, osWaitForever);
    ll_sys_bg_process();
    osMutexRelease(LINK_LAYER_Thread_MutexHandle);
  }
}

/**
  * @brief  Enable RTOS context switch.
  * @param  None
  * @retval None
  */
void LINKLAYER_PLAT_EnableOSContextSwitch(void)
{
}

/**
  * @brief  Disable RTOS context switch.
  * @param  None
  * @retval None
  */
void LINKLAYER_PLAT_DisableOSContextSwitch(void)
{
}
