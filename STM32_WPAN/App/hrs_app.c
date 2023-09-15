/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service1_app.c
  * @author  MCD Application Team
  * @brief   service1_app application definition.
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
#include "app_common.h"
#include "app_ble.h"
#include "ll_sys_if.h"
#include "dbg_trace.h"
#include "ble.h"
#include "hrs_app.h"
#include "hrs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef enum
{
  Hrme_NOTIFICATION_OFF,
  Hrme_NOTIFICATION_ON,
  /* USER CODE BEGIN Service1_APP_SendInformation_t */

  /* USER CODE END Service1_APP_SendInformation_t */
  HRS_APP_SENDINFORMATION_LAST
} HRS_APP_SendInformation_t;

typedef struct
{
  HRS_APP_SendInformation_t     Hrme_Notification_Status;
  /* USER CODE BEGIN Service1_APP_Context_t */

  /* USER CODE END Service1_APP_Context_t */
  uint16_t              ConnectionHandle;
} HRS_APP_Context_t;

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static HRS_APP_Context_t HRS_APP_Context;

uint8_t a_HRS_UpdateCharData[247];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void HRS_Hrme_SendNotification(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void HRS_Notification(HRS_NotificationEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service1_Notification_1 */

  /* USER CODE END Service1_Notification_1 */
  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service1_Notification_Service1_EvtOpcode */

    /* USER CODE END Service1_Notification_Service1_EvtOpcode */

    case HRS_HRME_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service1Char1_NOTIFY_ENABLED_EVT */

      /* USER CODE END Service1Char1_NOTIFY_ENABLED_EVT */
      break;

    case HRS_HRME_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service1Char1_NOTIFY_DISABLED_EVT */

      /* USER CODE END Service1Char1_NOTIFY_DISABLED_EVT */
      break;

    case HRS_BSL_READ_EVT:
      /* USER CODE BEGIN Service1Char2_READ_EVT */

      /* USER CODE END Service1Char2_READ_EVT */
      break;

    case HRS_HRCP_WRITE_EVT:
      /* USER CODE BEGIN Service1Char3_WRITE_EVT */

      /* USER CODE END Service1Char3_WRITE_EVT */
      break;

    default:
      /* USER CODE BEGIN Service1_Notification_default */

      /* USER CODE END Service1_Notification_default */
      break;
  }
  /* USER CODE BEGIN Service1_Notification_2 */

  /* USER CODE END Service1_Notification_2 */
  return;
}

void HRS_APP_EvtRx(HRS_APP_ConnHandleNotEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service1_APP_EvtRx_1 */

  /* USER CODE END Service1_APP_EvtRx_1 */

  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service1_APP_EvtRx_Service1_EvtOpcode */

    /* USER CODE END Service1_APP_EvtRx_Service1_EvtOpcode */
    case HRS_CONN_HANDLE_EVT :
      /* USER CODE BEGIN Service1_APP_CONN_HANDLE_EVT */

      /* USER CODE END Service1_APP_CONN_HANDLE_EVT */
      break;

    case HRS_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN Service1_APP_DISCON_HANDLE_EVT */

      /* USER CODE END Service1_APP_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN Service1_APP_EvtRx_default */

      /* USER CODE END Service1_APP_EvtRx_default */
      break;
  }

  /* USER CODE BEGIN Service1_APP_EvtRx_2 */

  /* USER CODE END Service1_APP_EvtRx_2 */

  return;
}

void HRS_APP_Init(void)
{
  UNUSED(HRS_APP_Context);
  HRS_Init();

  /* USER CODE BEGIN Service1_APP_Init */

  /* USER CODE END Service1_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
__USED void HRS_Hrme_SendNotification(void) /* Property Notification */
{
  HRS_APP_SendInformation_t notification_on_off = Hrme_NOTIFICATION_OFF;
  HRS_Data_t hrs_notification_data;

  hrs_notification_data.p_Payload = (uint8_t*)a_HRS_UpdateCharData;
  hrs_notification_data.Length = 0;

  /* USER CODE BEGIN Service1Char1_NS_1*/

  /* USER CODE END Service1Char1_NS_1*/

  if (notification_on_off != Hrme_NOTIFICATION_OFF)
  {
    HRS_UpdateValue(HRS_HRME, &hrs_notification_data);
  }

  /* USER CODE BEGIN Service1Char1_NS_Last*/

  /* USER CODE END Service1Char1_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
