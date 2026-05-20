/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device CDC ACM applicative source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Includes ------------------------------------------------------------------*/
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "om.h"
#include "crc.hpp"
#include "magicmsgs.hpp"
#include "config_comm.hpp"
#include "config_gimbal.hpp"
#include "tx_api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
TX_SEMAPHORE VisionRxSem;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
UX_SLAVE_CLASS_CDC_ACM  *cdc_acm;
UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER CDC_VCP_LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint32_t UserRxBufPtrIn;
uint32_t UserRxBufPtrOut;

uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
uint32_t UserTxBufPtrIn;
uint32_t UserTxBufPtrOut;

volatile UINT USB_TX_BUSY;
volatile UINT USB_TX_SUCCESS;
volatile UINT USB_RX_SUCCESS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */
  cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;

  /* Set device class_cdc_acm with default parameters */
  if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                    &CDC_VCP_LineCoding) != UX_SUCCESS)
  {
    Error_Handler();
  }
  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  cdc_acm = UX_NULL;
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  ULONG request;
  UX_SLAVE_TRANSFER *transfer_request;
  UX_SLAVE_DEVICE *device;

  /* Get the pointer to the device */
  device = &_ux_system_slave -> ux_system_slave_device;

  /* Get the pointer to the transfer request associated with the control endpoint */
  transfer_request = &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

  request = *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

  switch (request)
  {
    case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING :

      /* Get the Line Coding parameters */
      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING,
                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
      {
        Error_Handler();
      }

      /* Check if baudrate < 9600) then set it to 9600 */
      if (CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate < 9600)
      {
        CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate = 9600;
      }
      break;

    case UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING :

      /* Set the Line Coding parameters */
      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
      {
        Error_Handler();
      }

      break;

    case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE :
    default :
      break;
  }
  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 2 */
uint32_t new_data_ = 0;

struct msg_visionrx_t msg_visionrx;
struct msg_visionrx_t debug_visionrx;
/**
  * @brief  Function implementing USBX_DEVICE_CDC_ACM_Read_TASK.
  * @param  thread_input: Not used.
  * @retval none
  */
VOID usbx_cdc_acm_read_thread_entry(ULONG thread_input)
{
  ULONG actual_length;
  UX_SLAVE_DEVICE *device = &_ux_system_slave->ux_system_slave_device;

  UX_PARAMETER_NOT_USED(thread_input);

  om_topic_t *visionrx_topic = om_config_topic(NULL, "ca", "visionrx", sizeof(msg_visionrx));
  while (1)
  {
    if ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL))
    {
      // cdc_acm -> ux_slave_class_cdc_acm_transmission_status = UX_FALSE;
      ux_device_class_cdc_acm_read(cdc_acm,
                                           (UCHAR *)UserRxBufferFS,
                                           64, &actual_length);

      if (actual_length >= sizeof(msg_visionrx))
      {
        tx_semaphore_put(&VisionRxSem);
        tx_semaphore_put(&VisionRxSem);
        memcpy(&msg_visionrx, (UCHAR *)UserRxBufferFS, sizeof(msg_visionrx));
      }
      memcpy(&debug_visionrx, (UCHAR *)UserRxBufferFS, sizeof(msg_visionrx));
    }
    om_publish(visionrx_topic, &msg_visionrx, sizeof(msg_visionrx), true, false);
    msg_visionrx.header = 0;
    msg_visionrx.tracking = 0;
    tx_thread_sleep(2);
  }
}


struct msg_visiontx_t msg_visiontx;
struct msg_visiontx_t debug_visiontx;

/**
  * @brief  Function implementing usbx_cdc_acm_write_thread_entry.
  * @param  thread_input: Not used
  * @retval none
  */
VOID usbx_cdc_acm_write_thread_entry(ULONG thread_input) 
{
  ULONG actual_length;
  UX_SLAVE_DEVICE *device = &_ux_system_slave->ux_system_slave_device;

  om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
  om_suber_t *comm_suber = om_subscribe(om_find_topic("comm", UINT32_MAX));
  om_suber_t *cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
  struct comm_chassis_t comm;
  struct msg_ins_t ins;
  struct msg_cmd_t cmd;
  UX_PARAMETER_NOT_USED(thread_input);
  tx_thread_sleep(10);
  while (1)
  {
    om_suber_export(ins_suber, &ins, false);
    om_suber_export(comm_suber, &comm, false);
    om_suber_export(cmd_suber, &cmd, false);
    if ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL))
    {
      msg_visiontx.header = 0x5A;
      msg_visiontx.detect_color = 1-comm.color;
      msg_visiontx.reset_tracker = false;
      if (cmd.aim_rune)
        msg_visiontx.set_target = 0x08;
      else
        msg_visiontx.set_target = 0x00;
      msg_visiontx.q1 = ins.quaternion[0];
      msg_visiontx.q2 = ins.quaternion[1];
      msg_visiontx.q3 = ins.quaternion[2];
      msg_visiontx.q4 = ins.quaternion[3];
      msg_visiontx.gyro_yaw = ins.gyro_y;
      msg_visiontx.gyro_pitch = ins.gyro_p;
      Append_CRC16_Check_Sum((uint8_t *)&msg_visiontx, sizeof(msg_visiontx));      
      memcpy(&debug_visiontx, &msg_visiontx, sizeof(msg_visiontx));
      ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)&msg_visiontx , sizeof(msg_visiontx) , &actual_length);
    }
    tx_thread_sleep(2);
  }
}
/* USER CODE END 2 */
