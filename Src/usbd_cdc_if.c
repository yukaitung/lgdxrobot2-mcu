/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "lgdxrobot2.h"
#include "motor.h"
#include "stm32f4xx_hal.h"
#include "usbd_desc.h"

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  switch(Buf[0])
	{
		case MCU_SOFTWARE_EMERGENCY_STOP_COMMAND_TYPE:
      McuSoftwareEmergencyStopCommand cmd_e = {0};
      memcpy(&cmd_e, Buf, sizeof(McuSoftwareEmergencyStopCommand));
      MOTOR_Set_Emergency_Stop(software_emergency_stop, cmd_e.enable);
      break;
    case MCU_INVERSE_KINEMATICS_COMMAND_TYPE:
      McuInverseKinematicsCommand cmd_i = {0};
      memcpy(&cmd_i, Buf, sizeof(McuInverseKinematicsCommand));
      MOTOR_Set_Ik(cmd_i.velocity.x, cmd_i.velocity.y, cmd_i.velocity.rotation);
      break;
    case MCU_MOTOR_COMMAND_TYPE:
      McuMotorCommand cmd_m = {0};
      memcpy(&cmd_m, Buf, sizeof(McuMotorCommand));
      MOTOR_Set_Single_Motor(cmd_m.motor, cmd_m.velocity);
      break;
    case MCU_SET_PID_SPEED_COMMAND_TYPE:
      McuSetPidSpeedCommand cmd_l = {0};
      memcpy(&cmd_l, Buf, sizeof(McuSetPidSpeedCommand));
      MOTOR_Set_Temporary_Pid_Speed(cmd_l.pid_speed[0], cmd_l.pid_speed[1], cmd_l.pid_speed[2]);
      break;
    case MCU_GET_PID_COMMAND_TYPE:
      McuPid pid = {0};
      pid.header1 = MCU_HEADER1;
      pid.header2 = MCU_HEADER2;
      pid.header3 = MCU_HEADER3;
      pid.header4 = MCU_HEADER4;
      pid.type = MCU_PID_TYPE;
      for(int level = 0; level < PID_LEVEL; level++)
      {
        for(int motor = 0; motor < API_MOTOR_COUNT; motor++)
        {
          pid.pid_speed[level] = MOTOR_Get_Pid_Speed(level);
          pid.p[level][motor] = MOTOR_Get_Pid(motor, level, 0);
          pid.i[level][motor] = MOTOR_Get_Pid(motor, level, 1);
          pid.d[level][motor] = MOTOR_Get_Pid(motor, level, 2);
        }
      }
      for (int motor = 0; motor < API_MOTOR_COUNT; motor++)
      {
        pid.motors_maximum_speed[motor] = MOTOR_Get_Maximum_Speed(motor);
      }
      CDC_Transmit_FS((uint8_t*) &pid, sizeof(McuPid));
      break;
    case MCU_SET_PID_COMMAND_TYPE:
      McuSetPidCommand cmd_q = {0};
      memcpy(&cmd_q, Buf, sizeof(McuSetPidCommand));
      MOTOR_Set_Temporary_Pid(cmd_q.motor, cmd_q.level, cmd_q.p, cmd_q.i, cmd_q.d);
      break;
    case MCU_SAVE_PID_COMMAND_TYPE:
      MOTOR_Save_Pid();
      break;
    case MCU_SET_MOTOR_MAXIMUM_SPEED_COMMAND_TYPE:
      McuSetMotorMaximumSpeedCommand cmd_u = {0};
      memcpy(&cmd_u, Buf, sizeof(McuSetMotorMaximumSpeedCommand));
      MOTOR_Set_Temporary_Maximum_Speed(cmd_u.speed[0], cmd_u.speed[1], cmd_u.speed[2], cmd_u.speed[3]);
      MOTOR_Set_Ik(0, 0, 0);
      break;
    case MCU_GET_SERIAL_NUMBER_COMMAND_TYPE:
      McuSerialNumber serial_number = {0};
      serial_number.header1 = MCU_HEADER1;
      serial_number.header2 = MCU_HEADER2;
      serial_number.header3 = MCU_HEADER3;
      serial_number.header4 = MCU_HEADER4;
      serial_number.type = MCU_SERIAL_NUMBER_TYPE;
      serial_number.serial_number1 = HAL_GetUIDw0() + HAL_GetUIDw2();
      serial_number.serial_number2 = HAL_GetUIDw1();
      serial_number.serial_number3 = HAL_GetUIDw2();
      CDC_Transmit_FS((uint8_t*) &serial_number, sizeof(McuSerialNumber));
      break;
    case MCU_RESET_TRANSFORM_COMMAND_TYPE:
      MOTOR_Reset_Transform();
      break;
    default:
      break;
  }
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
