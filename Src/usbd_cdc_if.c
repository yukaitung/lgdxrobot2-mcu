/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include <string.h>
#include "motor.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t motor = 0, kp = 0, ki = 0, kd = 0;
uint32_t vx = 0, vy = 0, vw = 0;
uint32_t ax = 0, ay = 0, az= 0, gz = 0;
uint32_t enable = 0;
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
void Write_Uint32(uint32_t value, uint8_t *msb, uint8_t *byte2, uint8_t *byte3, uint8_t *lsb)
{
	*msb = (value & 4278190080) >> 24;
	*byte2 = (value & 16711680) >> 16;
	*byte3 = (value & 65280) >> 8;
	*lsb = value & 255;
}

float Uint32_To_Float(uint32_t n)
{
  return (float)(*(float*)&n);
}

uint32_t Combine_Byte(uint32_t a, uint32_t b, uint32_t c, uint32_t d) 
{
  return a << 24 | b << 16 | c << 8 | d;
}

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
		case 'E':
			/*
			 * Software E-Stop, expect 1 int variables, the length is 5 bytes
			 */
			if (*Len != 5)
				break;
			enable = Combine_Byte((uint8_t) Buf[1], (uint8_t) Buf[2], (uint8_t) Buf[3], (uint8_t) Buf[4]);
			MOTOR_Set_Software_E_Stop(enable != 0);
			break;
		case 'I':
			/*
			 * External IMU Data, expect 4 int variables, the length is 17 bytes
			 */
			if (*Len != 17)
				break;
			ax = Combine_Byte((uint8_t) Buf[1], (uint8_t) Buf[2], (uint8_t) Buf[3], (uint8_t) Buf[4]);
			ay = Combine_Byte((uint8_t) Buf[5], (uint8_t) Buf[6], (uint8_t) Buf[7], (uint8_t) Buf[8]);
			az = Combine_Byte((uint8_t) Buf[9], (uint8_t) Buf[10], (uint8_t) Buf[11], (uint8_t) Buf[12]);
			gz = Combine_Byte((uint8_t) Buf[13], (uint8_t) Buf[14], (uint8_t) Buf[15], (uint8_t) Buf[16]);
			MOTOR_Set_External_IMU(Uint32_To_Float(ax), Uint32_To_Float(ay), Uint32_To_Float(az), Uint32_To_Float(gz));
			break;
		case 'M':
			/*
			 * Motor Inverse Kinematics, expect 3 float variables, the length is 13 bytes
			 */
			if (*Len != 13)
				break;
			vx = Combine_Byte((uint8_t) Buf[1], (uint8_t) Buf[2], (uint8_t) Buf[3], (uint8_t) Buf[4]);
			vy = Combine_Byte((uint8_t) Buf[5], (uint8_t) Buf[6], (uint8_t) Buf[7], (uint8_t) Buf[8]);
			vw = Combine_Byte((uint8_t) Buf[9], (uint8_t) Buf[10], (uint8_t) Buf[11], (uint8_t) Buf[12]);
			MOTOR_Set_Ik(Uint32_To_Float(vx), Uint32_To_Float(vy), Uint32_To_Float(vw));
			break;
		case 'P':
			/*
			 * Motor PID, expect 4 int variables, the length is 17 bytes
			 */
			if (*Len != 17)
				break;
			motor = Combine_Byte((uint8_t) Buf[1], (uint8_t) Buf[2], (uint8_t) Buf[3], (uint8_t) Buf[4]);
			if(motor > 4)
				break;
			kp = Combine_Byte((uint8_t) Buf[5], (uint8_t) Buf[6], (uint8_t) Buf[7], (uint8_t) Buf[8]);
			ki = Combine_Byte((uint8_t) Buf[9], (uint8_t) Buf[10], (uint8_t) Buf[11], (uint8_t) Buf[12]);
			kd = Combine_Byte((uint8_t) Buf[13], (uint8_t) Buf[14], (uint8_t) Buf[15], (uint8_t) Buf[16]);
			MOTOR_Set_PID(motor, Uint32_To_Float(kp), Uint32_To_Float(ki), Uint32_To_Float(kd));
			break;
    case 'S':
      /*
       * Serial Number
       */
      if (*Len != 1)
        break;
      uint32_t stm32Uid[3];
      stm32Uid[0] = HAL_GetUIDw0();
      stm32Uid[1] = HAL_GetUIDw1();
      stm32Uid[2] = HAL_GetUIDw2();
      static uint8_t msg[97] = {'\0'};
      msg[0] = 0xAB;
      int index = 1;
      for(int i = 0; i < 3; i++)
      {
        Write_Uint32(stm32Uid[i], &msg[index], &msg[index + 1], &msg[index + 2], &msg[index + 3]);
        index += 4;
      }
      index++;
      CDC_Transmit_FS(msg, index);
      break;
		case 'T':
			/*
			 * Reset transform, the length is 1 byte
			 */
			if (*Len != 1)
				break;
			MOTOR_Reset_Transform();
			break;
		case 'V':
			/*
			 * Single Motor Velocity, expect 2 int variables, the length is 9 bytes
			 */
			if (*Len != 9)
				break;
			motor = Combine_Byte((uint8_t) Buf[1], (uint8_t) Buf[2], (uint8_t) Buf[3], (uint8_t) Buf[4]);
			if(motor > 4)
				break;
			vx = Combine_Byte((uint8_t) Buf[5], (uint8_t) Buf[6], (uint8_t) Buf[7], (uint8_t) Buf[8]);
			MOTOR_Set_Single_Velocity(motor, Uint32_To_Float(vx));
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
