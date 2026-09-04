/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "stm32_seq.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  APP_IDLE,
  APP_RX,
  APP_RX_TIMEOUT,
  APP_RX_ERROR,
  APP_TX,
  APP_TX_TIMEOUT,
} AppStates_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SUBGHZ_PHY_PROCESS_TASK   (1 << 0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */
static AppStates_t State = APP_IDLE;
static uint8_t Buffer[BUFFER_SIZE];
static uint16_t BufferSize = 0;
static int16_t RssiValue = 0;
static int8_t  SnrValue  = 0;
extern UART_HandleTypeDef huart1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
static void SubghzApp_Process(void);

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */
  UTIL_SEQ_RegTask(SUBGHZ_PHY_PROCESS_TASK, UTIL_SEQ_RFU, SubghzApp_Process);
  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                     LORA_CRC_ENABLED, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     0, LORA_CRC_ENABLED, 0, 0, LORA_IQ_INVERSION_ON, true);

  {
    char msg[64];
    int len = snprintf(msg, sizeof(msg), "\r\n--- RX Init Complete: Ready & Listening ---\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);
  }

  /* RX role: start listening immediately, continuously (timeout = 0) */
  Radio.Rx(0);
  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
	 State = APP_TX;
	  UTIL_SEQ_SetTask(SUBGHZ_PHY_PROCESS_TASK, UTIL_SEQ_RFU);
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
	BufferSize = size > BUFFER_SIZE ? BUFFER_SIZE : size;
	  memcpy(Buffer, payload, BufferSize);
	  RssiValue = rssi;
	  SnrValue = LoraSnr_FskCfo;
	  State = APP_RX;
	  UTIL_SEQ_SetTask(SUBGHZ_PHY_PROCESS_TASK, UTIL_SEQ_RFU);

  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
	State = APP_TX_TIMEOUT;
	UTIL_SEQ_SetTask(SUBGHZ_PHY_PROCESS_TASK, UTIL_SEQ_RFU);
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
	State = APP_RX_TIMEOUT;
    UTIL_SEQ_SetTask(SUBGHZ_PHY_PROCESS_TASK, UTIL_SEQ_RFU);
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
	State = APP_RX_ERROR;
	UTIL_SEQ_SetTask(SUBGHZ_PHY_PROCESS_TASK, UTIL_SEQ_RFU);
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */
static void SubghzApp_Process(void)
{
  switch (State)
  {
    case APP_RX:
    {
      if (BufferSize > 0)
      {
        /* Ensure null-terminated string for safe parsing */
        Buffer[BufferSize < BUFFER_SIZE ? BufferSize : BUFFER_SIZE - 1] = '\0';

        char ackMsg[BUFFER_SIZE];
        if (strncmp((char *)Buffer, "PING:", 5) == 0)
        {
          snprintf(ackMsg, sizeof(ackMsg), "ACK:%s", (char *)Buffer + 5);
        }
        else if (strncmp((char *)Buffer, "HELLO", 5) == 0 || strncmp((char *)Buffer, "PING", 4) == 0)
        {
          snprintf(ackMsg, sizeof(ackMsg), "PONG");
        }
        else
        {
          snprintf(ackMsg, sizeof(ackMsg), "ACK");
        }

        char msg[96];
        int len = snprintf(msg, sizeof(msg), "[RX] Received: '%s' (RSSI=%d dBm, SNR=%d dB) -> Replying %s\r\n",
                           (char *)Buffer, RssiValue, SnrValue, ackMsg);
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);

        uint16_t ackLen = (uint16_t)strlen(ackMsg);
        memcpy(Buffer, ackMsg, ackLen);
        BufferSize = ackLen;
        Radio.Send(Buffer, BufferSize);
      }
      else
      {
        Radio.Rx(0);
      }
      break;
    }

    case APP_TX:
    {
      char msg[48];
      int len = snprintf(msg, sizeof(msg), "[RX] ACK sent. Resuming continuous listen...\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);
      Radio.Rx(0);
      break;
    }

    case APP_RX_TIMEOUT:
    case APP_RX_ERROR:
    case APP_TX_TIMEOUT:
    {
      char msg[48];
      int len = snprintf(msg, sizeof(msg), "[RX] Timeout/error, resuming listen...\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);
      Radio.Rx(0);
      break;
    }

    case APP_IDLE:
    default:
      break;
  }
  State = APP_IDLE;
}
/* USER CODE END PrFD */
