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
#include "timer.h"
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
  APP_TX,
  APP_TX_TIMEOUT,
  APP_RX,
  APP_RX_TIMEOUT,
  APP_RX_ERROR,
} AppStates_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SUBGHZ_PHY_PROCESS_TASK   (1 << 0)
#define PING_TX_PERIOD_MS         1500U
#define PING_RETRY_PERIOD_MS      500U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */
typedef enum
{
  CONN_DISCONNECTED,
  CONN_CONNECTED
} ConnState_t;

static ConnState_t LinkState = CONN_DISCONNECTED;
static uint32_t PktSeqNum = 1;

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
static void SendPing(void);
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
                     0, LORA_CRC_ENABLED, 0, 0, LORA_IQ_INVERSION_ON, false);

  {
    char msg[64];
    int len = snprintf(msg, sizeof(msg), "\r\n--- TX Init Complete: Ready to Connect ---\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);
  }

  SendPing();
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
static void SendPing(void)
{
  snprintf((char *)Buffer, BUFFER_SIZE, "PING:%lu", (unsigned long)PktSeqNum);
  BufferSize = (uint16_t)strlen((char *)Buffer);
  Radio.Send(Buffer, BufferSize);
}

static void SubghzApp_Process(void)
{
  switch (State)
  {
    case APP_TX:
    {
      char msg[64];
      int len = snprintf(msg, sizeof(msg), "[TX] Sent Pkt #%lu, waiting for ACK...\r\n", (unsigned long)PktSeqNum);
      HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);
      Radio.Rx(RX_TIMEOUT_VALUE);
      break;
    }

    case APP_RX:
    {
      /* Ensure null-terminated string for clean logging */
      Buffer[BufferSize < BUFFER_SIZE ? BufferSize : BUFFER_SIZE - 1] = '\0';

      if (BufferSize > 0 && (strncmp((char *)Buffer, "ACK", 3) == 0 || strncmp((char *)Buffer, "PONG", 4) == 0))
      {
        if (LinkState == CONN_DISCONNECTED)
        {
          LinkState = CONN_CONNECTED;
          char connMsg[64];
          int cLen = snprintf(connMsg, sizeof(connMsg), "\r\n>>> [LINK CONNECTED] Handshake success with RX! <<<\r\n\r\n");
          HAL_UART_Transmit(&huart1, (uint8_t *)connMsg, cLen, 100);
        }

        char msg[96];
        int len = snprintf(msg, sizeof(msg), "[TX] Received %s | RSSI=%d dBm | SNR=%d dB\r\n", (char *)Buffer, RssiValue, SnrValue);
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);

        PktSeqNum++;
      }
      else
      {
        char msg[64];
        int len = snprintf(msg, sizeof(msg), "[TX] Received other packet (len=%d)\r\n", BufferSize);
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);
      }

      HAL_Delay(PING_TX_PERIOD_MS);
      SendPing();
      break;
    }

    case APP_RX_TIMEOUT:
    case APP_RX_ERROR:
    case APP_TX_TIMEOUT:
    {
      if (LinkState == CONN_CONNECTED)
      {
        LinkState = CONN_DISCONNECTED;
        char discMsg[64];
        int dLen = snprintf(discMsg, sizeof(discMsg), "\r\n>>> [LINK DISCONNECTED / RECONNECTING...] <<<\r\n\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)discMsg, dLen, 100);
      }

      char msg[80];
      int len = snprintf(msg, sizeof(msg), "[TX] No ACK for Pkt #%lu (timeout). Retrying...\r\n", (unsigned long)PktSeqNum);
      HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);

      HAL_Delay(PING_RETRY_PERIOD_MS);
      SendPing();
      break;
    }

    case APP_IDLE:
    default:
      break;
  }
  State = APP_IDLE;
}
/* USER CODE END PrFD */
