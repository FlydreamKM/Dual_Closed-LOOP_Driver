/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    vofa.h
  * @brief   VOFA JustFloat waveform output over UART (IT mode).
  *
  * Uses HAL_UART_Transmit_IT for non-blocking transmission.
  * Double-buffered to allow back-to-back frames.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __VOFA_H
#define __VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define VOFA_CH_NUM     10
#define VOFA_TAIL_0     0x00
#define VOFA_TAIL_1     0x00
#define VOFA_TAIL_2     0x80
#define VOFA_TAIL_3     0x7F

typedef struct {
    UART_HandleTypeDef *huart;
    float channels[VOFA_CH_NUM];
    uint8_t tx_busy;
    uint8_t buf[2][64];
    uint8_t buf_idx;
} VofaHandle_t;

extern VofaHandle_t g_vofa;

void Vofa_Init(VofaHandle_t *v, UART_HandleTypeDef *huart);
void Vofa_SetChannel(VofaHandle_t *v, uint8_t ch, float val);
void Vofa_Send(VofaHandle_t *v);
void Vofa_TxCpltCallback(VofaHandle_t *v);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H */
