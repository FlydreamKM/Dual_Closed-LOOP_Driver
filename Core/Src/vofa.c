/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    vofa.c
  * @brief   VOFA JustFloat waveform output over UART (IT mode).
  ******************************************************************************
  */
/* USER CODE END Header */

#include "vofa.h"
#include <string.h>

VofaHandle_t g_vofa;

void Vofa_Init(VofaHandle_t *v, UART_HandleTypeDef *huart)
{
    memset(v, 0, sizeof(*v));
    v->huart = huart;
}

void Vofa_SetChannel(VofaHandle_t *v, uint8_t ch, float val)
{
    if (ch < VOFA_CH_NUM) {
        v->channels[ch] = val;
    }
}

void Vofa_Send(VofaHandle_t *v)
{
    if (v->tx_busy) return;

    uint8_t *p = v->buf[v->buf_idx];
    memcpy(p, v->channels, VOFA_CH_NUM * 4);
    uint8_t idx = VOFA_CH_NUM * 4;

    p[idx++] = VOFA_TAIL_0;
    p[idx++] = VOFA_TAIL_1;
    p[idx++] = VOFA_TAIL_2;
    p[idx++] = VOFA_TAIL_3;

    if (HAL_UART_Transmit_IT(v->huart, p, idx) == HAL_OK) {
        v->tx_busy = 1;
        v->buf_idx = 1 - v->buf_idx;
    }
}

void Vofa_TxCpltCallback(VofaHandle_t *v)
{
    v->tx_busy = 0;
}
