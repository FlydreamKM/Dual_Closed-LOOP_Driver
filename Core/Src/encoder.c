/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   Quadrature encoder read-out with 16-bit overflow compensation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "encoder.h"
#include <math.h>

#define COUNTS_PER_REV  8192.0f
#define TWO_PI          6.283185307f

void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim)
{
    enc->htim = htim;
    enc->overflow_count = 0;
    enc->last_count = 0;
    enc->total_count = 0;
    enc->actual_speed = 0.0f;
    enc->actual_angle = 0.0f;
    enc->delta_count = 0;
    enc->initialized = 0;

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

void Encoder_ProcessOverflow(Encoder_t *enc)
{
    /* Clear the update interrupt flag before HAL_TIM_IRQHandler reads it */
    __HAL_TIM_CLEAR_IT(enc->htim, TIM_IT_UPDATE);

    /* In encoder mode DIR bit indicates counting direction at overflow instant */
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(enc->htim)) {
        enc->overflow_count--;  /* underflow */
    } else {
        enc->overflow_count++;  /* overflow */
    }
}

void Encoder_Update(Encoder_t *enc)
{
    int32_t current = (int32_t)__HAL_TIM_GET_COUNTER(enc->htim);
    int32_t last = enc->last_count;

    if (!enc->initialized) {
        enc->last_count = current;
        enc->initialized = 1;
        return;
    }

    int32_t diff = current - last;

    /* Compensate for multiple overflows that occurred between updates */
    diff += enc->overflow_count * 65536;
    enc->overflow_count = 0;

    enc->delta_count = diff;
    enc->total_count += diff;
    enc->last_count = current;

    enc->actual_angle = Encoder_CountsToAngle(enc->total_count);
    enc->actual_speed = Encoder_CountsToSpeed(diff, 1.0f);
}

void Encoder_Reset(Encoder_t *enc)
{
    enc->overflow_count = 0;
    enc->last_count = (int32_t)__HAL_TIM_GET_COUNTER(enc->htim);
    enc->total_count = 0;
    enc->actual_speed = 0.0f;
    enc->actual_angle = 0.0f;
    enc->delta_count = 0;
}

float Encoder_CountsToAngle(int32_t counts)
{
    return ((float)counts / COUNTS_PER_REV) * TWO_PI;
}

float Encoder_CountsToSpeed(int32_t delta_counts, float dt_ms)
{
    if (dt_ms <= 0.0f) dt_ms = 1.0f;
    return ((float)delta_counts * TWO_PI * 1000.0f) / (COUNTS_PER_REV * dt_ms);
}
