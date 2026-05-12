/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   Quadrature encoder interface with overflow tracking.
  *
  * Encoder spec: 1024-line, 4x decoding (TI12).
  * Gear chain: Motor(16T) -> Wheel(68T) -> Encoder(30T).
  * -> 4096 * (16/30) ≈ 2184.5 TIM counts per motor revolution.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define ENCODER_CPR                 1024
#define ENCODER_MULTIPLY            4

/* Gear chain: Motor(16T) -> Wheel(68T) -> Encoder(30T).
 * Effective ratio = (16/68) * (68/30) = 16/30 encoder revs per motor rev. */
#define MOTOR_GEAR_TEETH            16u
#define WHEEL_GEAR_TEETH            68u
#define ENCODER_GEAR_TEETH          30u
#define ENCODER_GEAR_RATIO          ((float)MOTOR_GEAR_TEETH / (float)WHEEL_GEAR_TEETH * (float)WHEEL_GEAR_TEETH / (float)ENCODER_GEAR_TEETH)

#define ENCODER_COUNTS_PER_MOTOR_REV ((float)ENCODER_CPR * (float)ENCODER_MULTIPLY * ENCODER_GEAR_RATIO)

typedef struct {
    TIM_HandleTypeDef *htim;
    int8_t invert;          /* 1 = 正常, -1 = 反转编码器方向（镜像安装时使用） */
    volatile int32_t overflow_count;
    int32_t last_count;
    volatile int32_t total_count;
    float actual_speed;     /* rad/s */
    float actual_angle;     /* rad (accumulated, can exceed one rev) */
    int32_t delta_count;
    uint8_t initialized;
} Encoder_t;

void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim);
void Encoder_Update(Encoder_t *enc);
void Encoder_Reset(Encoder_t *enc);
void Encoder_ProcessOverflow(Encoder_t *enc);

float Encoder_CountsToAngle(int32_t counts);
float Encoder_CountsToSpeed(int32_t delta_counts, float dt_ms);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
