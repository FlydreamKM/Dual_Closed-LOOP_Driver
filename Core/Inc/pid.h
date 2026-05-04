/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pid.h
  * @brief   PID controller module header.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float output_limit;
    float integral_limit;
    float output;
    uint8_t first_run;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_limit, float int_limit);
float PID_Update(PID_t *pid, float setpoint, float actual);
void PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
