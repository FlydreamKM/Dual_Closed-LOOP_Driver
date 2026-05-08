/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Position-type PID with anti-windup.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_limit, float int_limit)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->output_limit = out_limit;
    pid->integral_limit = int_limit;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->first_run = 1;
}

float PID_Update(PID_t *pid, float setpoint, float actual)
{
    float error = setpoint - actual;

    if (pid->first_run) {
        pid->prev_error = error;
        pid->first_run = 0;
    }

    /* Integral accumulation */
    pid->integral += error;

    /* Pre-calculate P+I for anti-windup check */
    float pre_output = pid->Kp * error + pid->Ki * pid->integral;

    /* Anti-windup: clamp integral if output is already saturated */
    if (pre_output > pid->output_limit) {
        pid->integral -= error;
        if (pid->integral > pid->integral_limit) {
            pid->integral = pid->integral_limit;
        }
        pre_output = pid->output_limit;
    } else if (pre_output < -pid->output_limit) {
        pid->integral -= error;
        if (pid->integral < -pid->integral_limit) {
            pid->integral = -pid->integral_limit;
        }
        pre_output = -pid->output_limit;
    }

    /* Integral limit (final guard) */
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }

    /* Re-calculate P+I after integral clamping */
    pre_output = pid->Kp * error + pid->Ki * pid->integral;

    /* Derivative */
    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    pid->output = pre_output + pid->Kd * derivative;

    /* Output limit */
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->first_run = 1;
}
