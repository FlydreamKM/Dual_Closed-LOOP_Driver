/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Motor driver implementation.
  *
  * Direction logic (per AGENTS.md):
  *   High level (SET)   = CW  (正转)
  *   Low level (RESET)  = CCW (反转)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motor.h"

void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t channel,
                GPIO_TypeDef *dir_port, uint16_t dir_pin, int16_t pwm_max)
{
    motor->htim_pwm = htim;
    motor->channel = channel;
    motor->dir_port = dir_port;
    motor->dir_pin = dir_pin;
    motor->pwm_max = pwm_max;
    motor->pwm_output = 0;
    motor->enabled = 0;
    motor->emergency_stop = 0;

    HAL_TIM_PWM_Start(htim, channel);
    __HAL_TIM_SET_COMPARE(htim, channel, 0);
}

void Motor_SetOutput(Motor_t *motor, int16_t pwm)
{
    if (motor->emergency_stop || !motor->enabled) {
        __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->channel, 0);
        motor->pwm_output = 0;
        return;
    }

    if (pwm > motor->pwm_max) {
        pwm = motor->pwm_max;
    } else if (pwm < -motor->pwm_max) {
        pwm = -motor->pwm_max;
    }

    motor->pwm_output = pwm;

    if (pwm >= 0) {
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);    /* CW */
        __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->channel, pwm);
    } else {
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);  /* CCW */
        __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->channel, -pwm);
    }
}

void Motor_Enable(Motor_t *motor)
{
    motor->enabled = 1;
}

void Motor_Disable(Motor_t *motor)
{
    motor->enabled = 0;
    __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->channel, 0);
    motor->pwm_output = 0;
}

void Motor_EmergencyStop(Motor_t *motor)
{
    motor->emergency_stop = 1;
    __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->channel, 0);
    motor->pwm_output = 0;
}

void Motor_ClearEmergency(Motor_t *motor)
{
    motor->emergency_stop = 0;
}
