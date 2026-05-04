/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Motor driver: PWM + direction GPIO.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim_pwm;
    uint32_t channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    int16_t pwm_max;
    int16_t pwm_output;
    uint8_t enabled;
    uint8_t emergency_stop;
} Motor_t;

void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t channel,
                GPIO_TypeDef *dir_port, uint16_t dir_pin, int16_t pwm_max);
void Motor_SetOutput(Motor_t *motor, int16_t pwm);
void Motor_Enable(Motor_t *motor);
void Motor_Disable(Motor_t *motor);
void Motor_EmergencyStop(Motor_t *motor);
void Motor_ClearEmergency(Motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
