/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app.h
  * @brief   Application layer: init, command dispatch, periodic status.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "controller.h"
#include "protocol.h"

#define MOTOR_NUM   2

typedef struct {
    Encoder_t encoder[MOTOR_NUM];
    Motor_t motor[MOTOR_NUM];
    MotorController_t controller[MOTOR_NUM];
    Protocol_t protocol;

    uint32_t control_tick;
    uint32_t status_tick;
    uint8_t status_interval_ms;  /* standard binary status rate: e.g. 10 -> 100 Hz */
    uint8_t vofa_interval_ms;    /* VOFA JustFloat output rate: 0=disabled, e.g. 5 -> 200 Hz */
} App_t;

extern App_t g_app;
extern volatile uint8_t g_app_initialized;

void App_Init(void);
void App_ControlUpdate(void);  /* 1 kHz, called from SysTick or main loop */
void App_Run(void);            /* main loop idle tasks */

void App_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void App_DMA1_Channel7_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
