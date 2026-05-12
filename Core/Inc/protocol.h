/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   Non-blocking UART protocol using DMA.
  *
  * Supports two input formats:
  *   1. Binary frame: [0xAA][0x55][LEN][CMD][DATA...][CHK]
  *   2. Text line (FireWater-style): plain ASCII command ending with '\n'
  *
  * Output formats:
  *   1. Binary STATUS / ACK frames
  *   2. VOFA JustFloat waveform frame
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Protocol mode switch:
 *   1 = VOFA-only (FireWater text commands + JustFloat output only)
 *   0 = Binary-only (0xAA 0x55 frames only, no JustFloat output)
 */
#define PROTOCOL_VOFA_ONLY      1

#define PROTOCOL_RX_BUF_SIZE    256
#define PROTOCOL_TX_BUF_SIZE    256
#define PROTOCOL_CMD_QUEUE_SIZE 12

/* Command codes */
#define CMD_SET_TARGET      0x01
#define CMD_SET_PID         0x02
#define CMD_CONTROL         0x03
#define CMD_REQ_STATUS      0x04
#define CMD_HEARTBEAT       0x05
#define CMD_SET_VOFA        0x06   /* interval_ms */
#define CMD_SET_PID_BOTH    0x07   /* 同时设置两个电机的 PID 参数 */

/* Response codes */
#define RSP_STATUS          0x81
#define RSP_ACK             0x82
#define RSP_HEARTBEAT       0x85

/* Motor IDs */
#define MOTOR_ID_1          0
#define MOTOR_ID_2          1
#define MOTOR_ID_BOTH       0xFF

/* Control sub-commands */
#define CTRL_ENABLE         0
#define CTRL_DISABLE        1
#define CTRL_HOME           2
#define CTRL_EMERGENCY      3
#define CTRL_CLEAR_FAULT    4

typedef struct {
    uint8_t cmd;
    uint8_t motor_id;
    union {
        struct {
            uint8_t mode;
            float target_speed;
            float target_angle;
            float accel;
            float decel;
        } target;
        struct {
            uint8_t pid_type;   /* 0=speed, 1=position */
            float kp;
            float ki;
            float kd;
        } pid;
        struct {
            uint8_t ctrl_cmd;
        } control;
        struct {
            uint8_t interval_ms; /* 0=disable VOFA JustFloat output */
        } vofa;
    } data;
} ProtocolCmd_t;

typedef struct {
    UART_HandleTypeDef *huart;

    /* RX: circular DMA buffer */
    uint8_t rx_dma_buf[PROTOCOL_RX_BUF_SIZE];
    volatile uint16_t rx_write_idx;
    uint16_t rx_read_idx;

    /* TX: single DMA buffer */
    uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
    volatile uint8_t tx_busy;

    /* Command queue */
    ProtocolCmd_t cmd_queue[PROTOCOL_CMD_QUEUE_SIZE];
    volatile uint8_t cmd_write_idx;
    volatile uint8_t cmd_read_idx;

    /* Text line buffer for FireWater / VOFA text commands */
    char text_rx_buf[80];
    uint8_t text_rx_idx;
} Protocol_t;

void Protocol_Init(Protocol_t *proto, UART_HandleTypeDef *huart);
void Protocol_Poll(Protocol_t *proto);
void Protocol_ProcessRx(Protocol_t *proto);
uint8_t Protocol_GetCommand(Protocol_t *proto, ProtocolCmd_t *cmd);

/* Standard binary frames */
void Protocol_SendStatus(Protocol_t *proto, uint8_t motor_id, uint8_t mode_state,
                         float actual_speed, float actual_angle,
                         float target_speed, float target_angle,
                         int16_t pwm_output, int32_t encoder_total, uint8_t fault);
void Protocol_SendAck(Protocol_t *proto, uint8_t cmd, uint8_t result);

#if PROTOCOL_VOFA_ONLY
/* VOFA JustFloat waveform output */
void Protocol_SendVofaJustFloat(Protocol_t *proto, float *channels, uint8_t num_channels);
#endif

void Protocol_TxCompleteCallback(Protocol_t *proto);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
