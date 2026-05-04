/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   UART DMA protocol parser and frame builder.
  *
  * Supports:
  *   - Binary frame protocol (see COMM_PROTOCOL.md)
  *   - FireWater-style text commands for VOFA tuning
  *   - VOFA JustFloat waveform output
  ******************************************************************************
  */
/* USER CODE END Header */

#include "protocol.h"
#include <string.h>

#define FRAME_SOF0  0xAA
#define FRAME_SOF1  0x55
#define VOFA_TAIL_0 0x00
#define VOFA_TAIL_1 0x00
#define VOFA_TAIL_2 0x80
#define VOFA_TAIL_3 0x7f

static uint8_t CalcChecksum(uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/* Lightweight ASCII to float parser (no libc dependency, no exp notation) */
static float simple_atof(const char *s, char **endptr)
{
    float val = 0.0f;
    float frac = 0.0f;
    float div = 1.0f;
    int sign = 1;
    const char *p = s;

    while (*p == ' ' || *p == '\t') p++;
    if (*p == '-') { sign = -1; p++; }
    else if (*p == '+') p++;

    while (*p >= '0' && *p <= '9') {
        val = val * 10.0f + (float)(*p - '0');
        p++;
    }

    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            frac = frac * 10.0f + (float)(*p - '0');
            div *= 10.0f;
            p++;
        }
    }

    if (endptr) *endptr = (char *)p;
    return sign * (val + frac / div);
}

static void Protocol_EnqueueCmd(Protocol_t *proto, ProtocolCmd_t *pcmd)
{
    uint8_t w = (proto->cmd_write_idx + 1) % PROTOCOL_CMD_QUEUE_SIZE;
    if (w != proto->cmd_read_idx) {
        proto->cmd_queue[proto->cmd_write_idx] = *pcmd;
        proto->cmd_write_idx = w;
    }
}

/* Parse FireWater-style text line and enqueue as binary command */
static void Protocol_ParseTextLine(Protocol_t *proto, char *line)
{
    while (*line == ' ' || *line == '\t') line++;
    char cmd_char = *line++;
    while (*line == ' ' || *line == '\t') line++;

    ProtocolCmd_t pcmd;
    memset(&pcmd, 0, sizeof(pcmd));

    switch (cmd_char) {
        case 'M': {  /* M motor mode speed angle accel decel */
            int motor = (int)simple_atof(line, &line);
            int mode  = (int)simple_atof(line, &line);
            float sp  = simple_atof(line, &line);
            float ang = simple_atof(line, &line);
            float acc = simple_atof(line, &line);
            float dec = simple_atof(line, &line);
            pcmd.cmd = CMD_SET_TARGET;
            pcmd.motor_id = (uint8_t)motor;
            pcmd.data.target.mode = (uint8_t)mode;
            pcmd.data.target.target_speed = sp;
            pcmd.data.target.target_angle = ang;
            pcmd.data.target.accel = acc;
            pcmd.data.target.decel = dec;
            Protocol_EnqueueCmd(proto, &pcmd);
            break;
        }
        case 'P': {  /* P motor pid_type kp ki kd */
            int motor = (int)simple_atof(line, &line);
            int ptype = (int)simple_atof(line, &line);
            float kp  = simple_atof(line, &line);
            float ki  = simple_atof(line, &line);
            float kd  = simple_atof(line, &line);
            pcmd.cmd = CMD_SET_PID;
            pcmd.motor_id = (uint8_t)motor;
            pcmd.data.pid.pid_type = (uint8_t)ptype;
            pcmd.data.pid.kp = kp;
            pcmd.data.pid.ki = ki;
            pcmd.data.pid.kd = kd;
            Protocol_EnqueueCmd(proto, &pcmd);
            break;
        }
        case 'C': {  /* C motor ctrl_cmd */
            int motor = (int)simple_atof(line, &line);
            int ctrl  = (int)simple_atof(line, &line);
            pcmd.cmd = CMD_CONTROL;
            pcmd.motor_id = (uint8_t)motor;
            pcmd.data.control.ctrl_cmd = (uint8_t)ctrl;
            Protocol_EnqueueCmd(proto, &pcmd);
            break;
        }
        case 'S': {  /* S motor */
            int motor = (int)simple_atof(line, &line);
            pcmd.cmd = CMD_REQ_STATUS;
            pcmd.motor_id = (uint8_t)motor;
            Protocol_EnqueueCmd(proto, &pcmd);
            break;
        }
        case 'V': {  /* V interval_ms  ->  set VOFA JustFloat interval */
            int interval = (int)simple_atof(line, &line);
            pcmd.cmd = CMD_SET_VOFA;
            pcmd.motor_id = MOTOR_ID_BOTH;
            pcmd.data.vofa.interval_ms = (uint8_t)interval;
            Protocol_EnqueueCmd(proto, &pcmd);
            break;
        }
        default:
            break;
    }
}

void Protocol_Init(Protocol_t *proto, UART_HandleTypeDef *huart)
{
    proto->huart = huart;
    proto->rx_write_idx = 0;
    proto->rx_read_idx = 0;
    proto->tx_busy = 0;
    proto->cmd_write_idx = 0;
    proto->cmd_read_idx = 0;
    proto->text_rx_idx = 0;
    memset(proto->rx_dma_buf, 0, PROTOCOL_RX_BUF_SIZE);
    memset(proto->tx_buf, 0, PROTOCOL_TX_BUF_SIZE);
    memset(proto->text_rx_buf, 0, sizeof(proto->text_rx_buf));

    HAL_UART_Receive_DMA(huart, proto->rx_dma_buf, PROTOCOL_RX_BUF_SIZE);
}

void Protocol_Poll(Protocol_t *proto)
{
    /* Update write index from DMA counter (circular mode) */
    proto->rx_write_idx = PROTOCOL_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(proto->huart->hdmarx);
}

void Protocol_ProcessRx(Protocol_t *proto)
{
    uint16_t write_idx = proto->rx_write_idx;
    uint16_t read_idx = proto->rx_read_idx;

    static uint8_t frame_buf[64];
    static uint8_t frame_state = 0;
    static uint8_t frame_len = 0;
    static uint8_t frame_idx = 0;

    while (read_idx != write_idx) {
        uint8_t byte = proto->rx_dma_buf[read_idx];
        read_idx = (read_idx + 1) % PROTOCOL_RX_BUF_SIZE;

        /* ---- FireWater text line handling ---- */
        if (byte == '\n' || byte == '\r') {
            if (proto->text_rx_idx > 0) {
                proto->text_rx_buf[proto->text_rx_idx] = '\0';
                Protocol_ParseTextLine(proto, proto->text_rx_buf);
                proto->text_rx_idx = 0;
            }
        } else if (proto->text_rx_idx < sizeof(proto->text_rx_buf) - 1) {
            proto->text_rx_buf[proto->text_rx_idx++] = (char)byte;
        }

#if !PROTOCOL_VOFA_ONLY
        /* ---- Binary frame state machine (disabled in VOFA-only mode) ---- */
        switch (frame_state) {
            case 0:
                if (byte == FRAME_SOF0) frame_state = 1;
                break;
            case 1:
                if (byte == FRAME_SOF1) frame_state = 2;
                else if (byte == FRAME_SOF0) frame_state = 1;
                else frame_state = 0;
                break;
            case 2:
                frame_len = byte;
                if (frame_len > 60) { frame_state = 0; break; }
                frame_idx = 0;
                frame_state = 3;
                break;
            case 3:
                frame_buf[frame_idx++] = byte;
                /* Need (CMD + DATA + CHK) = frame_len + 1 bytes */
                if (frame_idx >= frame_len + 1) {
                    uint8_t calc_chk = CalcChecksum(frame_buf, frame_len); /* CMD + DATA */
                    if (calc_chk == frame_buf[frame_len]) {
                        uint8_t cmd = frame_buf[0];
                        ProtocolCmd_t pcmd;
                        pcmd.cmd = cmd;

                        switch (cmd) {
                            case CMD_SET_TARGET:
                                if (frame_len >= 18) {
                                    pcmd.motor_id = frame_buf[1];
                                    pcmd.data.target.mode = frame_buf[2];
                                    memcpy(&pcmd.data.target.target_speed, &frame_buf[3], 4);
                                    memcpy(&pcmd.data.target.target_angle, &frame_buf[7], 4);
                                    memcpy(&pcmd.data.target.accel, &frame_buf[11], 4);
                                    memcpy(&pcmd.data.target.decel, &frame_buf[15], 4);
                                    Protocol_EnqueueCmd(proto, &pcmd);
                                }
                                break;

                            case CMD_SET_PID:
                                if (frame_len >= 14) {
                                    pcmd.motor_id = frame_buf[1];
                                    pcmd.data.pid.pid_type = frame_buf[2];
                                    memcpy(&pcmd.data.pid.kp, &frame_buf[3], 4);
                                    memcpy(&pcmd.data.pid.ki, &frame_buf[7], 4);
                                    memcpy(&pcmd.data.pid.kd, &frame_buf[11], 4);
                                    Protocol_EnqueueCmd(proto, &pcmd);
                                }
                                break;

                            case CMD_CONTROL:
                                if (frame_len >= 3) {
                                    pcmd.motor_id = frame_buf[1];
                                    pcmd.data.control.ctrl_cmd = frame_buf[2];
                                    Protocol_EnqueueCmd(proto, &pcmd);
                                }
                                break;

                            case CMD_REQ_STATUS:
                            case CMD_HEARTBEAT:
                                pcmd.motor_id = (frame_len >= 2) ? frame_buf[1] : MOTOR_ID_BOTH;
                                Protocol_EnqueueCmd(proto, &pcmd);
                                break;

                            case CMD_SET_VOFA:
                                if (frame_len >= 3) {
                                    pcmd.motor_id = frame_buf[1];
                                    pcmd.data.vofa.interval_ms = frame_buf[2];
                                    Protocol_EnqueueCmd(proto, &pcmd);
                                }
                                break;

                            default:
                                break;
                        }
                    }
                    frame_state = 0;
                }
                break;
        }
#endif /* !PROTOCOL_VOFA_ONLY */
    }

    proto->rx_read_idx = read_idx;
}

uint8_t Protocol_GetCommand(Protocol_t *proto, ProtocolCmd_t *cmd)
{
    if (proto->cmd_read_idx == proto->cmd_write_idx) return 0;
    *cmd = proto->cmd_queue[proto->cmd_read_idx];
    proto->cmd_read_idx = (proto->cmd_read_idx + 1) % PROTOCOL_CMD_QUEUE_SIZE;
    return 1;
}

void Protocol_SendStatus(Protocol_t *proto, uint8_t motor_id, uint8_t mode_state,
                         float actual_speed, float actual_angle,
                         float target_speed, float target_angle,
                         int16_t pwm_output, int32_t encoder_total, uint8_t fault)
{
#if PROTOCOL_VOFA_ONLY
    (void)proto; (void)motor_id; (void)mode_state;
    (void)actual_speed; (void)actual_angle; (void)target_speed; (void)target_angle;
    (void)pwm_output; (void)encoder_total; (void)fault;
    return;
#else
    if (proto->tx_busy) return;

    uint8_t *p = proto->tx_buf;
    uint8_t idx = 0;
    p[idx++] = FRAME_SOF0;
    p[idx++] = FRAME_SOF1;
    uint8_t len_idx = idx++;        /* reserve LEN position */
    p[idx++] = RSP_STATUS;          /* CMD */
    p[idx++] = motor_id;
    p[idx++] = mode_state;
    memcpy(&p[idx], &actual_speed, 4); idx += 4;
    memcpy(&p[idx], &actual_angle, 4); idx += 4;
    memcpy(&p[idx], &target_speed, 4); idx += 4;
    memcpy(&p[idx], &target_angle, 4); idx += 4;
    memcpy(&p[idx], &pwm_output,   2); idx += 2;
    memcpy(&p[idx], &encoder_total,4); idx += 4;
    p[idx++] = fault;

    uint8_t data_len = idx - len_idx - 1;  /* CMD + DATA length */
    p[len_idx] = data_len;
    p[idx++] = CalcChecksum(&p[len_idx + 1], data_len);  /* CHK over CMD+DATA */

    proto->tx_busy = 1;
    HAL_UART_Transmit_DMA(proto->huart, proto->tx_buf, idx);
#endif
}

void Protocol_SendAck(Protocol_t *proto, uint8_t cmd, uint8_t result)
{
#if PROTOCOL_VOFA_ONLY
    (void)proto; (void)cmd; (void)result;
    return;
#else
    if (proto->tx_busy) return;

    uint8_t *p = proto->tx_buf;
    p[0] = FRAME_SOF0;
    p[1] = FRAME_SOF1;
    p[2] = 3;           /* LEN = CMD(1) + DATA(2) */
    p[3] = RSP_ACK;     /* CMD */
    p[4] = cmd;         /* DATA[0] */
    p[5] = result;      /* DATA[1] */
    p[6] = CalcChecksum(&p[3], 3);  /* CHK over CMD+DATA */

    proto->tx_busy = 1;
    HAL_UART_Transmit_DMA(proto->huart, proto->tx_buf, 7);
#endif
}

void Protocol_SendVofaJustFloat(Protocol_t *proto, float *channels, uint8_t num_channels)
{
    if (proto->tx_busy) return;
    if (num_channels == 0) return;

    uint8_t *p = proto->tx_buf;
    uint8_t idx = 0;

    for (uint8_t i = 0; i < num_channels; i++) {
        memcpy(&p[idx], &channels[i], 4);
        idx += 4;
    }

    /* VOFA JustFloat frame tail */
    p[idx++] = VOFA_TAIL_0;
    p[idx++] = VOFA_TAIL_1;
    p[idx++] = VOFA_TAIL_2;
    p[idx++] = VOFA_TAIL_3;

    proto->tx_busy = 1;
    HAL_UART_Transmit_DMA(proto->huart, proto->tx_buf, idx);
}

void Protocol_TxCompleteCallback(Protocol_t *proto)
{
    proto->tx_busy = 0;
}
