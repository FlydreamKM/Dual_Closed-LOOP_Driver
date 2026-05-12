/**
 * @file    bp_frame.c
 * @brief   二进制通信协议帧打包与解析实现
 *
 * 实现说明：
 *   1. 字节序：所有多字节数据严格按小端（Little-Endian）处理。
 *      使用位移操作显式拼装/拆解，不依赖编译器的 struct packing。
 *   2. 浮点数：通过 memcpy 在 float 与 uint8_t 数组之间直接拷贝内存，
 *      保证 IEEE-754 字节序完整传递，不做任何数值转换。
 *   3. 打包器：采用"顺序写入"模式，先写 CMD，再依次写 DATA，
 *      最后由 finish 函数回填 LEN 并计算 CHK。
 */

#include "bp_frame.h"
#include <string.h>

/* ===================================================================
 * 内部辅助：小端字节序读写
 * =================================================================== */

/**
 * @brief 将 uint16_t 按小端写入缓冲区
 */
static void _write_u16_le(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

/**
 * @brief 将 uint32_t 按小端写入缓冲区
 */
static void _write_u32_le(uint8_t *buf, uint32_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

/**
 * @brief 从缓冲区按小端读取 uint16_t
 */
static uint16_t _read_u16_le(const uint8_t *buf)
{
    return ((uint16_t)buf[1] << 8) | buf[0];
}

/**
 * @brief 从缓冲区按小端读取 uint32_t
 */
static uint32_t _read_u32_le(const uint8_t *buf)
{
    return ((uint32_t)buf[3] << 24) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[1] << 8)  |
           buf[0];
}

/* ===================================================================
 * 校验和计算
 * =================================================================== */

uint8_t bp_calc_checksum(const uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/* ===================================================================
 * 打包器基础操作
 * =================================================================== */

void bp_builder_init(bp_builder_t *builder)
{
    memset(builder->buf, 0, sizeof(builder->buf));
    builder->idx = 0;
}

void bp_builder_begin(bp_builder_t *builder, uint8_t cmd)
{
    builder->idx = 0;
    builder->buf[builder->idx++] = BP_SOF0;     /* 帧头首字节 */
    builder->buf[builder->idx++] = BP_SOF1;     /* 帧头次字节 */
    builder->buf[builder->idx++] = 0;           /* LEN 占位，finish 时回填 */
    builder->buf[builder->idx++] = cmd;         /* 命令码 */
}

void bp_builder_write_u8(bp_builder_t *builder, uint8_t val)
{
    if (builder->idx < BP_MAX_FRAME_SIZE) {
        builder->buf[builder->idx++] = val;
    }
}

void bp_builder_write_u16(bp_builder_t *builder, uint16_t val)
{
    if (builder->idx + 2 <= BP_MAX_FRAME_SIZE) {
        _write_u16_le(&builder->buf[builder->idx], val);
        builder->idx += 2;
    }
}

void bp_builder_write_u32(bp_builder_t *builder, uint32_t val)
{
    if (builder->idx + 4 <= BP_MAX_FRAME_SIZE) {
        _write_u32_le(&builder->buf[builder->idx], val);
        builder->idx += 4;
    }
}

void bp_builder_write_i16(bp_builder_t *builder, int16_t val)
{
    /* 利用补码一致性：int16_t 的位模式与 uint16_t 相同 */
    bp_builder_write_u16(builder, (uint16_t)val);
}

void bp_builder_write_i32(bp_builder_t *builder, int32_t val)
{
    bp_builder_write_u32(builder, (uint32_t)val);
}

void bp_builder_write_float(bp_builder_t *builder, float val)
{
    if (builder->idx + 4 <= BP_MAX_FRAME_SIZE) {
        /* memcpy 保证按内存字节序原样拷贝，不做任何数值解释 */
        memcpy(&builder->buf[builder->idx], &val, 4);
        builder->idx += 4;
    }
}

void bp_builder_write_bytes(bp_builder_t *builder, const uint8_t *data, uint8_t len)
{
    if (builder->idx + len <= BP_MAX_FRAME_SIZE) {
        memcpy(&builder->buf[builder->idx], data, len);
        builder->idx += len;
    }
}

uint8_t bp_builder_finish(bp_builder_t *builder, uint8_t **out_frame)
{
    /*
     * 当前 idx 指向 DATA 最后一个字节的下一个位置。
     * 布局：buf[0]=SOF0, buf[1]=SOF1, buf[2]=LEN, buf[3]=CMD, buf[4..idx-1]=DATA
     *
     * LEN = CMD(1) + DATA(N) = idx - 3（因为 SOF0+SOF1+LEN 占了 3 字节）
     * 但 buf[3] 是 CMD，所以从 CMD 到 DATA 末尾共 idx - 3 字节。
     */
    uint8_t data_len = builder->idx - 3;  /* CMD + DATA 的总长度 */

    /* 回填 LEN */
    builder->buf[2] = data_len;

    /* 计算 CHK：范围是 CMD + DATA（即 buf[3] 到 buf[idx-1]） */
    uint8_t chk = bp_calc_checksum(&builder->buf[3], data_len);
    builder->buf[builder->idx++] = chk;

    if (out_frame != NULL) {
        *out_frame = builder->buf;
    }
    return builder->idx;  /* 返回完整帧长度（含 SOF0/SOF1/LEN/CMD/DATA/CHK） */
}

/* ===================================================================
 * 快捷打包函数
 * =================================================================== */

uint8_t bp_build_cmd_set_target(bp_builder_t *b, const bp_cmd_set_target_t *cmd, uint8_t **frame)
{
    bp_builder_begin(b, BP_CMD_SET_TARGET);
    bp_builder_write_u8 (b, cmd->motor_id);
    bp_builder_write_u8 (b, cmd->mode);
    bp_builder_write_float(b, cmd->target_speed);
    bp_builder_write_float(b, cmd->target_angle);
    bp_builder_write_float(b, cmd->accel);
    bp_builder_write_float(b, cmd->decel);
    return bp_builder_finish(b, frame);
}

uint8_t bp_build_cmd_set_pid(bp_builder_t *b, const bp_cmd_set_pid_t *cmd, uint8_t **frame)
{
    bp_builder_begin(b, BP_CMD_SET_PID);
    bp_builder_write_u8 (b, cmd->motor_id);
    bp_builder_write_u8 (b, cmd->pid_type);
    bp_builder_write_float(b, cmd->kp);
    bp_builder_write_float(b, cmd->ki);
    bp_builder_write_float(b, cmd->kd);
    return bp_builder_finish(b, frame);
}

uint8_t bp_build_cmd_set_pid_both(bp_builder_t *b, const bp_cmd_set_pid_both_t *cmd, uint8_t **frame)
{
    bp_builder_begin(b, BP_CMD_SET_PID_BOTH);
    bp_builder_write_u8 (b, cmd->pid_type);
    bp_builder_write_float(b, cmd->kp);
    bp_builder_write_float(b, cmd->ki);
    bp_builder_write_float(b, cmd->kd);
    return bp_builder_finish(b, frame);
}

uint8_t bp_build_cmd_control(bp_builder_t *b, const bp_cmd_control_t *cmd, uint8_t **frame)
{
    bp_builder_begin(b, BP_CMD_CONTROL);
    bp_builder_write_u8(b, cmd->motor_id);
    bp_builder_write_u8(b, cmd->ctrl_cmd);
    return bp_builder_finish(b, frame);
}

uint8_t bp_build_cmd_set_vofa(bp_builder_t *b, const bp_cmd_set_vofa_t *cmd, uint8_t **frame)
{
    bp_builder_begin(b, BP_CMD_SET_VOFA);
    bp_builder_write_u8(b, cmd->motor_id);
    bp_builder_write_u8(b, cmd->interval_ms);
    return bp_builder_finish(b, frame);
}

uint8_t bp_build_rsp_status(bp_builder_t *b, const bp_rsp_status_t *rsp, uint8_t **frame)
{
    bp_builder_begin(b, BP_RSP_STATUS);
    bp_builder_write_u8 (b, rsp->motor_id);
    bp_builder_write_u8 (b, rsp->mode_state);
    bp_builder_write_float(b, rsp->actual_speed);
    bp_builder_write_float(b, rsp->actual_angle);
    bp_builder_write_float(b, rsp->target_speed);
    bp_builder_write_float(b, rsp->target_angle);
    bp_builder_write_i16 (b, rsp->pwm_output);
    bp_builder_write_i32 (b, rsp->encoder_total);
    bp_builder_write_u8 (b, rsp->fault);
    return bp_builder_finish(b, frame);
}

uint8_t bp_build_rsp_ack(bp_builder_t *b, const bp_rsp_ack_t *rsp, uint8_t **frame)
{
    bp_builder_begin(b, BP_RSP_ACK);
    bp_builder_write_u8(b, rsp->cmd);
    bp_builder_write_u8(b, rsp->result);
    return bp_builder_finish(b, frame);
}

/* ===================================================================
 * 帧解析辅助函数
 * =================================================================== */

uint8_t bp_parse_cmd_set_target(const uint8_t *data, uint8_t len, bp_cmd_set_target_t *out)
{
    /* CMD(1) + motor_id(1) + mode(1) + speed(4) + angle(4) + accel(4) + decel(4) = 19 */
    if (len < 19) return 0;

    out->motor_id      = data[1];
    out->mode          = data[2];
    memcpy(&out->target_speed, &data[3],  4);
    memcpy(&out->target_angle, &data[7],  4);
    memcpy(&out->accel,        &data[11], 4);
    memcpy(&out->decel,        &data[15], 4);
    return 1;
}

uint8_t bp_parse_cmd_set_pid(const uint8_t *data, uint8_t len, bp_cmd_set_pid_t *out)
{
    /* CMD(1) + motor_id(1) + pid_type(1) + kp(4) + ki(4) + kd(4) = 15 */
    if (len < 15) return 0;

    out->motor_id   = data[1];
    out->pid_type   = data[2];
    memcpy(&out->kp, &data[3],  4);
    memcpy(&out->ki, &data[7],  4);
    memcpy(&out->kd, &data[11], 4);
    return 1;
}

uint8_t bp_parse_cmd_set_pid_both(const uint8_t *data, uint8_t len, bp_cmd_set_pid_both_t *out)
{
    /* CMD(1) + pid_type(1) + kp(4) + ki(4) + kd(4) = 13 */
    if (len < 13) return 0;

    out->pid_type   = data[1];
    memcpy(&out->kp, &data[2],  4);
    memcpy(&out->ki, &data[6],  4);
    memcpy(&out->kd, &data[10], 4);
    return 1;
}

uint8_t bp_parse_cmd_control(const uint8_t *data, uint8_t len, bp_cmd_control_t *out)
{
    /* CMD(1) + motor_id(1) + ctrl_cmd(1) = 3 */
    if (len < 3) return 0;

    out->motor_id  = data[1];
    out->ctrl_cmd  = data[2];
    return 1;
}

uint8_t bp_parse_cmd_set_vofa(const uint8_t *data, uint8_t len, bp_cmd_set_vofa_t *out)
{
    /* CMD(1) + motor_id(1) + interval_ms(1) = 3 */
    if (len < 3) return 0;

    out->motor_id     = data[1];
    out->interval_ms  = data[2];
    return 1;
}

uint8_t bp_parse_rsp_status(const uint8_t *data, uint8_t len, bp_rsp_status_t *out)
{
    /* CMD(1) + motor_id(1) + mode_state(1) + speed(4) + angle(4) + tgt_spd(4) + tgt_ang(4) + pwm(2) + enc(4) + fault(1) = 25 */
    if (len < 25) return 0;

    out->motor_id      = data[1];
    out->mode_state    = data[2];
    memcpy(&out->actual_speed,  &data[3],  4);
    memcpy(&out->actual_angle,  &data[7],  4);
    memcpy(&out->target_speed,  &data[11], 4);
    memcpy(&out->target_angle,  &data[15], 4);
    out->pwm_output    = (int16_t)_read_u16_le(&data[19]);
    out->encoder_total = (int32_t)_read_u32_le(&data[21]);
    out->fault         = data[25];
    return 1;
}

uint8_t bp_parse_rsp_ack(const uint8_t *data, uint8_t len, bp_rsp_ack_t *out)
{
    /* CMD(1) + cmd(1) + result(1) = 3 */
    if (len < 3) return 0;

    out->cmd    = data[1];
    out->result = data[2];
    return 1;
}
