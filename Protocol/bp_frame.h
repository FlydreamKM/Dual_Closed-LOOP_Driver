/**
 * @file    bp_frame.h
 * @brief   二进制通信协议帧数据结构、打包器与解析辅助函数
 *
 * 本模块提供面向对象的帧操作接口：
 *   - 将 C 结构体打包成协议字节流（上位机发下行帧 / 下位机组上行帧）
 *   - 将协议字节流解析回 C 结构体（下位机解下行帧 / 上位机解上行帧）
 *
 * 核心设计原则：
 *   1. 字节序：严格按小端（Little-Endian）处理，与 STM32 保持一致
 *   2. 浮点数：使用 memcpy 进行字节级拷贝，避免编译器优化导致的歧义
 *   3. 零依赖：除 bp_def.h 和标准 C 库外，不引用任何平台头文件
 */

#ifndef BP_FRAME_H
#define BP_FRAME_H

#include "bp_def.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * 下行命令数据结构（上位机 -> 下位机）
 * =================================================================== */

/**
 * @brief CMD_SET_TARGET (0x01) 数据载荷
 *
 * LEN = 19，DATA 布局：
 *   [motor_id:1][mode:1][target_speed:4][target_angle:4][accel:4][decel:4]
 */
typedef struct {
    uint8_t  motor_id;      /**< 电机 ID：0=电机1, 1=电机2, 0xFF=双电机 */
    uint8_t  mode;          /**< 控制模式：0=速度模式, 1=位置模式 */
    float    target_speed;  /**< 目标速度，单位 rad/s */
    float    target_angle;  /**< 目标角度，单位 rad（速度模式下可填 0） */
    float    accel;         /**< 加速度限制，单位 rad/s²，必须 > 0 */
    float    decel;         /**< 减速度限制，单位 rad/s²，必须 > 0 */
} bp_cmd_set_target_t;

/**
 * @brief CMD_SET_PID (0x02) 数据载荷
 *
 * LEN = 15，DATA 布局：
 *   [motor_id:1][pid_type:1][kp:4][ki:4][kd:4]
 */
typedef struct {
    uint8_t  motor_id;      /**< 电机 ID */
    uint8_t  pid_type;      /**< 0=速度环, 1=位置环 */
    float    kp;            /**< 比例增益 */
    float    ki;            /**< 积分增益 */
    float    kd;            /**< 微分增益 */
} bp_cmd_set_pid_t;

/**
 * @brief CMD_CONTROL (0x03) 数据载荷
 *
 * LEN = 3，DATA 布局：
 *   [motor_id:1][ctrl_cmd:1]
 */
typedef struct {
    uint8_t  motor_id;      /**< 电机 ID */
    uint8_t  ctrl_cmd;      /**< 控制子命令：0=使能, 1=失能, 2=回零, 3=急停, 4=清除故障 */
} bp_cmd_control_t;

/**
 * @brief CMD_REQ_STATUS (0x04) / CMD_HEARTBEAT (0x05) 数据载荷
 *
 * LEN = 1 或 2，DATA 布局：
 *   [motor_id:1]（可选，省略时默认 0xFF=双电机）
 */
typedef struct {
    uint8_t  motor_id;      /**< 电机 ID，0xFF=省略时默认值 */
    uint8_t  has_motor_id;  /**< 标记位：1 表示帧中包含 motor_id，0 表示省略 */
} bp_cmd_req_status_t;

typedef bp_cmd_req_status_t bp_cmd_heartbeat_t; /**< 心跳包与请求状态结构相同 */

/**
 * @brief CMD_SET_VOFA (0x06) 数据载荷
 *
 * LEN = 3，DATA 布局：
 *   [motor_id:1][interval_ms:1]
 */
typedef struct {
    uint8_t  motor_id;      /**< 固定 0xFF（全局设置） */
    uint8_t  interval_ms;   /**< 帧间隔毫秒数：0=关闭, 5=200Hz, 10=100Hz */
} bp_cmd_set_vofa_t;

/* ===================================================================
 * 上行响应数据结构（下位机 -> 上位机）
 * =================================================================== */

/**
 * @brief RSP_STATUS (0x81) 数据载荷
 *
 * LEN = 25，DATA 布局：
 *   [motor_id:1][mode_state:1][actual_speed:4][actual_angle:4]
 *   [target_speed:4][target_angle:4][pwm_output:2][encoder_total:4][fault:1]
 */
typedef struct {
    uint8_t  motor_id;          /**< 电机 ID */
    uint8_t  mode_state;        /**< 高 4 位=state，低 4 位=mode */
    float    actual_speed;      /**< 实际速度，rad/s */
    float    actual_angle;      /**< 实际角度，rad */
    float    target_speed;      /**< 轨迹目标速度，rad/s */
    float    target_angle;      /**< 轨迹目标角度，rad */
    int16_t  pwm_output;        /**< PWM 输出值，范围 ±1000 */
    int32_t  encoder_total;     /**< 编码器总脉冲计数（含溢出补偿） */
    uint8_t  fault;             /**< 故障码，0=无故障 */
} bp_rsp_status_t;

/**
 * @brief RSP_ACK (0x82) 数据载荷
 *
 * LEN = 3，DATA 布局：
 *   [cmd:1][result:1]
 */
typedef struct {
    uint8_t  cmd;               /**< 原始命令码 */
    uint8_t  result;            /**< 结果码：0=成功，非零=错误（保留） */
} bp_rsp_ack_t;

/* ===================================================================
 * 帧打包器（Builder）
 * =================================================================== */

/**
 * @brief 帧打包器句柄
 *
 * 使用流程：
 *   1. bp_builder_init(&builder)
 *   2. bp_builder_begin(&builder, cmd)
 *   3. bp_builder_write_xxx(...)  写入各字段
 *   4. len = bp_builder_finish(&builder, &frame_ptr)
 *   5. 将 frame_ptr 指向的 len 字节发送出去
 */
typedef struct {
    uint8_t buf[BP_MAX_FRAME_SIZE]; /**< 内部帧缓冲区 */
    uint8_t idx;                    /**< 当前写入位置 */
} bp_builder_t;

/* ---------- 打包器基础操作 ---------- */

void bp_builder_init(bp_builder_t *builder);
void bp_builder_begin(bp_builder_t *builder, uint8_t cmd);
void bp_builder_write_u8 (bp_builder_t *builder, uint8_t  val);
void bp_builder_write_u16(bp_builder_t *builder, uint16_t val);
void bp_builder_write_u32(bp_builder_t *builder, uint32_t val);
void bp_builder_write_i16(bp_builder_t *builder, int16_t  val);
void bp_builder_write_i32(bp_builder_t *builder, int32_t  val);
void bp_builder_write_float(bp_builder_t *builder, float val);
void bp_builder_write_bytes(bp_builder_t *builder, const uint8_t *data, uint8_t len);

/**
 * @brief 完成帧构建，计算 LEN 和 CHK
 * @param builder   打包器句柄
 * @param out_frame 输出：指向完整帧的首地址（包含 SOF0/SOF1/LEN/CMD/DATA/CHK）
 * @return 完整帧的字节数
 */
uint8_t bp_builder_finish(bp_builder_t *builder, uint8_t **out_frame);

/* ---------- 快捷打包函数（推荐直接使用） ---------- */

uint8_t bp_build_cmd_set_target (bp_builder_t *b, const bp_cmd_set_target_t  *cmd, uint8_t **frame);
uint8_t bp_build_cmd_set_pid    (bp_builder_t *b, const bp_cmd_set_pid_t     *cmd, uint8_t **frame);
uint8_t bp_build_cmd_control    (bp_builder_t *b, const bp_cmd_control_t     *cmd, uint8_t **frame);
uint8_t bp_build_cmd_set_vofa   (bp_builder_t *b, const bp_cmd_set_vofa_t    *cmd, uint8_t **frame);
uint8_t bp_build_rsp_status     (bp_builder_t *b, const bp_rsp_status_t      *rsp, uint8_t **frame);
uint8_t bp_build_rsp_ack        (bp_builder_t *b, const bp_rsp_ack_t         *rsp, uint8_t **frame);

/* ===================================================================
 * 帧解析辅助函数（将原始字节流解析到结构体）
 * =================================================================== */

/**
 * @brief 从字节流解析 CMD_SET_TARGET
 * @param data 指向 CMD 字节（即 frame_buf[0]，不含 SOF/LEN/CHK）
 * @param len  data 长度（= LEN）
 * @param out  输出结构体
 * @return 1=成功，0=长度不足或格式错误
 */
uint8_t bp_parse_cmd_set_target (const uint8_t *data, uint8_t len, bp_cmd_set_target_t  *out);
uint8_t bp_parse_cmd_set_pid    (const uint8_t *data, uint8_t len, bp_cmd_set_pid_t     *out);
uint8_t bp_parse_cmd_control    (const uint8_t *data, uint8_t len, bp_cmd_control_t     *out);
uint8_t bp_parse_cmd_set_vofa   (const uint8_t *data, uint8_t len, bp_cmd_set_vofa_t    *out);
uint8_t bp_parse_rsp_status     (const uint8_t *data, uint8_t len, bp_rsp_status_t      *out);
uint8_t bp_parse_rsp_ack        (const uint8_t *data, uint8_t len, bp_rsp_ack_t         *out);

/* ===================================================================
 * 校验和与辅助函数
 * =================================================================== */

/**
 * @brief 计算校验和
 * @param data 数据首地址
 * @param len  数据长度（字节）
 * @return 累加和的低 8 位
 */
uint8_t bp_calc_checksum(const uint8_t *data, uint8_t len);

/**
 * @brief 从 mode_state 字节中提取 mode（低 4 位）
 */
static inline uint8_t bp_get_mode_from_state(uint8_t mode_state) {
    return mode_state & 0x0F;
}

/**
 * @brief 从 mode_state 字节中提取 state（高 4 位）
 */
static inline uint8_t bp_get_state_from_state(uint8_t mode_state) {
    return (mode_state >> 4) & 0x0F;
}

/**
 * @brief 将 mode 和 state 合并为 mode_state 字节
 */
static inline uint8_t bp_make_mode_state(uint8_t state, uint8_t mode) {
    return ((state & 0x0F) << 4) | (mode & 0x0F);
}

#ifdef __cplusplus
}
#endif

#endif /* BP_FRAME_H */
