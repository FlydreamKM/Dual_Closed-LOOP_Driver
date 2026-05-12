/**
 * @file    main_example.c
 * @brief   二进制通信协议驱动使用示例
 *
 * 本示例展示如何在 STM32 + HAL 环境中集成 bp_driver 驱动：
 *   1. 定义全局适配器句柄
 *   2. 实现帧接收回调（命令分发）
 *   3. 在 main() 中初始化 UART 和驱动
 *   4. 在 HAL 中断回调中转发事件
 *   5. 在主循环中周期发送 STATUS 帧
 *
 * 复制到项目中的对应位置即可使用，无需修改驱动源码。
 */

/* ===================== 用户提供的 HAL 头文件 ===================== */
#include "stm32f1xx_hal.h"
#include <string.h>

/* ===================== 驱动头文件 ===================== */
#include "bp_hal_uart.h"

/* ===================================================================
 * 全局变量
 * =================================================================== */

/* HAL UART 句柄（由 CubeMX 生成，通常位于 main.c） */
extern UART_HandleTypeDef huart2;

/* 协议驱动适配器（全局唯一实例） */
bp_hal_uart_t g_bp_uart;

/* 模拟电机状态（实际项目中替换为真实的电机控制结构体） */
typedef struct {
    uint8_t  state;         /* BP_STATE_IDLE / RUNNING / EMERGENCY */
    uint8_t  mode;          /* BP_MODE_SPEED / POSITION */
    float    actual_speed;
    float    actual_angle;
    float    target_speed;
    float    target_angle;
    int16_t  pwm_output;
    int32_t  encoder_total;
    uint8_t  fault;
} MotorStatus_t;

static MotorStatus_t g_motor[2];    /* 电机 0 和电机 1 */

/* ===================================================================
 * 帧接收回调 — 命令分发
 * =================================================================== */

/**
 * @brief 解析器回调：每成功解析一帧有效数据时被调用
 *
 * 调用上下文：UART DMA 中断（或 HAL 回调链）
 * 注意事项：
 *   - 本函数在中断中执行，必须保持轻量（不可阻塞、不可长时间占用 CPU）
 *   - 如需与主循环共享数据，对全局变量使用 volatile 修饰
 */
static void on_frame_received(uint8_t cmd, const uint8_t *data, uint8_t data_len)
{
    /* data[0] 就是 cmd，data[1..data_len-1] 是 DATA 字段 */
    (void)data_len;  /* 某些命令分支会用到，先避免编译警告 */

    switch (cmd) {
        case BP_CMD_SET_TARGET: {
            bp_cmd_set_target_t target;
            if (bp_parse_cmd_set_target(data, data_len, &target)) {
                /* 根据 motor_id 选择电机（简化示例，只处理 motor 0） */
                if (target.motor_id == BP_MOTOR_1 || target.motor_id == BP_MOTOR_BOTH) {
                    g_motor[0].mode         = target.mode;
                    g_motor[0].target_speed = target.target_speed;
                    g_motor[0].target_angle = target.target_angle;
                    /* accel/decel 可存入全局变量供 Controller 使用 */
                }
            }
            break;
        }

        case BP_CMD_SET_PID: {
            bp_cmd_set_pid_t pid;
            if (bp_parse_cmd_set_pid(data, data_len, &pid)) {
                if (pid.motor_id == BP_MOTOR_1 || pid.motor_id == BP_MOTOR_BOTH) {
                    /* 调用 PID 更新函数（实际项目中替换） */
                    /* Controller_SetSpeedPID(&ctrl[0], pid.kp, pid.ki, pid.kd); */
                }
            }
            break;
        }

        case BP_CMD_SET_PID_BOTH: {
            bp_cmd_set_pid_both_t pid;
            if (bp_parse_cmd_set_pid_both(data, data_len, &pid)) {
                /* 同时设置两个电机的 PID（共用同一组参数） */
                /* Controller_SetSpeedPID(&ctrl[0], pid.kp, pid.ki, pid.kd); */
                /* Controller_SetSpeedPID(&ctrl[1], pid.kp, pid.ki, pid.kd); */
            }
            break;
        }

        case BP_CMD_CONTROL: {
            bp_cmd_control_t ctrl;
            if (bp_parse_cmd_control(data, data_len, &ctrl)) {
                uint8_t m = (ctrl.motor_id == BP_MOTOR_2) ? 1 : 0;
                switch (ctrl.ctrl_cmd) {
                    case BP_CTRL_ENABLE:
                        g_motor[m].state = BP_STATE_RUNNING;
                        break;
                    case BP_CTRL_DISABLE:
                        g_motor[m].state = BP_STATE_IDLE;
                        g_motor[m].pwm_output = 0;
                        break;
                    case BP_CTRL_EMERGENCY:
                        g_motor[m].state = BP_STATE_EMERGENCY;
                        g_motor[m].pwm_output = 0;
                        break;
                    case BP_CTRL_CLEAR_FAULT:
                        g_motor[m].state = BP_STATE_IDLE;
                        g_motor[m].fault = 0;
                        break;
                    case BP_CTRL_HOME:
                        g_motor[m].encoder_total = 0;
                        g_motor[m].actual_angle = 0.0f;
                        break;
                }
            }
            break;
        }

        case BP_CMD_REQ_STATUS: {
            /* 上位机请求状态，立即发送一帧 STATUS */
            bp_rsp_status_t status;
            status.motor_id      = BP_MOTOR_1;
            status.mode_state    = bp_make_mode_state(g_motor[0].state, g_motor[0].mode);
            status.actual_speed  = g_motor[0].actual_speed;
            status.actual_angle  = g_motor[0].actual_angle;
            status.target_speed  = g_motor[0].target_speed;
            status.target_angle  = g_motor[0].target_angle;
            status.pwm_output    = g_motor[0].pwm_output;
            status.encoder_total = g_motor[0].encoder_total;
            status.fault         = g_motor[0].fault;

            /* 中断中发送：使用短暂超时（2ms），避免阻塞控制节拍 */
            bp_hal_uart_send_status(&g_bp_uart, &status, 2);
            break;
        }

        case BP_CMD_HEARTBEAT: {
            /* 心跳应答：发送 ACK */
            bp_hal_uart_send_ack(&g_bp_uart, BP_CMD_HEARTBEAT, 0, 2);
            break;
        }

        case BP_CMD_SET_VOFA: {
            bp_cmd_set_vofa_t vofa;
            if (bp_parse_cmd_set_vofa(data, data_len, &vofa)) {
                /* 设置 JustFloat 输出频率（如 interval_ms = 5 表示 200Hz） */
                /* g_vofa_interval_ms = vofa.interval_ms; */
            }
            break;
        }

        default:
            /* 未知命令，静默丢弃 */
            break;
    }
}

/* ===================================================================
 * HAL 中断回调转发
 * =================================================================== */

/**
 * @brief UART DMA 半传输完成回调
 * 位置：main.c 或 stm32f1xx_it.c 中的 HAL_UART_RxHalfCpltCallback
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_bp_uart.huart) {
        bp_hal_uart_rx_half_callback(&g_bp_uart);
    }
}

/**
 * @brief UART DMA 传输完成回调
 * 位置：main.c 或 stm32f1xx_it.c 中的 HAL_UART_RxCpltCallback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_bp_uart.huart) {
        bp_hal_uart_rx_cplt_callback(&g_bp_uart);
    }
}

/**
 * @brief UART 错误回调
 * 位置：main.c 或 stm32f1xx_it.c 中的 HAL_UART_ErrorCallback
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_bp_uart.huart) {
        bp_hal_uart_error_callback(&g_bp_uart);
    }
}

/* ===================================================================
 * main() 中的初始化和主循环
 * =================================================================== */

/* 以下代码应插入到你的 main() 函数中 */

void example_init(void)
{
    /* 1. 初始化电机状态（实际项目中由 Controller_Init 完成） */
    memset(g_motor, 0, sizeof(g_motor));

    /* 2. 初始化协议驱动：绑定 UART2、设置帧接收回调 */
    bp_hal_uart_init(&g_bp_uart, &huart2, on_frame_received);
}

void example_main_loop(void)
{
    static uint32_t last_status_tick = 0;
    uint32_t now = HAL_GetTick();

    /* 每 100ms 发送一次电机 1 的 STATUS 帧（10 Hz） */
    if (now - last_status_tick >= 100) {
        last_status_tick = now;

        bp_rsp_status_t status;
        status.motor_id      = BP_MOTOR_1;
        status.mode_state    = bp_make_mode_state(g_motor[0].state, g_motor[0].mode);
        status.actual_speed  = g_motor[0].actual_speed;
        status.actual_angle  = g_motor[0].actual_angle;
        status.target_speed  = g_motor[0].target_speed;
        status.target_angle  = g_motor[0].target_angle;
        status.pwm_output    = g_motor[0].pwm_output;
        status.encoder_total = g_motor[0].encoder_total;
        status.fault         = g_motor[0].fault;

        /* 主循环中发送，使用 10ms 超时 */
        bp_hal_uart_send_status(&g_bp_uart, &status, 10);
    }
}

/* ===================================================================
 * 手动组帧发送示例（不通过快捷封装函数）
 * =================================================================== */

void example_manual_build(void)
{
    bp_builder_t builder;
    uint8_t *frame = NULL;

    /* 示例：手动打包一个 "使能电机1" 的控制帧 */
    bp_cmd_control_t ctrl = {
        .motor_id  = BP_MOTOR_1,
        .ctrl_cmd  = BP_CTRL_ENABLE
    };
    uint8_t len = bp_build_cmd_control(&builder, &ctrl, &frame);

    /* frame 指向 builder 内部缓冲区，len 为完整帧长度 */
    if (len > 0 && frame != NULL) {
        /* 可以在此观察帧内容，或发送出去 */
        /* HAL_UART_Transmit(&huart2, frame, len, 100); */
    }
}
