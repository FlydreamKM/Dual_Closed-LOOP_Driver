/**
 * @file    bp_hal_uart.h
 * @brief   二进制通信协议 HAL UART 适配层
 *
 * 本文件是整个驱动中**唯一引用 STM32 HAL 头文件**的地方。
 * 如果你不使用 HAL（如使用标准库、LL 库、或其他 MCU），
 * 只需舍弃本文件，用 bp_parser.h + bp_frame.h 自行实现底层接口即可。
 *
 * 设计要点：
 *   1. 采用 UART + DMA Circular 模式接收，CPU 零拷贝
 *   2. 半传输中断 + 传输完成中断双触发，确保低延迟
 *   3. 发送采用阻塞方式（适合低频响应帧），也可自行扩展为 DMA 发送
 */

#ifndef BP_HAL_UART_H
#define BP_HAL_UART_H

#include "bp_def.h"
#include "bp_parser.h"
#include "bp_frame.h"
#include "stm32f1xx_hal.h"  /* 驱动中唯一引用 HAL 的地方 */

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * 配置常量
 * =================================================================== */

#define BP_HAL_RX_BUF_SIZE  256     /**< UART DMA 接收缓冲区大小（必须是 2 的幂，便于环形处理） */

/* ===================================================================
 * HAL UART 适配器句柄
 * =================================================================== */

typedef struct {
    UART_HandleTypeDef *huart;          /**< HAL UART 句柄，由用户传入 */
    bp_parser_t         parser;         /**< 协议解析器 */
    bp_builder_t        builder;        /**< 帧打包器（复用，避免栈分配） */
    uint8_t             rx_dma_buf[BP_HAL_RX_BUF_SIZE]; /**< DMA 循环接收缓冲区 */
    uint16_t            rx_read_idx;    /**< 应用层已读取位置 */
} bp_hal_uart_t;

/* ===================================================================
 * 生命周期函数
 * =================================================================== */

/**
 * @brief 初始化 HAL UART 适配器并启动 DMA 循环接收
 *
 * 调用本函数后，UART 立即进入 DMA Circular 接收模式，
 * 所有收到的字节会自动进入 bp_parser 状态机处理。
 *
 * @param bp    适配器句柄（由用户分配内存，可为全局变量或堆内存）
 * @param huart HAL UART 句柄（如 &huart2）
 * @param callback  协议帧解析成功后的回调（参见 bp_parser.h）
 */
void bp_hal_uart_init(bp_hal_uart_t *bp, UART_HandleTypeDef *huart, bp_parser_callback_t callback);

/**
 * @brief 停止 DMA 接收并复位状态
 */
void bp_hal_uart_deinit(bp_hal_uart_t *bp);

/* ===================================================================
 * 发送接口
 * =================================================================== */

/**
 * @brief 发送一帧（阻塞方式，带超时）
 *
 * 适用于低频场景（如 STATUS 周期上报、ACK 应答）。
 * 若需高频发送，建议自行改用 DMA + 双缓冲模式。
 *
 * @param bp      适配器句柄
 * @param frame   帧数据首地址（完整帧，含 SOF0/SOF1/LEN/CMD/DATA/CHK）
 * @param len     帧长度
 * @param timeout HAL 超时时间（毫秒）
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef bp_hal_uart_send(bp_hal_uart_t *bp, const uint8_t *frame, uint8_t len, uint32_t timeout);

/**
 * @brief 发送上行 STATUS 帧（快捷封装）
 */
HAL_StatusTypeDef bp_hal_uart_send_status(bp_hal_uart_t *bp, const bp_rsp_status_t *status, uint32_t timeout);

/**
 * @brief 发送上行 ACK 帧（快捷封装）
 */
HAL_StatusTypeDef bp_hal_uart_send_ack(bp_hal_uart_t *bp, uint8_t cmd, uint8_t result, uint32_t timeout);

/* ===================================================================
 * 中断回调（必须在对应 HAL 回调中调用）
 * =================================================================== */

/**
 * @brief 在 HAL_UART_RxHalfCpltCallback 中调用
 *
 * DMA 半传输完成时触发，表示 rx_dma_buf 的前半区（0 ~ BP_HAL_RX_BUF_SIZE/2 - 1）
 * 已收满新数据。
 */
void bp_hal_uart_rx_half_callback(bp_hal_uart_t *bp);

/**
 * @brief 在 HAL_UART_RxCpltCallback 中调用
 *
 * DMA 传输完成时触发（Circular 模式下表示整个缓冲区绕回），
 * 表示 rx_dma_buf 的后半区（BP_HAL_RX_BUF_SIZE/2 ~ BP_HAL_RX_BUF_SIZE - 1）
 * 已收满新数据。
 */
void bp_hal_uart_rx_cplt_callback(bp_hal_uart_t *bp);

/**
 * @brief 在 HAL_UART_ErrorCallback 中调用
 *
 * UART 发生错误（如帧错误、噪声、溢出）时复位 DMA 接收。
 */
void bp_hal_uart_error_callback(bp_hal_uart_t *bp);

#ifdef __cplusplus
}
#endif

#endif /* BP_HAL_UART_H */
