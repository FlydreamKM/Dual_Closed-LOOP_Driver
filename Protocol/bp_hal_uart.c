/**
 * @file    bp_hal_uart.c
 * @brief   二进制通信协议 HAL UART 适配层实现
 *
 * 核心机制：UART + DMA Circular 双缓冲接收
 *   - DMA 将接收到的字节循环写入 rx_dma_buf[BP_HAL_RX_BUF_SIZE]
 *   - 半传输中断（HT）：前半区满，处理 buf[0] ~ buf[size/2 - 1]
 *   - 传输完成中断（TC）：后半区满，处理 buf[size/2] ~ buf[size - 1]
 *   - 应用层维护 rx_read_idx，避免与 DMA 写指针冲突
 *
 * 线程安全说明：
 *   - 解析器状态机（bp_parser）在 UART 中断上下文运行，必须保持轻量
 *   - 回调函数也在中断上下文执行，若需与主循环共享数据，请使用 volatile + 临界区保护
 */

#include "bp_hal_uart.h"
#include <string.h>

/* ===================================================================
 * 内部辅助：处理接收缓冲区中尚未解析的字节
 * =================================================================== */

/**
 * @brief 处理 rx_dma_buf 中 read_idx 到 write_idx 之间的数据
 * @param bp 适配器句柄
 */
static void _bp_hal_process_rx(bp_hal_uart_t *bp)
{
    /* 获取 DMA 当前写位置（剩余未传输字节数 = CNDTR） */
    uint16_t write_idx = BP_HAL_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(bp->huart->hdmarx);
    uint16_t read_idx = bp->rx_read_idx;

    /* 处理 read_idx 到 write_idx 之间的所有字节 */
    while (read_idx != write_idx) {
        uint8_t byte = bp->rx_dma_buf[read_idx];
        bp_parser_feed(&bp->parser, byte);
        read_idx = (read_idx + 1) & (BP_HAL_RX_BUF_SIZE - 1);
    }

    bp->rx_read_idx = write_idx;
}

/* ===================================================================
 * 生命周期
 * =================================================================== */

void bp_hal_uart_init(bp_hal_uart_t *bp, UART_HandleTypeDef *huart, bp_parser_callback_t callback)
{
    memset(bp, 0, sizeof(bp_hal_uart_t));
    bp->huart = huart;
    bp->rx_read_idx = 0;

    /* 初始化解析器和打包器 */
    bp_parser_init(&bp->parser, callback);
    bp_builder_init(&bp->builder);

    /* 启动 DMA Circular 接收 */
    HAL_UART_Receive_DMA(huart, bp->rx_dma_buf, BP_HAL_RX_BUF_SIZE);
}

void bp_hal_uart_deinit(bp_hal_uart_t *bp)
{
    if (bp->huart != NULL) {
        HAL_UART_DMAStop(bp->huart);
    }
    bp_parser_reset(&bp->parser);
    bp->rx_read_idx = 0;
}

/* ===================================================================
 * 发送接口
 * =================================================================== */

HAL_StatusTypeDef bp_hal_uart_send(bp_hal_uart_t *bp, const uint8_t *frame, uint8_t len, uint32_t timeout)
{
    if (bp->huart == NULL || frame == NULL || len == 0) {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit(bp->huart, (uint8_t *)frame, len, timeout);
}

HAL_StatusTypeDef bp_hal_uart_send_status(bp_hal_uart_t *bp, const bp_rsp_status_t *status, uint32_t timeout)
{
    uint8_t *frame = NULL;
    uint8_t len = bp_build_rsp_status(&bp->builder, status, &frame);
    if (len == 0 || frame == NULL) {
        return HAL_ERROR;
    }
    return bp_hal_uart_send(bp, frame, len, timeout);
}

HAL_StatusTypeDef bp_hal_uart_send_ack(bp_hal_uart_t *bp, uint8_t cmd, uint8_t result, uint32_t timeout)
{
    bp_rsp_ack_t ack = { .cmd = cmd, .result = result };
    uint8_t *frame = NULL;
    uint8_t len = bp_build_rsp_ack(&bp->builder, &ack, &frame);
    if (len == 0 || frame == NULL) {
        return HAL_ERROR;
    }
    return bp_hal_uart_send(bp, frame, len, timeout);
}

/* ===================================================================
 * 中断回调（用户需在对应 HAL 回调中转发）
 * =================================================================== */

void bp_hal_uart_rx_half_callback(bp_hal_uart_t *bp)
{
    /*
     * HAL 在半传输完成时调用此函数。
     * 对于 Circular DMA，此时 DMA 正在写后半区，前半区数据已稳定可读。
     * 直接处理全部未读数据即可（_bp_hal_process_rx 内部会自动判断 write_idx）。
     */
    _bp_hal_process_rx(bp);
}

void bp_hal_uart_rx_cplt_callback(bp_hal_uart_t *bp)
{
    /*
     * HAL 在传输完成（缓冲区绕回）时调用此函数。
     * 此时 DMA 已从头开始覆盖 buf[0]，后半区数据已稳定可读。
     * 同样调用 _bp_hal_process_rx 处理所有未读字节。
     */
    _bp_hal_process_rx(bp);
}

void bp_hal_uart_error_callback(bp_hal_uart_t *bp)
{
    /* UART 错误（帧错误、噪声、溢出）时，停止并重启 DMA 接收 */
    HAL_UART_DMAStop(bp->huart);
    bp_parser_reset(&bp->parser);
    bp->rx_read_idx = 0;
    HAL_UART_Receive_DMA(bp->huart, bp->rx_dma_buf, BP_HAL_RX_BUF_SIZE);
}
