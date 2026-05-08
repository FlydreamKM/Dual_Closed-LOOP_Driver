/**
 * @file    bp_parser.h
 * @brief   二进制通信协议通用字节流解析器
 *
 * 设计目标：
 *   1. 纯 C 实现，零平台依赖（除 bp_def.h 和标准 C 库外不引用任何头文件）
 *   2. 采用字节级状态机，支持从任意字节边界开始同步
 *   3. 单字节喂入（bp_parser_feed）和多字节批量喂入（bp_parser_feed_bytes）双接口
 *   4. 解析成功后通过回调函数通知上层，上层无需轮询状态
 *
 * 使用方式：
 *   1. 定义回调函数：void my_callback(uint8_t cmd, const uint8_t *data, uint8_t len)
 *   2. 初始化：bp_parser_init(&parser, my_callback)
 *   3. 在 UART 中断或 DMA 完成回调中喂入数据：
 *      bp_parser_feed(&parser, rx_byte);
 *      或
 *      bp_parser_feed_bytes(&parser, dma_buf, dma_len);
 */

#ifndef BP_PARSER_H
#define BP_PARSER_H

#include "bp_def.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * 解析器状态枚举
 * =================================================================== */

typedef enum {
    BP_STATE_WAIT_SOF0 = 0,     /**< 等待帧头首字节 0xAA */
    BP_STATE_WAIT_SOF1,         /**< 等待帧头次字节 0x55 */
    BP_STATE_WAIT_LEN,          /**< 等待长度字段 LEN */
    BP_STATE_RECV_PAYLOAD,      /**< 接收 CMD + DATA + CHK */
} bp_parser_state_t;

/* ===================================================================
 * 解析结果枚举
 * =================================================================== */

typedef enum {
    BP_RESULT_NONE = 0,         /**< 当前字节处理完毕，暂无完整帧 */
    BP_RESULT_OK,               /**< 成功解析出一帧有效数据 */
    BP_RESULT_ERROR_LEN,        /**< 长度字段超限（LEN > 60），帧被丢弃 */
    BP_RESULT_ERROR_CHK,        /**< 校验和错误，帧被丢弃 */
} bp_parser_result_t;

/* ===================================================================
 * 帧接收回调函数类型
 *
 * @param cmd       命令码/响应码（如 0x01, 0x81）
 * @param data      指向 CMD + DATA 的首地址（不含 SOF/LEN/CHK）
 * @param data_len  data 的字节数（= LEN）
 *
 * 注意：data 指向的是解析器内部缓冲区的地址，回调返回后该内存可能被复用。
 *       如果上层需要长期保存，请在回调内自行 memcpy。
 * =================================================================== */
typedef void (*bp_parser_callback_t)(uint8_t cmd, const uint8_t *data, uint8_t data_len);

/* ===================================================================
 * 解析器句柄
 * =================================================================== */

typedef struct {
    bp_parser_state_t    state;                     /**< 当前状态机状态 */
    uint8_t              frame_len;                 /**< LEN 字段值（CMD+DATA 长度） */
    uint8_t              recv_idx;                  /**< 当前已接收的 CMD+DATA+CHK 字节数 */
    uint8_t              rx_buf[BP_MAX_FRAME_SIZE]; /**< 内部接收缓冲区，存 CMD+DATA+CHK */
    bp_parser_callback_t callback;                  /**< 帧解析成功回调 */
} bp_parser_t;

/* ===================================================================
 * API 函数
 * =================================================================== */

/**
 * @brief 初始化解析器
 * @param parser    解析器句柄
 * @param callback  帧解析成功后的回调函数（不能为 NULL）
 */
void bp_parser_init(bp_parser_t *parser, bp_parser_callback_t callback);

/**
 * @brief 向解析器喂入单个字节
 * @param parser 解析器句柄
 * @param byte   接收到的字节
 * @return       解析结果
 *
 * 典型调用场景：
 *   - UART 中断中（每收到一个字节调用一次）
 *   - 从 DMA 缓冲区逐字节遍历
 */
bp_parser_result_t bp_parser_feed(bp_parser_t *parser, uint8_t byte);

/**
 * @brief 向解析器批量喂入多个字节
 * @param parser 解析器句柄
 * @param data   数据首地址
 * @param len    数据长度
 *
 * 典型调用场景：
 *   - DMA 半传输/传输完成中断中，一次性处理半缓冲区的数据
 *   - 从文件或网络 socket 读取的数据块
 */
void bp_parser_feed_bytes(bp_parser_t *parser, const uint8_t *data, uint16_t len);

/**
 * @brief 重置解析器到初始状态
 * @param parser 解析器句柄
 *
 * 当通信链路出现严重错误（如波特率切换、长时间丢包）时，
 * 可调用此函数强制复位状态机，避免陷入死状态。
 */
void bp_parser_reset(bp_parser_t *parser);

#ifdef __cplusplus
}
#endif

#endif /* BP_PARSER_H */
