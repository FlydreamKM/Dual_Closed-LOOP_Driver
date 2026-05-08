/**
 * @file    bp_parser.c
 * @brief   二进制通信协议通用字节流解析器实现
 *
 * 状态机流程：
 *   WAIT_SOF0 -> WAIT_SOF1 -> WAIT_LEN -> RECV_PAYLOAD -> (完成或出错) -> WAIT_SOF0
 *
 * 错误恢复策略：
 *   - LEN 超限：立即丢弃，回到 WAIT_SOF0
 *   - 校验和错误：丢弃当前帧，回到 WAIT_SOF0
 *   - 在任意状态下收到 0xAA，都会优先进入 WAIT_SOF1（处理连续帧头）
 */

#include "bp_parser.h"
#include <string.h>

/* ===================================================================
 * 初始化与复位
 * =================================================================== */

void bp_parser_init(bp_parser_t *parser, bp_parser_callback_t callback)
{
    memset(parser, 0, sizeof(bp_parser_t));
    parser->state = BP_STATE_WAIT_SOF0;
    parser->callback = callback;
}

void bp_parser_reset(bp_parser_t *parser)
{
    parser->state = BP_STATE_WAIT_SOF0;
    parser->frame_len = 0;
    parser->recv_idx = 0;
}

/* ===================================================================
 * 单字节处理（状态机核心）
 * =================================================================== */

bp_parser_result_t bp_parser_feed(bp_parser_t *parser, uint8_t byte)
{
    switch (parser->state) {

        /* -----------------------------------------------------------
         * State 0: 等待帧头首字节 0xAA
         * ----------------------------------------------------------- */
        case BP_STATE_WAIT_SOF0:
            if (byte == BP_SOF0) {
                parser->state = BP_STATE_WAIT_SOF1;
            }
            /* 不是 0xAA 则继续等待 */
            break;

        /* -----------------------------------------------------------
         * State 1: 等待帧头次字节 0x55
         * ----------------------------------------------------------- */
        case BP_STATE_WAIT_SOF1:
            if (byte == BP_SOF1) {
                parser->state = BP_STATE_WAIT_LEN;
            } else if (byte == BP_SOF0) {
                /* 再次收到 0xAA，保持在当前状态（处理连续 0xAA 0xAA... 的情况） */
                parser->state = BP_STATE_WAIT_SOF1;
            } else {
                /* 帧头不匹配，回到初始状态 */
                parser->state = BP_STATE_WAIT_SOF0;
            }
            break;

        /* -----------------------------------------------------------
         * State 2: 等待长度字段 LEN
         * ----------------------------------------------------------- */
        case BP_STATE_WAIT_LEN:
            parser->frame_len = byte;
            if (parser->frame_len > BP_MAX_DATA_LEN) {
                /* 长度超限，丢弃并重新同步 */
                parser->state = BP_STATE_WAIT_SOF0;
                return BP_RESULT_ERROR_LEN;
            }
            parser->recv_idx = 0;
            parser->state = BP_STATE_RECV_PAYLOAD;
            break;

        /* -----------------------------------------------------------
         * State 3: 接收 CMD + DATA + CHK
         * ----------------------------------------------------------- */
        case BP_STATE_RECV_PAYLOAD:
            /*
             * 需要接收的字节数 = frame_len (CMD+DATA) + 1 (CHK)
             * rx_buf 中按顺序存储：buf[0]=CMD, buf[1..frame_len-1]=DATA, buf[frame_len]=CHK
             */
            if (parser->recv_idx < BP_MAX_FRAME_SIZE) {
                parser->rx_buf[parser->recv_idx++] = byte;
            }

            /* 检查是否收完 */
            if (parser->recv_idx >= parser->frame_len + 1) {
                /* 收完了一帧，进行校验 */
                uint8_t calc_chk = bp_calc_checksum(parser->rx_buf, parser->frame_len);
                uint8_t recv_chk = parser->rx_buf[parser->frame_len];

                if (calc_chk == recv_chk) {
                    /* 校验通过，提取 CMD 并通过回调通知上层 */
                    uint8_t cmd = parser->rx_buf[0];
                    if (parser->callback != NULL) {
                        /*
                         * 回调参数说明：
                         *   cmd      = rx_buf[0]（命令码）
                         *   data     = &rx_buf[0]（CMD + DATA 的首地址）
                         *   data_len = frame_len（= CMD长度1 + DATA长度）
                         */
                        parser->callback(cmd, parser->rx_buf, parser->frame_len);
                    }
                    parser->state = BP_STATE_WAIT_SOF0;
                    return BP_RESULT_OK;
                } else {
                    /* 校验和错误，丢弃当前帧 */
                    parser->state = BP_STATE_WAIT_SOF0;
                    return BP_RESULT_ERROR_CHK;
                }
            }
            break;

        default:
            /* 防御性编程：未知状态强制复位 */
            parser->state = BP_STATE_WAIT_SOF0;
            break;
    }

    /* 如果在 RECV_PAYLOAD 状态下意外收到 0xAA，可能是下一帧的 SOF0 */
    if (parser->state == BP_STATE_RECV_PAYLOAD && byte == BP_SOF0) {
        /*
         * 这是一个设计抉择：
         *   当前帧还没收完就收到 0xAA，可能是当前 DATA 中恰好有 0xAA（正常情况），
         *   也可能是前一帧已损坏、0xAA 是真正的下一帧开头。
         *   我们选择保守策略：继续接收当前帧，让状态机按 LEN 正常走完。
         *   如果当前帧 CHK 错误，会自动回到 WAIT_SOF0 并识别这个 0xAA。
         */
    }

    return BP_RESULT_NONE;
}

/* ===================================================================
 * 批量喂入（内部逐字节调用单字节处理）
 * =================================================================== */

void bp_parser_feed_bytes(bp_parser_t *parser, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        bp_parser_feed(parser, data[i]);
    }
}
