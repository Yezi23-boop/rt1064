#ifndef _openart_uart_h_
#define _openart_uart_h_

#include "zf_common_typedef.h"
#include "map_types.h"

/**
 * @brief 初始化 OpenART 地图接收状态。
 *
 * UART1 硬件由 `debug_init()` 初始化；本函数只清协议状态和接收缓冲。
 */
void openart_uart_init(void);

/**
 * @brief 在主循环中解析 OpenART 地图帧。
 */
void openart_uart_poll(void);

/**
 * @brief 检查是否收到过完整地图。
 */
uint8 openart_map_ready(void);

/**
 * @brief 获取最近一次收到的完整地图。
 *
 * 地图使用 RT 字符：# 墙，. 空地，B 箱子，T 目标点，C 小车，X 炸弹/障碍。
 */
const map_source_struct *openart_map_get(void);

/**
 * @brief 获取最近一次收到 OpenART 字节的时间戳，单位 ms。
 */
uint32 openart_last_rx_ms(void);

/**
 * @brief 获取成功接收的完整地图帧数。
 */
uint32 openart_uart_get_frame_count(void);

/**
 * @brief 从 UART1 ISR 投递一个接收字节。
 *
 * ISR 只写入接收环形缓冲；实际协议解析在 `openart_uart_poll()` 中完成。
 */
void openart_uart_push_byte(uint8 data);

#endif
