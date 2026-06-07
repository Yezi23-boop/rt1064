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
 * @brief 查找最近一帧 OpenART 地图中的小车格子 `C`。
 * @param[out] row  唯一 `C` 所在行；未找到或多于一个时返回第一个 `C` 或 0。
 * @param[out] col  唯一 `C` 所在列；未找到或多于一个时返回第一个 `C` 或 0。
 * @param[out] count 地图中 `C` 的数量，可传 NULL。
 * @return 1 表示恰好找到一个 `C`，0 表示无有效地图、没有 `C` 或存在多个 `C`。
 */
uint8 openart_find_player_cell(uint8 *row, uint8 *col, uint8 *count);

/**
 * @brief 获取最近一帧 OpenART 地图中缓存的小车格子。
 *
 * 与 `openart_find_player_cell()` 返回值一致，但由接收完整地图时预先缓存，
 * 适合控制环只读查询。
 */
uint8 openart_get_player_cell(uint8 *row, uint8 *col, uint8 *count, uint32 *frame);

/**
 * @brief 获取最近一帧 OpenART 地图中缓存的箱子格子集合。
 * @param[out] boxes 输出箱子 cell 数组，长度至少为 MAX_BOXES；可传 NULL 只取数量。
 * @param[out] count 箱子数量，可传 NULL。
 * @param[out] frame 当前地图帧号，可传 NULL。
 * @return 1 表示存在有效 OpenART 地图，0 表示尚无有效地图。
 */
uint8 openart_get_box_cells(uint16 boxes[MAX_BOXES], uint8 *count, uint32 *frame);

/**
 * @brief 从 UART1 ISR 投递一个接收字节。
 *
 * ISR 只写入接收环形缓冲；实际协议解析在 `openart_uart_poll()` 中完成。
 */
void openart_uart_push_byte(uint8 data);

#endif
