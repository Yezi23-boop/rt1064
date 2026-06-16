#ifndef _openart_uart_h_
#define _openart_uart_h_

#include "zf_common_typedef.h"
#include "map_types.h"

/**
 * @brief 初始化 OpenART 地图接收状态。
 *
 * UART1 硬件由 `debug_init()` 初始化；本函数只清协议状态和接收缓冲。
 *
 * @note 应在主循环开始解析 OpenART 数据前调用一次；不会重新配置 UART 波特率或引脚。
 */
void openart_uart_init(void);

/**
 * @brief 在主循环中解析 OpenART 地图帧。
 *
 * 从 ISR 投递的环形缓冲中取出字节，按 `MAP_BEGIN`/12 行地图/`MAP_END` 协议更新
 * 最近完整地图快照。
 *
 * @note 必须在主循环中高频调用，避免环形缓冲被 UART ISR 写满；不要在 ISR 中调用。
 */
void openart_uart_poll(void);

/**
 * @brief 检查是否收到过完整地图。
 *
 * @return 1 表示至少成功接收一帧 12x16 地图；0 表示尚无有效快照。
 */
uint8 openart_map_ready(void);

/**
 * @brief 获取最近一次收到的完整地图。
 *
 * 地图使用 RT 字符：# 墙，. 空地，B 箱子，T 目标点，C 小车，X 炸弹/障碍。
 *
 * @return 指向模块内部只读快照；尚无有效地图时返回 NULL。
 *
 * @note 返回指针在下一帧成功接收后内容会更新，调用方若要长期使用必须自行复制。
 */
const map_source_struct *openart_map_get(void);

/**
 * @brief 获取最近一次解析到 OpenART 文本行的时间戳，单位 ms。
 *
 * @return `time_ms()` 同一时间基准下的毫秒值；从未解析到文本行时返回 0。
 */
uint32 openart_last_rx_ms(void);

/**
 * @brief 获取成功接收的完整地图帧数。
 *
 * @return 自 `openart_uart_init()` 后累计接受的完整帧数量。
 */
uint32 openart_uart_get_frame_count(void);

/**
 * @brief 丢弃 UART 环形缓冲中尚未解析的字节，并重置当前半帧解析状态。
 *
 * 已接收完成的最近一帧地图仍然保留；本函数只用于等待后续新帧前清理旧串口数据。
 *
 * @note 会短暂关闭全局中断以同步 ISR 写指针和主循环读指针；不要在高频路径中滥用。
 */
void openart_uart_discard_pending(void);

/**
 * @brief 查找最近一帧 OpenART 地图中的小车格子 `C`。
 * @param[out] row  唯一 `C` 所在行；未找到或多于一个时返回第一个 `C` 或 0。
 * @param[out] col  唯一 `C` 所在列；未找到或多于一个时返回第一个 `C` 或 0。
 * @param[out] count 地图中 `C` 的数量，可传 NULL。
 * @return 1 表示恰好找到一个 `C`，0 表示无有效地图、没有 `C` 或存在多个 `C`。
 */
uint8 openart_find_player_cell(uint8 *row, uint8 *col, uint8 *count);

/**
 * @brief 从 UART1 ISR 投递一个接收字节。
 *
 * @param[in] data UART1 收到的原始字节。
 *
 * ISR 只写入接收环形缓冲；实际协议解析在 `openart_uart_poll()` 中完成。
 *
 * @note 环形缓冲满时会丢弃当前字节并累计溢出计数，以保证 ISR 不阻塞。
 */
void openart_uart_push_byte(uint8 data);

#endif
