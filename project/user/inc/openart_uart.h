#ifndef _openart_uart_h_
#define _openart_uart_h_

#include "zf_common_typedef.h"
#include "map_types.h"

typedef struct
{
    uint16 car_col_q;
    uint16 car_row_q;
    uint16 box_col_q;
    uint16 box_row_q;
} openart_observation_sample_struct;

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
 * 从 ISR 投递的环形缓冲中取出字节，按 `MAP_BEGIN`/12 行地图/
 * `PLAYER_CENTER_GRID <col_q>,<row_q> <valid>`/`MAP_END` 协议原子更新最近完整地图。
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
 * OpenART 输入使用 `#`/`.`/`B`/`T`/`C`/`+`/`X`；MCU 保持 OpenART 的 `+` 不变。
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
 * @brief 查找最近一帧 OpenART 地图中的唯一小车格子 `C/+`。
 * @param[out] row 唯一小车所在行；未找到或多于一个时为 0xFF。
 * @param[out] col 唯一小车所在列；未找到或多于一个时为 0xFF。
 * @param[out] count 地图中 `C/+` 的数量，可传 NULL。
 * @return 1 表示恰好找到一个 `C/+`，0 表示无有效地图、没有小车或存在多个小车。
 */
uint8 openart_find_player_cell(uint8 *row, uint8 *col, uint8 *count);

/**
 * @brief 获取最近完整地图帧中的配套小车中心字段。
 *
 * `valid=1` 时 MCU 已校验中心和该帧唯一 `C/+` 位于同一格；中心行缺失或
 * `valid=0` 时地图仍可用，但本接口返回无效中心。
 *
 * @param[out] col_q 列坐标，单位为 1/100 格，可传 NULL。
 * @param[out] row_q 行坐标，单位为 1/100 格，可传 NULL。
 * @param[out] valid 1 表示当前样本有效；0 表示无有效中心点。
 * @return 配套地图帧号；只在完整地图成功发布时递增。
 */
uint32 openart_get_player_center(uint16 *col_q, uint16 *row_q, uint8 *valid);

/**
 * @brief 请求 OpenART 临时开启中心识别并逐帧返回多帧有效精确中心。
 * @note 会清除上一轮请求样本和配套地图，并通过 UART1 发送一次 `CENTER_REQ`；调用方负责停车等待。
 */
void openart_request_player_center(void);

/**
 * @brief 按接收顺序弹出当前请求的一条中心样本。
 * @param[out] col_q 中心列坐标，单位 1/100 格，可传 NULL。
 * @param[out] row_q 中心行坐标，单位 1/100 格，可传 NULL。
 * @param[out] yaw_q ART yaw，单位 0.01 degree，可传 NULL。
 * @param[out] yaw_valid 1 表示 yaw 有效；0 表示本帧只有中心，可传 NULL。
 * @return 样本序号 1..ART_CENTER_SAMPLE_COUNT；0 表示当前没有待取样本。
 */
uint8 openart_get_requested_center_sample(uint16 *col_q, uint16 *row_q,
                                          uint16 *yaw_q, uint8 *yaw_valid);

/**
 * @brief 获取当前 CENTER_REQ 最近一条已接受样本的配套完整地图。
 * @return 配套地图快照；本轮尚无通过一致性校验的样本时返回 NULL。
 * @note 快照不会被普通周期地图覆盖，但下一次 CENTER_REQ 会使其失效。
 */
const map_source_struct *openart_get_requested_center_map(void);

/** 请求 ART1 返回指定箱子格对应的多帧小车中心与箱子中心。 */
void openart_request_observation(uint8 box_row, uint8 box_col);

/** 弹出当前观察请求的一条样本；每条样本均配套一张新的规范地图。 */
uint8 openart_get_observation_sample(openart_observation_sample_struct *sample);

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
