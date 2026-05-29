#ifndef _openart_uart_h_
#define _openart_uart_h_

#include "zf_common_typedef.h"
#include "map_types.h"

/**
 * @file openart_uart.h
 * @brief OpenART UART 解析模块公开接口。
 *
 * 从专用 UART 接收 OpenART 发送的结构化消息，包括地图、小车位置、
 * 箱子位置、目标位置和状态文本。协议为行文本格式，每行以 '\n' 结尾。
 *
 * 协议格式：
 *   MAP_BEGIN\n
 *   XXXXXXXXXXXXXXXX\n   （12 行，每行 16 字符，字符集：#.TBXC-*）
 *   MAP_END\n
 *   POS:row,col\n         （小车当前位置，十进制整数）
 *   BOX:r1,c1,r2,c2,...\n （箱子位置列表，十进制整数对）
 *   TGT:r1,c1,r2,c2,...\n （目标位置列表，十进制整数对）
 *   STS:text\n            （状态文本，最长 OPENART_STS_MAX_LEN）
 */

/** 单行最大缓存长度，含结尾 '\0'。 */
#define OPENART_LINE_SIZE       (64u)
/** 行间超时，超过则丢弃当前帧，单位 ms。 */
#define OPENART_RX_TIMEOUT_MS   (1000u)
/** 单次从 FIFO 取出的最大字节数。 */
#define OPENART_RX_CHUNK_SIZE   (32u)
/** 状态文本最大长度，含结尾 '\0'。 */
#define OPENART_STS_MAX_LEN     (32u)
/** 最大箱子/目标位置数。 */
#define OPENART_MAX_POSITIONS   (8u)

/**
 * @brief OpenART 接收到的消息类型。
 */
typedef enum
{
    OPENART_MSG_NONE = 0,      /**< 无新消息。 */
    OPENART_MSG_MAP,           /**< 完整地图已接收。 */
    OPENART_MSG_POSITION,      /**< 小车位置已接收。 */
    OPENART_MSG_BOXES,         /**< 箱子位置列表已接收。 */
    OPENART_MSG_TARGETS,       /**< 目标位置列表已接收。 */
    OPENART_MSG_STATUS,        /**< 状态文本已接收。 */
} openart_msg_type_enum;

/**
 * @brief OpenART 解析结果。
 *
 * 调用 `openart_uart_poll()` 后通过 `openart_uart_get_result()` 获取。
 * 每次 poll 最多返回一条消息；重复调用 `get_result()` 直到返回
 * `OPENART_MSG_NONE` 可消费所有已解析消息。
 */
typedef struct
{
    openart_msg_type_enum type;              /**< 消息类型。 */

    /* MAP 消息字段 */
    map_source_struct map;                   /**< 地图源，type == OPENART_MSG_MAP 时有效。 */

    /* POSITION 消息字段 */
    uint8 pos_row;                           /**< 小车行号，范围 [0, MAP_ROWS-1]。 */
    uint8 pos_col;                           /**< 小车列号，范围 [0, MAP_COLS-1]。 */

    /* BOXES / TARGETS 消息字段 */
    uint8 positions[OPENART_MAX_POSITIONS];  /**< 位置编码：row * MAP_COLS + col。 */
    uint8 position_count;                    /**< 有效位置数量。 */

    /* STATUS 消息字段 */
    char status_text[OPENART_STS_MAX_LEN];   /**< 状态文本，以 '\0' 结尾。 */
} openart_result_struct;

/**
 * @brief 初始化 OpenART UART 解析模块。
 *
 * 配置接收缓冲区和状态机，准备接收来自 OpenART 的数据。
 * 应在 `app_init()` 中调用。
 */
void openart_uart_init(void);

/**
 * @brief 非阻塞轮询 OpenART UART 数据并推进解析状态机。
 *
 * 从 UART FIFO 读取字节，按行解析协议消息。解析结果通过
 * `openart_uart_get_result()` 获取。
 *
 * @note 在主循环中调用；不要放到 ISR 中。
 */
void openart_uart_poll(void);

/**
 * @brief 取出下一条已解析的消息。
 *
 * 每次调用返回队列中的一条消息，直到返回 `OPENART_MSG_NONE`。
 * 消息被取出后即从队列中移除。
 *
 * @param[out] result 消息内容写入该结构体。
 * @return 消息类型；`OPENART_MSG_NONE` 表示队列已空。
 */
openart_msg_type_enum openart_uart_get_result(openart_result_struct *result);

/**
 * @brief 检查是否收到过完整的 OpenART 地图。
 *
 * @return 1 表示已收到完整地图；0 表示尚未收到。
 */
uint8 openart_map_ready(void);

/**
 * @brief 获取最近一次通过 OpenART 接收到的完整地图。
 *
 * @return 地图源结构体指针；若未收到过完整地图，返回 NULL。
 */
const map_source_struct *openart_map_get(void);

/**
 * @brief 获取最近一次 OpenART 数据接收的时间戳。
 *
 * @return 最后一次收到有效消息的时间，单位 ms；未收到过则返回 0。
 */
uint32 openart_last_rx_ms(void);

/**
 * @brief 向解析器送入一个字节（专用 UART 模式）。
 *
 * 在 ISR 中调用，将 UART 接收到的字节送入解析器。
 * 仅在 OPENART_USE_WIRELESS_UART=0 时使用。
 *
 * @param data 接收到的字节。
 */
void openart_uart_push_byte(uint8 data);

#endif
