/**
 * @file openart_uart.c
 * @brief OpenART UART 解析模块实现。
 *
 * 从专用 UART 逐字节读取，按行解析 OpenART 发送的结构化消息。
 * 协议格式：
 *   MAP_BEGIN\n
 *   XXXXXXXXXXXXXXXX\n   （12 行，每行 16 字符）
 *   MAP_END\n
 *   POS:row,col\n
 *   BOX:r1,c1,r2,c2,...\n
 *   TGT:r1,c1,r2,c2,...\n
 *   STS:text\n
 *
 * 状态机：
 *   STATE_IDLE     -> 收到 "MAP_BEGIN" 进入 STATE_RECV_MAP
 *                   -> 收到其他前缀行，解析为 POS/BOX/TGT/STS
 *   STATE_RECV_MAP -> 收到 "MAP_END" 提交地图，回到 STATE_IDLE
 *                   -> 收到非法行或超时则丢弃，回到 STATE_IDLE
 *
 * 消息通过环形队列缓存，主循环通过 openart_uart_get_result() 消费。
 */

#include "zf_common_headfile.h"
#include "openart_uart.h"
#include "timebase.h"

/**
 * UART 数据源选择。
 * 定义 OPENART_USE_WIRELESS_UART 为 1 时从无线串口 FIFO 读取（共享 LPUART8）；
 * 为 0 时从专用 UART 硬件读取（需在 isr.c 中对接 LPUART 中断）。
 */
#ifndef OPENART_USE_WIRELESS_UART
#define OPENART_USE_WIRELESS_UART (0)
#endif

#if OPENART_USE_WIRELESS_UART
#include "zf_device_wireless_uart.h"
#endif

/** 消息队列深度；主循环消费速度远快于 UART 收包速度，4 条足够。 */
#define RESULT_QUEUE_SIZE   (4u)

/* ------------------------------------------------------------------ */
/*  解析器状态机                                                       */
/* ------------------------------------------------------------------ */

typedef enum
{
    PARSE_STATE_IDLE = 0,
    PARSE_STATE_RECV_MAP,
} parse_state_enum;

/** 接收行缓存。 */
static char rx_line_buf[OPENART_LINE_SIZE];
/** 当前行已接收字节数（不含结尾 '\0'）。 */
static uint8 rx_line_len;
/** 解析状态机当前状态。 */
static parse_state_enum rx_state;
/** 地图接收模式下已收到的行号。 */
static uint8 recv_row;
/** 地图行暂存缓冲区，+1 用于结尾 '\0'。 */
static char map_rows_buf[MAP_ROWS][MAP_COLS + 1];
/** 最后一次收到有效数据的时间戳。 */
static uint32 last_rx_ms;

/* ------------------------------------------------------------------ */
/*  地图快照（供 openart_map_get() 访问）                              */
/* ------------------------------------------------------------------ */

static uint8 map_valid;
static map_source_struct openart_map_source;
/** 地图行指针数组副本，避免 map_rows_buf 被后续接收覆盖。 */
static char map_snapshot[MAP_ROWS][MAP_COLS + 1];

/* ------------------------------------------------------------------ */
/*  消息队列                                                           */
/* ------------------------------------------------------------------ */

static openart_result_struct result_queue[RESULT_QUEUE_SIZE];
static uint8 queue_head;
static uint8 queue_tail;
static uint8 queue_count;

static uint8 queue_full(void)
{
    return (queue_count >= RESULT_QUEUE_SIZE) ? 1u : 0u;
}

static void queue_push(const openart_result_struct *item)
{
    if (0 != queue_full())
    {
        /* 队列满时覆盖最旧条目。 */
        queue_tail = (uint8)((queue_tail + 1u) % RESULT_QUEUE_SIZE);
        queue_count--;
    }
    result_queue[queue_head] = *item;
    queue_head = (uint8)((queue_head + 1u) % RESULT_QUEUE_SIZE);
    queue_count++;
}

/* ------------------------------------------------------------------ */
/*  字符串工具                                                         */
/* ------------------------------------------------------------------ */

static uint8 is_map_char(char ch)
{
    return (ch == '#' || ch == '.' || ch == 'T' || ch == 'B' ||
            ch == 'X' || ch == 'C' || ch == '-' || ch == '$' || ch == '*')
           ? 1u : 0u;
}

static uint8 validate_map_line(const char *line)
{
    uint8 i;

    for (i = 0; i < MAP_COLS; i++)
    {
        if ('\0' == line[i])
        {
            return 0;
        }
        if (0 == is_map_char(line[i]))
        {
            return 0;
        }
    }
    return 1;
}

static uint8 prefix_match(const char *line, const char *prefix)
{
    while ('\0' != *prefix)
    {
        if (*line != *prefix)
        {
            return 0;
        }
        line++;
        prefix++;
    }
    return 1;
}

/**
 * @brief 从行首解析一个无符号十进制整数。
 *
 * @param[in,out] cursor 输入当前位置，输出更新到数字之后。
 * @param[out] value 解析结果。
 * @return 1 成功，0 无数字。
 */
static uint8 parse_uint(const char **cursor, uint8 *value)
{
    const char *p = *cursor;
    uint16 result = 0;
    uint8 has_digit = 0;

    /* 跳过前导非数字字符。 */
    while (('\0' != *p) && ((*p < '0') || (*p > '9')))
    {
        p++;
    }

    while (('\0' != *p) && (*p >= '0') && (*p <= '9'))
    {
        result = (uint16)(result * 10u + (uint16)(*p - '0'));
        has_digit = 1;
        p++;
    }

    *cursor = p;
    if ((0 == has_digit) || (result > 255u))
    {
        return 0;
    }
    *value = (uint8)result;
    return 1;
}

/* ------------------------------------------------------------------ */
/*  消息解析                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief 解析 "BOX:..." 或 "TGT:..." 行，提取坐标对列表。
 *
 * @param[in] payload 前缀之后的起始位置。
 * @param[out] positions 位置编码数组。
 * @param[out] count 有效位置数量。
 */
static void parse_position_list(const char *payload, uint8 *positions, uint8 *count)
{
    const char *p = payload;
    uint8 row;
    uint8 col;
    uint8 n = 0;

    while ('\0' != *p)
    {
        if (0 == parse_uint(&p, &row))
        {
            break;
        }
        if (0 == parse_uint(&p, &col))
        {
            break;
        }
        if ((row < MAP_ROWS) && (col < MAP_COLS) && (n < OPENART_MAX_POSITIONS))
        {
            positions[n] = (uint8)(row * MAP_COLS + col);
            n++;
        }
    }
    *count = n;
}

/**
 * @brief 处理一条完整行。
 *
 * 根据当前状态和行内容，更新地图暂存区或向消息队列提交解析结果。
 */
static void process_line(void)
{
    openart_result_struct result;
    const char *payload;

    rx_line_buf[rx_line_len] = '\0';

    if (PARSE_STATE_IDLE == rx_state)
    {
        if (0 == strcmp(rx_line_buf, "MAP_BEGIN"))
        {
            rx_state = PARSE_STATE_RECV_MAP;
            recv_row = 0;
            last_rx_ms = time_ms();
            return;
        }

        /* 非地图消息解析。 */
        memset(&result, 0, sizeof(result));
        last_rx_ms = time_ms();

        if (1 == prefix_match(rx_line_buf, "POS:"))
        {
            payload = rx_line_buf + 4;
            result.type = OPENART_MSG_POSITION;
            if ((0 != parse_uint(&payload, &result.pos_row)) &&
                (0 != parse_uint(&payload, &result.pos_col)) &&
                (result.pos_row < MAP_ROWS) &&
                (result.pos_col < MAP_COLS))
            {
                queue_push(&result);
            }
        }
        else if (1 == prefix_match(rx_line_buf, "BOX:"))
        {
            payload = rx_line_buf + 4;
            result.type = OPENART_MSG_BOXES;
            parse_position_list(payload, result.positions, &result.position_count);
            if (result.position_count > 0)
            {
                queue_push(&result);
            }
        }
        else if (1 == prefix_match(rx_line_buf, "TGT:"))
        {
            payload = rx_line_buf + 4;
            result.type = OPENART_MSG_TARGETS;
            parse_position_list(payload, result.positions, &result.position_count);
            if (result.position_count > 0)
            {
                queue_push(&result);
            }
        }
        else if (1 == prefix_match(rx_line_buf, "STS:"))
        {
            result.type = OPENART_MSG_STATUS;
            strncpy(result.status_text, rx_line_buf + 4, OPENART_STS_MAX_LEN - 1);
            result.status_text[OPENART_STS_MAX_LEN - 1] = '\0';
            queue_push(&result);
        }

        return;
    }

    /* PARSE_STATE_RECV_MAP: 地图接收模式。 */
    last_rx_ms = time_ms();

    if (0 == strcmp(rx_line_buf, "MAP_END"))
    {
        if (recv_row == MAP_ROWS)
        {
            /* 完整地图接收成功，更新快照。 */
            uint8 r;
            for (r = 0; r < MAP_ROWS; r++)
            {
                memcpy(map_snapshot[r], map_rows_buf[r], MAP_COLS + 1);
                openart_map_source.rows[r] = map_snapshot[r];
            }
            openart_map_source.name = "OpenART";
            openart_map_source.format = MAP_FORMAT_RT;
            map_valid = 1;

            /* 向队列提交 MAP 消息。 */
            memset(&result, 0, sizeof(result));
            result.type = OPENART_MSG_MAP;
            result.map = openart_map_source;
            queue_push(&result);

            printf("OPENART_MAP_OK rows=%d\r\n", recv_row);
        }
        else
        {
            printf("OPENART_MAP_INCOMPLETE rows=%d\r\n", recv_row);
        }
        rx_state = PARSE_STATE_IDLE;
        return;
    }

    if (recv_row >= MAP_ROWS)
    {
        rx_state = PARSE_STATE_IDLE;
        return;
    }

    if ((rx_line_len == MAP_COLS) && (0 != validate_map_line(rx_line_buf)))
    {
        memcpy(map_rows_buf[recv_row], rx_line_buf, MAP_COLS);
        map_rows_buf[recv_row][MAP_COLS] = '\0';
        recv_row++;
    }
    else
    {
        printf("OPENART_MAP_BAD_LINE row=%d len=%d\r\n", recv_row, rx_line_len);
        rx_state = PARSE_STATE_IDLE;
    }
}

/**
 * @brief 向解析器送入一个字节。
 *
 * 按 '\r'/'\n' 分行；行内容过长时丢弃当前行并回到 IDLE。
 */
static void push_byte(uint8 data)
{
    if (('\r' == data) || ('\n' == data))
    {
        if (0 != rx_line_len)
        {
            process_line();
            rx_line_len = 0;
        }
        return;
    }

    if (rx_line_len < (OPENART_LINE_SIZE - 1u))
    {
        rx_line_buf[rx_line_len] = (char)data;
        rx_line_len++;
    }
    else
    {
        /* 行过长，丢弃并重置。 */
        rx_line_len = 0;
        rx_state = PARSE_STATE_IDLE;
    }
}

/* ------------------------------------------------------------------ */
/*  公开接口                                                           */
/* ------------------------------------------------------------------ */

void openart_uart_init(void)
{
    rx_line_len = 0;
    rx_state = PARSE_STATE_IDLE;
    recv_row = 0;
    map_valid = 0;
    last_rx_ms = 0;
    memset(map_rows_buf, 0, sizeof(map_rows_buf));
    memset(map_snapshot, 0, sizeof(map_snapshot));
    memset(result_queue, 0, sizeof(result_queue));
    queue_head = 0;
    queue_tail = 0;
    queue_count = 0;
    openart_map_source.name = "OpenART";
    openart_map_source.format = MAP_FORMAT_RT;
}

void openart_uart_poll(void)
{
    uint8 buffer[OPENART_RX_CHUNK_SIZE];
    uint32 length;
    uint32 i;
    uint32 now_ms;

    /* 检查行间超时。 */
    now_ms = time_ms();
    if ((PARSE_STATE_RECV_MAP == rx_state) &&
        (0 != last_rx_ms) &&
        ((now_ms - last_rx_ms) >= OPENART_RX_TIMEOUT_MS))
    {
        printf("OPENART_TIMEOUT row=%d\r\n", recv_row);
        rx_state = PARSE_STATE_IDLE;
        rx_line_len = 0;
    }

    /* 从 UART FIFO 读取字节。 */
#if OPENART_USE_WIRELESS_UART
    do
    {
        length = wireless_uart_read_buffer(buffer, OPENART_RX_CHUNK_SIZE);
        for (i = 0; i < length; i++)
        {
            push_byte(buffer[i]);
        }
    } while (OPENART_RX_CHUNK_SIZE == length);
#else
    /* 专用 UART 模式：需在 isr.c 中将接收到的字节送入本模块。
     * 此处预留接口，由外部调用 openart_uart_push_byte()。 */
    (void)buffer;
    (void)length;
    (void)i;
#endif
}

openart_msg_type_enum openart_uart_get_result(openart_result_struct *result)
{
    if (0 == queue_count)
    {
        return OPENART_MSG_NONE;
    }
    *result = result_queue[queue_tail];
    queue_tail = (uint8)((queue_tail + 1u) % RESULT_QUEUE_SIZE);
    queue_count--;
    return result->type;
}

uint8 openart_map_ready(void)
{
    return map_valid;
}

const map_source_struct *openart_map_get(void)
{
    if (0 == map_valid)
    {
        return NULL;
    }
    return &openart_map_source;
}

uint32 openart_last_rx_ms(void)
{
    return last_rx_ms;
}

/**
 * @brief 向解析器送入一个字节（专用 UART 模式）。
 *
 * 在 ISR 中调用，将 UART 接收到的字节送入解析器。
 * 仅在 OPENART_USE_WIRELESS_UART=0 时使用。
 *
 * @param data 接收到的字节。
 */
void openart_uart_push_byte(uint8 data)
{
    push_byte(data);
}
