/**
 * @file openart_uart.c
 * @brief OpenART 12x16 地图帧接收。
 *
 * 协议：
 *   MAP_BEGIN
 *   12 行 RT 地图，每行 16 字符：
 *   # 墙，. 空地，B 箱子，T 目标点，C 小车，X 炸弹/障碍
 *   MAP_END
 *
 * UART1 底层初始化由 debug_init() 完成；运行期 LPUART1_IRQHandler()
 * 只投递字节到本模块，协议解析在主循环 openart_uart_poll() 中完成。
 */

#include "zf_common_headfile.h"
#include "openart_uart.h"
#include "timebase.h"

#define OPENART_UART_INDEX          (UART_1)
#define OPENART_HW_RX_BUFFER_SIZE   (384u)
#define OPENART_LINE_SIZE           (24u)
#define OPENART_RX_TIMEOUT_MS       (1000u)

typedef enum
{
    PARSE_WAIT_BEGIN = 0,
    PARSE_READ_MAP,
} openart_parse_state_enum;

static volatile uint8 hw_rx_buffer[OPENART_HW_RX_BUFFER_SIZE];
static volatile uint16 hw_rx_write_index;
static volatile uint16 hw_rx_read_index;
static volatile uint32 hw_rx_overflow_count;

static openart_parse_state_enum parse_state;
static char line_buffer[OPENART_LINE_SIZE];
static uint8 line_length;
static uint8 recv_row;
static uint32 last_rx_ms;

static char staging_map[MAP_ROWS][MAP_COLS + 1];
static char map_snapshot[MAP_ROWS][MAP_COLS + 1];
static map_source_struct openart_map_source;
static uint8 map_valid;
static uint32 frame_count;
static uint32 error_count;

static uint16 next_hw_rx_index(uint16 index)
{
    index++;
    if(index >= OPENART_HW_RX_BUFFER_SIZE)
    {
        index = 0;
    }
    return index;
}

static uint8 hw_rx_pop(uint8 *data)
{
    if(hw_rx_read_index == hw_rx_write_index)
    {
        return 0;
    }

    *data = hw_rx_buffer[hw_rx_read_index];
    hw_rx_read_index = next_hw_rx_index(hw_rx_read_index);
    return 1;
}

static uint8 str_equal(const char *left, const char *right)
{
    while(('\0' != *left) && ('\0' != *right))
    {
        if(*left != *right)
        {
            return 0;
        }
        left++;
        right++;
    }
    return (('\0' == *left) && ('\0' == *right)) ? 1u : 0u;
}

static uint8 is_map_char(char ch)
{
    return (('#' == ch) || ('.' == ch) || ('B' == ch) ||
            ('T' == ch) || ('C' == ch) || ('X' == ch)) ? 1u : 0u;
}

static uint8 is_valid_map_line(const char *line)
{
    uint8 col;

    for(col = 0; col < MAP_COLS; col++)
    {
        if(0 == is_map_char(line[col]))
        {
            return 0;
        }
    }

    return ('\0' == line[MAP_COLS]) ? 1u : 0u;
}

static void reset_frame_parser(void)
{
    parse_state = PARSE_WAIT_BEGIN;
    line_length = 0;
    recv_row = 0;
}

static void accept_map(void)
{
    uint8 row;

    for(row = 0; row < MAP_ROWS; row++)
    {
        memcpy(map_snapshot[row], staging_map[row], MAP_COLS + 1);
        openart_map_source.rows[row] = map_snapshot[row];
    }

    openart_map_source.name = "OpenART";
    map_valid = 1;
    frame_count++;
    uart_write_string(OPENART_UART_INDEX, "MAP_OK rows=12 cols=16\r\n");
}

static void parse_line(void)
{
    line_buffer[line_length] = '\0';
    last_rx_ms = time_ms();

    if(str_equal(line_buffer, "MAP_BEGIN"))
    {
        parse_state = PARSE_READ_MAP;
        recv_row = 0;
        return;
    }

    if(PARSE_WAIT_BEGIN == parse_state)
    {
        return;
    }

    if(str_equal(line_buffer, "MAP_END"))
    {
        if(MAP_ROWS == recv_row)
        {
            accept_map();
        }
        else
        {
            error_count++;
        }
        reset_frame_parser();
        return;
    }

    if((recv_row < MAP_ROWS) && (0 != is_valid_map_line(line_buffer)))
    {
        memcpy(staging_map[recv_row], line_buffer, MAP_COLS + 1);
        recv_row++;
        return;
    }

    error_count++;
    reset_frame_parser();
}

static void parse_byte(uint8 data)
{
    if('\r' == data)
    {
        return;
    }

    if('\n' == data)
    {
        if(0 != line_length)
        {
            parse_line();
            line_length = 0;
        }
        return;
    }

    if(line_length >= (OPENART_LINE_SIZE - 1u))
    {
        error_count++;
        reset_frame_parser();
        return;
    }

    line_buffer[line_length] = (char)data;
    line_length++;
}

void openart_uart_init(void)
{
    uint8 row;

    hw_rx_write_index = 0;
    hw_rx_read_index = 0;
    hw_rx_overflow_count = 0;
    map_valid = 0;
    frame_count = 0;
    error_count = 0;
    last_rx_ms = 0;
    reset_frame_parser();

    for(row = 0; row < MAP_ROWS; row++)
    {
        memset(staging_map[row], 0, MAP_COLS + 1);
        memset(map_snapshot[row], 0, MAP_COLS + 1);
        openart_map_source.rows[row] = map_snapshot[row];
    }
    openart_map_source.name = "OpenART";
}

void openart_uart_poll(void)
{
    uint8 data;
    uint32 now_ms = time_ms();

    if((PARSE_READ_MAP == parse_state) &&
       (0 != last_rx_ms) &&
       ((now_ms - last_rx_ms) >= OPENART_RX_TIMEOUT_MS))
    {
        error_count++;
        reset_frame_parser();
    }

    while(0 != hw_rx_pop(&data))
    {
        parse_byte(data);
    }
}

uint8 openart_map_ready(void)
{
    return map_valid;
}

const map_source_struct *openart_map_get(void)
{
    if(0 == map_valid)
    {
        return NULL;
    }
    return &openart_map_source;
}

uint32 openart_last_rx_ms(void)
{
    return last_rx_ms;
}

uint32 openart_uart_get_frame_count(void)
{
    return frame_count;
}

void openart_uart_push_byte(uint8 data)
{
    uint16 next_index = next_hw_rx_index(hw_rx_write_index);

    if(next_index == hw_rx_read_index)
    {
        hw_rx_overflow_count++;
        return;
    }

    hw_rx_buffer[hw_rx_write_index] = data;
    hw_rx_write_index = next_index;
}
