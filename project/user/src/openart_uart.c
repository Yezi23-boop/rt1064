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
#include "map_utils.h"
#include "timebase.h"

#define OPENART_UART_INDEX          (UART_1)
#define OPENART_HW_RX_BUFFER_SIZE   (384u)
#define OPENART_LINE_SIZE           (24u)
#define OPENART_RX_TIMEOUT_MS       (1000u)

typedef enum
{
    PARSE_WAIT_BEGIN = 0, /**< 等待 MAP_BEGIN；其它文本行都会被忽略。 */
    PARSE_READ_MAP,       /**< 已进入 12 行地图读取阶段，任何非法行都会放弃本帧。 */
} openart_parse_state_enum;

static volatile uint8 hw_rx_buffer[OPENART_HW_RX_BUFFER_SIZE];   // UART1 ISR 写入、主循环读取；只缓存原始字节，不存整帧历史。
static volatile uint16 hw_rx_write_index;                        // ISR 推进的写下标；主循环不写，避免无锁双写。
static volatile uint16 hw_rx_read_index;                         // 主循环推进的读下标；丢弃旧帧时在临界区追平写下标。
static volatile uint32 hw_rx_overflow_count;                     // 环形缓冲满时累加，保留给串口吞吐诊断。

static openart_parse_state_enum parse_state;                     // 主循环协议解析状态，ISR 不访问。
static char line_buffer[OPENART_LINE_SIZE];                      // 单行文本缓存，容量覆盖 16 字符地图行和 MAP_* 标记。
static uint8 line_length;                                        // 当前行已接收字符数，不含结尾 NUL。
static uint8 recv_row;                                           // 当前帧已经接收的地图行数，必须到 MAP_ROWS 才接受 MAP_END。
static uint32 last_rx_ms;                                        // 最近一次解析到完整文本行的时间戳，单位 ms，用于半帧超时。

static char staging_map[MAP_ROWS][MAP_COLS + 1];                 // 当前正在接收的半帧地图，只有 MAP_END 合法后才发布。
static char map_snapshot[MAP_ROWS][MAP_COLS + 1];                // 最近一次完整帧快照，供屏幕/菜单读取稳定数据。
static map_source_struct openart_map_source;                     // 指向 `map_snapshot` 行缓存的地图描述符，生命周期覆盖整个运行期。
static uint8 map_valid;                                          // 1 表示至少接收过一帧完整合法地图。
static uint32 frame_count;                                       // 完整合法地图帧计数，用于等待新帧和诊断延迟。
static uint32 error_count;                                       // 协议错误/半帧超时次数，当前主要供后续调试扩展。
static uint8 player_row;                                         // 最近完整帧中第一个 `C` 的行号。
static uint8 player_col;                                         // 最近完整帧中第一个 `C` 的列号。
static uint8 player_count;                                       // 最近完整帧中 `C` 的数量，非 1 时位置只作诊断。

static void update_map_object_cache(void)
{
    (void)map_find_car(&openart_map_source, &player_row, &player_col, &player_count);
}

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
    // UART ISR 只推进 write_index，主循环只推进 read_index；读空表示当前没有待解析字节。
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
    // 只重置半帧解析器，不清除最近一次成功地图；屏幕和重解算仍可使用旧完整快照。
    parse_state = PARSE_WAIT_BEGIN;
    line_length = 0;
    recv_row = 0;
}

static void accept_map(void)
{
    uint8 row;

    // staging_map 只保存正在接收的帧；完整帧通过 MAP_END 校验后再一次性发布快照。
    // 这样屏幕和求解器不会读到半帧地图。
    for(row = 0; row < MAP_ROWS; row++)
    {
        memcpy(map_snapshot[row], staging_map[row], MAP_COLS + 1);
        openart_map_source.rows[row] = map_snapshot[row];
    }

    openart_map_source.name = "OpenART";
    map_valid = 1;
    update_map_object_cache();
    frame_count++;
    uart_write_string(OPENART_UART_INDEX, "MAP_OK rows=12 cols=16\r\n");
}

static void parse_line(void)
{
    line_buffer[line_length] = '\0';
    last_rx_ms = time_ms();

    if(str_equal(line_buffer, "MAP_BEGIN"))
    {
        // 新 MAP_BEGIN 直接开始新帧；若上一帧残缺，后续 MAP_END 行数校验会让它失效。
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
    player_row = 0;
    player_col = 0;
    player_count = 0;
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
        // OpenART 串口可能在一帧中途断开；超时后丢弃半帧，避免下一帧尾部拼到旧数据上。
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

void openart_uart_discard_pending(void)
{
    uint32 primask;

    // read/write 指针由主循环和 UART ISR 分别访问，调整读指针时短暂关中断保证一致性。
    primask = interrupt_global_disable();
    hw_rx_read_index = hw_rx_write_index;
    interrupt_global_enable(primask);
    reset_frame_parser();
}

uint8 openart_find_player_cell(uint8 *row, uint8 *col, uint8 *count)
{
    if(0 != row)
    {
        *row = player_row;
    }
    if(0 != col)
    {
        *col = player_col;
    }
    if(0 != count)
    {
        *count = player_count;
    }

    return ((0 != map_valid) && (1u == player_count)) ? 1u : 0u;
}

void openart_uart_push_byte(uint8 data)
{
    uint16 next_index = next_hw_rx_index(hw_rx_write_index);

    if(next_index == hw_rx_read_index)
    {
        // ISR 中不能阻塞等待主循环解析；溢出时丢当前字节，下一次 MAP_BEGIN 会恢复同步。
        hw_rx_overflow_count++;
        return;
    }

    hw_rx_buffer[hw_rx_write_index] = data;
    hw_rx_write_index = next_index;
}
