/**
 * @file openart_uart.c
 * @brief OpenART 12x16 地图帧接收。
 *
 * 协议：
 *   MAP_BEGIN
 *   12 行 RT 地图，每行 16 字符：
 *   # 墙，. 空地，B 箱子，T 目标点，C 小车，X 炸弹/障碍
 *   PLAYER_CENTER_GRID col_q,row_q valid
 *   MAP_END
 * 精确中心由 MCU 发送 CENTER_REQ 后，通过独立的 CENTER_SAMPLE 1..N 返回。
 *
 * UART1 底层初始化由 debug_init() 完成；运行期 LPUART1_IRQHandler()
 * 只投递字节到本模块，协议解析在主循环 openart_uart_poll() 中完成。
 */

#include "zf_common_headfile.h"
#include "openart_uart.h"
#include "drive_config.h"
#include "map_utils.h"
#include "timebase.h"

#define OPENART_UART_INDEX          (UART_1)
#define OPENART_HW_RX_BUFFER_SIZE   (384u)
#define OPENART_LINE_SIZE           (48u)
#define OPENART_RX_TIMEOUT_MS       (1000u)
#define OPENART_REQUESTED_CENTER_SAMPLE_COUNT (ART_CENTER_SAMPLE_COUNT)
#define OPENART_OBSERVATION_SAMPLE_COUNT      (ART_BOX_OBSERVE_SAMPLE_COUNT)

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
static uint16 staging_player_center_col_q;                        // 普通帧兼容中心列；请求式模式下应为0。
static uint16 staging_player_center_row_q;                        // 普通帧兼容中心行；请求式模式下应为0。
static uint8 staging_player_center_valid;                         // 普通帧兼容 valid；请求式模式下应为0。
static uint8 staging_player_center_received;                      // 1 表示当前半帧已收到 PLAYER_CENTER_GRID 行。
static uint16 player_center_col_q;                               // 最近视觉中心列坐标，单位 1/100 格。
static uint16 player_center_row_q;                               // 最近视觉中心行坐标，单位 1/100 格。
static uint8 player_center_valid;                                // 1 表示最近一次样本有效。
static uint32 player_center_count;                               // 视觉中心点样本累计计数。
static uint8 requested_center_active;                            // 1 表示 MCU 已发送 CENTER_REQ，正在接收 1..N 号样本。
static uint8 requested_center_count;                             // 当前请求已按顺序接收并入队的样本数。
static uint8 requested_center_read_index;                        // 主循环下一条待取样本下标。
static uint16 requested_center_col_q[OPENART_REQUESTED_CENTER_SAMPLE_COUNT]; // 请求样本列队列，单位 1/100 格。
static uint16 requested_center_row_q[OPENART_REQUESTED_CENTER_SAMPLE_COUNT]; // 请求样本行队列，单位 1/100 格。
static uint16 requested_center_yaw_q[OPENART_REQUESTED_CENTER_SAMPLE_COUNT]; // 请求样本 ART yaw，单位 0.01 degree。
static uint8 requested_center_yaw_valid[OPENART_REQUESTED_CENTER_SAMPLE_COUNT]; // 1 表示对应 yaw 样本有效。
static char requested_center_map_rows[MAP_ROWS][MAP_COLS + 1];   // 最近一条有效 CENTER_SAMPLE 前的配套完整地图。
static map_source_struct requested_center_map_source;            // 指向请求中心配套地图快照。
static uint8 requested_center_map_valid;                         // 1 表示配套地图与当前请求样本已通过一致性校验。
static uint32 requested_center_last_map_frame;                   // 上一条已接受样本使用的地图帧号。
static uint8 observation_active;
static uint8 observation_count;
static uint8 observation_read_index;
static openart_observation_sample_struct observation_samples[OPENART_OBSERVATION_SAMPLE_COUNT];
static uint8 observation_request_row;
static uint8 observation_request_col;
static uint32 observation_last_map_frame;

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
            ('T' == ch) || ('C' == ch) || ('+' == ch) ||
            ('X' == ch)) ? 1u : 0u;
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

static void reset_staging_player_center(void)
{
    staging_player_center_col_q = 0;
    staging_player_center_row_q = 0;
    staging_player_center_valid = 0;
    staging_player_center_received = 0;
}

static void reset_frame_parser(void)
{
    // 只重置半帧解析器，不清除最近一次成功地图；屏幕和重解算仍可使用旧完整快照。
    parse_state = PARSE_WAIT_BEGIN;
    line_length = 0;
    recv_row = 0;
    reset_staging_player_center();
}

static uint8 find_unique_staging_player(uint8 *player_row_out,
                                        uint8 *player_col_out)
{
    uint8 row;
    uint8 col;
    uint8 count = 0u;

    for(row = 0u; row < MAP_ROWS; row++)
    {
        for(col = 0u; col < MAP_COLS; col++)
        {
            if(('C' == staging_map[row][col]) || ('+' == staging_map[row][col]))
            {
                *player_row_out = row;
                *player_col_out = col;
                count++;
            }
        }
    }
    return (1u == count) ? 1u : 0u;
}

static uint8 validate_staging_player(void)
{
    uint8 row;
    uint8 col;

    if(0 == find_unique_staging_player(&row, &col))
    {
        return 0u;
    }

    if((0 == staging_player_center_received) ||
       (0 == staging_player_center_valid))
    {
        return 1u;
    }

    if((staging_player_center_col_q >= (MAP_COLS * 100u)) ||
       (staging_player_center_row_q >= (MAP_ROWS * 100u)))
    {
        return 0u;
    }

    return (((uint8)(staging_player_center_col_q / 100u) == col) &&
            ((uint8)(staging_player_center_row_q / 100u) == row)) ? 1u : 0u;
}

static void accept_map(void)
{
    uint8 row;

    // OpenART 负责生成唯一 C/+；MCU 只校验，不再依据中心点重写字符地图。
    if(0 == validate_staging_player())
    {
        error_count++;
        return;
    }

    // staging_map 只保存正在接收的帧；完整帧通过 MAP_END 校验后再一次性发布快照。
    // 这样屏幕和求解器不会读到半帧地图。
    for(row = 0; row < MAP_ROWS; row++)
    {
        memcpy(map_snapshot[row], staging_map[row], MAP_COLS + 1);
        openart_map_source.rows[row] = map_snapshot[row];
    }

    openart_map_source.name = "OpenART";
    if(0 != staging_player_center_received)
    {
        player_center_col_q = staging_player_center_col_q;
        player_center_row_q = staging_player_center_row_q;
        player_center_valid = staging_player_center_valid;
    }
    else
    {
        player_center_col_q = 0;
        player_center_row_q = 0;
        player_center_valid = 0;
    }
    map_valid = 1;
    update_map_object_cache();
    frame_count++;
    player_center_count = frame_count;
    uart_write_string(OPENART_UART_INDEX, "MAP_OK rows=12 cols=16\r\n");
}

static uint8 parse_uint16_text(const char *text, uint16 *value)
{
    uint32 result = 0;
    uint8 index = 0;

    if((0 == text) || ('\0' == text[0]))
    {
        return 0;
    }

    while('\0' != text[index])
    {
        if((text[index] < '0') || (text[index] > '9'))
        {
            return 0;
        }
        result = (result * 10u) + (uint32)(text[index] - '0');
        if(result > 65535u)
        {
            return 0;
        }
        index++;
    }

    *value = (uint16)result;
    return 1u;
}

static void skip_spaces(char **text)
{
    while((' ' == **text) || ('\t' == **text))
    {
        (*text)++;
    }
}

static uint8 parse_u16_pair_valid_line(const char *prefix,
                                       uint16 *first_value,
                                       uint16 *second_value,
                                       uint8 *valid_value)
{
    char *text;
    char *first_text;
    char *second_text;
    char *valid_text;
    uint16 prefix_len;
    uint16 valid_u16;

    prefix_len = (uint16)strlen(prefix);
    text = line_buffer;
    if(0 != strncmp(text, prefix, prefix_len))
    {
        return 0;
    }

    text += prefix_len;
    skip_spaces(&text);
    first_text = text;
    while(('\0' != *text) && (',' != *text))
    {
        text++;
    }
    if(',' != *text)
    {
        return 0;
    }
    *text = '\0';
    text++;
    skip_spaces(&text);
    second_text = text;
    while(('\0' != *text) && (' ' != *text))
    {
        text++;
    }
    if((' ' == *text) || ('\t' == *text))
    {
        *text = '\0';
        text++;
    }
    skip_spaces(&text);
    valid_text = text;

    if((0 != parse_uint16_text(first_text, first_value)) &&
       (0 != parse_uint16_text(second_text, second_value)) &&
       (0 != parse_uint16_text(valid_text, &valid_u16)))
    {
        *valid_value = (0 != valid_u16) ? 1u : 0u;
        return 1;
    }
    return 0;
}

static void parse_player_center_line(void)
{
    uint16 first_value;
    uint16 second_value;
    uint8 valid_value;

    if(0 != parse_u16_pair_valid_line("PLAYER_CENTER_GRID ",
                                      &first_value,
                                      &second_value,
                                      &valid_value))
    {
        staging_player_center_col_q = first_value;
        staging_player_center_row_q = second_value;
        staging_player_center_valid = valid_value;
        staging_player_center_received = 1;
    }
}

static uint8 parse_center_sample_line(uint16 *sample_index,
                                     uint16 *center_col_q,
                                     uint16 *center_row_q,
                                     uint16 *yaw_q,
                                     uint8 *yaw_valid)
{
    const char *prefix = "CENTER_SAMPLE ";
    char *text = line_buffer + strlen(prefix);
    char *index_text;
    char *col_text;
    char *row_text;
    char *yaw_text;
    char *valid_text;
    uint16 valid_value;

    if(0 != strncmp(line_buffer, prefix, strlen(prefix)))
    {
        return 0;
    }

    index_text = text;
    while(('\0' != *text) && (',' != *text))
    {
        text++;
    }
    if(',' != *text)
    {
        return 0;
    }
    *text++ = '\0';

    col_text = text;
    while(('\0' != *text) && (',' != *text))
    {
        text++;
    }
    if(',' != *text)
    {
        return 0;
    }
    *text++ = '\0';
    row_text = text;
    while(('\0' != *text) && (',' != *text))
    {
        text++;
    }
    if(',' != *text)
    {
        return 0;
    }
    *text++ = '\0';

    yaw_text = text;
    while(('\0' != *text) && (',' != *text))
    {
        text++;
    }
    if(',' != *text)
    {
        return 0;
    }
    *text++ = '\0';
    valid_text = text;

    if((0 == parse_uint16_text(index_text, sample_index)) ||
       (0 == parse_uint16_text(col_text, center_col_q)) ||
       (0 == parse_uint16_text(row_text, center_row_q)) ||
       (0 == parse_uint16_text(yaw_text, yaw_q)) ||
       (0 == parse_uint16_text(valid_text, &valid_value)) ||
       (valid_value > 1u) || (*yaw_q >= 36000u))
    {
        return 0u;
    }
    if((0u == valid_value) && (0u != *yaw_q))
    {
        return 0u;
    }
    *yaw_valid = (uint8)valid_value;
    return 1u;
}

static void parse_requested_center_line(void)
{
    uint16 sample_index;
    uint16 center_col_q;
    uint16 center_row_q;
    uint16 yaw_q;
    uint8 yaw_valid;

    if((0 == requested_center_active) ||
       (0 == parse_center_sample_line(&sample_index, &center_col_q, &center_row_q,
                                      &yaw_q, &yaw_valid)))
    {
        return;
    }
    if((sample_index != (uint16)(requested_center_count + 1u)) ||
       (sample_index > OPENART_REQUESTED_CENTER_SAMPLE_COUNT))
    {
        return;
    }
    if((0 == map_valid) ||
       (0 == player_center_valid) ||
       (frame_count == requested_center_last_map_frame) ||
       (center_col_q != player_center_col_q) ||
       (center_row_q != player_center_row_q) ||
       (1u != player_count))
    {
        return;
    }

    requested_center_col_q[requested_center_count] = center_col_q;
    requested_center_row_q[requested_center_count] = center_row_q;
    requested_center_yaw_q[requested_center_count] = yaw_q;
    requested_center_yaw_valid[requested_center_count] = yaw_valid;
    map_source_snapshot(&requested_center_map_source,
                        requested_center_map_rows,
                        &openart_map_source);
    requested_center_map_valid = 1u;
    requested_center_last_map_frame = frame_count;
    requested_center_count++;
    if(requested_center_count >= OPENART_REQUESTED_CENTER_SAMPLE_COUNT)
    {
        requested_center_active = 0;
    }
}

static void append_u16(char *text, uint8 *length, uint16 value)
{
    char reverse[5];
    uint8 count = 0u;

    do
    {
        reverse[count++] = (char)('0' + (value % 10u));
        value = (uint16)(value / 10u);
    } while((0u != value) && (count < sizeof(reverse)));
    while(0u != count)
    {
        text[(*length)++] = reverse[--count];
    }
}

static uint8 parse_observation_sample_line(uint16 values[5])
{
    const char *prefix = "OBSERVE_SAMPLE ";
    char *text = line_buffer + strlen(prefix);
    char *tokens[5];
    uint8 index;

    if(0 != strncmp(line_buffer, prefix, strlen(prefix)))
    {
        return 0u;
    }
    for(index = 0u; index < 4u; index++)
    {
        tokens[index] = text;
        while(('\0' != *text) && (',' != *text))
        {
            text++;
        }
        if(',' != *text)
        {
            return 0u;
        }
        *text++ = '\0';
    }
    tokens[4] = text;
    for(index = 0u; index < 5u; index++)
    {
        if(0 == parse_uint16_text(tokens[index], &values[index]))
        {
            return 0u;
        }
    }
    return 1u;
}

static void parse_observation_line(void)
{
    uint16 values[5];
    openart_observation_sample_struct *sample;
    int16 row_offset_q;
    int16 col_offset_q;

    if((0u == observation_active) ||
       (0u == parse_observation_sample_line(values)))
    {
        return;
    }
    if((values[0] != (uint16)(observation_count + 1u)) ||
       (values[0] > OPENART_OBSERVATION_SAMPLE_COUNT) ||
       (values[1] >= (MAP_COLS * 100u)) ||
       (values[2] >= (MAP_ROWS * 100u)) ||
       (values[3] >= (MAP_COLS * 100u)) ||
       (values[4] >= (MAP_ROWS * 100u)))
    {
        return;
    }

    if((0u == map_valid) ||
       (0u == player_center_valid) ||
       (frame_count == observation_last_map_frame) ||
       (values[1] != player_center_col_q) ||
       (values[2] != player_center_row_q) ||
       (1u != player_count) ||
       (observation_request_row >= MAP_ROWS) ||
       (observation_request_col >= MAP_COLS) ||
       ('B' != openart_map_source.rows[observation_request_row][observation_request_col]))
    {
        return;
    }

    col_offset_q = (int16)values[3] -
                   (int16)(observation_request_col * 100u + 50u);
    row_offset_q = (int16)values[4] -
                   (int16)(observation_request_row * 100u + 50u);
    if((col_offset_q < -(int16)ART_BOX_MATCH_MAX_OFFSET_Q) ||
       (col_offset_q > (int16)ART_BOX_MATCH_MAX_OFFSET_Q) ||
       (row_offset_q < -(int16)ART_BOX_MATCH_MAX_OFFSET_Q) ||
       (row_offset_q > (int16)ART_BOX_MATCH_MAX_OFFSET_Q))
    {
        return;
    }

    sample = &observation_samples[observation_count];
    sample->car_col_q = values[1];
    sample->car_row_q = values[2];
    sample->box_col_q = values[3];
    sample->box_row_q = values[4];
    observation_last_map_frame = frame_count;
    observation_count++;
    if(observation_count >= OPENART_OBSERVATION_SAMPLE_COUNT)
    {
        observation_active = 0u;
    }
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
        reset_staging_player_center();
        return;
    }

    if(0 == strncmp(line_buffer, "CENTER_SAMPLE ", 14))
    {
        parse_requested_center_line();
        return;
    }

    if(0 == strncmp(line_buffer, "OBSERVE_SAMPLE ", 15))
    {
        parse_observation_line();
        return;
    }

    if(0 == strncmp(line_buffer, "PLAYER_CENTER_GRID ", 19))
    {
        if(PARSE_READ_MAP == parse_state)
        {
            parse_player_center_line();
        }
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
    player_center_col_q = 0;
    player_center_row_q = 0;
    player_center_valid = 0;
    player_center_count = 0;
    requested_center_active = 0;
    requested_center_count = 0;
    requested_center_read_index = 0;
    requested_center_map_valid = 0;
    requested_center_last_map_frame = 0;
    memset(requested_center_col_q, 0, sizeof(requested_center_col_q));
    memset(requested_center_row_q, 0, sizeof(requested_center_row_q));
    memset(requested_center_yaw_q, 0, sizeof(requested_center_yaw_q));
    memset(requested_center_yaw_valid, 0, sizeof(requested_center_yaw_valid));
    observation_active = 0u;
    observation_count = 0u;
    observation_read_index = 0u;
    observation_request_row = 0u;
    observation_request_col = 0u;
    observation_last_map_frame = 0u;
    memset(observation_samples, 0, sizeof(observation_samples));
    reset_frame_parser();

    for(row = 0; row < MAP_ROWS; row++)
    {
        memset(staging_map[row], 0, MAP_COLS + 1);
        memset(map_snapshot[row], 0, MAP_COLS + 1);
        memset(requested_center_map_rows[row], 0, MAP_COLS + 1);
        openart_map_source.rows[row] = map_snapshot[row];
        requested_center_map_source.rows[row] = requested_center_map_rows[row];
    }
    openart_map_source.name = "OpenART";
    requested_center_map_source.name = "OpenART Center";
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

uint32 openart_get_player_center(uint16 *col_q, uint16 *row_q, uint8 *valid)
{
    if(0 != col_q)
    {
        *col_q = player_center_col_q;
    }
    if(0 != row_q)
    {
        *row_q = player_center_row_q;
    }
    if(0 != valid)
    {
        *valid = player_center_valid;
    }
    return player_center_count;
}

void openart_request_player_center(void)
{
    requested_center_active = 1;
    requested_center_count = 0;
    requested_center_read_index = 0;
    requested_center_map_valid = 0;
    requested_center_last_map_frame = frame_count;
    memset(requested_center_col_q, 0, sizeof(requested_center_col_q));
    memset(requested_center_row_q, 0, sizeof(requested_center_row_q));
    memset(requested_center_yaw_q, 0, sizeof(requested_center_yaw_q));
    memset(requested_center_yaw_valid, 0, sizeof(requested_center_yaw_valid));
    uart_write_string(OPENART_UART_INDEX, "CENTER_REQ\n");
}

uint8 openart_get_requested_center_sample(uint16 *col_q, uint16 *row_q,
                                          uint16 *yaw_q, uint8 *yaw_valid)
{
    uint8 sample_index;

    if(requested_center_read_index >= requested_center_count)
    {
        if(0 != col_q)
        {
            *col_q = 0;
        }
        if(0 != row_q)
        {
            *row_q = 0;
        }
        if(0 != yaw_q)
        {
            *yaw_q = 0;
        }
        if(0 != yaw_valid)
        {
            *yaw_valid = 0;
        }
        return 0;
    }

    sample_index = (uint8)(requested_center_read_index + 1u);
    if(0 != col_q)
    {
        *col_q = requested_center_col_q[requested_center_read_index];
    }
    if(0 != row_q)
    {
        *row_q = requested_center_row_q[requested_center_read_index];
    }
    if(0 != yaw_q)
    {
        *yaw_q = requested_center_yaw_q[requested_center_read_index];
    }
    if(0 != yaw_valid)
    {
        *yaw_valid = requested_center_yaw_valid[requested_center_read_index];
    }
    requested_center_read_index++;
    return sample_index;
}

const map_source_struct *openart_get_requested_center_map(void)
{
    return (0 != requested_center_map_valid) ? &requested_center_map_source : NULL;
}

void openart_request_observation(uint8 box_row, uint8 box_col)
{
    char command[24];
    uint8 length = 0u;
    const char *prefix = "OBSERVE_REQ ";

    observation_active = 1u;
    observation_count = 0u;
    observation_read_index = 0u;
    observation_request_row = box_row;
    observation_request_col = box_col;
    observation_last_map_frame = frame_count;
    memset(observation_samples, 0, sizeof(observation_samples));
    while('\0' != *prefix)
    {
        command[length++] = *prefix++;
    }
    append_u16(command, &length, box_row);
    command[length++] = ',';
    append_u16(command, &length, box_col);
    command[length++] = '\n';
    command[length] = '\0';
    uart_write_string(OPENART_UART_INDEX, command);
}

uint8 openart_get_observation_sample(openart_observation_sample_struct *sample)
{
    if((0 == sample) || (observation_read_index >= observation_count))
    {
        return 0u;
    }
    *sample = observation_samples[observation_read_index++];
    return 1u;
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
