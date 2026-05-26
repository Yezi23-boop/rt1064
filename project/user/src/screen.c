#include "zf_common_headfile.h"
#include "screen.h"
#include "maps.h"

#define IPS200_TYPE             (IPS200_TYPE_SPI)
#define LINE_H                  (16)    // IPS200 8x16 字体的行高，单位 pixel。
#define MAP_X                   (8)     // 地图左上角 X 坐标，单位 pixel。
#define MAP_Y                   (112)   // 地图左上角 Y 坐标，避开 Run/Playback 顶部文字区。
#define CELL_SIZE               (14)    // 12x16 地图在 240x320 屏上的单格边长，单位 pixel。
#define KEY_HINT_Y1             (288)   // 底部按键提示第一行，单位 pixel。
#define KEY_HINT_Y2             (304)   // 底部按键提示第二行，单位 pixel。
#define SCREEN_TEXT_COLOR       (RGB565_BLACK)
#define SCREEN_BG_COLOR         (RGB565_WHITE)
#define WALL_COLOR              (RGB565_BLACK)
#define BOX_COLOR               (RGB565_YELLOW)
#define TARGET_COLOR            (RGB565_PURPLE)
#define CAR_COLOR               (RGB565_CYAN)
#define EMPTY_COLOR             (RGB565_WHITE)
#define GRID_COLOR              (RGB565_GRAY)
#define FILL_BUFFER_PIXELS      (240u * LINE_H) // 单行文字清屏所需最大像素数，复用作小矩形填充缓存。

typedef enum
{
    SCREEN_PAGE_NONE = 0,
    SCREEN_PAGE_HOME,
    SCREEN_PAGE_RUN,
    SCREEN_PAGE_MODE_SELECT,
    SCREEN_PAGE_PLAYBACK,
    SCREEN_PAGE_DEMO,
    SCREEN_PAGE_DEBUG,
    SCREEN_PAGE_INFO,
} screen_page_enum;

static screen_page_enum active_page = SCREEN_PAGE_NONE; // 只在页面切换时整屏清空，减少白底页面刷新闪烁。
AT_SDRAM_SECTION_ALIGN(uint16 screen_fill_buffer[FILL_BUFFER_PIXELS], 64); // 显示填充缓存不参与 BFS，放 cacheable SDRAM 减少片上 RAM 压力。

static void begin_page(screen_page_enum page)
{
    if(active_page != page)
    {
        ips200_clear();
        active_page = page;
    }
}

static uint16 cell_index_local(uint8 row, uint8 col)
{
    return (uint16)(row * MAP_COLS + col);
}

static uint8 cell_row_local(uint16 cell)
{
    return (uint8)(cell / MAP_COLS);
}

static uint8 cell_col_local(uint16 cell)
{
    return (uint8)(cell % MAP_COLS);
}

static void fill_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color)
{
    uint32 pixels = (uint32)w * h;
    uint32 i;
    uint16 row;

    if(pixels <= FILL_BUFFER_PIXELS)
    {
        for(i = 0; i < pixels; i++)
        {
            screen_fill_buffer[i] = color;
        }
        ips200_show_rgb565_image(x, y, screen_fill_buffer, w, h, w, h, 0); // 整块写入比逐点画线更适合地图色块刷新。
        return;
    }

    for(row = 0; row < h; row++)
    {
        ips200_draw_line(x, (uint16)(y + row), (uint16)(x + w - 1u), (uint16)(y + row), color);
    }
}

static void add_cell(uint16 *cells, uint8 *count, uint8 row, uint8 col)
{
    if(*count < MAX_BOXES)
    {
        cells[*count] = cell_index_local(row, col);
        (*count)++;
    }
}

static void clear_line(uint16 y)
{
    fill_rect(0, y, 240, LINE_H, SCREEN_BG_COLOR);
}

static void show_line(uint16 y, const char *text)
{
    clear_line(y);
    ips200_show_string(0, y, text);
}

static void show_hint(const char *line1, const char *line2)
{
    show_line(KEY_HINT_Y1, line1);
    show_line(KEY_HINT_Y2, line2);
}

static uint8 is_target_cell(const uint16 *targets, uint8 target_count, uint16 cell)
{
    uint8 i;

    for(i = 0; i < target_count; i++)
    {
        if(targets[i] == cell)
        {
            return 1;
        }
    }
    return 0;
}

static uint8 is_box_cell(const uint16 *boxes, uint8 box_count, uint16 cell)
{
    uint8 i;

    for(i = 0; i < box_count; i++)
    {
        if(boxes[i] == cell)
        {
            return 1;
        }
    }
    return 0;
}

static void remove_index(uint16 *list, uint8 *count, uint8 index)
{
    uint8 i;

    for(i = index; (i + 1u) < *count; i++)
    {
        list[i] = list[i + 1u];
    }
    if(0 < *count)
    {
        (*count)--;
    }
}

static void remove_solved_boxes(uint16 *boxes, uint8 *box_count, uint16 *targets, uint8 *target_count)
{
    uint8 box_i = 0;
    uint8 target_i;
    uint8 removed;

    while(box_i < *box_count)
    {
        removed = 0;
        for(target_i = 0; target_i < *target_count; target_i++)
        {
            if(boxes[box_i] == targets[target_i])
            {
                remove_index(boxes, box_count, box_i);
                remove_index(targets, target_count, target_i);
                removed = 1;
                break;
            }
        }
        if(0 == removed)
        {
            box_i++;
        }
    }
}

static void parse_source(const map_source_struct *source, char grid[MAP_ROWS][MAP_COLS], uint16 *car, uint16 *boxes, uint8 *box_count, uint16 *targets, uint8 *target_count)
{
    uint8 row;
    uint8 col;
    char value;

    *car = 0;
    *box_count = 0;
    *target_count = 0;

    for(row = 0; row < MAP_ROWS; row++)
    {
        for(col = 0; col < MAP_COLS; col++)
        {
            value = source->rows[row][col];
            grid[row][col] = '.';

            if('#' == value)
            {
                grid[row][col] = '#';
            }
            else if('C' == value)
            {
                *car = cell_index_local(row, col);
            }
            else if(('B' == value) || ('$' == value))
            {
                add_cell(boxes, box_count, row, col);
            }
            else if(('T' == value) || ('.' == value))
            {
                if((MAP_FORMAT_SEEKFREE == source->format) || ('T' == value))
                {
                    add_cell(targets, target_count, row, col);
                }
            }
            else if('*' == value)
            {
                grid[row][col] = '#';
            }
        }
    }
}

static void apply_actions(uint16 *car, uint16 *boxes, uint8 *box_count, uint16 *targets, uint8 *target_count, const char *actions, uint16 step)
{
    uint16 i;
    int8 dr;
    int8 dc;
    uint16 next_car;
    uint16 next_box;
    uint8 box_i;
    char action;

    for(i = 0; (i < step) && ('\0' != actions[i]); i++)
    {
        action = actions[i];
        if('|' == action)
        {
            remove_solved_boxes(boxes, box_count, targets, target_count);
            continue;
        }

        dr = 0;
        dc = 0;
        if(('u' == action) || ('U' == action))
        {
            dr = -1;
        }
        else if(('d' == action) || ('D' == action))
        {
            dr = 1;
        }
        else if(('l' == action) || ('L' == action))
        {
            dc = -1;
        }
        else if(('r' == action) || ('R' == action))
        {
            dc = 1;
        }

        next_car = cell_index_local((uint8)((int16)cell_row_local(*car) + dr), (uint8)((int16)cell_col_local(*car) + dc));
        if((action >= 'A') && (action <= 'Z'))
        {
            for(box_i = 0; box_i < *box_count; box_i++)
            {
                if(boxes[box_i] == next_car)
                {
                    next_box = cell_index_local((uint8)((int16)cell_row_local(boxes[box_i]) + dr), (uint8)((int16)cell_col_local(boxes[box_i]) + dc));
                    boxes[box_i] = next_box;
                    break;
                }
            }
        }
        *car = next_car;
    }
}

static uint16 color_for_cell(char grid_value, uint16 cell, uint16 car, const uint16 *boxes, uint8 box_count, const uint16 *targets, uint8 target_count)
{
    if('#' == grid_value)
    {
        return WALL_COLOR;
    }
    if(car == cell)
    {
        return CAR_COLOR;
    }
    if(0 != is_box_cell(boxes, box_count, cell))
    {
        return BOX_COLOR;
    }
    if(0 != is_target_cell(targets, target_count, cell))
    {
        return TARGET_COLOR;
    }
    return EMPTY_COLOR;
}

static void draw_color_map(const map_source_struct *source, const solve_result_struct *result, uint16 step)
{
    char grid[MAP_ROWS][MAP_COLS];
    uint16 car;
    uint16 boxes[MAX_BOXES];
    uint16 targets[MAX_BOXES];
    uint8 box_count;
    uint8 target_count;
    uint8 row;
    uint8 col;
    uint16 cell;
    uint16 color;

    parse_source(source, grid, &car, boxes, &box_count, targets, &target_count);
    if((0 != result) && (0 != result->solved) && (0 < step))
    {
        apply_actions(&car, boxes, &box_count, targets, &target_count, result->actions, step);
    }

    for(row = 0; row < MAP_ROWS; row++)
    {
        for(col = 0; col < MAP_COLS; col++)
        {
            cell = cell_index_local(row, col);
            color = color_for_cell(grid[row][col], cell, car, boxes, box_count, targets, target_count);
            fill_rect((uint16)(MAP_X + col * CELL_SIZE), (uint16)(MAP_Y + row * CELL_SIZE), (CELL_SIZE - 1u), (CELL_SIZE - 1u), color);
        }
    }
}

void screen_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(SCREEN_TEXT_COLOR, SCREEN_BG_COLOR);
    ips200_init(IPS200_TYPE);
    ips200_clear();
    active_page = SCREEN_PAGE_NONE;
}

static uint16 count_visible_actions(const solve_result_struct *result)
{
    uint16 i;
    uint16 count = 0;

    for(i = 0; i < result->action_count; i++)
    {
        if('|' != result->actions[i])
        {
            count++;
        }
    }
    return count;
}

static uint16 count_visible_actions_to_step(const solve_result_struct *result, uint16 step)
{
    uint16 i;
    uint16 count = 0;

    for(i = 0; (i < step) && (i < result->action_count); i++)
    {
        if('|' != result->actions[i])
        {
            count++;
        }
    }
    return count;
}

void screen_draw_home(const char *const *items, uint8 item_count, uint8 cursor, uint8 current_map, run_mode_enum mode, save_state_enum save_state)
{
    char line[40];
    uint8 i;

    begin_page(SCREEN_PAGE_HOME);
    show_line(0, "Home");
    for(i = 0; i < item_count; i++)
    {
        snprintf(line, sizeof(line), "%c %s", (i == cursor) ? '>' : ' ', items[i]);
        show_line((uint16)(LINE_H * (i + 1u)), line);
    }
    snprintf(line, sizeof(line), "Map : V%02d", current_map + 1);
    show_line(LINE_H * 5u, line);
    snprintf(line, sizeof(line), "Mode: %s", mode_name(mode));
    show_line(LINE_H * 6u, line);
    snprintf(line, sizeof(line), "Save: %s", save_state_name(save_state));
    show_line(LINE_H * 7u, line);
    show_hint("K1/K2 Move  K3 Enter", "K4 Save  K4L Home");
}

void screen_draw_run(uint8 current_map, uint8 candidate_map, run_mode_enum mode, save_state_enum save_state, const char *state, uint32 elapsed_ms)
{
    char line[48];

    begin_page(SCREEN_PAGE_RUN);
    show_line(0, "Run");
    snprintf(line, sizeof(line), "Map : V%02d", current_map + 1);
    show_line(LINE_H, line);
    snprintf(line, sizeof(line), "Pick: V%02d", candidate_map + 1);
    show_line(LINE_H * 2u, line);
    snprintf(line, sizeof(line), "Mode: %s", mode_name(mode));
    show_line(LINE_H * 3u, line);
    snprintf(line, sizeof(line), "Save: %s", save_state_name(save_state));
    show_line(LINE_H * 4u, line);
    snprintf(line, sizeof(line), "State: %s", state);
    show_line(LINE_H * 5u, line);
    snprintf(line, sizeof(line), "Time: %lu ms", (unsigned long)elapsed_ms);
    show_line(LINE_H * 6u, line);
    draw_color_map(map_get(candidate_map), 0, 0);
    show_hint("K1/K2 Map  K3 OK/Run", "K2L Mode  K4 Home");
}

void screen_draw_mode_select(run_mode_enum candidate_mode)
{
    uint8 i;
    char line[32];

    begin_page(SCREEN_PAGE_MODE_SELECT);
    show_line(0, "Mode Select");
    for(i = 0; i < RUN_MODE_COUNT; i++)
    {
        snprintf(line, sizeof(line), "%c %s", (i == (uint8)candidate_mode) ? '>' : ' ', mode_name((run_mode_enum)i));
        show_line((uint16)(LINE_H * (i + 1u)), line);
    }
    show_hint("K1/K2 Mode  K3 OK", "K4 Cancel  K4L Home");
}

void screen_draw_playback(uint8 map_index, const map_source_struct *source, const solve_result_struct *result, uint16 step, uint32 elapsed_ms, playback_state_enum state)
{
    char line[48];
    char action = '-';
    const char *state_name = "Paused";
    uint16 visible_step;
    uint16 visible_total;

    if((0 < step) && (step <= result->action_count))
    {
        action = result->actions[step - 1u];
        if('|' == action)
        {
            action = '-';
        }
    }
    if(PLAYBACK_STATE_PLAYING == state)
    {
        state_name = "Playing";
    }
    else if(PLAYBACK_STATE_DONE == state)
    {
        state_name = "Done";
    }
    else if(PLAYBACK_STATE_FAIL == state)
    {
        state_name = "Fail";
    }

    visible_step = count_visible_actions_to_step(result, step);
    visible_total = count_visible_actions(result);

    begin_page(SCREEN_PAGE_PLAYBACK);
    snprintf(line, sizeof(line), "Playback V%02d", map_index + 1);
    show_line(0, line);
    snprintf(line, sizeof(line), "Step: %03u/%03u", visible_step, visible_total);
    show_line(LINE_H, line);
    snprintf(line, sizeof(line), "Act : %c", action);
    show_line(LINE_H * 2u, line);
    snprintf(line, sizeof(line), "Time: %lu ms", (unsigned long)elapsed_ms);
    show_line(LINE_H * 3u, line);
    snprintf(line, sizeof(line), "State: %s", state_name);
    show_line(LINE_H * 4u, line);
    draw_color_map(source, result, step);
    if(PLAYBACK_STATE_FAIL == state)
    {
        show_line(LINE_H * 5u, result->message);
    }
    else
    {
        clear_line(LINE_H * 5u);
    }
    show_hint("K1 Prev K2 Next K3 Play", "K4 Run  K4L Home");
}

void screen_draw_demo(uint8 index, uint8 total, uint8 ok_count, uint8 fail_count, uint32 elapsed_ms, const char *state)
{
    char line[40];

    begin_page(SCREEN_PAGE_DEMO);
    show_line(0, "Demo");
    snprintf(line, sizeof(line), "Map : %03d/%03d", index, total);
    show_line(LINE_H, line);
    snprintf(line, sizeof(line), "OK  : %03d", ok_count);
    show_line(LINE_H * 2u, line);
    snprintf(line, sizeof(line), "Fail: %03d", fail_count);
    show_line(LINE_H * 3u, line);
    snprintf(line, sizeof(line), "Time: %lu ms", (unsigned long)elapsed_ms);
    show_line(LINE_H * 4u, line);
    snprintf(line, sizeof(line), "State: %s", state);
    show_line(LINE_H * 5u, line);
    show_hint("K4 Stop", "K4L Home");
}

void screen_draw_debug(void)
{
    begin_page(SCREEN_PAGE_DEBUG);
    show_line(0, "Debug");
    show_line(LINE_H, "Reserved");
    show_hint("K4 Home", "K4L Home");
}

void screen_draw_info(uint8 map_count_value, save_state_enum save_state)
{
    char line[48];

    begin_page(SCREEN_PAGE_INFO);
    show_line(0, "Info");
    snprintf(line, sizeof(line), "Map count: %d", map_count_value);
    show_line(LINE_H, line);
    snprintf(line, sizeof(line), "Flash   : %s", flash_state_name(save_state));
    show_line(LINE_H * 2u, line);
    snprintf(line, sizeof(line), "Save    : %s", save_state_name(save_state));
    show_line(LINE_H * 3u, line);
    show_line(LINE_H * 4u, "Version : 1");
    show_hint("K4 Home", "K4L Home");
}
