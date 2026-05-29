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
#define MENU_CURSOR_X           (0)
#define MENU_TEXT_X             (16)
#define MENU_ROW_Y(row)         ((uint16)(((row) + 1) * LINE_H))
#define SCREEN_TEXT_COLOR       (RGB565_BLACK)
#define SCREEN_BG_COLOR         (RGB565_WHITE)
#define WALL_COLOR              (RGB565_BLACK)
#define BOX_COLOR               (RGB565_YELLOW)
#define TARGET_COLOR            (RGB565_PURPLE)
#define CAR_COLOR               (RGB565_CYAN)
#define POSE_CAR_COLOR          (RGB565_BLUE)
#define EMPTY_COLOR             (RGB565_WHITE)
#define GRID_COLOR              (RGB565_GRAY)
#define GRID_CELL_SIZE_CM       (20.0f) // 推箱子地图物理尺度，默认现实 20cm 对应屏幕一格。
#define FILL_BUFFER_PIXELS      (240 * LINE_H) // 单行文字清屏所需最大像素数，复用作小矩形填充缓存。

typedef enum
{
    SCREEN_PAGE_NONE = 0,
    SCREEN_PAGE_HOME,
    SCREEN_PAGE_RUN,
    SCREEN_PAGE_RUN_ROOT,
    SCREEN_PAGE_RUN_MAP,
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
        ips200_draw_line(x, (uint16)(y + row), (uint16)(x + w - 1), (uint16)(y + row), color);
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

static void clear_text_area(uint16 x, uint16 y, uint8 char_count)
{
    fill_rect(x, y, (uint16)(char_count * 8), LINE_H, SCREEN_BG_COLOR);
}

static void show_text_value(uint16 x, uint16 y, const char *text, uint8 max_chars)
{
    clear_text_area(x, y, max_chars);
    ips200_show_string(x, y, text);
}

static void show_hint(const char *line1, const char *line2)
{
    ips200_show_string(0, KEY_HINT_Y1, line1);
    ips200_show_string(0, KEY_HINT_Y2, line2);
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

    for(i = index; (i + 1) < *count; i++)
    {
        list[i] = list[i + 1];
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

            if(('#' == value) || ('X' == value))
            {
                grid[row][col] = '#';
            }
            else if('C' == value)
            {
                *car = cell_index_local(row, col);
            }
            else if('B' == value)
            {
                add_cell(boxes, box_count, row, col);
            }
            else if('T' == value)
            {
                add_cell(targets, target_count, row, col);
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

static uint16 color_for_pose_cell(char grid_value, uint16 cell, uint16 pose_car, const uint16 *boxes, uint8 box_count, const uint16 *targets, uint8 target_count)
{
    if(pose_car == cell)
    {
        return POSE_CAR_COLOR;
    }
    if('#' == grid_value)
    {
        return WALL_COLOR;
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

static int16 round_cm_to_grid_delta(float value_cm)
{
    if(value_cm >= 0.0f)
    {
        return (int16)((value_cm + GRID_CELL_SIZE_CM * 0.5f) / GRID_CELL_SIZE_CM);
    }
    return (int16)((value_cm - GRID_CELL_SIZE_CM * 0.5f) / GRID_CELL_SIZE_CM);
}

static uint8 clamp_grid_index(int16 value, uint8 max_value)
{
    if(value < 0)
    {
        return 0;
    }
    if(value >= (int16)max_value)
    {
        return (uint8)(max_value - 1);
    }
    return (uint8)value;
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
            fill_rect((uint16)(MAP_X + col * CELL_SIZE), (uint16)(MAP_Y + row * CELL_SIZE), (CELL_SIZE - 1), (CELL_SIZE - 1), color);
        }
    }
}

static void draw_pose_map(const map_source_struct *source, float pose_x_cm, float pose_y_cm, uint8 *pose_row, uint8 *pose_col)
{
    char grid[MAP_ROWS][MAP_COLS];
    uint16 start_car;
    uint16 pose_car;
    uint16 boxes[MAX_BOXES];
    uint16 targets[MAX_BOXES];
    uint8 box_count;
    uint8 target_count;
    int16 row;
    int16 col;
    uint8 draw_row;
    uint8 draw_col;
    uint16 cell;
    uint16 color;

    parse_source(source, grid, &start_car, boxes, &box_count, targets, &target_count);

    col = (int16)cell_col_local(start_car) + round_cm_to_grid_delta(pose_x_cm);
    row = (int16)cell_row_local(start_car) - round_cm_to_grid_delta(pose_y_cm);
    *pose_row = clamp_grid_index(row, MAP_ROWS);
    *pose_col = clamp_grid_index(col, MAP_COLS);
    pose_car = cell_index_local(*pose_row, *pose_col);

    for(draw_row = 0; draw_row < MAP_ROWS; draw_row++)
    {
        for(draw_col = 0; draw_col < MAP_COLS; draw_col++)
        {
            cell = cell_index_local(draw_row, draw_col);
            color = color_for_pose_cell(grid[draw_row][draw_col], cell, pose_car, boxes, box_count, targets, target_count);
            fill_rect((uint16)(MAP_X + draw_col * CELL_SIZE), (uint16)(MAP_Y + draw_row * CELL_SIZE), (CELL_SIZE - 1), (CELL_SIZE - 1), color);
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

static void draw_home_status_labels(void)
{
    ips200_show_string(0, LINE_H * 8, "Enc LF:");
    ips200_show_string(112, LINE_H * 8, "RF:");

    ips200_show_string(0, LINE_H * 9, "Enc LB:");
    ips200_show_string(112, LINE_H * 9, "RB:");

    ips200_show_string(0, LINE_H * 10, "IMU R:");
    ips200_show_string(96, LINE_H * 10, "P:");

    ips200_show_string(0, LINE_H * 11, "IMU Y:");

    ips200_show_string(0, LINE_H * 12, "Yaw T:");
    ips200_show_string(96, LINE_H * 12, " C:");

    ips200_show_string(0, LINE_H * 13, "E:");
    ips200_show_string(72, LINE_H * 13, " Z:");
    ips200_show_string(136, LINE_H * 13, " T:");

    ips200_show_string(0, LINE_H * 14, "X:");
    ips200_show_string(96, LINE_H * 14, " Y:");

    ips200_show_string(0, LINE_H * 15, "ART:");
}

static void draw_home_status_values(const float encoder_count[WHEEL_COUNT], float imu_roll, float imu_pitch, float imu_yaw, float target_yaw, float yaw_error, float vz, float vzt, float pose_x_cm, float pose_y_cm, uint32 openart_frame_count)
{
    ips200_show_int(56, LINE_H * 8, (int16)encoder_count[WHEEL_LF], 5);
    ips200_show_int(136, LINE_H * 8, (int16)encoder_count[WHEEL_RF], 5);

    ips200_show_int(56, LINE_H * 9, (int16)encoder_count[WHEEL_LB], 5);
    ips200_show_int(136, LINE_H * 9, (int16)encoder_count[WHEEL_RB], 5);

    ips200_show_int(48, LINE_H * 10, (int16)imu_roll, 4);
    ips200_show_int(112, LINE_H * 10, (int16)imu_pitch, 4);

    ips200_show_int(48, LINE_H * 11, (int16)imu_yaw, 4);

    ips200_show_float(48, LINE_H * 12, (double)target_yaw, 3, 1);
    ips200_show_float(120, LINE_H * 12, (double)imu_yaw, 3, 1);

    ips200_show_float(16, LINE_H * 13, (double)yaw_error, 3, 2);
    ips200_show_float(96, LINE_H * 13, (double)vz, 1, 2);
    ips200_show_float(160, LINE_H * 13, (double)vzt, 1, 2);

    ips200_show_float(16, LINE_H * 14, (double)pose_x_cm, 4, 1);
    ips200_show_float(120, LINE_H * 14, (double)pose_y_cm, 4, 1);

    ips200_show_uint(32, LINE_H * 15, openart_frame_count, 5);
}

void screen_draw_home(const char *const *items, uint8 item_count, uint8 cursor, uint8 current_map, run_mode_enum mode, save_state_enum save_state, const float encoder_count[WHEEL_COUNT], float imu_roll, float imu_pitch, float imu_yaw, float target_yaw, float yaw_error, float vz, float vzt, float pose_x_cm, float pose_y_cm, uint32 openart_frame_count)
{
    uint8 i;

    begin_page(SCREEN_PAGE_HOME);
    ips200_show_string(0, 0, "Home");
    for(i = 0; i < item_count; i++)
    {
        ips200_show_string(0, (uint16)(LINE_H * (i + 1)), (i == cursor) ? ">" : " ");
        ips200_show_string(16, (uint16)(LINE_H * (i + 1)), items[i]);
    }
    ips200_show_string(0, LINE_H * 5, "Map : V");
    ips200_show_uint(56, LINE_H * 5, current_map + 1, 2);
    ips200_show_string(0, LINE_H * 6, "Mode: ");
    show_text_value(48, LINE_H * 6, mode_name(mode), 8);
    ips200_show_string(0, LINE_H * 7, "Save: ");
    show_text_value(48, LINE_H * 7, save_state_name(save_state), 12);
    draw_home_status_labels();
    draw_home_status_values(encoder_count, imu_roll, imu_pitch, imu_yaw, target_yaw, yaw_error, vz, vzt, pose_x_cm, pose_y_cm, openart_frame_count);
    show_hint("K1/K2 Move  K3 Enter", "K4 Save  K4L Home");
}

void screen_draw_home_status(const float encoder_count[WHEEL_COUNT], float imu_roll, float imu_pitch, float imu_yaw, float target_yaw, float yaw_error, float vz, float vzt, float pose_x_cm, float pose_y_cm, uint32 openart_frame_count)
{
    begin_page(SCREEN_PAGE_HOME);
    draw_home_status_values(encoder_count, imu_roll, imu_pitch, imu_yaw, target_yaw, yaw_error, vz, vzt, pose_x_cm, pose_y_cm, openart_frame_count);
}

void screen_draw_nav_cursor(uint8 previous_cursor, uint8 cursor)
{
    ips200_show_string(MENU_CURSOR_X, MENU_ROW_Y(previous_cursor), " ");
    ips200_show_string(MENU_CURSOR_X, MENU_ROW_Y(cursor), ">");
}

void screen_draw_run_root(const char *const *items, uint8 item_count, uint8 cursor, uint8 current_map, run_mode_enum mode, save_state_enum save_state, const char *state, uint32 elapsed_ms)
{
    uint8 i;

    begin_page(SCREEN_PAGE_RUN_ROOT);
    ips200_show_string(0, 0, "Run");
    for(i = 0; i < item_count; i++)
    {
        ips200_show_string(MENU_CURSOR_X, MENU_ROW_Y(i), (i == cursor) ? ">" : " ");
        ips200_show_string(MENU_TEXT_X, MENU_ROW_Y(i), items[i]);
    }

    ips200_show_string(0, LINE_H * 5, "Map : V");
    ips200_show_uint(56, LINE_H * 5, current_map + 1, 2);
    ips200_show_string(0, LINE_H * 6, "Mode: ");
    show_text_value(48, LINE_H * 6, mode_name(mode), 8);
    ips200_show_string(0, LINE_H * 7, "Save: ");
    show_text_value(48, LINE_H * 7, save_state_name(save_state), 12);
    ips200_show_string(0, LINE_H * 8, "State: ");
    show_text_value(56, LINE_H * 8, state, 10);
    ips200_show_string(0, LINE_H * 9, "Last: ");
    ips200_show_uint(48, LINE_H * 9, elapsed_ms, 5);
    ips200_show_string(96, LINE_H * 9, "ms");
    show_hint("K1/K2 Move  K3 Enter", "K4 Home  K4L Home");
}

void screen_draw_map_select(uint8 current_map, uint8 candidate_map, save_state_enum save_state, const char *state)
{
    begin_page(SCREEN_PAGE_RUN_MAP);
    ips200_show_string(0, 0, "Run/Map");
    ips200_show_string(0, LINE_H, "Current: V");
    ips200_show_uint(80, LINE_H, current_map + 1, 2);
    ips200_show_string(0, LINE_H * 2, "Select : V");
    ips200_show_uint(80, LINE_H * 2, candidate_map + 1, 2);
    ips200_show_string(0, LINE_H * 3, "Save   : ");
    show_text_value(80, LINE_H * 3, save_state_name(save_state), 12);
    ips200_show_string(0, LINE_H * 4, "State  : ");
    show_text_value(80, LINE_H * 4, state, 10);
    draw_color_map(map_get(candidate_map), 0, 0);
    show_hint("K1/K2 Map  K3 OK", "K4 Cancel  K4L Home");
}

void screen_draw_mode_page(run_mode_enum candidate_mode)
{
    screen_draw_mode_select(candidate_mode);
}

void screen_draw_run_workbench(uint8 current_map, run_mode_enum mode, save_state_enum save_state, const char *state, uint32 elapsed_ms)
{
    begin_page(SCREEN_PAGE_RUN);
    ips200_show_string(0, 0, "Run");
    ips200_show_string(0, LINE_H, "Map : V");
    ips200_show_uint(56, LINE_H, current_map + 1, 2);
    ips200_show_string(96, LINE_H, "Mode:");
    show_text_value(136, LINE_H, mode_name(mode), 8);
    ips200_show_string(0, LINE_H * 2, "Save: ");
    show_text_value(48, LINE_H * 2, save_state_name(save_state), 12);
    ips200_show_string(0, LINE_H * 3, "State:");
    show_text_value(48, LINE_H * 3, state, 10);
    ips200_show_string(120, LINE_H * 3, "Last:");
    ips200_show_uint(160, LINE_H * 3, elapsed_ms, 5);
    draw_color_map(map_get(current_map), 0, 0);
    show_hint("K1/K2 Map  K3 Run", "K3L Mode  K4 Home");
}

void screen_draw_run(uint8 current_map, uint8 candidate_map, run_mode_enum mode, save_state_enum save_state, const char *state, uint32 elapsed_ms)
{
    begin_page(SCREEN_PAGE_RUN);
    ips200_show_string(0, 0, "Run");
    ips200_show_string(0, LINE_H, "Map : V");
    ips200_show_uint(56, LINE_H, current_map + 1, 2);
    ips200_show_string(0, LINE_H * 2, "Pick: V");
    ips200_show_uint(56, LINE_H * 2, candidate_map + 1, 2);
    ips200_show_string(0, LINE_H * 3, "Mode: ");
    show_text_value(48, LINE_H * 3, mode_name(mode), 8);
    ips200_show_string(0, LINE_H * 4, "Save: ");
    show_text_value(48, LINE_H * 4, save_state_name(save_state), 12);
    ips200_show_string(0, LINE_H * 5, "State: ");
    show_text_value(56, LINE_H * 5, state, 10);
    ips200_show_string(0, LINE_H * 6, "Time: ");
    ips200_show_uint(48, LINE_H * 6, elapsed_ms, 5);
    ips200_show_string(96, LINE_H * 6, "ms");
    draw_color_map(map_get(candidate_map), 0, 0);
    show_hint("K1/K2 Map  K3 OK/Run", "K2L Mode  K4 Home");
}

void screen_draw_mode_select(run_mode_enum candidate_mode)
{
    uint8 i;

    begin_page(SCREEN_PAGE_MODE_SELECT);
    ips200_show_string(0, 0, "Mode Select");
    for(i = 0; i < RUN_MODE_COUNT; i++)
    {
        ips200_show_string(0, (uint16)(LINE_H * (i + 1)), (i == (uint8)candidate_mode) ? ">" : " ");
        ips200_show_string(16, (uint16)(LINE_H * (i + 1)), mode_name((run_mode_enum)i));
    }
    show_hint("K1/K2 Mode  K3 OK", "K4 Cancel  K4L Home");
}

void screen_draw_playback(uint8 map_index, const map_source_struct *source, const solve_result_struct *result, uint16 step, uint32 elapsed_ms, playback_state_enum state)
{
    char action = '-';
    const char *state_name = "Paused";
    uint16 visible_step;
    uint16 visible_total;

    if((0 < step) && (step <= result->action_count))
    {
        action = result->actions[step - 1];
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
    ips200_show_string(0, 0, "Playback V");
    ips200_show_uint(80, 0, map_index + 1, 2);
    ips200_show_string(0, LINE_H, "Step: ");
    ips200_show_uint(48, LINE_H, visible_step, 3);
    ips200_show_string(80, LINE_H, "/");
    ips200_show_uint(88, LINE_H, visible_total, 3);
    ips200_show_string(0, LINE_H * 2, "Act : ");
    ips200_show_char(48, LINE_H * 2, action);
    ips200_show_string(0, LINE_H * 3, "Time: ");
    ips200_show_uint(48, LINE_H * 3, elapsed_ms, 5);
    ips200_show_string(96, LINE_H * 3, "ms");
    ips200_show_string(0, LINE_H * 4, "State: ");
    show_text_value(56, LINE_H * 4, state_name, 8);
    draw_color_map(source, result, step);
    if(PLAYBACK_STATE_FAIL == state)
    {
        clear_text_area(0, LINE_H * 5, 30);
        ips200_show_string(0, LINE_H * 5, result->message);
    }
    else
    {
        clear_text_area(0, LINE_H * 5, 30);
    }
    show_hint("K1 Prev K2 Next K3 Play", "K4 Run  K4L Home");
}

void screen_draw_demo(uint8 index, uint8 total, uint8 ok_count, uint8 fail_count, uint32 elapsed_ms, const char *state)
{
    begin_page(SCREEN_PAGE_DEMO);
    ips200_show_string(0, 0, "Demo");
    ips200_show_string(0, LINE_H, "Map : ");
    ips200_show_uint(48, LINE_H, index, 3);
    ips200_show_string(80, LINE_H, "/");
    ips200_show_uint(88, LINE_H, total, 3);
    ips200_show_string(0, LINE_H * 2, "OK  : ");
    ips200_show_uint(48, LINE_H * 2, ok_count, 3);
    ips200_show_string(0, LINE_H * 3, "Fail: ");
    ips200_show_uint(48, LINE_H * 3, fail_count, 3);
    ips200_show_string(0, LINE_H * 4, "Time: ");
    ips200_show_uint(48, LINE_H * 4, elapsed_ms, 5);
    ips200_show_string(96, LINE_H * 4, "ms");
    ips200_show_string(0, LINE_H * 5, "State: ");
    show_text_value(56, LINE_H * 5, state, 10);
    show_hint("K4 Stop", "K4L Home");
}

void screen_draw_debug(uint8 map_index, const map_source_struct *source, float pose_x_cm, float pose_y_cm)
{
    uint8 pose_row;
    uint8 pose_col;

    begin_page(SCREEN_PAGE_DEBUG);
    ips200_show_string(0, 0, "Pose Map");
    ips200_show_string(0, LINE_H, "Map : V");
    ips200_show_uint(56, LINE_H, map_index + 1, 2);
    ips200_show_string(0, LINE_H * 2, "X:");
    ips200_show_float(16, LINE_H * 2, (double)pose_x_cm, 4, 1);
    ips200_show_string(96, LINE_H * 2, " Y:");
    ips200_show_float(120, LINE_H * 2, (double)pose_y_cm, 4, 1);
    draw_pose_map(source, pose_x_cm, pose_y_cm, &pose_row, &pose_col);
    ips200_show_string(0, LINE_H * 3, "R:");
    ips200_show_uint(16, LINE_H * 3, pose_row, 2);
    ips200_show_string(40, LINE_H * 3, "C:");
    ips200_show_uint(56, LINE_H * 3, pose_col, 2);
    show_hint("K4 Home", "K4L Home");
}

void screen_draw_info(uint8 map_count_value, save_state_enum save_state)
{
    begin_page(SCREEN_PAGE_INFO);
    ips200_show_string(0, 0, "Info");
    ips200_show_string(0, LINE_H, "Map count: ");
    ips200_show_uint(88, LINE_H, map_count_value, 3);
    ips200_show_string(0, LINE_H * 2, "Flash   : ");
    show_text_value(80, LINE_H * 2, flash_state_name(save_state), 18);
    ips200_show_string(0, LINE_H * 3, "Save    : ");
    show_text_value(80, LINE_H * 3, save_state_name(save_state), 12);
    ips200_show_string(0, LINE_H * 4, "Version : 1");
    show_hint("K4 Home", "K4L Home");
}
