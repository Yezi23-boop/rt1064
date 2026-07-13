#include "zf_common_headfile.h"
#include "screen.h"
#include "drive_config.h"
#include "map_utils.h"
#include "vision_uart.h"

#define IPS200_TYPE             (IPS200_TYPE_SPI)
#define LINE_H                  (16)    // IPS200 8x16 字体的行高，单位 pixel。
#define MAP_X                   (8)     // 地图左上角 X 坐标，单位 pixel。
#define MAP_Y                   (112)   // 地图左上角 Y 坐标，避开 Run/Playback 顶部文字区。
#define CELL_SIZE               (14)    // 12x16 地图在 240x320 屏上的单格边长，单位 pixel。
#define KEY_HINT_Y1             (288)   // 底部按键提示第一行，单位 pixel。
#define KEY_HINT_Y2             (304)   // 底部按键提示第二行，单位 pixel。
#define EXEC_LINE_Y(n)          (MAP_Y + MAP_ROWS * CELL_SIZE + 2 + (n) * LINE_H) // executor 状态行：地图下方，第 n 行。
#define MENU_CURSOR_X           (0)
#define MENU_TEXT_X             (16)
#define MENU_ROW_Y(row)         ((uint16)(((row) + 1) * LINE_H))
#define SCREEN_TEXT_COLOR       (RGB565_BLACK)
#define SCREEN_BG_COLOR         (RGB565_WHITE)
// 地图色块语义与推箱子元素一致：墙/障碍黑色、箱子黄色、目标紫色、虚拟车青色。
// 执行页的蓝色车格表示 MCU 根据起点和里程计推算的位置，用于和 ART 识别车格对照。
#define WALL_COLOR              (RGB565_BLACK)
#define BOX_COLOR               (RGB565_YELLOW)
#define TARGET_COLOR            (RGB565_PURPLE)
#define CAR_COLOR               (RGB565_CYAN)
#define POSE_CAR_COLOR          (RGB565_BLUE)
#define EMPTY_COLOR             (RGB565_WHITE)
#define FILL_BUFFER_PIXELS      (240 * LINE_H) // 单行文字清屏所需最大像素数，复用作小矩形填充缓存。

typedef enum
{
    SCREEN_PAGE_NONE = 0,    /**< 尚未绘制任何页面，下一次 begin_page 必须清屏。 */
    SCREEN_PAGE_HOME,        /**< Home 菜单页。 */
    SCREEN_PAGE_RUN,         /**< Run 工作台页。 */
    SCREEN_PAGE_MODE_SELECT, /**< Run 子页：模式选择。 */
    SCREEN_PAGE_PLAYBACK,    /**< 求解结果回放页。 */
    SCREEN_PAGE_DEBUG,       /**< 本地 pose 地图调试页。 */
    SCREEN_PAGE_RUN_EXECUTE, /**< executor 执行页。 */
    SCREEN_PAGE_INFO,        /**< 信息页。 */
    SCREEN_PAGE_ART_MAP,     /**< OpenART 最近完整帧预览页。 */
} screen_page_enum;

typedef struct
{
    const map_source_struct *source;     /**< 待绘制的地图来源，可为 NULL。 */
    const solve_result_struct *result;   /**< 可选求解结果；用于按 action step 回放动态箱子位置。 */
    uint16 step;                         /**< 当前 action/waypoint 对应的动作步，下游会跳过 `|` 分隔符。 */
    uint8 use_pose;                      /**< 非 0 时叠加 MCU 本地 pose 推算车格。 */
    uint8 start_row;                     /**< pose 坐标原点对应的地图行，仅 `use_pose` 时使用。 */
    uint8 start_col;                     /**< pose 坐标原点对应的地图列，仅 `use_pose` 时使用。 */
    float pose_x_cm;                     /**< MCU 本地 X 位移，单位 cm，右移为正。 */
    float pose_y_cm;                     /**< MCU 本地 Y 位移，单位 cm，前进为正。 */
    uint8 *pose_row;                     /**< 输出：pose 换算后的行号，可为 NULL。 */
    uint8 *pose_col;                     /**< 输出：pose 换算后的列号，可为 NULL。 */
} screen_map_render_struct;

static screen_page_enum active_page = SCREEN_PAGE_NONE; // 只在页面切换时整屏清空，减少白底页面刷新闪烁。
AT_SDRAM_SECTION_ALIGN(uint16 screen_fill_buffer[FILL_BUFFER_PIXELS], 64); // 显示填充缓存不参与 BFS，放 cacheable SDRAM 减少片上 RAM 压力。

static void draw_mode_select(run_mode_enum candidate_mode);
static void draw_pose_map_from_start(const map_source_struct *source, const solve_result_struct *result, uint16 step, float pose_x_cm, float pose_y_cm, uint8 start_row, uint8 start_col, uint8 *pose_row, uint8 *pose_col);

static void begin_page(screen_page_enum page)
{
    if(active_page != page)
    {
        ips200_clear();
        active_page = page;
    }
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
        cells[*count] = map_cell_index(row, col);
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

static void show_float_value(uint16 x, uint16 y, float value, uint8 int_digits, uint8 frac_digits)
{
    ips200_show_float(x, y, (double)value, int_digits, frac_digits);
}

static void show_hint(const char *line1, const char *line2)
{
    clear_text_area(0, KEY_HINT_Y1, 30);
    clear_text_area(0, KEY_HINT_Y2, 30);
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
                // `|` 分隔符表示一个箱子任务完成；完成后的箱子/目标不再参与后续回放绘制。
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
                // 屏幕和求解器都把 X 当作不可通行障碍显示，避免动态地图与实际约束不一致。
                grid[row][col] = '#';
            }
            else if('C' == value)
            {
                *car = map_cell_index(row, col);
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
            // 分隔符不是车辆动作，只触发“已完成目标消失”的回放效果。
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

        next_car = map_cell_index((uint8)((int16)map_cell_row(*car) + dr), (uint8)((int16)map_cell_col(*car) + dc));
        if((action >= 'A') && (action <= 'Z'))
        {
            for(box_i = 0; box_i < *box_count; box_i++)
            {
                if(boxes[box_i] == next_car)
                {
                    next_box = map_cell_index((uint8)((int16)map_cell_row(boxes[box_i]) + dr), (uint8)((int16)map_cell_col(boxes[box_i]) + dc));
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
        return (int16)((value_cm + GRID_SIZE_CM * 0.5f) / GRID_SIZE_CM);
    }
    return (int16)((value_cm - GRID_SIZE_CM * 0.5f) / GRID_SIZE_CM);
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

static uint16 action_step_from_waypoint_step(const solve_result_struct *result, uint16 waypoint_step)
{
    if((0 == result) || (0 == waypoint_step))
    {
        return 0;
    }

    if(waypoint_step >= result->waypoint_count)
    {
        return result->action_count;
    }

    return result->waypoints[waypoint_step].action_start;
}

static uint8 waypoint_is_push(const waypoint_struct *wp)
{
    return ((0 != wp) && (wp->action >= 'A') && (wp->action <= 'Z')) ? 1u : 0u;
}

static uint16 action_step_for_execute_preview(const solve_result_struct *result, uint16 waypoint_step)
{
    uint16 action_step = action_step_from_waypoint_step(result, waypoint_step);
    const waypoint_struct *wp;

    if((0 == result) || (0 == result->solved) || (waypoint_step >= result->waypoint_count))
    {
        return action_step;
    }

    wp = &result->waypoints[waypoint_step];
    if(0 == waypoint_is_push(wp))
    {
        return action_step;
    }

    // 执行页按 waypoint 显示，推箱 waypoint 需要预览到 action_end，才能让箱子位置与当前目标一致。
    if(wp->action_end > result->action_count)
    {
        return result->action_count;
    }

    return wp->action_end;
}

static void display_task_progress(const solve_result_struct *result, uint16 waypoint_step, uint16 *current_task, uint16 *total_tasks)
{
    uint16 action_step;
    uint16 i;
    uint16 completed_tasks = 0;

    *current_task = 0;
    *total_tasks = 0;

    if((0 == result) || (0 == result->solved) || (0 == result->task_count))
    {
        return;
    }

    *total_tasks = result->task_count;
    action_step = action_step_from_waypoint_step(result, waypoint_step);

    for(i = 0; (i < action_step) && (i < result->action_count); i++)
    {
        if('|' == result->actions[i])
        {
            completed_tasks++;
        }
    }

    if(action_step >= result->action_count)
    {
        *current_task = *total_tasks;
    }
    else
    {
        *current_task = (uint16)(completed_tasks + 1u);
        if(*current_task > *total_tasks)
        {
            *current_task = *total_tasks;
        }
    }
}

static void draw_map_render(const screen_map_render_struct *render)
{
    char grid[MAP_ROWS][MAP_COLS];
    uint16 car;
    uint16 pose_car;
    uint16 boxes[MAX_BOXES];
    uint16 targets[MAX_BOXES];
    uint8 box_count;
    uint8 target_count;
    uint8 row;
    uint8 col;
    int16 pose_row_value;
    int16 pose_col_value;
    uint16 cell;
    uint16 color;
    uint16 action_step;

    parse_source(render->source, grid, &car, boxes, &box_count, targets, &target_count);

    if((0 != render->result) && (0 != render->result->solved) && (0 < render->step))
    {
        if(0 != render->use_pose)
        {
            action_step = action_step_for_execute_preview(render->result, render->step);
        }
        else
        {
            action_step = render->step;
        }
        apply_actions(&car, boxes, &box_count, targets, &target_count, render->result->actions, action_step);
    }

    if(0 != render->use_pose)
    {
        // pose_x 向右为正、pose_y 向前为正；地图行号向下增加，所以 Y 位移要反向换算成行偏移。
        // 越界时夹到屏幕边界，只影响显示诊断，不改变执行器或求解状态。
        pose_col_value = (int16)render->start_col + round_cm_to_grid_delta(render->pose_x_cm);
        pose_row_value = (int16)render->start_row - round_cm_to_grid_delta(render->pose_y_cm);
        *render->pose_row = clamp_grid_index(pose_row_value, MAP_ROWS);
        *render->pose_col = clamp_grid_index(pose_col_value, MAP_COLS);
        pose_car = map_cell_index(*render->pose_row, *render->pose_col);
    }
    else
    {
        pose_car = car;
    }

    for(row = 0; row < MAP_ROWS; row++)
    {
        for(col = 0; col < MAP_COLS; col++)
        {
            cell = map_cell_index(row, col);
            if(0 != render->use_pose)
            {
                color = color_for_pose_cell(grid[row][col], cell, pose_car, boxes, box_count, targets, target_count);
            }
            else
            {
                color = color_for_cell(grid[row][col], cell, car, boxes, box_count, targets, target_count);
            }
            fill_rect((uint16)(MAP_X + col * CELL_SIZE), (uint16)(MAP_Y + row * CELL_SIZE), (CELL_SIZE - 1), (CELL_SIZE - 1), color);
        }
    }
}

static void draw_color_map(const map_source_struct *source, const solve_result_struct *result, uint16 step)
{
    screen_map_render_struct render;

    // 回放/ART Map 使用虚拟车位置；执行页才叠加 MCU 本地 pose 推算车格。
    render.source = source;
    render.result = result;
    render.step = step;
    render.use_pose = 0;
    render.start_row = 0;
    render.start_col = 0;
    render.pose_x_cm = 0.0f;
    render.pose_y_cm = 0.0f;
    render.pose_row = 0;
    render.pose_col = 0;
    draw_map_render(&render);
}

static uint16 find_start_car(const map_source_struct *source)
{
    uint8 row = 0;
    uint8 col = 0;
    uint8 count = 0;

    (void)map_find_car(source, &row, &col, &count);
    if(0 != count)
    {
        return map_cell_index(row, col);
    }
    return 0;
}

static void draw_pose_map(const map_source_struct *source, float pose_x_cm, float pose_y_cm, uint8 *pose_row, uint8 *pose_col)
{
    uint16 start_car;

    start_car = find_start_car(source);
    draw_pose_map_from_start(source, 0, 0, pose_x_cm, pose_y_cm,
                             map_cell_row(start_car),
                             map_cell_col(start_car),
                             pose_row,
                             pose_col);
}

static void draw_pose_map_from_start(const map_source_struct *source, const solve_result_struct *result, uint16 step, float pose_x_cm, float pose_y_cm, uint8 start_row, uint8 start_col, uint8 *pose_row, uint8 *pose_col)
{
    screen_map_render_struct render;

    render.source = source;
    render.result = result;
    render.step = step;
    render.use_pose = 1;
    render.start_row = start_row;
    render.start_col = start_col;
    render.pose_x_cm = pose_x_cm;
    render.pose_y_cm = pose_y_cm;
    render.pose_row = pose_row;
    render.pose_col = pose_col;
    draw_map_render(&render);
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

    ips200_show_string(0, LINE_H * 10, "IMU Y:");

    ips200_show_string(0, LINE_H * 11, "Yaw T:");
    ips200_show_string(96, LINE_H * 11, " C:");

    ips200_show_string(0, LINE_H * 12, "E:");
    ips200_show_string(72, LINE_H * 12, " Z:");
    ips200_show_string(136, LINE_H * 12, " T:");

    ips200_show_string(0, LINE_H * 13, "X:");
    ips200_show_string(96, LINE_H * 13, " Y:");

}

static const char *vision_board_test_state_text(uint8 state)
{
    switch(state)
    {
        case VISION_UART_BOARD_TEST_WAIT_READY:  return "MODE";
        case VISION_UART_BOARD_TEST_WAIT_SAMPLE: return "REQ";
        case VISION_UART_BOARD_TEST_PASS:        return "PASS";
        case VISION_UART_BOARD_TEST_FAIL:        return "FAIL";
        default:                                 return "OFF";
    }
}

static void draw_home_status_values(const screen_home_view_struct *view)
{
    ips200_show_int(56, LINE_H * 8, (int16)view->encoder_count[WHEEL_LF], 5);
    ips200_show_int(136, LINE_H * 8, (int16)view->encoder_count[WHEEL_RF], 5);

    ips200_show_int(56, LINE_H * 9, (int16)view->encoder_count[WHEEL_LB], 5);
    ips200_show_int(136, LINE_H * 9, (int16)view->encoder_count[WHEEL_RB], 5);

    ips200_show_int(48, LINE_H * 10, (int16)view->imu_yaw, 4);

    show_float_value(48, LINE_H * 11, view->target_yaw, 3, 1);
    show_float_value(120, LINE_H * 11, view->imu_yaw, 3, 1);

    show_float_value(16, LINE_H * 12, view->yaw_error, 3, 2);
    show_float_value(96, LINE_H * 12, view->vz, 1, 2);
    show_float_value(160, LINE_H * 12, view->vzt, 1, 2);

    show_float_value(16, LINE_H * 13, view->pose_x_cm, 4, 1);
    show_float_value(120, LINE_H * 13, view->pose_y_cm, 4, 1);

    clear_text_area(0, LINE_H * 14, 30);
    if(0 != view->vision_test_enabled)
    {
        ips200_show_string(0, LINE_H * 14, "V4:");
        ips200_show_string(24, LINE_H * 14,
                           vision_board_test_state_text(view->vision_test_state));
        ips200_show_string(64, LINE_H * 14, "N:");
        ips200_show_uint(80, LINE_H * 14, view->vision_test_samples, 1);
        ips200_show_string(96, LINE_H * 14, "C:");
        ips200_show_uint(112, LINE_H * 14, view->vision_test_class, 1);
        ips200_show_string(128, LINE_H * 14, "Q:");
        ips200_show_uint(144, LINE_H * 14, view->vision_test_confidence_q, 4);
    }
    else
    {
        ips200_show_string(0, LINE_H * 14, "ART:");
        ips200_show_uint(32, LINE_H * 14, view->openart_frame_count, 5);
    }
}

void screen_draw_home(const screen_home_view_struct *view)
{
    uint8 i;

    begin_page(SCREEN_PAGE_HOME);
    ips200_show_string(0, 0, "Home");
    for(i = 0; i < view->item_count; i++)
    {
        ips200_show_string(0, (uint16)(LINE_H * (i + 1)), (i == view->cursor) ? ">" : " ");
        ips200_show_string(16, (uint16)(LINE_H * (i + 1)), view->items[i]);
    }
    ips200_show_string(0, LINE_H * 5, "Map : V");
    ips200_show_uint(56, LINE_H * 5, view->current_map + 1, 2);
    ips200_show_string(0, LINE_H * 6, "Mode: ");
    show_text_value(48, LINE_H * 6, mode_name(view->mode), 8);
    ips200_show_string(0, LINE_H * 7, "Save: ");
    show_text_value(48, LINE_H * 7, save_state_name(view->save_state), 12);
    draw_home_status_labels();
    draw_home_status_values(view);
    show_hint("K1/K2 Move  K3 Enter", "K4 Save  K4L Home");
}

void screen_draw_home_status(const screen_home_view_struct *view)
{
    begin_page(SCREEN_PAGE_HOME);
    draw_home_status_values(view);
}

void screen_draw_nav_cursor(uint8 previous_cursor, uint8 cursor)
{
    ips200_show_string(MENU_CURSOR_X, MENU_ROW_Y(previous_cursor), " ");
    ips200_show_string(MENU_CURSOR_X, MENU_ROW_Y(cursor), ">");
}

void screen_draw_mode_page(run_mode_enum candidate_mode)
{
    draw_mode_select(candidate_mode);
}

static const char *executor_state_text(executor_state_enum state)
{
    switch(state)
    {
        case EXEC_STATE_IDLE:    return "IDLE";
        case EXEC_STATE_RUNNING: return "RUN";
        case EXEC_STATE_PAUSED:  return "PAUSE";
        case EXEC_STATE_DONE:    return "DONE";
        case EXEC_STATE_ERROR:   return "ERROR";
        default:                 return "?";
    }
}

static const char *executor_error_text(executor_error_enum error)
{
    switch(error)
    {
        case EXEC_ERROR_MAP:        return "E:MAP";
        case EXEC_ERROR_ART_TIMEOUT:return "E:ATO";
        case EXEC_ERROR_ART_SYNC:   return "E:SYN";
        case EXEC_ERROR_ART_PLAN:   return "E:PLN";
        case EXEC_ERROR_ART_CENTER: return "E:Ctr";
        case EXEC_ERROR_NONE:       return "E:OK";
        default:                    return "E:?";
    }
}

static void draw_executor_status(const screen_run_view_struct *view)
{
    uint16 y0 = EXEC_LINE_Y(0);
    uint16 y1 = EXEC_LINE_Y(1);
    uint16 current_task;
    uint16 total_tasks;

    display_task_progress(view->result, view->current_step, &current_task, &total_tasks);

    // Run 页只在执行器活跃时显示压缩状态，给地图区域保留固定尺寸，避免页面跳动。
    ips200_show_string(0, y0, "S:");
    clear_text_area(16, y0, 10);
    ips200_show_string(16, y0, executor_state_text(view->executor_state));
    ips200_show_string(104, y0, "St:");
    ips200_show_uint(124, y0, view->current_step, 3);
    ips200_show_string(148, y0, "/");
    ips200_show_uint(156, y0, view->total_steps, 3);

    ips200_show_string(0, y1, "B:");
    ips200_show_uint(16, y1, current_task, 2);
    ips200_show_string(32, y1, "/");
    ips200_show_uint(40, y1, total_tasks, 2);
    ips200_show_string(64, y1, "X:");
    ips200_show_float(76, y1, (double)view->pose_x_cm, 3, 1);
    ips200_show_string(120, y1, "Y:");
    ips200_show_float(132, y1, (double)view->pose_y_cm, 3, 1);
    if(EXEC_STATE_ERROR == view->executor_state)
    {
        ips200_show_string(176, y1, executor_error_text(view->executor_error));
    }
}

void screen_draw_run_workbench(const screen_run_view_struct *view)
{
    begin_page(SCREEN_PAGE_RUN);
    ips200_show_string(0, 0, "Run");
    ips200_show_string(0, LINE_H, "Map : V");
    ips200_show_uint(56, LINE_H, view->current_map + 1, 2);
    ips200_show_string(96, LINE_H, "Src:");
    show_text_value(128, LINE_H, source_name(view->source_type), 8);
    ips200_show_string(0, LINE_H * 2, "Mode: ");
    show_text_value(48, LINE_H * 2, mode_name(view->mode), 8);
    ips200_show_string(0, LINE_H * 3, "Save: ");
    show_text_value(48, LINE_H * 3, save_state_name(view->save_state), 12);
    ips200_show_string(0, LINE_H * 4, "State:");
    show_text_value(48, LINE_H * 4, view->state_text, 10);
    ips200_show_string(120, LINE_H * 4, "Last:");
    ips200_show_uint(160, LINE_H * 4, view->elapsed_ms, 5);
    if(0 != view->source)
    {
        draw_color_map(view->source, 0, 0);
    }

    if(0 != view->executor_active)
    {
        draw_executor_status(view);
    }
    else
    {
        show_hint("K1/K2 Map  K3 Run", "K3L Mode  K4 Src");
    }
}

static void draw_mode_select(run_mode_enum candidate_mode)
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

    // `|` 是任务边界，不是用户可执行动作；可见步数隐藏它，方便人工逐步核对路径。
    visible_step = count_visible_actions_to_step(result, step);
    visible_total = count_visible_actions(result);

    begin_page(SCREEN_PAGE_PLAYBACK);
    ips200_show_string(0, 0, "Playback V");
    ips200_show_uint(80, 0, map_index + 1, 2);
    if(0 == source)
    {
        ips200_show_string(0, LINE_H, "No Map");
        show_hint("K4 Run", "K4L Home");
        return;
    }
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

void screen_draw_execute(const screen_execute_view_struct *view)
{
    uint8 pose_row, pose_col;
    uint16 current_task;
    uint16 total_tasks;
    uint16 total_waypoints = 0;
    const waypoint_struct *wp = 0;

    if(0 != view->result)
    {
        total_waypoints = view->result->waypoint_count;
        if(view->current_step < total_waypoints)
        {
            wp = &view->result->waypoints[view->current_step];
        }
    }
    display_task_progress(view->result, view->current_step, &current_task, &total_tasks);

    // 执行页固定显示同一张求解快照，避免 ART 实时帧刷新导致车辆还在跑时地图底图跳变。
    begin_page(SCREEN_PAGE_RUN_EXECUTE);
    ips200_show_string(0, 0, "Execute");
    ips200_show_string(0, LINE_H, "Map:");
    ips200_show_uint(32, LINE_H, view->current_map + 1, 2);
    ips200_show_string(0, LINE_H * 2, "S:");
    show_text_value(16, LINE_H * 2,
                    (0 != view->state_text) ? view->state_text : executor_state_text(view->state),
                    8);

    if(0 == view->source)
    {
        ips200_show_string(0, LINE_H * 3, "No Map");
        show_hint("", "K4 Stop");
        return;
    }

    // 本地 pose 必须从执行启动格换算，不能从当前 ART 车格换算；否则识别抖动会污染里程计诊断。
    draw_pose_map_from_start(view->source,
                             view->result,
                             view->current_step,
                             view->pose_x_cm,
                             view->pose_y_cm,
                             view->start_row,
                             view->start_col,
                             &pose_row,
                             &pose_col);

    ips200_show_string(0, LINE_H * 2, "S:");
    show_text_value(16, LINE_H * 2,
                    (0 != view->state_text) ? view->state_text : executor_state_text(view->state),
                    8);
    ips200_show_string(80, LINE_H * 2, "St:");
    ips200_show_uint(104, LINE_H * 2, view->current_step, 3);
    ips200_show_string(128, LINE_H * 2, "/");
    ips200_show_uint(136, LINE_H * 2, total_waypoints, 3);
    if(EXEC_STATE_ERROR == view->state)
    {
        ips200_show_string(176, LINE_H * 2, executor_error_text(view->error));
    }

    ips200_show_string(0, LINE_H * 3, "B:");
    ips200_show_uint(16, LINE_H * 3, current_task, 2);
    ips200_show_string(32, LINE_H * 3, "/");
    ips200_show_uint(40, LINE_H * 3, total_tasks, 2);

    // M 是 MCU 本地推算格，W 是当前 waypoint；两者分开显示便于区分控制偏差和规划目标。
    ips200_show_string(64, LINE_H * 3, "M:");
    ips200_show_uint(80, LINE_H * 3, pose_row, 2);
    ips200_show_string(96, LINE_H * 3, ",");
    ips200_show_uint(104, LINE_H * 3, pose_col, 2);

    ips200_show_string(0, LINE_H * 4, "W:");
    if(0 != wp)
    {
        ips200_show_uint(16, LINE_H * 4, wp->row, 2);
        ips200_show_string(32, LINE_H * 4, ",");
        ips200_show_uint(40, LINE_H * 4, wp->col, 2);
        ips200_show_string(64, LINE_H * 4, "A:");
        ips200_show_char(80, LINE_H * 4, wp->action);
    }
    else
    {
        ips200_show_string(16, LINE_H * 4, "--,--");
        ips200_show_string(64, LINE_H * 4, "A:-");
    }

    if(0 != view->art_player_enabled)
    {
        // A/N 显示 OpenART 再识别到的 C 位置和数量；N!=1 时该坐标不可信，只用于诊断。
        ips200_show_string(0, LINE_H * 5, "A:");
        if(0 != view->art_player_valid)
        {
            ips200_show_uint(16, LINE_H * 5, view->art_row, 2);
            ips200_show_string(32, LINE_H * 5, ",");
            ips200_show_uint(40, LINE_H * 5, view->art_col, 2);
        }
        else
        {
            ips200_show_string(16, LINE_H * 5, "--,--");
        }
        ips200_show_string(80, LINE_H * 5, "N:");
        ips200_show_uint(96, LINE_H * 5, view->art_player_count, 2);
    }

    if(0 != view->art_launch_pending)
    {
        show_hint("K3 Launch", "K4 Stop");
    }
    else if(EXEC_STATE_PAUSED == view->state)
    {
        show_hint("K3 Resume", "K4 Stop");
    }
    else if((EXEC_STATE_ERROR == view->state) || (EXEC_STATE_DONE == view->state))
    {
        show_hint("K4 Back", "K4L Home");
    }
    else
    {
        show_hint("", "K4 Stop");
    }
}

void screen_draw_art_map(uint32 frame_count, uint32 age_s, const map_source_struct *source)
{
    begin_page(SCREEN_PAGE_ART_MAP);
    ips200_show_string(0, 0, "OpenART Map");
    ips200_show_string(0, LINE_H, "Frames:");
    ips200_show_uint(56, LINE_H, frame_count, 5);
    ips200_show_string(0, LINE_H * 2, "Age  :");

    if(0 == source)
    {
        clear_text_area(56, LINE_H * 2, 6);
        ips200_show_string(56, LINE_H * 2, "-");
        clear_text_area(0, LINE_H * 4, 30);
        ips200_show_string(0, LINE_H * 4, "No OpenART map");
    }
    else
    {
        ips200_show_uint(56, LINE_H * 2, age_s, 5);
        ips200_show_string(96, LINE_H * 2, "s");
        // ART Map 页显示最近完整帧快照；若串口正在接收下一帧，也不会把半帧画出来。
        draw_color_map(source, 0, 0);
    }

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
