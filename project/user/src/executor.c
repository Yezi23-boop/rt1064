#include "executor.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "motion_math.h"
#include "openart_uart.h"
#include <math.h>

/* 执行器内部状态 */
static executor_state_enum exec_state = EXEC_STATE_IDLE;
static executor_error_enum exec_error = EXEC_ERROR_NONE;

/* 路径数据 */
static const waypoint_struct *exec_waypoints = NULL;
static uint16 exec_waypoint_count = 0;
static uint16 current_step = 0;
static uint8 start_row = 0;
static uint8 start_col = 0;
static uint16 expected_player_cell = 0;
static uint16 expected_boxes[MAX_BOXES];
static uint8 expected_box_count = 0;
static uint16 expected_targets[MAX_BOXES];
static uint8 expected_target_count = 0;
static uint8 expected_state_valid = 0;

/* 路径跟踪PID实例 */
static path_pid_struct x_pid;
static path_pid_struct y_pid;

/* 单步模式 */
static uint8 single_step_mode = 0;
static uint8 arrival_stable_ticks = 0;
static uint8 segment_settling = 0;
static uint16 segment_settle_elapsed_ms = 0;
static uint8 art_verify_enabled = 0;
static uint8 segment_waiting_art = 0;
static uint16 art_verify_elapsed_ms = 0;
static uint8 art_confirm_count = 0;
static uint32 art_last_checked_frame = 0;

/* 将网格坐标转换为物理坐标（以起点为原点） */
static void grid_to_physical(uint8 row, uint8 col, float *x_cm, float *y_cm)
{
    *x_cm = (float)(col - start_col) * GRID_SIZE_CM;
    *y_cm = -(float)(row - start_row) * GRID_SIZE_CM;
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

static uint8 abs_int16_to_u8(int16 value)
{
    if(value < 0)
    {
        value = (int16)-value;
    }
    return (uint8)value;
}

static uint8 cell_step(uint16 cell, int8 dr, int8 dc, uint16 *next_cell)
{
    int16 row = (int16)cell_row_local(cell) + dr;
    int16 col = (int16)cell_col_local(cell) + dc;

    if((row < 0) || (row >= MAP_ROWS) || (col < 0) || (col >= MAP_COLS))
    {
        return 0;
    }

    *next_cell = cell_index_local((uint8)row, (uint8)col);
    return 1;
}

static int8 waypoint_row_delta(char action)
{
    if(('u' == action) || ('U' == action))
    {
        return -1;
    }
    if(('d' == action) || ('D' == action))
    {
        return 1;
    }
    return 0;
}

static int8 waypoint_col_delta(char action)
{
    if(('l' == action) || ('L' == action))
    {
        return -1;
    }
    if(('r' == action) || ('R' == action))
    {
        return 1;
    }
    return 0;
}

static uint8 cell_in_list(const uint16 *list, uint8 count, uint16 cell)
{
    uint8 i;

    for(i = 0; i < count; i++)
    {
        if(list[i] == cell)
        {
            return 1;
        }
    }
    return 0;
}

static uint8 find_box_index(const uint16 *boxes, uint8 count, uint16 cell, uint8 *index)
{
    uint8 i;

    for(i = 0; i < count; i++)
    {
        if(boxes[i] == cell)
        {
            *index = i;
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
    (*count)--;
}

static void remove_solved_boxes_from_expected(uint16 *boxes, uint8 *box_count)
{
    uint8 i = 0;

    while(i < *box_count)
    {
        if(0 != cell_in_list(expected_targets, expected_target_count, boxes[i]))
        {
            remove_index(boxes, box_count, i);
        }
        else
        {
            i++;
        }
    }
}

static uint8 unordered_cells_equal(const uint16 *left, uint8 left_count, const uint16 *right, uint8 right_count)
{
    uint8 i;

    if(left_count != right_count)
    {
        return 0;
    }

    for(i = 0; i < left_count; i++)
    {
        if(0 == cell_in_list(right, right_count, left[i]))
        {
            return 0;
        }
    }
    return 1;
}

static void expected_state_reset(void)
{
    expected_player_cell = 0;
    expected_box_count = 0;
    expected_target_count = 0;
    expected_state_valid = 0;
}

static void expected_state_init_from_source(const map_source_struct *source)
{
    uint8 row;
    uint8 col;
    char value;

    expected_state_reset();
    if(0 == source)
    {
        return;
    }

    for(row = 0; row < MAP_ROWS; row++)
    {
        for(col = 0; col < MAP_COLS; col++)
        {
            value = source->rows[row][col];
            if('C' == value)
            {
                expected_player_cell = cell_index_local(row, col);
                expected_state_valid = 1;
            }
            else if(('B' == value) && (expected_box_count < MAX_BOXES))
            {
                expected_boxes[expected_box_count] = cell_index_local(row, col);
                expected_box_count++;
            }
            else if(('T' == value) && (expected_target_count < MAX_BOXES))
            {
                expected_targets[expected_target_count] = cell_index_local(row, col);
                expected_target_count++;
            }
        }
    }
}

static uint8 expected_state_after_waypoint(const waypoint_struct *wp,
                                           uint16 out_boxes[MAX_BOXES],
                                           uint8 *out_box_count,
                                           uint16 *out_player)
{
    uint16 player = expected_player_cell;
    uint16 next_player;
    uint16 next_box;
    uint16 target_cell;
    uint8 temp_box_count = expected_box_count;
    uint8 box_index;
    uint8 steps;
    uint8 i;
    int8 dr = waypoint_row_delta(wp->action);
    int8 dc = waypoint_col_delta(wp->action);

    for(i = 0; i < expected_box_count; i++)
    {
        out_boxes[i] = expected_boxes[i];
    }

    target_cell = cell_index_local(wp->row, wp->col);
    steps = (uint8)(abs_int16_to_u8((int16)wp->row - (int16)cell_row_local(player)) +
                    abs_int16_to_u8((int16)wp->col - (int16)cell_col_local(player)));

    for(i = 0; i < steps; i++)
    {
        if(0 == cell_step(player, dr, dc, &next_player))
        {
            return 0;
        }

        if((wp->action >= 'A') && (wp->action <= 'Z'))
        {
            if(0 == find_box_index(out_boxes, temp_box_count, next_player, &box_index))
            {
                return 0;
            }
            if(0 == cell_step(out_boxes[box_index], dr, dc, &next_box))
            {
                return 0;
            }
            out_boxes[box_index] = next_box;
        }
        player = next_player;
    }

    if(player != target_cell)
    {
        return 0;
    }

    remove_solved_boxes_from_expected(out_boxes, &temp_box_count);
    *out_box_count = temp_box_count;
    *out_player = player;
    return 1;
}

static void expected_state_commit(uint16 player, const uint16 boxes[MAX_BOXES], uint8 box_count)
{
    uint8 i;

    expected_player_cell = player;
    expected_box_count = box_count;
    for(i = 0; i < box_count; i++)
    {
        expected_boxes[i] = boxes[i];
    }
}

void executor_init(void)
{
    /* 初始化路径跟踪PID */
    path_pid_init(&x_pid, PATH_KP, PATH_KI, PATH_KD, PATH_MAX_SPEED, PATH_MAX_INTEGRAL);
    path_pid_init(&y_pid, PATH_KP, PATH_KI, PATH_KD, PATH_MAX_SPEED, PATH_MAX_INTEGRAL);
}

static void executor_reset_segment_state(void)
{
    arrival_stable_ticks = 0;
    segment_settling = 0;
    segment_settle_elapsed_ms = 0;
    segment_waiting_art = 0;
    art_verify_elapsed_ms = 0;
    art_confirm_count = 0;
    art_last_checked_frame = 0;
    path_pid_reset(&x_pid);
    path_pid_reset(&y_pid);
}

static uint8 action_is_x_axis(char action)
{
    return (('l' == action) || ('L' == action) || ('r' == action) || ('R' == action)) ? 1u : 0u;
}

static uint8 action_is_y_axis(char action)
{
    return (('u' == action) || ('U' == action) || ('d' == action) || ('D' == action)) ? 1u : 0u;
}

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 检查是否到达当前 action 对应的目标轴 */
static uint8 is_axis_arrived(float target_x, float target_y, char action)
{
    const drive_pose_struct *pose = drive_pose_get();
    float dx = target_x - pose->x_cm;
    float dy = target_y - pose->y_cm;
    float distance = sqrtf(dx * dx + dy * dy);

    if (0 != action_is_x_axis(action))
    {
        return ((abs_float(dx) < PATH_ARRIVAL_THRESHOLD_CM) &&
                (abs_float(dy) < PATH_ARRIVAL_THRESHOLD_CM));
    }
    if (0 != action_is_y_axis(action))
    {
        return ((abs_float(dy) < PATH_ARRIVAL_THRESHOLD_CM) &&
                (abs_float(dx) < PATH_ARRIVAL_THRESHOLD_CM));
    }
    return (distance < PATH_ARRIVAL_THRESHOLD_CM);
}

/* 世界坐标转车体坐标 */
static void world_velocity_to_body(float vx_world, float vy_world, float yaw_deg, float *vx_body, float *vy_body)
{
    float yaw_rad = yaw_deg * 3.1415926f / 180.0f;
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    *vx_body = vx_world * cos_yaw + vy_world * sin_yaw;
    *vy_body = -vx_world * sin_yaw + vy_world * cos_yaw;
}

static float path_pid_update_with_arrival_deadband(path_pid_struct *pid, float error)
{
    if(abs_float(error) < PATH_ARRIVAL_THRESHOLD_CM)
    {
        path_pid_reset(pid);
        return 0.0f;
    }

    return path_pid_update(pid, error, CONTROL_DT_S);
}

/* 向目标位置移动（使用PID） */
static void move_to_target(float target_x, float target_y, char action)
{
    const drive_pose_struct *pose = drive_pose_get();
    float dx = target_x - pose->x_cm;
    float dy = target_y - pose->y_cm;
    float vx_world, vy_world;
    float vx_body, vy_body;

    /* 使用PID计算世界坐标速度 */
    vx_world = path_pid_update_with_arrival_deadband(&x_pid, dx);
    vy_world = path_pid_update_with_arrival_deadband(&y_pid, dy);

    /* 世界坐标转车体坐标 */
    world_velocity_to_body(vx_world, vy_world, pose->yaw_deg, &vx_body, &vy_body);

    /* 设置运动 */
    set_motion(vx_body, vy_body);
}

void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 start_row_param, uint8 start_col_param, uint8 single_step,
                    uint8 art_verify, const map_source_struct *source)
{
    /* 参数检查 */
    if (waypoints == NULL || count == 0)
    {
        exec_state = EXEC_STATE_ERROR;
        exec_error = EXEC_ERROR_MAP;
        return;
    }

    /* 保存路径数据 */
    exec_waypoints = waypoints;
    exec_waypoint_count = count;
    start_row = start_row_param;
    start_col = start_col_param;
    single_step_mode = single_step;
    art_verify_enabled = art_verify;
    expected_state_init_from_source(source);

    /* 重置状态 */
    current_step = 0;
    exec_error = EXEC_ERROR_NONE;
    executor_reset_segment_state();

    /* 重置位姿，以起点为原点 */
    drive_pose_reset(0.0f, 0.0f, 0.0f);

    /* 设置初始状态 */
    if (single_step_mode)
    {
        exec_state = EXEC_STATE_PAUSED;
    }
    else
    {
        exec_state = EXEC_STATE_RUNNING;
    }
}

void executor_stop(void)
{
    stop_motion();
    exec_state = EXEC_STATE_IDLE;
    exec_error = EXEC_ERROR_NONE;
    exec_waypoints = NULL;
    exec_waypoint_count = 0;
    current_step = 0;
    art_verify_enabled = 0;
    expected_state_reset();
    executor_reset_segment_state();
}

void executor_resume(void)
{
    if (exec_state == EXEC_STATE_PAUSED)
    {
        executor_reset_segment_state();
        exec_state = EXEC_STATE_RUNNING;
    }
}

static void executor_enter_segment_settle(void)
{
    reset_motion_segment();
    executor_reset_segment_state();
    segment_settling = 1;
}

static void executor_advance_after_segment(void)
{
    current_step++;
    executor_reset_segment_state();

    if (current_step >= exec_waypoint_count)
    {
        stop_motion();
        exec_state = EXEC_STATE_DONE;
        return;
    }

    if (single_step_mode)
    {
        stop_motion();
        exec_state = EXEC_STATE_PAUSED;
    }
}

static void executor_correct_pose_and_advance(float target_x, float target_y)
{
    const drive_pose_struct *pose = drive_pose_get();

    drive_pose_reset(target_x, target_y, pose->yaw_deg);
    executor_advance_after_segment();
}

static void executor_enter_art_wait(void)
{
    segment_settling = 0;
    segment_settle_elapsed_ms = 0;
    segment_waiting_art = 1;
    art_verify_elapsed_ms = 0;
    art_confirm_count = 0;
    art_last_checked_frame = openart_uart_get_frame_count();
}

static void executor_finish_segment_settle(float target_x, float target_y)
{
    segment_settling = 0;
    segment_settle_elapsed_ms = 0;

    if (0 != art_verify_enabled)
    {
        executor_enter_art_wait();
        return;
    }

    executor_advance_after_segment();
}

static void executor_fail_art(executor_error_enum error)
{
    stop_motion();
    executor_reset_segment_state();
    exec_error = error;
    exec_state = EXEC_STATE_ERROR;
}

static void executor_update_art_wait_20ms(float target_x, float target_y, const waypoint_struct *wp)
{
    uint8 art_row = 0;
    uint8 art_col = 0;
    uint8 art_count = 0;
    uint32 art_frame = 0;
    uint16 art_boxes[MAX_BOXES];
    uint8 art_box_count = 0;
    uint16 next_expected_boxes[MAX_BOXES];
    uint8 next_expected_box_count = 0;
    uint16 next_expected_player = 0;
    uint8 boxes_match = 0;

    reset_motion_segment();

    if (art_verify_elapsed_ms >= EXEC_ART_VERIFY_TIMEOUT_MS)
    {
        executor_fail_art(EXEC_ERROR_ART_TIMEOUT);
        return;
    }

    if (0 != openart_get_player_cell(&art_row, &art_col, &art_count, &art_frame))
    {
        if (art_frame != art_last_checked_frame)
        {
            art_last_checked_frame = art_frame;
            if((0 != expected_state_valid) &&
               (0 != expected_state_after_waypoint(wp, next_expected_boxes, &next_expected_box_count, &next_expected_player)) &&
               (0 != openart_get_box_cells(art_boxes, &art_box_count, 0)))
            {
                boxes_match = unordered_cells_equal(next_expected_boxes, next_expected_box_count,
                                                   art_boxes, art_box_count);
            }

            if ((art_row == wp->row) && (art_col == wp->col) && (0 != boxes_match))
            {
                if (art_confirm_count < EXEC_ART_CONFIRM_FRAMES)
                {
                    art_confirm_count++;
                }
                if (art_confirm_count >= EXEC_ART_CONFIRM_FRAMES)
                {
                    expected_state_commit(next_expected_player, next_expected_boxes, next_expected_box_count);
                    executor_correct_pose_and_advance(target_x, target_y);
                    return;
                }
            }
            else
            {
                art_confirm_count = 0;
            }
        }
    }
    else if((0 != art_frame) && (art_frame != art_last_checked_frame) && (1u != art_count))
    {
        art_last_checked_frame = art_frame;
        art_confirm_count = 0;
    }

    art_verify_elapsed_ms += CONTROL_PERIOD_MS;
}

static void executor_update_segment_settle_20ms(float target_x, float target_y)
{
    reset_motion_segment();
    if (segment_settle_elapsed_ms >= EXEC_SEGMENT_SETTLE_MS)
    {
        executor_finish_segment_settle(target_x, target_y);
        return;
    }

    segment_settle_elapsed_ms += CONTROL_PERIOD_MS;
}

void executor_update_20ms(void)
{
    /* 只在运行状态执行 */
    if (exec_state != EXEC_STATE_RUNNING)
    {
        return;
    }

    /* 检查是否完成所有步骤 */
    if (current_step >= exec_waypoint_count)
    {
        stop_motion();
        exec_state = EXEC_STATE_DONE;
        return;
    }

    /* 获取当前目标 */
    const waypoint_struct *wp = &exec_waypoints[current_step];
    float target_x, target_y;
    grid_to_physical(wp->row, wp->col, &target_x, &target_y);

    if (0 != segment_settling)
    {
        executor_update_segment_settle_20ms(target_x, target_y);
        return;
    }

    if (0 != segment_waiting_art)
    {
        executor_update_art_wait_20ms(target_x, target_y, wp);
        return;
    }

    if (is_axis_arrived(target_x, target_y, wp->action))
    {
        reset_motion_segment();
        if (arrival_stable_ticks < EXEC_ARRIVAL_STABLE_TICKS)
        {
            arrival_stable_ticks++;
        }

        if (arrival_stable_ticks >= EXEC_ARRIVAL_STABLE_TICKS)
        {
            executor_enter_segment_settle();
        }
    }
    else
    {
        arrival_stable_ticks = 0;
        /* 向目标移动 */
        move_to_target(target_x, target_y, wp->action);
    }
}

executor_state_enum executor_get_state(void)
{
    return exec_state;
}

executor_error_enum executor_get_error(void)
{
    return exec_error;
}

const char *executor_state_name(void)
{
    switch (exec_state)
    {
    case EXEC_STATE_IDLE:
        return "Idle";
    case EXEC_STATE_RUNNING:
        return "Running";
    case EXEC_STATE_PAUSED:
        return "Paused";
    case EXEC_STATE_DONE:
        return "Done";
    case EXEC_STATE_ERROR:
        return "Error";
    default:
        return "Unknown";
    }
}

uint16 executor_get_current_step(void)
{
    return current_step;
}

uint16 executor_get_total_steps(void)
{
    return exec_waypoint_count;
}

uint16 executor_get_current_box(void)
{
    if (exec_waypoints == NULL || exec_waypoint_count == 0)
    {
        return 0;
    }
    /* 计算当前是第几个箱子 */
    uint16 box = 0;
    for (uint16 i = 0; i < current_step && i < exec_waypoint_count; i++)
    {
        if (exec_waypoints[i].action >= 'A' && exec_waypoints[i].action <= 'Z')
        {
            box++;
        }
    }
    return box;
}

uint16 executor_get_total_boxes(void)
{
    if (exec_waypoints == NULL || exec_waypoint_count == 0)
    {
        return 0;
    }
    /* 计算总共有多少个箱子 */
    uint16 box = 0;
    for (uint16 i = 0; i < exec_waypoint_count; i++)
    {
        if (exec_waypoints[i].action >= 'A' && exec_waypoints[i].action <= 'Z')
        {
            box++;
        }
    }
    return box;
}

void executor_debug_output(void)
{
    const drive_pose_struct *pose = drive_pose_get();
    const waypoint_struct *wp;
    float target_x, target_y;

    if (exec_state != EXEC_STATE_RUNNING)
    {
        return;
    }

    if (current_step >= exec_waypoint_count)
    {
        return;
    }

    wp = &exec_waypoints[current_step];
    grid_to_physical(wp->row, wp->col, &target_x, &target_y);

    printf("EXEC: target=(%.2f,%.2f) current=(%.2f,%.2f) error=(%.2f,%.2f)\r\n",
           target_x, target_y, pose->x_cm, pose->y_cm,
           target_x - pose->x_cm, target_y - pose->y_cm);
    printf("PID: x_integral=%.3f y_integral=%.3f\r\n",
           path_pid_get_integral(&x_pid), path_pid_get_integral(&y_pid));
}
