#include "executor.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "motion_math.h"
#include "timebase.h"
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

/* 单步模式 */
static uint8 single_step_mode = 0;

/* 超时检测 */
static uint32 step_start_ms = 0;
#define EXEC_TIMEOUT_MS 3000

/* 到位阈值 */
#define ARRIVAL_THRESHOLD_CM 2.0f
#define ARRIVAL_THRESHOLD_DEG 5.0f

/* 速度 */
#define EXEC_MOVE_SPEED 1.0f

/* 将网格坐标转换为物理坐标（以起点为原点） */
static void grid_to_physical(uint8 row, uint8 col, float *x_cm, float *y_cm)
{
    *x_cm = (float)(col - start_col) * 20.0f;
    *y_cm = -(float)(row - start_row) * 20.0f;
}

/* 检查是否到达目标位置 */
static uint8 is_arrived(float target_x, float target_y)
{
    const drive_pose_struct *pose = drive_pose_get();
    float dx = target_x - pose->x_cm;
    float dy = target_y - pose->y_cm;
    float distance = sqrtf(dx * dx + dy * dy);
    return (distance < ARRIVAL_THRESHOLD_CM);
}

/* 向目标位置移动 */
static void move_to_target(float target_x, float target_y)
{
    const drive_pose_struct *pose = drive_pose_get();
    float dx = target_x - pose->x_cm;
    float dy = target_y - pose->y_cm;

    /* 选择主移动方向 */
    if (fabsf(dx) > fabsf(dy)) {
        /* X 方向误差大，左右移动 */
        if (dx > 0) {
            set_motion_command(MOTION_RIGHT, EXEC_MOVE_SPEED, 0.0f);
        } else {
            set_motion_command(MOTION_LEFT, EXEC_MOVE_SPEED, 0.0f);
        }
    } else {
        /* Y 方向误差大，前后移动 */
        if (dy > 0) {
            set_motion_command(MOTION_FORWARD, EXEC_MOVE_SPEED, 0.0f);
        } else {
            set_motion_command(MOTION_BACKWARD, EXEC_MOVE_SPEED, 0.0f);
        }
    }
}

void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 start_row_param, uint8 start_col_param, uint8 single_step)
{
    /* 参数检查 */
    if (waypoints == NULL || count == 0) {
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

    /* 重置状态 */
    current_step = 0;
    exec_error = EXEC_ERROR_NONE;

    /* 重置位姿，以起点为原点 */
    drive_pose_reset(0.0f, 0.0f, 0.0f);

    /* 设置初始状态 */
    if (single_step_mode) {
        exec_state = EXEC_STATE_PAUSED;
    } else {
        exec_state = EXEC_STATE_RUNNING;
        step_start_ms = time_ms();
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
}

void executor_resume(void)
{
    if (exec_state == EXEC_STATE_PAUSED) {
        exec_state = EXEC_STATE_RUNNING;
        step_start_ms = time_ms();
    }
}

void executor_update_20ms(void)
{
    /* 只在运行状态执行 */
    if (exec_state != EXEC_STATE_RUNNING) {
        return;
    }

    /* 检查超时 */
    if (time_ms() - step_start_ms > EXEC_TIMEOUT_MS) {
        stop_motion();
        exec_state = EXEC_STATE_ERROR;
        exec_error = EXEC_ERROR_TIMEOUT;
        return;
    }

    /* 检查是否完成所有步骤 */
    if (current_step >= exec_waypoint_count) {
        stop_motion();
        exec_state = EXEC_STATE_DONE;
        return;
    }

    /* 获取当前目标 */
    const waypoint_struct *wp = &exec_waypoints[current_step];
    float target_x, target_y;
    grid_to_physical(wp->row, wp->col, &target_x, &target_y);

    if (is_arrived(target_x, target_y)) {
        current_step++;
        step_start_ms = time_ms();

        if (single_step_mode) {
            stop_motion();
            exec_state = EXEC_STATE_PAUSED;
        }
    } else {
        move_to_target(target_x, target_y);
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
    switch (exec_state) {
        case EXEC_STATE_IDLE:    return "Idle";
        case EXEC_STATE_RUNNING: return "Running";
        case EXEC_STATE_PAUSED:  return "Paused";
        case EXEC_STATE_DONE:    return "Done";
        case EXEC_STATE_ERROR:   return "Error";
        default:                 return "Unknown";
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
    if (exec_waypoints == NULL || exec_waypoint_count == 0) {
        return 0;
    }
    /* 计算当前是第几个箱子 */
    uint16 box = 0;
    for (uint16 i = 0; i < current_step && i < exec_waypoint_count; i++) {
        if (exec_waypoints[i].action >= 'A' && exec_waypoints[i].action <= 'Z') {
            box++;
        }
    }
    return box;
}

uint16 executor_get_total_boxes(void)
{
    if (exec_waypoints == NULL || exec_waypoint_count == 0) {
        return 0;
    }
    /* 计算总共有多少个箱子 */
    uint16 box = 0;
    for (uint16 i = 0; i < exec_waypoint_count; i++) {
        if (exec_waypoints[i].action >= 'A' && exec_waypoints[i].action <= 'Z') {
            box++;
        }
    }
    return box;
}
