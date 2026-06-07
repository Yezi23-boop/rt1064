#include "executor.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "motion_math.h"
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

/* 路径跟踪PID实例 */
static path_pid_struct x_pid;
static path_pid_struct y_pid;

/* 单步模式 */
static uint8 single_step_mode = 0;
static uint8 arrival_stable_ticks = 0;
static uint8 segment_settling = 0;
static uint16 segment_settle_elapsed_ms = 0;
static uint8 art_sync_enabled = 0;
static uint8 segment_waiting_art = 0;

/* 将网格坐标转换为物理坐标（以起点为原点） */
static void grid_to_physical(uint8 row, uint8 col, float *x_cm, float *y_cm)
{
    *x_cm = (float)(col - start_col) * GRID_SIZE_CM;
    *y_cm = -(float)(row - start_row) * GRID_SIZE_CM;
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
                    uint8 art_sync)
{
    const drive_pose_struct *pose;

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
    art_sync_enabled = art_sync;

    /* 重置状态 */
    current_step = 0;
    exec_error = EXEC_ERROR_NONE;
    executor_reset_segment_state();

    /* 重置局部位置，以最新 C 格为原点；yaw 保留当前 IMU 相对航向。 */
    pose = drive_pose_get();
    drive_pose_reset(0.0f, 0.0f, pose->yaw_deg);

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
    art_sync_enabled = 0;
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

uint8 executor_art_sync_pending(void)
{
    return ((EXEC_STATE_RUNNING == exec_state) && (0 != segment_waiting_art)) ? 1u : 0u;
}

void executor_finish_done(void)
{
    stop_motion();
    exec_error = EXEC_ERROR_NONE;
    exec_state = EXEC_STATE_DONE;
    executor_reset_segment_state();
}

void executor_set_error(executor_error_enum error)
{
    stop_motion();
    exec_error = error;
    exec_state = EXEC_STATE_ERROR;
    executor_reset_segment_state();
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

static void executor_enter_art_wait(void)
{
    segment_settling = 0;
    segment_settle_elapsed_ms = 0;
    segment_waiting_art = 1;
}

static void executor_finish_segment_settle(void)
{
    segment_settling = 0;
    segment_settle_elapsed_ms = 0;

    if (0 != art_sync_enabled)
    {
        executor_enter_art_wait();
        return;
    }

    executor_advance_after_segment();
}

static void executor_update_segment_settle_20ms(void)
{
    reset_motion_segment();
    if (segment_settle_elapsed_ms >= EXEC_SEGMENT_SETTLE_MS)
    {
        executor_finish_segment_settle();
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
        executor_update_segment_settle_20ms();
        return;
    }

    if (0 != segment_waiting_art)
    {
        reset_motion_segment();
        return;
    }

    if (0 != is_axis_arrived(target_x, target_y, wp->action))
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
