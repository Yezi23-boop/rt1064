#include "executor.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "motion_math.h"
#include <math.h>

/* 执行器状态由主循环启动/停止、PIT_CH1 20ms 推进共同访问；
 * 这里不做阻塞等待，ART 同步等待交给主循环处理。 */
static executor_state_enum exec_state = EXEC_STATE_IDLE; // 主循环查询、PIT_CH1 更新；非 IDLE 时底盘可能被执行器占用。
static executor_error_enum exec_error = EXEC_ERROR_NONE; // 最近一次执行错误，供 Execute 页显示和 ART 失败路径区分。

/* waypoints 指向求解器输出缓冲区，executor_start() 后调用方必须保证其生命周期覆盖执行过程。 */
static const waypoint_struct *exec_waypoints = NULL; // 指向 last_result.waypoints，不拥有内存；重算/清空结果前必须先停止 executor。
static uint16 exec_waypoint_count = 0;               // 当前路径点总数，决定 DONE 判定边界。
static uint16 current_step = 0;                      // 当前正在逼近的 waypoint 下标，由 20ms 周期推进。
static uint8 start_row = 0;                          // 执行开始时 C 的行号，作为本地 pose 到地图坐标的原点。
static uint8 start_col = 0;                          // 执行开始时 C 的列号，作为本地 pose 到地图坐标的原点。

/* X/Y 两个位置式 PID 的误差单位为 cm，输出为世界坐标归一化速度分量。 */
static path_pid_struct x_pid; // 世界 X 方向位置环；到点或切段时重置，避免上一段积分残留。
static path_pid_struct y_pid; // 世界 Y 方向位置环；与 X 独立限幅后再旋转到车体系。

/* 段内状态均以 20ms 为时间基准；到点稳定计数用于滤掉里程计瞬时越界。 */
static uint8 single_step_mode = 0;        // 1 表示每个 waypoint 后暂停等待 K3，便于低速调试路径。
static uint8 arrival_stable_ticks = 0;    // 连续到点计数，过滤横移惯性和里程计抖动导致的瞬时命中。
static uint8 segment_settling = 0;        // 1 表示已到 waypoint，正在段间停稳窗口内保持停止。
static uint16 segment_settle_elapsed_ms = 0; // 段间停稳累计时间，单位 ms，由 20ms 周期累加。
static uint8 art_sync_enabled = 0;        // ART 来源执行时置 1，段末到点后交给主循环重识别/重解算。
static uint8 segment_waiting_art = 0;     // 1 表示已停车并等待 ART 重解算，PIT 内只保持停止不做求解。

/* 地图 row 向下增大，而本地物理 Y 约定前进为正，因此 row 差值需要取反。 */
static void grid_to_physical(uint8 row, uint8 col, float *x_cm, float *y_cm)
{
    *x_cm = (float)(col - start_col) * GRID_SIZE_CM;
    *y_cm = -(float)(row - start_row) * GRID_SIZE_CM;
}

void executor_init(void)
{
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

/* 到点判定同时约束主轴和副轴，避免只穿过目标线但横向偏差仍很大时提前切段。 */
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

/* 路径 PID 先在局部世界坐标算速度，再旋转到车体系 vx/vy；底盘混控只接受车体系分量。 */
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

/* 每个 20ms 周期只给一组速度分量；真实 PWM 仍由后续底盘控制链路限幅和闭环。 */
static void move_to_target(float target_x, float target_y, char action)
{
    const drive_pose_struct *pose = drive_pose_get();
    float dx = target_x - pose->x_cm;
    float dy = target_y - pose->y_cm;
    float vx_world, vy_world;
    float vx_body, vy_body;

    vx_world = path_pid_update_with_arrival_deadband(&x_pid, dx);
    vy_world = path_pid_update_with_arrival_deadband(&y_pid, dy);

    world_velocity_to_body(vx_world, vy_world, pose->yaw_deg, &vx_body, &vy_body);

    set_motion(vx_body, vy_body);
}

void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 start_row_param, uint8 start_col_param, uint8 single_step,
                    uint8 art_sync)
{
    const drive_pose_struct *pose;

    if (waypoints == NULL || count == 0)
    {
        exec_state = EXEC_STATE_ERROR;
        exec_error = EXEC_ERROR_MAP;
        return;
    }

    exec_waypoints = waypoints;
    exec_waypoint_count = count;
    start_row = start_row_param;
    start_col = start_col_param;
    single_step_mode = single_step;
    art_sync_enabled = art_sync;

    current_step = 0;
    exec_error = EXEC_ERROR_NONE;
    executor_reset_segment_state();

    /* 重置局部位置，以最新 C 格为原点；yaw 保留当前 IMU 相对航向。 */
    pose = drive_pose_get();
    drive_pose_reset(0.0f, 0.0f, pose->yaw_deg);

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
    /* ART 识别和重解算可能耗时，不能放在 PIT ISR；这里只停车并暴露 pending 状态给主循环。 */
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
    if (exec_state != EXEC_STATE_RUNNING)
    {
        return;
    }

    if (current_step >= exec_waypoint_count)
    {
        stop_motion();
        exec_state = EXEC_STATE_DONE;
        return;
    }

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
        /* 等 ART 期间每个 20ms 都保持段间停止，避免低频重定位时车继续滑动。 */
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
