#include "executor.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "motion_math.h"
#include <math.h>

#define ART_PLAYER_CENTER_FILTER_WINDOW (3u)

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
static char last_completed_action = '\0'; // 最近完成并触发 ART 等待的 waypoint 动作；主循环用它区分普通移动/推箱确认。
static uint16 art_player_center_x_history[ART_PLAYER_CENTER_FILTER_WINDOW];
static uint16 art_player_center_y_history[ART_PLAYER_CENTER_FILTER_WINDOW];
static uint8 art_player_center_history_count = 0;
static uint8 art_player_center_history_head = 0;
static uint32 art_player_center_history_frame[ART_PLAYER_CENTER_FILTER_WINDOW];
static uint16 art_player_center_median_col_q = 0;
static uint16 art_player_center_median_row_q = 0;
static uint8 art_player_center_median_valid = 0;

/* 地图 row 向下增大，而本地物理 Y 约定前进为正，因此 row 差值需要取反。 */
static void grid_to_physical(uint8 row, uint8 col, float *x_cm, float *y_cm)
{
    *x_cm = (float)(col - start_col) * GRID_SIZE_CM;
    *y_cm = -(float)(row - start_row) * GRID_SIZE_CM;
}

static void grid_q_to_physical(uint16 row_q, uint16 col_q, float *x_cm, float *y_cm)
{
    float col_center_q = ((float)start_col + 0.5f) * 100.0f;
    float row_center_q = ((float)start_row + 0.5f) * 100.0f;

    *x_cm = (((float)col_q - col_center_q) / 100.0f) * GRID_SIZE_CM;
    *y_cm = -(((float)row_q - row_center_q) / 100.0f) * GRID_SIZE_CM;
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

static uint16 median_u16_3(uint16 a, uint16 b, uint16 c)
{
    if(a > b)
    {
        uint16 t = a; a = b; b = t;
    }
    if(b > c)
    {
        uint16 t = b; b = c; c = t;
    }
    if(a > b)
    {
        uint16 t = a; a = b; b = t;
    }
    return b;
}

static uint8 center_q_matches_current_cell(uint16 row_q, uint16 col_q)
{
    uint8 center_row = (uint8)(row_q / 100u);
    uint8 center_col = (uint8)(col_q / 100u);

    return ((center_row == start_row) && (center_col == start_col)) ? 1u : 0u;
}

static uint8 action_is_x_axis(char action)
{
    return (('l' == action) || ('L' == action) || ('r' == action) || ('R' == action)) ? 1u : 0u;
}

static uint8 action_is_y_axis(char action)
{
    return (('u' == action) || ('U' == action) || ('d' == action) || ('D' == action)) ? 1u : 0u;
}

static uint8 action_is_push(char action)
{
    return ((action >= 'A') && (action <= 'Z')) ? 1u : 0u;
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

static void apply_push_overshoot(float *target_x, float *target_y, char action)
{
#if EXEC_PUSH_OVERSHOOT_ENABLE
    float overshoot_cm = GRID_SIZE_CM * EXEC_PUSH_OVERSHOOT_RATIO;

    if(0 == action_is_push(action))
    {
        return;
    }

    if('L' == action)
    {
        *target_x -= overshoot_cm;
    }
    else if('R' == action)
    {
        *target_x += overshoot_cm;
    }
    else if('U' == action)
    {
        *target_y += overshoot_cm;
    }
    else if('D' == action)
    {
        *target_y -= overshoot_cm;
    }
#else
    (void)target_x;
    (void)target_y;
    (void)action;
#endif
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
    last_completed_action = '\0';

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
    last_completed_action = '\0';
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

uint8 executor_art_center_sampling_active(void)
{
    if((EXEC_STATE_RUNNING != exec_state) || (0 == art_sync_enabled))
    {
        return 0;
    }
    return ((0 != segment_settling) || (0 != segment_waiting_art)) ? 1u : 0u;
}

char executor_get_art_sync_action(void)
{
    if(0 == executor_art_sync_pending())
    {
        return '\0';
    }
    return last_completed_action;
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

uint8 executor_apply_art_player_center(uint16 center_col_q, uint16 center_row_q, uint32 sample_count)
{
    uint8 index;
    uint8 sample_index;

    index = art_player_center_history_head;
    for(sample_index = 0; sample_index < art_player_center_history_count; sample_index++)
    {
        if(sample_count == art_player_center_history_frame[sample_index])
        {
            return 0;
        }
    }

    art_player_center_x_history[index] = center_col_q;
    art_player_center_y_history[index] = center_row_q;
    art_player_center_history_frame[index] = sample_count;
    art_player_center_history_head++;
    if(art_player_center_history_head >= ART_PLAYER_CENTER_FILTER_WINDOW)
    {
        art_player_center_history_head = 0;
    }
    if(art_player_center_history_count < ART_PLAYER_CENTER_FILTER_WINDOW)
    {
        art_player_center_history_count++;
    }

    if(art_player_center_history_count < ART_PLAYER_CENTER_FILTER_WINDOW)
    {
        return 0;
    }

    art_player_center_median_col_q = median_u16_3(art_player_center_x_history[0],
                                                  art_player_center_x_history[1],
                                                  art_player_center_x_history[2]);
    art_player_center_median_row_q = median_u16_3(art_player_center_y_history[0],
                                                  art_player_center_y_history[1],
                                                  art_player_center_y_history[2]);
    art_player_center_median_valid = 1;
    return 1;
}

uint8 executor_commit_art_player_center(void)
{
    const drive_pose_struct *pose;
    float corrected_x_cm;
    float corrected_y_cm;
    float pose_yaw;

    if(0 == art_player_center_median_valid)
    {
        return 0;
    }
    if(0 == center_q_matches_current_cell(art_player_center_median_row_q,
                                          art_player_center_median_col_q))
    {
        art_player_center_median_valid = 0;
        return 0;
    }

    grid_q_to_physical(art_player_center_median_row_q,
                       art_player_center_median_col_q,
                       &corrected_x_cm,
                       &corrected_y_cm);
    pose = drive_pose_get();
    pose_yaw = pose->yaw_deg;
    drive_pose_reset(corrected_x_cm, corrected_y_cm, pose_yaw);
    art_player_center_median_valid = 0;
    return 1;
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

static void executor_enter_art_wait(char action)
{
    /* ART 识别和重解算可能耗时，不能放在 PIT ISR；这里只停车并暴露 pending 状态给主循环。 */
    segment_settling = 0;
    segment_settle_elapsed_ms = 0;
    last_completed_action = action;
    segment_waiting_art = 1;
    art_player_center_history_count = 0;
    art_player_center_history_head = 0;
    art_player_center_median_valid = 0;
}

static void executor_finish_segment_settle(void)
{
    char action = '\0';

    segment_settling = 0;
    segment_settle_elapsed_ms = 0;

    if((0 != exec_waypoints) && (current_step < exec_waypoint_count))
    {
        action = exec_waypoints[current_step].action;
    }

    if (0 != art_sync_enabled)
    {
        executor_enter_art_wait(action);
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
    apply_push_overshoot(&target_x, &target_y, wp->action);

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
