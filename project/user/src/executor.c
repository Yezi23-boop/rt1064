#include "executor.h"
#include "drive_config.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "motion_math.h"
#include "zf_common_interrupt.h"
#include <math.h>

#define ART_PLAYER_CENTER_FILTER_WINDOW (ART_CENTER_SAMPLE_COUNT)
#define ART_BOX_OBSERVATION_FILTER_WINDOW (3u)
#define ART_BOX_MATCH_MAX_OFFSET_Q (80)

typedef enum
{
    PRE_PUSH_BOX_IDLE = 0,
    PRE_PUSH_BOX_RETRY_NUDGE,
    PRE_PUSH_BOX_SAFE_GAP,
    PRE_PUSH_BOX_ALIGN_PERP,
    PRE_PUSH_BOX_APPROACH
} pre_push_box_phase_enum;

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
static uint8 pre_push_center_waiting = 0; // 1 表示当前方向转折 waypoint 正在等待中心矫正。
static uint8 pre_push_center_confirmed = 0; // 1 表示当前 waypoint 已完成中心矫正，可以运动。
static uint8 pre_push_alignment_active = 0; // 1 表示视觉 pose 已提交，正在沿推箱垂直轴回到 C 格中心线。
static uint8 pre_push_alignment_stable_ticks = 0; // 单轴误差连续进入到点阈值的 20ms 周期数。
static uint16 pre_push_alignment_elapsed_ms = 0; // 单轴对齐累计时间，达到 ART 同步超时后报中心错误。
static float pre_push_alignment_target_x_cm = 0.0f; // 当前 C 格中心的本地世界 X。
static float pre_push_alignment_target_y_cm = 0.0f; // 当前 C 格中心的本地世界 Y。
static pre_push_box_phase_enum pre_push_box_phase = PRE_PUSH_BOX_IDLE;
static uint8 pre_push_box_consumes_waypoint = 0u;
static char pre_push_box_action = '\0';
static float pre_push_box_target_x_cm = 0.0f;
static float pre_push_box_target_y_cm = 0.0f;
static uint16 art_box_car_col_samples[ART_BOX_OBSERVATION_FILTER_WINDOW];
static uint16 art_box_car_row_samples[ART_BOX_OBSERVATION_FILTER_WINDOW];
static uint16 art_box_col_samples[ART_BOX_OBSERVATION_FILTER_WINDOW];
static uint16 art_box_row_samples[ART_BOX_OBSERVATION_FILTER_WINDOW];
static uint8 art_box_observation_count = 0u;
static uint8 art_box_observation_valid = 0u;
static uint16 art_box_car_median_col_q = 0u;
static uint16 art_box_car_median_row_q = 0u;
static uint16 art_box_median_col_q = 0u;
static uint16 art_box_median_row_q = 0u;
static char last_completed_action = '\0'; // 最近完成并触发 ART 等待的 waypoint 动作；主循环用它区分普通移动/推箱确认。
static uint16 art_player_center_x_history[ART_PLAYER_CENTER_FILTER_WINDOW];
static uint16 art_player_center_y_history[ART_PLAYER_CENTER_FILTER_WINDOW];
static uint8 art_player_center_history_count = 0;
static uint8 art_player_center_history_head = 0;
static uint32 art_player_center_history_frame[ART_PLAYER_CENTER_FILTER_WINDOW];
static uint16 art_player_center_median_col_q = 0;
static uint16 art_player_center_median_row_q = 0;
static uint8 art_player_center_median_valid = 0;
static executor_art_center_result_enum last_art_center_result = EXEC_ART_CENTER_NONE;
static float last_art_center_dx_cm = 0.0f;
static float last_art_center_dy_cm = 0.0f;
static float last_art_center_diff_cm = 0.0f;
static uint8 position_correction_active = 0u;
static float position_correction_target_x_cm = 0.0f;
static float position_correction_target_y_cm = 0.0f;

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
    pre_push_center_waiting = 0;
    pre_push_center_confirmed = 0;
    pre_push_alignment_active = 0;
    pre_push_alignment_stable_ticks = 0;
    pre_push_alignment_elapsed_ms = 0;
    pre_push_alignment_target_x_cm = 0.0f;
    pre_push_alignment_target_y_cm = 0.0f;
    pre_push_box_phase = PRE_PUSH_BOX_IDLE;
    pre_push_box_consumes_waypoint = 0u;
    pre_push_box_action = '\0';
    pre_push_box_target_x_cm = 0.0f;
    pre_push_box_target_y_cm = 0.0f;
    path_pid_reset(&x_pid);
    path_pid_reset(&y_pid);
}

static void executor_clear_art_center_history(void)
{
    art_player_center_history_count = 0;
    art_player_center_history_head = 0;
    art_player_center_median_valid = 0;
}

static void executor_clear_art_box_observation(void)
{
    art_box_observation_count = 0u;
    art_box_observation_valid = 0u;
    art_box_car_median_col_q = 0u;
    art_box_car_median_row_q = 0u;
    art_box_median_col_q = 0u;
    art_box_median_row_q = 0u;
}

static uint16 median_u16_values(const uint16 *values, uint8 count)
{
    uint16 sorted[ART_PLAYER_CENTER_FILTER_WINDOW];
    uint8 i;
    uint8 j;

    for(i = 0; i < count; i++)
    {
        sorted[i] = values[i];
    }

    for(i = 1; i < count; i++)
    {
        uint16 key = sorted[i];
        j = i;
        while((j > 0) && (sorted[j - 1] > key))
        {
            sorted[j] = sorted[j - 1];
            j--;
        }
        sorted[j] = key;
    }

    return sorted[count / 2u];
}

static uint8 center_q_is_near_cell(uint16 row_q, uint16 col_q,
                                  uint8 cell_row, uint8 cell_col)
{
    uint8 center_row = (uint8)(row_q / 100u);
    uint8 center_col = (uint8)(col_q / 100u);
    int16 row_delta = (int16)center_row - (int16)cell_row;
    int16 col_delta = (int16)center_col - (int16)cell_col;

    if((0 == row_delta) && (0 == col_delta))
    {
        return 1u;
    }
#if EXEC_ART_CENTER_ALLOW_NEIGHBOR_CELL
    return ((row_delta >= -1) && (row_delta <= 1) &&
            (col_delta >= -1) && (col_delta <= 1)) ? 1u : 0u;
#else
    return 0u;
#endif
}

static uint8 box_center_q_matches_cell(uint16 row_q, uint16 col_q,
                                      uint8 cell_row, uint8 cell_col)
{
    int16 row_offset_q = (int16)row_q - (int16)(cell_row * 100u + 50u);
    int16 col_offset_q = (int16)col_q - (int16)(cell_col * 100u + 50u);

    return ((row_offset_q >= -ART_BOX_MATCH_MAX_OFFSET_Q) &&
            (row_offset_q <= ART_BOX_MATCH_MAX_OFFSET_Q) &&
            (col_offset_q >= -ART_BOX_MATCH_MAX_OFFSET_Q) &&
            (col_offset_q <= ART_BOX_MATCH_MAX_OFFSET_Q)) ? 1u : 0u;
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

static uint8 executor_get_pre_push_box_context_at(uint16 prepare_step,
                                                  uint16 *push_step,
                                                  uint8 *consume_current)
{
    if((0 == exec_waypoints) || (prepare_step >= exec_waypoint_count))
    {
        return 0u;
    }
    if(0 != action_is_push(exec_waypoints[prepare_step].action))
    {
        if(0 != push_step)
        {
            *push_step = prepare_step;
        }
        if(0 != consume_current)
        {
            *consume_current = 0u;
        }
        return 1u;
    }
    if(((prepare_step + 1u) < exec_waypoint_count) &&
       (0 != action_is_push(exec_waypoints[prepare_step + 1u].action)))
    {
        if(0 != push_step)
        {
            *push_step = (uint16)(prepare_step + 1u);
        }
        if(0 != consume_current)
        {
            *consume_current = 1u;
        }
        return 1u;
    }
    return 0u;
}

static uint8 executor_get_pre_push_box_context(uint16 *push_step,
                                               uint8 *consume_current)
{
    return executor_get_pre_push_box_context_at(current_step,
                                                push_step,
                                                consume_current);
}

static char action_direction(char action)
{
    return ((action >= 'A') && (action <= 'Z')) ?
           (char)(action + ('a' - 'A')) : action;
}

static uint8 waypoint_needs_art_sync(const waypoint_struct *wp)
{
    if(0 == wp)
    {
        return 0;
    }
    if(0 == action_is_push(wp->action))
    {
        return 0;
    }
    return (0 != wp->task_end) ? 1u : 0u;
}

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float sqrt_distance_cm(float x_cm, float y_cm)
{
    return sqrtf((x_cm * x_cm) + (y_cm * y_cm));
}

static void set_art_center_debug(executor_art_center_result_enum result,
                                 float dx_cm,
                                 float dy_cm,
                                 float diff_cm)
{
    last_art_center_result = result;
    last_art_center_dx_cm = dx_cm;
    last_art_center_dy_cm = dy_cm;
    last_art_center_diff_cm = diff_cm;
}

/* 到点判定同时约束主轴和副轴，避免只穿过目标线但横向偏差仍很大时提前切段。 */
static uint8 is_axis_arrived(float target_x, float target_y, char action)
{
    const drive_pose_struct *pose = drive_pose_get();
    float dx = target_x - pose->x_cm;
    float dy = target_y - pose->y_cm;

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
    return ((abs_float(dx) < PATH_ARRIVAL_THRESHOLD_CM) &&
            (abs_float(dy) < PATH_ARRIVAL_THRESHOLD_CM));
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
                    uint8 start_row_param, uint8 start_col_param,
                    float initial_pose_x_cm, float initial_pose_y_cm,
                    uint8 single_step,
                    uint8 art_sync)
{
    const drive_pose_struct *pose;
    uint32 primask;

    primask = interrupt_global_disable();

    if (waypoints == NULL || count == 0)
    {
        exec_state = EXEC_STATE_ERROR;
        exec_error = EXEC_ERROR_MAP;
        interrupt_global_enable(primask);
        return;
    }

    exec_waypoints = waypoints;
    exec_waypoint_count = count;
    start_row = start_row_param;
    start_col = start_col_param;
    single_step_mode = single_step;
    art_sync_enabled = art_sync;
    position_correction_active = 0u;

    current_step = 0;
    exec_error = EXEC_ERROR_NONE;
    executor_reset_segment_state();
    last_completed_action = '\0';

    /* ART 可带入相对 C 格中心的偏移；离线地图仍传 0,0。 */
    pose = drive_pose_get();
    drive_pose_reset(initial_pose_x_cm, initial_pose_y_cm, pose->yaw_deg);

    if (single_step_mode)
    {
        exec_state = EXEC_STATE_PAUSED;
    }
    else
    {
        exec_state = EXEC_STATE_RUNNING;
    }
    interrupt_global_enable(primask);
}

uint8 executor_start_position_correction(float target_x_cm, float target_y_cm)
{
    uint8 started = 0u;
    uint32 primask = interrupt_global_disable();

    if((EXEC_STATE_RUNNING != exec_state) && (EXEC_STATE_PAUSED != exec_state))
    {
        exec_waypoints = NULL;
        exec_waypoint_count = 0u;
        current_step = 0u;
        single_step_mode = 0u;
        art_sync_enabled = 0u;
        position_correction_target_x_cm = target_x_cm;
        position_correction_target_y_cm = target_y_cm;
        position_correction_active = 1u;
        exec_error = EXEC_ERROR_NONE;
        executor_reset_segment_state();
        exec_state = EXEC_STATE_RUNNING;
        started = 1u;
    }
    interrupt_global_enable(primask);
    return started;
}

void executor_stop(void)
{
    uint32 primask = interrupt_global_disable();

    stop_motion();
    exec_state = EXEC_STATE_IDLE;
    exec_error = EXEC_ERROR_NONE;
    exec_waypoints = NULL;
    exec_waypoint_count = 0;
    current_step = 0;
    art_sync_enabled = 0;
    position_correction_active = 0u;
    last_completed_action = '\0';
    executor_reset_segment_state();
    interrupt_global_enable(primask);
}

void executor_resume(void)
{
    uint32 primask = interrupt_global_disable();

    if (exec_state == EXEC_STATE_PAUSED)
    {
        executor_reset_segment_state();
        exec_state = EXEC_STATE_RUNNING;
    }
    interrupt_global_enable(primask);
}

uint8 executor_art_sync_pending(void)
{
    return ((EXEC_STATE_RUNNING == exec_state) && (0 != segment_waiting_art)) ? 1u : 0u;
}

uint8 executor_art_pre_push_pending(void)
{
    return ((EXEC_STATE_RUNNING == exec_state) && (0 != pre_push_center_waiting)) ? 1u : 0u;
}

uint8 executor_center_requires_push_alignment(void)
{
    if((0 == executor_art_pre_push_pending()) ||
       (0 == exec_waypoints) ||
       (current_step >= exec_waypoint_count))
    {
        return 0u;
    }
    return action_is_push(exec_waypoints[current_step].action);
}

uint8 executor_get_pre_push_box_request(uint8 *box_row, uint8 *box_col)
{
    uint16 push_step;

    if((0 == box_row) || (0 == box_col) ||
       (0 == executor_art_pre_push_pending()) ||
       (0 == executor_get_pre_push_box_context(&push_step, 0)))
    {
        return 0u;
    }
    *box_row = exec_waypoints[push_step].row;
    *box_col = exec_waypoints[push_step].col;
    return 1u;
}

uint8 executor_get_pre_push_box_prefetch_request(uint8 *box_row, uint8 *box_col)
{
    uint16 prepare_step;
    uint16 push_step;
    uint8 found = 0u;
    uint32 primask;

    if((0 == box_row) || (0 == box_col))
    {
        return 0u;
    }

    primask = interrupt_global_disable();
    if((EXEC_STATE_RUNNING != exec_state) || (0u == art_sync_enabled) ||
       (0u != pre_push_center_waiting) || (0 == exec_waypoints))
    {
        goto done;
    }

    prepare_step = current_step;
    if(((prepare_step + 1u) < exec_waypoint_count) &&
       (0u != exec_waypoints[prepare_step + 1u].center_correct_before) &&
       (0u != executor_get_pre_push_box_context_at(
                   (uint16)(prepare_step + 1u), &push_step, 0)))
    {
        found = 1u;
    }
    else if((prepare_step < exec_waypoint_count) &&
            (0u != exec_waypoints[prepare_step].center_correct_before) &&
            (0u != executor_get_pre_push_box_context_at(
                        prepare_step, &push_step, 0)))
    {
        found = 1u;
    }

    if(0u != found)
    {
        *box_row = exec_waypoints[push_step].row;
        *box_col = exec_waypoints[push_step].col;
    }

done:
    interrupt_global_enable(primask);
    return found;
}

uint8 executor_continue_after_pre_push_center(void)
{
    uint8 continued = 0;
    uint32 primask = interrupt_global_disable();

    if(0 != executor_art_pre_push_pending())
    {
        pre_push_center_waiting = 0;
        pre_push_center_confirmed = 1;
        arrival_stable_ticks = 0;
        path_pid_reset(&x_pid);
        path_pid_reset(&y_pid);
        continued = 1;
    }
    interrupt_global_enable(primask);
    return continued;
}

uint8 executor_start_pre_push_alignment(uint8 reference_row, uint8 reference_col)
{
    const waypoint_struct *wp;
    uint8 started = 0u;
    uint32 primask = interrupt_global_disable();

    if((0 != executor_art_pre_push_pending()) &&
       (0 != exec_waypoints) &&
       (current_step < exec_waypoint_count))
    {
        wp = &exec_waypoints[current_step];
        if(0 != action_is_push(wp->action))
        {
            grid_to_physical(reference_row, reference_col,
                             &pre_push_alignment_target_x_cm,
                             &pre_push_alignment_target_y_cm);
            pre_push_center_waiting = 0;
            pre_push_center_confirmed = 0;
            pre_push_alignment_active = 1;
            pre_push_alignment_stable_ticks = 0;
            pre_push_alignment_elapsed_ms = 0;
            arrival_stable_ticks = 0;
            path_pid_reset(&x_pid);
            path_pid_reset(&y_pid);
            started = 1u;
        }
    }
    interrupt_global_enable(primask);
    return started;
}

uint8 executor_art_center_sampling_active(void)
{
    if((EXEC_STATE_RUNNING != exec_state) || (0 == art_sync_enabled))
    {
        return 0;
    }
    return ((0 != segment_settling) ||
            (0 != segment_waiting_art) ||
            (0 != pre_push_center_waiting)) ? 1u : 0u;
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
    uint32 primask = interrupt_global_disable();

    stop_motion();
    exec_error = EXEC_ERROR_NONE;
    exec_state = EXEC_STATE_DONE;
    position_correction_active = 0u;
    executor_reset_segment_state();
    interrupt_global_enable(primask);
}

void executor_set_error(executor_error_enum error)
{
    uint32 primask = interrupt_global_disable();

    stop_motion();
    exec_error = error;
    exec_state = EXEC_STATE_ERROR;
    position_correction_active = 0u;
    executor_reset_segment_state();
    interrupt_global_enable(primask);
}

uint8 executor_apply_art_player_center(uint16 center_col_q, uint16 center_row_q, uint32 sample_count)
{
    uint8 index;
    uint8 sample_index;
    uint8 ready = 0;
    uint32 primask = interrupt_global_disable();

    index = art_player_center_history_head;
    for(sample_index = 0; sample_index < art_player_center_history_count; sample_index++)
    {
        if(sample_count == art_player_center_history_frame[sample_index])
        {
            goto done;
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
        goto done;
    }

    art_player_center_median_col_q = median_u16_values(art_player_center_x_history,
                                                       ART_PLAYER_CENTER_FILTER_WINDOW);
    art_player_center_median_row_q = median_u16_values(art_player_center_y_history,
                                                       ART_PLAYER_CENTER_FILTER_WINDOW);
    art_player_center_median_valid = 1;
    ready = 1;

done:
    interrupt_global_enable(primask);
    return ready;
}

void executor_reset_art_player_center_samples(void)
{
    uint32 primask = interrupt_global_disable();
    executor_clear_art_center_history();
    interrupt_global_enable(primask);
}

static executor_art_center_result_enum executor_commit_art_center_values(
    uint16 center_col_q, uint16 center_row_q,
    uint8 current_car_row, uint8 current_car_col)
{
    const drive_pose_struct *pose;
    float corrected_x_cm;
    float corrected_y_cm;
    float pose_yaw;
    executor_art_center_result_enum result;

    if(0 == center_q_is_near_cell(center_row_q,
                                  center_col_q,
                                  current_car_row,
                                  current_car_col))
    {
        set_art_center_debug(EXEC_ART_CENTER_REJECTED, 0.0f, 0.0f, 0.0f);
        return EXEC_ART_CENTER_REJECTED;
    }

    grid_q_to_physical(center_row_q,
                       center_col_q,
                       &corrected_x_cm,
                       &corrected_y_cm);
    pose = drive_pose_get();
    pose_yaw = pose->yaw_deg;

    {
        float dx = corrected_x_cm - pose->x_cm;
        float dy = corrected_y_cm - pose->y_cm;
        float diff_cm = sqrt_distance_cm(dx, dy);

        if(diff_cm < EXEC_ART_CENTER_IGNORE_CM)
        {
            set_art_center_debug(EXEC_ART_CENTER_IGNORED, dx, dy, diff_cm);
            return EXEC_ART_CENTER_IGNORED;
        }

        if(diff_cm > EXEC_ART_CENTER_ABNORMAL_CM)
        {
            set_art_center_debug(EXEC_ART_CENTER_ABNORMAL, dx, dy, diff_cm);
            return EXEC_ART_CENTER_ABNORMAL;
        }

        if(diff_cm > EXEC_ART_CENTER_FUSE_MAX_CM)
        {
            set_art_center_debug(EXEC_ART_CENTER_REJECTED, dx, dy, diff_cm);
            return EXEC_ART_CENTER_REJECTED;
        }

        corrected_x_cm = pose->x_cm + (dx * EXEC_ART_CENTER_FUSE_ALPHA);
        corrected_y_cm = pose->y_cm + (dy * EXEC_ART_CENTER_FUSE_ALPHA);
        drive_pose_reset(corrected_x_cm, corrected_y_cm, pose_yaw);
        set_art_center_debug(EXEC_ART_CENTER_APPLIED, dx, dy, diff_cm);
    }
    result = EXEC_ART_CENTER_APPLIED;
    return result;
}

executor_art_center_result_enum executor_commit_art_player_center(uint8 current_car_row,
                                                                  uint8 current_car_col)
{
    executor_art_center_result_enum result;
    uint32 primask = interrupt_global_disable();

    if(0 == art_player_center_median_valid)
    {
        set_art_center_debug(EXEC_ART_CENTER_NONE, 0.0f, 0.0f, 0.0f);
        result = EXEC_ART_CENTER_NONE;
        goto done;
    }
    result = executor_commit_art_center_values(
        art_player_center_median_col_q,
        art_player_center_median_row_q,
        current_car_row,
        current_car_col);
    art_player_center_median_valid = 0;

done:
    interrupt_global_enable(primask);
    return result;
}

void executor_reset_art_box_observation_samples(void)
{
    uint32 primask = interrupt_global_disable();
    executor_clear_art_box_observation();
    interrupt_global_enable(primask);
}

uint8 executor_apply_art_box_observation(uint16 car_col_q, uint16 car_row_q,
                                         uint16 box_col_q, uint16 box_row_q)
{
    uint8 ready = 0u;
    uint32 primask = interrupt_global_disable();

    if(art_box_observation_count < ART_BOX_OBSERVATION_FILTER_WINDOW)
    {
        uint8 index = art_box_observation_count;
        art_box_car_col_samples[index] = car_col_q;
        art_box_car_row_samples[index] = car_row_q;
        art_box_col_samples[index] = box_col_q;
        art_box_row_samples[index] = box_row_q;
        art_box_observation_count++;
    }
    if(art_box_observation_count >= ART_BOX_OBSERVATION_FILTER_WINDOW)
    {
        art_box_car_median_col_q = median_u16_values(
            art_box_car_col_samples, ART_BOX_OBSERVATION_FILTER_WINDOW);
        art_box_car_median_row_q = median_u16_values(
            art_box_car_row_samples, ART_BOX_OBSERVATION_FILTER_WINDOW);
        art_box_median_col_q = median_u16_values(
            art_box_col_samples, ART_BOX_OBSERVATION_FILTER_WINDOW);
        art_box_median_row_q = median_u16_values(
            art_box_row_samples, ART_BOX_OBSERVATION_FILTER_WINDOW);
        art_box_observation_valid = 1u;
        ready = 1u;
    }
    interrupt_global_enable(primask);
    return ready;
}

executor_art_box_prep_result_enum executor_start_pre_push_box_preparation(void)
{
    const waypoint_struct *push_wp;
    executor_art_center_result_enum center_result;
    uint16 push_step;
    uint8 consume_current;
    uint8 current_row;
    uint8 current_col;
    float car_x_cm;
    float car_y_cm;
    float box_x_cm;
    float box_y_cm;
    float signed_gap_cm;
    executor_art_box_prep_result_enum result = EXEC_ART_BOX_PREP_NONE;
    uint32 primask = interrupt_global_disable();

    if((0 == executor_art_pre_push_pending()) ||
       (0 == art_box_observation_valid) ||
       (0 == executor_get_pre_push_box_context(&push_step, &consume_current)))
    {
        goto done;
    }

    push_wp = &exec_waypoints[push_step];
    if(0u == current_step)
    {
        current_row = start_row;
        current_col = start_col;
    }
    else
    {
        current_row = exec_waypoints[current_step - 1u].row;
        current_col = exec_waypoints[current_step - 1u].col;
    }
    if((0 == center_q_is_near_cell(art_box_car_median_row_q,
                                   art_box_car_median_col_q,
                                   current_row, current_col)) ||
       (0 == box_center_q_matches_cell(art_box_median_row_q,
                                      art_box_median_col_q,
                                      push_wp->row, push_wp->col)))
    {
        result = EXEC_ART_BOX_PREP_GEOMETRY_ERROR;
        goto clear_samples;
    }

    grid_q_to_physical(art_box_car_median_row_q,
                       art_box_car_median_col_q,
                       &car_x_cm, &car_y_cm);
    grid_q_to_physical(art_box_median_row_q,
                       art_box_median_col_q,
                       &box_x_cm, &box_y_cm);

    pre_push_box_target_x_cm = box_x_cm;
    pre_push_box_target_y_cm = box_y_cm;
    if('R' == push_wp->action)
    {
        signed_gap_cm = box_x_cm - car_x_cm;
        pre_push_box_target_x_cm -= GRID_SIZE_CM;
    }
    else if('L' == push_wp->action)
    {
        signed_gap_cm = car_x_cm - box_x_cm;
        pre_push_box_target_x_cm += GRID_SIZE_CM;
    }
    else if('U' == push_wp->action)
    {
        signed_gap_cm = box_y_cm - car_y_cm;
        pre_push_box_target_y_cm -= GRID_SIZE_CM;
    }
    else if('D' == push_wp->action)
    {
        signed_gap_cm = car_y_cm - box_y_cm;
        pre_push_box_target_y_cm += GRID_SIZE_CM;
    }
    else
    {
        result = EXEC_ART_BOX_PREP_GEOMETRY_ERROR;
        goto clear_samples;
    }
    if(signed_gap_cm <= PATH_ARRIVAL_THRESHOLD_CM)
    {
        result = EXEC_ART_BOX_PREP_GEOMETRY_ERROR;
        goto clear_samples;
    }

    center_result = executor_commit_art_center_values(
        art_box_car_median_col_q,
        art_box_car_median_row_q,
        current_row,
        current_col);
    if((EXEC_ART_CENTER_APPLIED != center_result) &&
       (EXEC_ART_CENTER_IGNORED != center_result))
    {
        result = EXEC_ART_BOX_PREP_GEOMETRY_ERROR;
        goto clear_samples;
    }

    pre_push_box_consumes_waypoint = consume_current;
    pre_push_box_action = push_wp->action;
    pre_push_center_waiting = 0u;
    pre_push_center_confirmed = 0u;
    pre_push_alignment_stable_ticks = 0u;
    pre_push_alignment_elapsed_ms = 0u;
    arrival_stable_ticks = 0u;
    path_pid_reset(&x_pid);
    path_pid_reset(&y_pid);
    pre_push_box_phase =
        (signed_gap_cm < (GRID_SIZE_CM - PATH_ARRIVAL_THRESHOLD_CM)) ?
        PRE_PUSH_BOX_SAFE_GAP : PRE_PUSH_BOX_ALIGN_PERP;
    result = EXEC_ART_BOX_PREP_STARTED;

clear_samples:
    executor_clear_art_box_observation();
done:
    interrupt_global_enable(primask);
    return result;
}

uint8 executor_start_pre_push_box_retry_nudge(float distance_cm)
{
    const drive_pose_struct *pose;
    uint16 push_step;
    char action;
    uint8 started = 0u;
    uint32 primask = interrupt_global_disable();

    if((distance_cm <= 0.0f) ||
       (0 == executor_art_pre_push_pending()) ||
       (0 == executor_get_pre_push_box_context(&push_step, 0)))
    {
        goto done;
    }

    action = exec_waypoints[push_step].action;
    pose = drive_pose_get();
    pre_push_box_target_x_cm = pose->x_cm;
    pre_push_box_target_y_cm = pose->y_cm;
    if('R' == action)
    {
        pre_push_box_target_x_cm -= distance_cm;
    }
    else if('L' == action)
    {
        pre_push_box_target_x_cm += distance_cm;
    }
    else if('U' == action)
    {
        pre_push_box_target_y_cm -= distance_cm;
    }
    else if('D' == action)
    {
        pre_push_box_target_y_cm += distance_cm;
    }
    else
    {
        goto done;
    }

    pre_push_box_action = action;
    pre_push_box_phase = PRE_PUSH_BOX_RETRY_NUDGE;
    pre_push_alignment_stable_ticks = 0u;
    pre_push_alignment_elapsed_ms = 0u;
    arrival_stable_ticks = 0u;
    path_pid_reset(&x_pid);
    path_pid_reset(&y_pid);
    started = 1u;

done:
    interrupt_global_enable(primask);
    return started;
}

uint8 executor_pre_push_box_preparation_active(void)
{
    return ((EXEC_STATE_RUNNING == exec_state) &&
            (PRE_PUSH_BOX_IDLE != pre_push_box_phase)) ? 1u : 0u;
}

const char *executor_pre_push_box_state_name(void)
{
    if(PRE_PUSH_BOX_RETRY_NUDGE == pre_push_box_phase)
    {
        return "BRetry";
    }
    if(PRE_PUSH_BOX_SAFE_GAP == pre_push_box_phase)
    {
        return "BGap";
    }
    if(PRE_PUSH_BOX_ALIGN_PERP == pre_push_box_phase)
    {
        return "BAlign";
    }
    if(PRE_PUSH_BOX_APPROACH == pre_push_box_phase)
    {
        return "BNear";
    }
    return "";
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

uint8 executor_continue_after_art_sync(void)
{
    uint8 continued = 0;
    uint32 primask = interrupt_global_disable();

    if(0 != executor_art_sync_pending())
    {
        executor_advance_after_segment();
        continued = 1;
    }
    interrupt_global_enable(primask);
    return continued;
}

static void executor_enter_art_wait(char action)
{
    /* ART 识别和重解算可能耗时，不能放在 PIT ISR；这里只停车并暴露 pending 状态给主循环。 */
    segment_settling = 0;
    segment_settle_elapsed_ms = 0;
    last_completed_action = action;
    segment_waiting_art = 1;
    executor_clear_art_center_history();
}

static void executor_enter_pre_push_center_wait(void)
{
    reset_motion_segment();
    arrival_stable_ticks = 0;
    pre_push_center_waiting = 1;
    pre_push_center_confirmed = 0;
    pre_push_alignment_active = 0;
    pre_push_alignment_stable_ticks = 0;
    pre_push_alignment_elapsed_ms = 0;
    path_pid_reset(&x_pid);
    path_pid_reset(&y_pid);
    executor_clear_art_center_history();
    executor_clear_art_box_observation();
}

static void executor_update_pre_push_alignment_20ms(void)
{
    const drive_pose_struct *pose = drive_pose_get();
    const waypoint_struct *wp;
    float error;
    float vx_world = 0.0f;
    float vy_world = 0.0f;
    float vx_body;
    float vy_body;

    if((0 == exec_waypoints) || (current_step >= exec_waypoint_count))
    {
        executor_set_error(EXEC_ERROR_ART_CENTER);
        return;
    }
    if(pre_push_alignment_elapsed_ms >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        executor_set_error(EXEC_ERROR_ART_CENTER);
        return;
    }
    pre_push_alignment_elapsed_ms += CONTROL_PERIOD_MS;

    wp = &exec_waypoints[current_step];
    if(0 != action_is_x_axis(wp->action))
    {
        error = pre_push_alignment_target_y_cm - pose->y_cm;
        vy_world = path_pid_update_with_arrival_deadband(&y_pid, error);
    }
    else if(0 != action_is_y_axis(wp->action))
    {
        error = pre_push_alignment_target_x_cm - pose->x_cm;
        vx_world = path_pid_update_with_arrival_deadband(&x_pid, error);
    }
    else
    {
        executor_set_error(EXEC_ERROR_ART_CENTER);
        return;
    }

    if(abs_float(error) < PATH_ARRIVAL_THRESHOLD_CM)
    {
        reset_motion_segment();
        if(pre_push_alignment_stable_ticks < EXEC_ARRIVAL_STABLE_TICKS)
        {
            pre_push_alignment_stable_ticks++;
        }
        if(pre_push_alignment_stable_ticks >= EXEC_ARRIVAL_STABLE_TICKS)
        {
            pre_push_alignment_active = 0;
            pre_push_alignment_stable_ticks = 0;
            pre_push_alignment_elapsed_ms = 0;
            pre_push_center_confirmed = 1;
            arrival_stable_ticks = 0;
            path_pid_reset(&x_pid);
            path_pid_reset(&y_pid);
        }
        return;
    }

    pre_push_alignment_stable_ticks = 0;
    world_velocity_to_body(vx_world, vy_world, pose->yaw_deg, &vx_body, &vy_body);
    set_motion(vx_body, vy_body);
}

static void executor_finish_pre_push_box_preparation(void)
{
    uint8 consume_current = pre_push_box_consumes_waypoint;

    pre_push_box_phase = PRE_PUSH_BOX_IDLE;
    pre_push_box_consumes_waypoint = 0u;
    pre_push_alignment_stable_ticks = 0u;
    pre_push_alignment_elapsed_ms = 0u;
    arrival_stable_ticks = 0u;
    path_pid_reset(&x_pid);
    path_pid_reset(&y_pid);

    if(0u != consume_current)
    {
        executor_advance_after_segment();
    }
    else
    {
        pre_push_center_confirmed = 1u;
    }
}

static void executor_update_pre_push_box_20ms(void)
{
    const drive_pose_struct *pose = drive_pose_get();
    float error;
    float vx_world = 0.0f;
    float vy_world = 0.0f;
    float vx_body;
    float vy_body;

    if(pre_push_alignment_elapsed_ms >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        executor_set_error(EXEC_ERROR_ART_CENTER);
        return;
    }
    pre_push_alignment_elapsed_ms += CONTROL_PERIOD_MS;

    if(PRE_PUSH_BOX_ALIGN_PERP == pre_push_box_phase)
    {
        if(0 != action_is_x_axis(pre_push_box_action))
        {
            error = pre_push_box_target_y_cm - pose->y_cm;
            vy_world = path_pid_update_with_arrival_deadband(&y_pid, error);
        }
        else
        {
            error = pre_push_box_target_x_cm - pose->x_cm;
            vx_world = path_pid_update_with_arrival_deadband(&x_pid, error);
        }
    }
    else if((PRE_PUSH_BOX_RETRY_NUDGE == pre_push_box_phase) ||
            (PRE_PUSH_BOX_SAFE_GAP == pre_push_box_phase) ||
            (PRE_PUSH_BOX_APPROACH == pre_push_box_phase))
    {
        if(0 != action_is_x_axis(pre_push_box_action))
        {
            error = pre_push_box_target_x_cm - pose->x_cm;
            vx_world = path_pid_update_with_arrival_deadband(&x_pid, error);
        }
        else
        {
            error = pre_push_box_target_y_cm - pose->y_cm;
            vy_world = path_pid_update_with_arrival_deadband(&y_pid, error);
        }
    }
    else
    {
        executor_set_error(EXEC_ERROR_ART_CENTER);
        return;
    }

    if(abs_float(error) < PATH_ARRIVAL_THRESHOLD_CM)
    {
        reset_motion_segment();
        if(pre_push_alignment_stable_ticks < EXEC_ARRIVAL_STABLE_TICKS)
        {
            pre_push_alignment_stable_ticks++;
        }
        if(pre_push_alignment_stable_ticks >= EXEC_ARRIVAL_STABLE_TICKS)
        {
            pre_push_alignment_stable_ticks = 0u;
            path_pid_reset(&x_pid);
            path_pid_reset(&y_pid);
            if(PRE_PUSH_BOX_RETRY_NUDGE == pre_push_box_phase)
            {
                pre_push_box_phase = PRE_PUSH_BOX_IDLE;
                pre_push_box_action = '\0';
                pre_push_alignment_elapsed_ms = 0u;
            }
            else if(PRE_PUSH_BOX_SAFE_GAP == pre_push_box_phase)
            {
                pre_push_box_phase = PRE_PUSH_BOX_ALIGN_PERP;
            }
            else if(PRE_PUSH_BOX_ALIGN_PERP == pre_push_box_phase)
            {
                pre_push_box_phase = PRE_PUSH_BOX_APPROACH;
            }
            else
            {
                executor_finish_pre_push_box_preparation();
            }
        }
        return;
    }

    pre_push_alignment_stable_ticks = 0u;
    world_velocity_to_body(vx_world, vy_world, pose->yaw_deg, &vx_body, &vy_body);
    set_motion(vx_body, vy_body);
}

/* Run 模式把同方向进入推箱及连续推箱前视到本任务直线终点，避免每格减速再加速。 */
static uint16 executor_continuous_target_step(void)
{
    const waypoint_struct *wp;
    const waypoint_struct *next_wp;
    uint16 target_step = current_step;

    if((0 != single_step_mode) || (0 == exec_waypoints))
    {
        return target_step;
    }

    while((target_step + 1u) < exec_waypoint_count)
    {
        wp = &exec_waypoints[target_step];
        next_wp = &exec_waypoints[target_step + 1u];

        if((0 != wp->task_end) ||
           (0 != next_wp->center_correct_before) ||
           (0 == action_is_push(next_wp->action)) ||
           (action_direction(wp->action) != action_direction(next_wp->action)))
        {
            break;
        }
        target_step++;
    }

    return target_step;
}

static void executor_finish_segment_settle(void)
{
    const waypoint_struct *wp = NULL;

    segment_settling = 0;
    segment_settle_elapsed_ms = 0;

    if((0 != exec_waypoints) && (current_step < exec_waypoint_count))
    {
        wp = &exec_waypoints[current_step];
    }

    if((0 != art_sync_enabled) && (0 != waypoint_needs_art_sync(wp)))
    {
        executor_enter_art_wait(wp->action);
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

static void executor_update_position_correction_20ms(void)
{
    if(0 != is_axis_arrived(position_correction_target_x_cm,
                            position_correction_target_y_cm, '\0'))
    {
        stop_motion();
        if(arrival_stable_ticks < EXEC_ARRIVAL_STABLE_TICKS)
        {
            arrival_stable_ticks++;
        }
        if(arrival_stable_ticks >= EXEC_ARRIVAL_STABLE_TICKS)
        {
            position_correction_active = 0u;
            exec_state = EXEC_STATE_DONE;
        }
        return;
    }

    arrival_stable_ticks = 0u;
    move_to_target(position_correction_target_x_cm,
                   position_correction_target_y_cm, '\0');
}

void executor_update_20ms(void)
{
    if (exec_state != EXEC_STATE_RUNNING)
    {
        return;
    }

    if(0u != position_correction_active)
    {
        executor_update_position_correction_20ms();
        return;
    }

    if (current_step >= exec_waypoint_count)
    {
        stop_motion();
        exec_state = EXEC_STATE_DONE;
        return;
    }

    const waypoint_struct *wp = &exec_waypoints[current_step];
    const waypoint_struct *target_wp;
    uint16 target_step;
    float target_x, target_y;

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

    if(PRE_PUSH_BOX_IDLE != pre_push_box_phase)
    {
        executor_update_pre_push_box_20ms();
        return;
    }

    if(0 != pre_push_alignment_active)
    {
        executor_update_pre_push_alignment_20ms();
        return;
    }

#if EXEC_ART_WAYPOINT_CENTER_CORRECT_ENABLE
    if((0 != art_sync_enabled) &&
       (0 != wp->center_correct_before) &&
       (0 == pre_push_center_confirmed))
    {
        if(0 == pre_push_center_waiting)
        {
            executor_enter_pre_push_center_wait();
        }
        else
        {
            reset_motion_segment();
        }
        return;
    }
#endif

    target_step = executor_continuous_target_step();
    target_wp = &exec_waypoints[target_step];
    grid_to_physical(target_wp->row, target_wp->col, &target_x, &target_y);
    apply_push_overshoot(&target_x, &target_y, target_wp->action);

    if (0 != is_axis_arrived(target_x, target_y, target_wp->action))
    {
        current_step = target_step;
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
        move_to_target(target_x, target_y, target_wp->action);
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

void executor_get_debug_status(executor_debug_status_struct *status)
{
    const drive_pose_struct *pose;
    float target_x = 0.0f;
    float target_y = 0.0f;
    char action = last_completed_action;

    if(0 == status)
    {
        return;
    }

    if(0u != position_correction_active)
    {
        target_x = position_correction_target_x_cm;
        target_y = position_correction_target_y_cm;
        action = 'c';
    }
    else if((0 != exec_waypoints) && (current_step < exec_waypoint_count))
    {
        const waypoint_struct *wp = &exec_waypoints[current_step];
        grid_to_physical(wp->row, wp->col, &target_x, &target_y);
        apply_push_overshoot(&target_x, &target_y, wp->action);
        action = wp->action;
    }

    pose = drive_pose_get();
    status->target_x_cm = target_x;
    status->target_y_cm = target_y;
    status->error_x_cm = target_x - pose->x_cm;
    status->error_y_cm = target_y - pose->y_cm;
    status->art_center_dx_cm = last_art_center_dx_cm;
    status->art_center_dy_cm = last_art_center_dy_cm;
    status->art_center_diff_cm = last_art_center_diff_cm;
    status->current_step = current_step;
    status->total_steps = exec_waypoint_count;
    status->action = action;
    status->state = (uint8)exec_state;
    status->art_sync_pending = executor_art_sync_pending();
    status->art_center_result = (uint8)last_art_center_result;
}
