#include "zf_common_headfile.h"
#include "art_observation.h"
#include "art_replan.h"
#include "drive_control.h"
#include "drive_config.h"
#include "drive_pose.h"
#include "executor.h"
#include "map_utils.h"
#include "openart_uart.h"
#include "solver.h"
#include "timebase.h"

#define ART_LAUNCH_DELAY_MS  (5000u)  /**< 发车前等人离场延迟，单位 ms。 */

typedef enum
{
    ART_REPLAN_IDLE = 0,         /**< 空闲态：不主动等待 ART，只响应 executor 触发的同步请求。 */
    ART_REPLAN_WAIT_LAUNCH,      /**< 发车前等待态：持续收图，但只计时不求解。 */
    ART_REPLAN_WAIT_CENTER,      /**< 发车区中心等待态：等待 CENTER_REQ 的5个精确中心样本。 */
    ART_REPLAN_LAUNCH_MOVE,      /**< 按 ART 中心计算距离，移动到外面第一可走格中心。 */
    ART_REPLAN_INITIAL,          /**< 出发车区后等待真实推箱地图并首次求解。 */
    ART_REPLAN_INITIAL_CENTER,   /**< 初次稳定地图已冻结，等待请求式精确中心后再求解。 */
    ART_REPLAN_SEGMENT,          /**< waypoint 段末同步：按 ART 最新地图做确认。 */
    ART_REPLAN_SEGMENT_CENTER,   /**< 单箱任务稳定地图已冻结，等待请求式精确中心后再重算。 */
    ART_REPLAN_PRE_PUSH_CENTER,  /**< waypoint 前中心校正；推箱链使用车箱配对观察。 */
    ART_REPLAN_RETURN_GRID,      /**< 推箱完成后执行网格 BFS 返航到左侧出口。 */
    ART_REPLAN_RETURN_CENTER,    /**< 到达出口后采集中心，用于计算返航精确偏移。 */
    ART_REPLAN_RETURN_ALIGN_Y,   /**< 在出口内先对齐发车中心所在 Y。 */
    ART_REPLAN_RETURN_MOVE_X,    /**< 沿 X 轴向左进入发车区。 */
    ART_REPLAN_RETURN_VERIFY,    /**< 停车复核是否回到保存的发车中心。 */
} art_replan_phase_enum;

static art_replan_phase_enum art_replan_phase = ART_REPLAN_IDLE; // 主循环写入和读取；决定稳定帧成功后的启动策略。
static map_stability_tracker_struct art_map_tracker;
static uint32 art_wait_start_ms = 0;                              // 本轮 ART 等待起点，单位 ms；用于统一初始/段末超时。
static uint8 art_launch_subject2 = 0;                             // 1 表示发车后把地图交给科目二，不运行任意配对求解。
static uint32 art_launch_delay_start_ms = 0;                      // 5 秒延迟起点。
static uint8 confirmed_box_count = 0;                             // 上一次被 ART 认定为“同步完成”的箱子数基线。
static uint8 confirmed_target_count = 0;                          // 上一次被 ART 认定为“同步完成”的目标数基线。
static uint8 confirmed_counts_valid = 0;                           // 1 表示上面的 B/T 基线有效，可用于判断是否发生了消除。
static uint32 art_monitor_last_frame = 0;                           // 执行空闲态已检查的最新地图帧号。
static uint8 art_external_change_pending = 0;                       // 1 表示上位机提前消除 B/T，正在接管旧路径。
static art_center_batch_struct art_center_batch;
static uint16 art_requested_center_col_q = 0;                       // 当前请求列中值，单位 1/100 格。
static uint16 art_requested_center_row_q = 0;                       // 当前请求行中值，单位 1/100 格。
static uint8 art_requested_center_valid = 0;                        // 1 表示当前请求已得到多帧样本中值。
static uint8 art_requested_center_fallback = 0;                     // 1 表示当前中心由超时后的地图格中心回退生成。
static uint8 art_pre_push_box_request_active = 0;                   // 1 表示当前 waypoint 前等待使用 OBSERVE_REQ。
static uint8 art_pre_push_box_preparation_started = 0;              // 1 表示 executor 已接管二维安全准备位。
static art_box_observation_session_struct art_box_session;
static uint8 art_pre_push_box_retry_count = 0u;
static uint8 art_pre_push_box_retry_moving = 0u;
static uint8 art_pre_push_box_retry_settling = 0u;
static uint32 art_pre_push_box_retry_settle_start_ms = 0u;
static uint32 art_launch_move_start_ms = 0;                         // 发车移动开始时间。
static uint16 art_home_center_col_q = 0;                            // 本轮启动时发车中心列，单位 1/100 格。
static uint16 art_home_center_row_q = 0;                            // 本轮启动时发车中心行，单位 1/100 格。
static uint8 art_home_center_valid = 0;                             // 1 表示本轮已保存发车中心，可执行自动返航。
static float art_return_pending_x_cm = 0.0f;                       // Y 对齐后待执行的 X 返航距离。
static uint8 art_return_correction_count = 0;                       // 最终视觉复核失败后的再次校正次数。
static uint32 art_return_phase_start_ms = 0;                        // 返航采样或单轴移动阶段起点。

static float art_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void art_requested_center_clear(void)
{
    art_center_batch_reset(&art_center_batch);
    art_requested_center_col_q = 0;
    art_requested_center_row_q = 0;
    art_requested_center_valid = 0;
    art_requested_center_fallback = 0;
}

#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
static uint8 art_requested_center_use_map_cell(const map_source_struct *source)
{
    map_scan_stats_struct stats;

    map_scan_stats(source, &stats);
    if(1u != stats.car_count)
    {
        return 0u;
    }
    art_requested_center_col_q = (uint16)(stats.car_col * 100u + 50u);
    art_requested_center_row_q = (uint16)(stats.car_row * 100u + 50u);
    art_requested_center_valid = 1u;
    art_requested_center_fallback = 1u;
    return 1u;
}
#endif

static void art_replan_center_timeout_error(executor_error_enum error,
                                            const char *state,
                                            art_replan_update_struct *update)
{
    stop_motion();
    executor_set_error(error);
    art_replan_cancel();
    if(0 != update)
    {
        update->run_state = state;
        update->redraw = 1u;
    }
}

static float art_grid_q_to_x_cm(uint16 col_q)
{
    return ((float)col_q * GRID_SIZE_CM) / 100.0f;
}

static uint8 art_replan_calculate_pose_offset(const map_source_struct *source,
                                               uint16 center_col_q,
                                               uint16 center_row_q,
                                               float *offset_x_cm,
                                               float *offset_y_cm)
{
    uint8 car_col;
    uint8 car_row;

    *offset_x_cm = 0.0f;
    *offset_y_cm = 0.0f;
    if(0 == map_validate_player_center(source, center_col_q, center_row_q,
                                       &car_row, &car_col))
    {
        return 0;
    }

    *offset_x_cm = ((float)center_col_q - ((float)car_col * 100.0f + 50.0f))
                   * GRID_SIZE_CM / 100.0f;
    *offset_y_cm = -(((float)center_row_q - ((float)car_row * 100.0f + 50.0f))
                     * GRID_SIZE_CM / 100.0f);
    return 1;
}

static void art_replan_retry_center_request(const char *state,
                                            art_replan_update_struct *update)
{
    art_requested_center_clear();
    executor_reset_art_player_center_samples();
    openart_request_player_center();
    if(0 != update)
    {
        update->run_state = state;
        update->redraw = 1u;
    }
}

static void art_replan_update_reset(art_replan_update_struct *update)
{
    if(0 == update)
    {
        return;
    }
    memset(update, 0, sizeof(*update));
    update->playback = ART_REPLAN_PLAYBACK_KEEP;
}

static void art_replan_wait_fresh_frame(void)
{
    // 切入等待态时，先丢掉半帧和旧缓冲。
    // 这样后续只有“等待之后新产生的完整帧”才会参与稳定性判断。
    openart_uart_discard_pending();
    map_stability_tracker_reset(&art_map_tracker,
                                openart_uart_get_frame_count());
}

static void art_replan_begin_center_request(art_replan_phase_enum phase,
                                            const char *state,
                                            art_replan_update_struct *update)
{
    stop_motion();
    art_replan_phase = phase;
    art_wait_start_ms = time_ms();
    art_requested_center_clear();
    openart_request_player_center();
    if(0 != update)
    {
        update->run_state = state;
        update->redraw = 1;
    }
}

static void art_replan_begin(art_replan_phase_enum phase, art_replan_update_struct *update)
{
    if(ART_REPLAN_PRE_PUSH_CENTER == phase)
    {
        uint8 box_row;
        uint8 box_col;

        art_pre_push_box_request_active = 0u;
        art_pre_push_box_preparation_started = 0u;
        art_pre_push_box_retry_count = 0u;
        art_pre_push_box_retry_moving = 0u;
        art_pre_push_box_retry_settling = 0u;
        art_pre_push_box_retry_settle_start_ms = 0u;
        if(0 != executor_get_pre_push_box_request(&box_row, &box_col))
        {
            stop_motion();
            art_replan_phase = phase;
            art_wait_start_ms = time_ms();
            art_box_observation_session_request(&art_box_session,
                                                box_row, box_col);
            art_pre_push_box_request_active = 1u;
            if(0 != update)
            {
                update->run_state = "BCtr";
                update->redraw = 1u;
            }
        }
        else
        {
            art_replan_begin_center_request(phase, "PCtr", update);
        }
        return;
    }

    art_replan_phase = phase;
    art_replan_wait_fresh_frame();
    art_wait_start_ms = time_ms();
    if(0 != update)
    {
        if(ART_REPLAN_INITIAL == phase)
        {
            update->run_state = "WMAP";
        }
        else if(ART_REPLAN_PRE_PUSH_CENTER == phase)
        {
            update->run_state = "PCtr";
        }
        else
        {
            update->run_state = "ART Sync";
        }
        update->redraw = 1;
    }
}

static void art_replan_restart_stability(void)
{
    art_replan_wait_fresh_frame();
}

static uint8 art_get_stable_map(const map_source_struct **source_out)
{
    const map_source_struct *source = openart_map_get();
    uint32 frame = openart_uart_get_frame_count();
    map_stability_result_enum result;

    if((0 == source) || (0 == frame))
    {
        return 0;
    }

    result = map_stability_tracker_push(&art_map_tracker, frame, source,
                                        EXEC_ART_STABLE_FRAMES);
    if(MAP_STABILITY_READY == result)
    {
        *source_out = source;
        return 1;
    }
    return 0;
}

static uint8 art_stats_done(const map_scan_stats_struct *stats)
{
    // 箱子和目标都消失表示虚拟推箱子任务已经完成；仍要求有且只有一个车格由上层判断。
    return ((0 == stats->box_count) && (0 == stats->target_count)) ? 1u : 0u;
}

static uint8 art_stats_car_inside_playable_area(const map_scan_stats_struct *stats)
{
    if((0u == stats->car_row) || (stats->car_row >= (MAP_ROWS - 1u)))
    {
        return 0;
    }
    if((0u == stats->car_col) || (stats->car_col >= (MAP_COLS - 1u)))
    {
        return 0;
    }
    return 1;
}

static uint8 art_stats_valid_for_solve(const map_scan_stats_struct *stats)
{
    // BFS 只接受真实推箱地图：单车在内圈，且 B/T 非零、数量一致。
    // 不满足时继续等新稳定帧，不拿可疑识别结果硬算路径。
    if(1u != stats->car_count)
    {
        return 0;
    }
    if(0 == art_stats_car_inside_playable_area(stats))
    {
        return 0;
    }
    if((0u == stats->box_count) || (0u == stats->target_count))
    {
        return 0;
    }
    if(stats->box_count != stats->target_count)
    {
        return 0;
    }
    if((stats->box_count > MAX_BOXES) || (stats->target_count > MAX_BOXES))
    {
        return 0;
    }
    return (0 != stats->box_count) ? 1u : 0u;
}

static void art_update_confirmed_counts(const map_scan_stats_struct *stats)
{
    // 只提交即将执行或返航的配套地图数量，作为下一次段末同步的对照基线。
    confirmed_box_count = stats->box_count;
    confirmed_target_count = stats->target_count;
    confirmed_counts_valid = 1;
}

static uint8 art_stats_count_decreased(const map_scan_stats_struct *stats)
{
    if(0 == confirmed_counts_valid)
    {
        return 0;
    }
    // 只要 B 或 T 真的减少，就说明上位机已经把这次推箱结果消掉了。
    if(stats->box_count < confirmed_box_count)
    {
        return 1;
    }
    if(stats->target_count < confirmed_target_count)
    {
        return 1;
    }
    return 0;
}

static uint8 art_stats_paired_count_decreased(const map_scan_stats_struct *stats)
{
    uint8 box_reduction;
    uint8 target_reduction;

    if((0 == confirmed_counts_valid) ||
       (stats->box_count >= confirmed_box_count) ||
       (stats->target_count >= confirmed_target_count))
    {
        return 0;
    }

    box_reduction = (uint8)(confirmed_box_count - stats->box_count);
    target_reduction = (uint8)(confirmed_target_count - stats->target_count);
    return (box_reduction == target_reduction) ? 1u : 0u;
}

static uint8 art_replan_handle_host_completion(art_replan_update_struct *update)
{
    const map_source_struct *source;
    map_scan_stats_struct stats;
    uint32 frame = openart_uart_get_frame_count();
    executor_state_enum state = executor_get_state();

    if((0u == frame) || (frame == art_monitor_last_frame) ||
       ((EXEC_STATE_RUNNING != state) && (EXEC_STATE_PAUSED != state)))
    {
        return 0u;
    }
    art_monitor_last_frame = frame;

    source = openart_map_get();
    if(0 == source)
    {
        return 0u;
    }
    map_scan_stats(source, &stats);
    if((1u != stats.car_count) ||
       (0 == art_stats_paired_count_decreased(&stats)))
    {
        return 0u;
    }

    executor_stop();
    art_box_observation_session_reset(&art_box_session);
    art_pre_push_box_request_active = 0u;
    art_pre_push_box_preparation_started = 0u;
    art_pre_push_box_retry_count = 0u;
    art_pre_push_box_retry_moving = 0u;
    art_pre_push_box_retry_settling = 0u;
    art_pre_push_box_retry_settle_start_ms = 0u;
    art_external_change_pending = 1u;
    art_replan_begin(ART_REPLAN_SEGMENT, update);
    if(0 != update)
    {
        update->run_state = "Host Sync";
        update->redraw = 1u;
    }
    return 1u;
}

static uint8 art_action_is_push(char action)
{
    // 大写动作保留为“推箱语义”；普通小写只表示普通移动同步。
    return ((action >= 'A') && (action <= 'Z')) ? 1u : 0u;
}

static uint8 art_center_result_needs_map(executor_art_center_result_enum center_result)
{
    return (EXEC_ART_CENTER_ABNORMAL == center_result) ? 1u : 0u;
}

static void art_replan_save_snapshot(const art_replan_context_struct *context,
                                     const map_source_struct *source)
{
    map_source_snapshot(context->snapshot, context->snapshot_rows, source);
    *context->snapshot_valid = 1;
}

static void art_replan_start_executor(const art_replan_context_struct *context,
                                      const map_scan_stats_struct *stats,
                                      float initial_pose_x_cm,
                                      float initial_pose_y_cm,
                                      art_replan_update_struct *update)
{
    uint8 single_step = (RUN_MODE_STEP == context->run_mode) ? 1u : 0u;

    *context->start_row = stats->car_row;
    *context->start_col = stats->car_col;
    executor_start(context->result->waypoints, context->result->waypoint_count,
                   *context->start_row, *context->start_col,
                   initial_pose_x_cm, initial_pose_y_cm,
                   single_step, 1u);
    if(0 != update)
    {
        update->run_state = (0 != single_step) ? "Paused" : "Running";
    }
}

static uint8 art_replan_collect_requested_center(void)
{
    if(0u == art_center_batch_collect(&art_center_batch))
    {
        return 0;
    }

    if(0 == art_requested_center_valid)
    {
        (void)art_center_batch_get_median(&art_center_batch,
                                          &art_requested_center_col_q,
                                          &art_requested_center_row_q);
        art_requested_center_valid = 1;
        art_requested_center_fallback = 0;
    }
    return 1;
}

static uint8 art_pre_push_center_succeeded(executor_art_center_result_enum result)
{
    return ((EXEC_ART_CENTER_APPLIED == result) ||
            (EXEC_ART_CENTER_IGNORED == result)) ? 1u : 0u;
}

static void art_replan_fail_pre_push_center(art_replan_update_struct *update)
{
    executor_set_error(EXEC_ERROR_ART_CENTER);
    art_replan_cancel();
    if(0 != update)
    {
        update->run_state = "E:Ctr";
        update->redraw = 1;
    }
}

static void art_replan_fail_pre_push_box(const char *state,
                                         art_replan_update_struct *update)
{
    executor_set_error(EXEC_ERROR_ART_CENTER);
    art_replan_cancel();
    if(0 != update)
    {
        update->run_state = state;
        update->redraw = 1u;
    }
}

static void art_replan_tick_pre_push_box(art_replan_update_struct *update)
{
    executor_art_box_prep_result_enum prep_result;

    if(0u != art_pre_push_box_retry_moving)
    {
        if(0u != executor_pre_push_box_preparation_active())
        {
            if(0 != update) update->run_state = "BRetry";
            return;
        }
        if(EXEC_STATE_ERROR == executor_get_state())
        {
            art_replan_cancel();
            if(0 != update)
            {
                update->run_state = "E:BTim";
                update->redraw = 1u;
            }
            return;
        }
        if(0u == art_pre_push_box_retry_settling)
        {
            art_pre_push_box_retry_settling = 1u;
            art_pre_push_box_retry_settle_start_ms = time_ms();
        }
        if((time_ms() - art_pre_push_box_retry_settle_start_ms) <
           ART_BOX_OBSERVE_RETRY_SETTLE_MS)
        {
            if(0 != update) update->run_state = "BSet";
            return;
        }

        art_pre_push_box_retry_moving = 0u;
        art_pre_push_box_retry_settling = 0u;
        art_box_observation_session_request(&art_box_session,
                                            art_box_session.box_row,
                                            art_box_session.box_col);
        art_wait_start_ms = time_ms();
        if(0 != update)
        {
            update->run_state = "BCtr";
            update->redraw = 1u;
        }
        return;
    }

    if(0u != art_pre_push_box_preparation_started)
    {
        if(0u != executor_pre_push_box_preparation_active())
        {
            if(0 != update)
            {
                update->run_state = executor_pre_push_box_state_name();
            }
            return;
        }
        if(EXEC_STATE_ERROR == executor_get_state())
        {
            art_replan_cancel();
            if(0 != update)
            {
                update->run_state = "E:BTim";
                update->redraw = 1u;
            }
            return;
        }

        art_replan_cancel();
        if(0 != update)
        {
            update->run_state = executor_state_name();
            update->redraw = 1u;
        }
        return;
    }

    if(0u == art_box_observation_session_collect(&art_box_session))
    {
        if((time_ms() - art_wait_start_ms) >= ART_BOX_OBSERVE_WAIT_MS)
        {
            if(art_pre_push_box_retry_count < ART_BOX_OBSERVE_MAX_RETRIES)
            {
                art_box_session.active = 0u;
                art_box_session.ready = 0u;
                executor_reset_art_box_observation_samples();
                if(0 == executor_start_pre_push_box_retry_nudge(
                             ART_BOX_OBSERVE_RETRY_MOVE_CM))
                {
                    art_replan_fail_pre_push_box("E:BTim", update);
                    return;
                }
                art_pre_push_box_retry_count++;
                art_pre_push_box_retry_moving = 1u;
                if(0 != update)
                {
                    update->run_state = "BRetry";
                    update->redraw = 1u;
                }
            }
            else
            {
                art_replan_fail_pre_push_box("E:BObs", update);
            }
        }
        else if(0 != update)
        {
            update->run_state = "BCtr";
        }
        return;
    }

    prep_result = executor_start_pre_push_box_preparation();
    if(EXEC_ART_BOX_PREP_STARTED != prep_result)
    {
        if(0 == executor_continue_after_pre_push_center())
        {
            art_replan_fail_pre_push_center(update);
            return;
        }
        art_replan_cancel();
        if(0 != update)
        {
            update->run_state = executor_state_name();
            update->redraw = 1u;
        }
        return;
    }

    art_box_observation_session_reset(&art_box_session);
    art_pre_push_box_preparation_started = 1u;
    if(0 != update)
    {
        update->run_state = executor_pre_push_box_state_name();
        update->redraw = 1u;
    }
}

static void art_replan_tick_pre_push_center(const art_replan_context_struct *context,
                                             uint8 center_ready,
                                             art_replan_update_struct *update)
{
    const map_source_struct *source;
    executor_art_center_result_enum center_result;
    uint8 reference_row;
    uint8 reference_col;

    (void)context;

    if(0 == center_ready)
    {
        if((time_ms() - art_wait_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
        {
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
            if(0 == executor_continue_after_pre_push_center())
            {
                art_replan_fail_pre_push_center(update);
                return;
            }
            art_replan_cancel();
            if(0 != update)
            {
                update->run_state = executor_state_name();
                update->redraw = 1;
            }
#else
            art_replan_fail_pre_push_center(update);
#endif
            return;
        }
        if(0 != update)
        {
            update->run_state = "PCtr";
        }
        return;
    }

    source = openart_get_requested_center_map();
    if(0 == map_validate_player_center(source,
                                       art_requested_center_col_q,
                                       art_requested_center_row_q,
                                       &reference_row,
                                       &reference_col))
    {
        art_replan_retry_center_request("PCtr", update);
        return;
    }

    (void)art_center_batch_apply_to_executor(&art_center_batch);
    center_result = executor_commit_art_player_center(reference_row, reference_col);
    if(0 == art_pre_push_center_succeeded(center_result))
    {
        art_replan_fail_pre_push_center(update);
        return;
    }
    if(0 != executor_center_requires_push_alignment())
    {
        if(0 == executor_start_pre_push_alignment(reference_row, reference_col))
        {
            art_replan_fail_pre_push_center(update);
            return;
        }
    }
    else if(0 == executor_continue_after_pre_push_center())
    {
        art_replan_fail_pre_push_center(update);
        return;
    }

    art_replan_cancel();
    if(0 != update)
    {
        update->run_state = executor_state_name();
        update->redraw = 1;
    }
}

static void art_replan_begin_wait_center(art_replan_update_struct *update)
{
    art_replan_begin_center_request(ART_REPLAN_WAIT_CENTER, "WCTR", update);
}

static void art_replan_begin_launch_move(float move_cm, art_replan_update_struct *update)
{
    executor_stop();
    art_launch_move_start_ms = time_ms();
    art_replan_phase = ART_REPLAN_LAUNCH_MOVE;
    if(0 == executor_start_position_correction_with_pose_reset(
                 0.0f, 0.0f, move_cm, 0.0f))
    {
        art_replan_center_timeout_error(EXEC_ERROR_ART_TIMEOUT,
                                        "E:LMov", update);
        return;
    }
    if(0 != update)
    {
        update->run_state = "LCH";
        update->redraw = 1;
    }
}

static void art_replan_tick_wait_center(art_replan_update_struct *update)
{
    if(0 != art_replan_collect_requested_center())
    {
        float current_x_cm = art_grid_q_to_x_cm(art_requested_center_col_q);
        float move_cm = ART_LAUNCH_TARGET_X_CM - current_x_cm;

        art_home_center_col_q = art_requested_center_col_q;
        art_home_center_row_q = art_requested_center_row_q;
        art_home_center_valid = 1;

        if(move_cm <= 0.0f)
        {
            art_replan_begin(ART_REPLAN_INITIAL, update);
            return;
        }

        art_replan_begin_launch_move(move_cm, update);
        return;
    }

    if((time_ms() - art_wait_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
        art_home_center_col_q = (uint16)(((ART_LAUNCH_TARGET_X_CM -
                                           ART_LAUNCH_FALLBACK_MOVE_CM) * 100.0f /
                                          GRID_SIZE_CM) + 0.5f);
        art_home_center_row_q = (uint16)(ART_RETURN_GATE_ROW_MIN * 100u + 50u);
        art_home_center_valid = 1u;
        art_replan_begin_launch_move(ART_LAUNCH_FALLBACK_MOVE_CM, update);
#else
        art_replan_center_timeout_error(EXEC_ERROR_ART_SYNC, "E:LCtr", update);
#endif
        return;
    }

    if(0 != update)
    {
        update->run_state = "WCTR";
    }
}

static void art_replan_tick_launch_move(art_replan_update_struct *update)
{
    executor_state_enum state = executor_get_state();

    if((time_ms() - art_launch_move_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        executor_stop();
        art_replan_begin(ART_REPLAN_INITIAL, update);
        return;
    }

    if(EXEC_STATE_DONE == state)
    {
        executor_stop();
        art_replan_begin(ART_REPLAN_INITIAL, update);
        return;
    }
    if(EXEC_STATE_ERROR == state)
    {
        art_replan_center_timeout_error(EXEC_ERROR_ART_TIMEOUT,
                                        "E:LMov", update);
        return;
    }

    if(0 != update)
    {
        update->run_state = "LCH";
    }
}

static uint16 art_u16_difference(uint16 left, uint16 right)
{
    return (left >= right) ? (uint16)(left - right) : (uint16)(right - left);
}

static void art_replan_return_error(executor_error_enum error,
                                    art_replan_update_struct *update)
{
    stop_motion();
    art_replan_cancel();
    executor_set_error(error);
    if(0 != update)
    {
        update->run_state = "RetErr";
        update->redraw = 1;
    }
}

static void art_replan_finish_return(art_replan_update_struct *update)
{
    stop_motion();
    art_replan_cancel();
    executor_finish_done();
    if(0 != update)
    {
        update->playback = ART_REPLAN_PLAYBACK_DONE;
        update->return_complete = 1u;
        update->run_state = "Done";
        update->redraw = 1;
    }
}

static void art_replan_begin_return_center(art_replan_phase_enum phase,
                                            art_replan_update_struct *update)
{
    art_return_phase_start_ms = time_ms();
    art_replan_begin_center_request(
        phase,
        (ART_REPLAN_RETURN_VERIFY == phase) ? "RetChk" : "RetCtr",
        update);
}

static void art_replan_begin_return_axis(art_replan_phase_enum phase,
                                         float target_cm,
                                         art_replan_update_struct *update)
{
    float target_x_cm = 0.0f;
    float target_y_cm = 0.0f;

    executor_stop();
    art_replan_phase = phase;
    art_return_phase_start_ms = time_ms();
    if(ART_REPLAN_RETURN_ALIGN_Y == phase)
    {
        target_y_cm = target_cm;
    }
    else
    {
        target_x_cm = target_cm;
    }
    if(0 == executor_start_position_correction_with_pose_reset(
                 0.0f, 0.0f, target_x_cm, target_y_cm))
    {
        art_replan_return_error(EXEC_ERROR_ART_TIMEOUT, update);
        return;
    }
    if(0 != update)
    {
        update->run_state = (ART_REPLAN_RETURN_ALIGN_Y == phase) ? "RetY" : "RetX";
        update->redraw = 1;
    }
}

static void art_replan_begin_return_x_or_verify(art_replan_update_struct *update)
{
    if(art_abs_float(art_return_pending_x_cm) <= PATH_ARRIVAL_THRESHOLD_CM)
    {
        art_replan_begin_return_center(ART_REPLAN_RETURN_VERIFY, update);
    }
    else
    {
        art_replan_begin_return_axis(ART_REPLAN_RETURN_MOVE_X,
                                     art_return_pending_x_cm,
                                     update);
    }
}

static void art_replan_start_return_correction(uint16 current_col_q,
                                               uint16 current_row_q,
                                               art_replan_update_struct *update)
{
    float target_y_cm;

    art_return_pending_x_cm = ((float)art_home_center_col_q - (float)current_col_q)
                              * GRID_SIZE_CM / 100.0f;
    target_y_cm = -(((float)art_home_center_row_q - (float)current_row_q)
                    * GRID_SIZE_CM / 100.0f);
    if(art_abs_float(target_y_cm) <= PATH_ARRIVAL_THRESHOLD_CM)
    {
        art_replan_begin_return_x_or_verify(update);
    }
    else
    {
        art_replan_begin_return_axis(ART_REPLAN_RETURN_ALIGN_Y, target_y_cm, update);
    }
}

static void art_replan_tick_return_center(art_replan_update_struct *update)
{
    art_replan_phase_enum phase = art_replan_phase;

    if(0 != art_replan_collect_requested_center())
    {
        if(ART_REPLAN_RETURN_VERIFY == phase)
        {
            if((art_u16_difference(art_requested_center_col_q, art_home_center_col_q) <=
                ART_RETURN_HOME_TOLERANCE_Q) &&
               (art_u16_difference(art_requested_center_row_q, art_home_center_row_q) <=
                ART_RETURN_HOME_TOLERANCE_Q))
            {
                art_replan_finish_return(update);
                return;
            }
            if(art_return_correction_count >= ART_RETURN_MAX_CORRECTIONS)
            {
                art_replan_return_error(EXEC_ERROR_ART_SYNC, update);
                return;
            }
            art_return_correction_count++;
        }

        art_replan_start_return_correction(art_requested_center_col_q,
                                           art_requested_center_row_q,
                                           update);
        return;
    }

    if((time_ms() - art_wait_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        if(ART_REPLAN_RETURN_VERIFY == phase)
        {
            art_replan_return_error(EXEC_ERROR_ART_SYNC, update);
            return;
        }
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
        if(0 == art_requested_center_use_map_cell(openart_map_get()))
        {
            art_requested_center_col_q = (uint16)(ART_RETURN_GATE_COL * 100u + 50u);
            art_requested_center_row_q = (uint16)(ART_RETURN_GATE_ROW_MIN * 100u + 50u);
            art_requested_center_valid = 1u;
        }
        art_replan_start_return_correction(art_requested_center_col_q,
                                           art_requested_center_row_q,
                                           update);
#else
        art_replan_return_error(EXEC_ERROR_ART_SYNC, update);
#endif
        return;
    }
}

static void art_replan_tick_return_axis(art_replan_update_struct *update)
{
    uint8 is_y_axis = (ART_REPLAN_RETURN_ALIGN_Y == art_replan_phase) ? 1u : 0u;
    executor_state_enum state = executor_get_state();

    if((time_ms() - art_return_phase_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        art_replan_return_error(EXEC_ERROR_ART_TIMEOUT, update);
        return;
    }

    if(EXEC_STATE_DONE == state)
    {
        executor_stop();
        if(0 != is_y_axis)
        {
            art_replan_begin_return_x_or_verify(update);
        }
        else
        {
            art_replan_begin_return_center(ART_REPLAN_RETURN_VERIFY, update);
        }
        return;
    }
    if(EXEC_STATE_ERROR == state)
    {
        art_replan_return_error(EXEC_ERROR_ART_TIMEOUT, update);
    }
}

static uint8 art_replan_solve_return_gate(const art_replan_context_struct *context)
{
    uint8 preferred_row = (uint8)(art_home_center_row_q / 100u);
    uint8 alternate_row;

    if(preferred_row < ART_RETURN_GATE_ROW_MIN)
    {
        preferred_row = ART_RETURN_GATE_ROW_MIN;
    }
    else if(preferred_row > ART_RETURN_GATE_ROW_MAX)
    {
        preferred_row = ART_RETURN_GATE_ROW_MAX;
    }

    if(0 != solve_navigation_path(context->snapshot,
                                  preferred_row,
                                  ART_RETURN_GATE_COL,
                                  context->result))
    {
        return 1;
    }

    alternate_row = (ART_RETURN_GATE_ROW_MIN == preferred_row) ?
                    ART_RETURN_GATE_ROW_MAX : ART_RETURN_GATE_ROW_MIN;
    if((alternate_row != preferred_row) &&
       (0 != solve_navigation_path(context->snapshot,
                                   alternate_row,
                                   ART_RETURN_GATE_COL,
                                   context->result)))
    {
        return 1;
    }
    return 0;
}

static void art_replan_start_return(const art_replan_context_struct *context,
                                     const map_scan_stats_struct *stats,
                                     float initial_pose_x_cm,
                                     float initial_pose_y_cm,
                                     art_replan_update_struct *update)
{
    uint8 single_step = (RUN_MODE_STEP == context->run_mode) ? 1u : 0u;
    uint32 start_ms;

    if(0 == art_home_center_valid)
    {
        art_replan_return_error(EXEC_ERROR_ART_SYNC, update);
        return;
    }

    executor_stop();
    start_ms = time_ms();
    if(0 == art_replan_solve_return_gate(context))
    {
        *context->elapsed_ms = time_ms() - start_ms;
        art_replan_return_error(EXEC_ERROR_ART_PLAN, update);
        return;
    }
    *context->elapsed_ms = time_ms() - start_ms;
    *context->start_row = stats->car_row;
    *context->start_col = stats->car_col;
    art_return_correction_count = 0;
    if(0 != update)
    {
        update->reset_playback_step = 1;
        update->playback = ART_REPLAN_PLAYBACK_PAUSED;
        update->run_state = "RetPlan";
        update->redraw = 1;
    }

    if(0 == context->result->waypoint_count)
    {
        art_replan_begin_return_center(ART_REPLAN_RETURN_CENTER, update);
        return;
    }

    executor_start(context->result->waypoints, context->result->waypoint_count,
                   *context->start_row, *context->start_col,
                   initial_pose_x_cm, initial_pose_y_cm,
                   single_step, 0u);
    art_replan_phase = ART_REPLAN_RETURN_GRID;
    if(0 != update)
    {
        update->run_state = "RetGrid";
    }
}

static void art_handle_stable_map(const art_replan_context_struct *context,
                                   const map_source_struct *source,
                                   art_replan_update_struct *update)
{
    map_scan_stats_struct stats;
    art_replan_phase_enum phase = art_replan_phase;
    executor_art_center_result_enum center_result = EXEC_ART_CENTER_NONE;

    art_replan_save_snapshot(context, source);
    map_scan_stats(context->snapshot, &stats);

    if((ART_REPLAN_SEGMENT == phase) &&
       (1u == stats.car_count) &&
       (0 != art_stats_done(&stats)))
    {
        art_replan_begin_center_request(ART_REPLAN_SEGMENT_CENTER, "RCtr", update);
        return;
    }

    if(0 == art_stats_valid_for_solve(&stats))
    {
        // 地图还没稳定到可解，重新进入稳定性等待，不在这张图上继续推进。
        art_replan_restart_stability();
        if(0 != update)
        {
            update->run_state = (ART_REPLAN_INITIAL == phase) ? "WMAP" : "ART Retry";
            update->redraw = 1;
        }
        return;
    }

    if((ART_REPLAN_SEGMENT == phase) && (0u != art_external_change_pending))
    {
        art_replan_begin_center_request(ART_REPLAN_SEGMENT_CENTER, "RCtr", update);
        return;
    }

    if(ART_REPLAN_SEGMENT == phase)
    {
        char sync_action = executor_get_art_sync_action();

        if(0 == art_action_is_push(sync_action))
        {
#if EXEC_ART_CENTER_CORRECT_ENABLE
            center_result = executor_commit_art_player_center(stats.car_row, stats.car_col);
#else
            center_result = EXEC_ART_CENTER_NONE;
#endif
            if(0 == art_center_result_needs_map(center_result))
            {
                (void)executor_continue_after_art_sync();
                art_replan_cancel();
                if(0 != update)
                {
                    update->run_state = executor_state_name();
                    update->redraw = 1;
                }
                return;
            }
            if(0 != update)
            {
                update->run_state = "ART Replan";
            }
        }
        else if(0 != art_stats_count_decreased(&stats))
        {
            // 单箱任务结束点：必须看到 B/T 真的减少，才算上位机已确认这次箱子消除。
            if(0 != update)
            {
                update->run_state = "Box OK";
            }
        }
        else
        {
            // 单箱任务结束后 B/T 没变，说明这次箱子任务还没被 ART 接受，后续应重算/重试。
            if(0 != update)
            {
                update->run_state = "Push Retry";
            }
        }
    }

    art_replan_begin_center_request(
        (ART_REPLAN_INITIAL == phase) ? ART_REPLAN_INITIAL_CENTER : ART_REPLAN_SEGMENT_CENTER,
        (ART_REPLAN_INITIAL == phase) ? "ICtr" : "RCtr",
        update);
}

static void art_replan_solve_snapshot_after_center(
    const art_replan_context_struct *context,
    art_replan_phase_enum center_phase,
    art_replan_update_struct *update)
{
    const map_source_struct *paired_source;
    map_scan_stats_struct stats;
    art_replan_phase_enum map_phase =
        (ART_REPLAN_INITIAL_CENTER == center_phase) ? ART_REPLAN_INITIAL : ART_REPLAN_SEGMENT;
    float initial_pose_x_cm;
    float initial_pose_y_cm;
    uint32 start_ms;

    paired_source = openart_get_requested_center_map();
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    if((0 == paired_source) && (0 != art_requested_center_fallback))
    {
        paired_source = context->snapshot;
    }
#endif
    if(0 == art_replan_calculate_pose_offset(paired_source,
                                             art_requested_center_col_q,
                                             art_requested_center_row_q,
                                             &initial_pose_x_cm,
                                             &initial_pose_y_cm))
    {
        art_replan_retry_center_request(
            (ART_REPLAN_INITIAL_CENTER == center_phase) ? "ICtr" : "RCtr",
            update);
        return;
    }
    map_scan_stats(paired_source, &stats);
    if((0 == art_stats_valid_for_solve(&stats)) &&
       !((ART_REPLAN_SEGMENT == map_phase) && (0 != art_stats_done(&stats))))
    {
        art_replan_begin(map_phase, update);
        return;
    }
    art_replan_save_snapshot(context, paired_source);

    if((ART_REPLAN_INITIAL == map_phase) && (0u != art_launch_subject2))
    {
        *context->start_row = stats.car_row;
        *context->start_col = stats.car_col;
        memset(context->result, 0, sizeof(*context->result));
        *context->elapsed_ms = 0u;
        art_replan_cancel();
        if(0 != update)
        {
            update->subject2_map_ready = 1u;
            update->initial_pose_x_cm = initial_pose_x_cm;
            update->initial_pose_y_cm = initial_pose_y_cm;
            update->run_state = "BScan";
            update->enter_execute = 1u;
            update->redraw = 1u;
        }
        return;
    }

    if((ART_REPLAN_SEGMENT == map_phase) && (0 != art_stats_done(&stats)))
    {
        if(0 != ART_RETURN_HOME_ENABLE)
        {
            art_replan_start_return(context,
                                    &stats,
                                    initial_pose_x_cm,
                                    initial_pose_y_cm,
                                    update);
            if(ART_REPLAN_IDLE != art_replan_phase)
            {
                art_update_confirmed_counts(&stats);
            }
        }
        else
        {
            art_update_confirmed_counts(&stats);
            *context->start_row = stats.car_row;
            *context->start_col = stats.car_col;
            clear_result(context->result);
            *context->elapsed_ms = 0;
            art_replan_cancel();
            executor_finish_done();
            if(0 != update)
            {
                update->playback = ART_REPLAN_PLAYBACK_DONE;
                update->run_state = "Done";
                update->redraw = 1;
            }
        }
        return;
    }

    // 求解可能占用较长主循环时间；求解前后清空未解析字节，避免旧画面积压进下一轮同步。
    openart_uart_discard_pending();
    start_ms = time_ms();
    if(0 != solve_map(context->snapshot, context->result))
    {
        *context->elapsed_ms = time_ms() - start_ms;
        openart_uart_discard_pending();
        art_update_confirmed_counts(&stats);
        if(0 != update)
        {
            update->reset_playback_step = 1;
            update->playback = ART_REPLAN_PLAYBACK_PAUSED;
        }
        if(ART_REPLAN_INITIAL == map_phase)
        {
            // 初始阶段求解成功后直接进入执行。
            art_replan_start_executor(context,
                                      &stats,
                                      initial_pose_x_cm,
                                      initial_pose_y_cm,
                                      update);
            art_replan_cancel();
        }
        else
        {
            // 段末重解算来自执行器的同步请求，稳定帧通过后直接续跑，不再额外停一层。
            art_replan_start_executor(context,
                                      &stats,
                                      initial_pose_x_cm,
                                      initial_pose_y_cm,
                                      update);
            art_replan_cancel();
            if(0 != update)
            {
                update->redraw = 1;
            }
        }
        if((ART_REPLAN_INITIAL == map_phase) && (0 != update))
        {
            update->enter_execute = 1;
            update->redraw = 1;
        }
    }
    else
    {
        *context->elapsed_ms = time_ms() - start_ms;
        art_replan_begin(map_phase, update);
        if(0 != update)
        {
            update->run_state = "ART Retry";
            update->redraw = 1;
        }
    }
}

void art_replan_cancel(void)
{
    art_replan_phase = ART_REPLAN_IDLE;
    map_stability_tracker_reset(&art_map_tracker, 0u);
    art_wait_start_ms = 0;
    art_launch_subject2 = 0u;
    art_launch_delay_start_ms = 0;
    art_launch_move_start_ms = 0;
    art_return_pending_x_cm = 0.0f;
    art_return_correction_count = 0;
    art_return_phase_start_ms = 0;
    art_monitor_last_frame = 0;
    art_external_change_pending = 0;
    art_requested_center_clear();
    art_pre_push_box_request_active = 0u;
    art_pre_push_box_preparation_started = 0u;
    art_box_observation_session_reset(&art_box_session);
    art_pre_push_box_retry_count = 0u;
    art_pre_push_box_retry_moving = 0u;
    art_pre_push_box_retry_settling = 0u;
    art_pre_push_box_retry_settle_start_ms = 0u;
}

void art_replan_begin_initial(art_replan_update_struct *update)
{
    art_replan_update_reset(update);
    art_replan_phase = ART_REPLAN_WAIT_LAUNCH;
    art_launch_subject2 = 0u;
    art_launch_delay_start_ms = time_ms();
    confirmed_box_count = 0;
    confirmed_target_count = 0;
    confirmed_counts_valid = 0;
    art_monitor_last_frame = 0;
    art_external_change_pending = 0;
    art_home_center_col_q = 0;
    art_home_center_row_q = 0;
    art_home_center_valid = 0;
    art_launch_move_start_ms = 0;
    art_requested_center_clear();
    if(0 != update)
    {
        update->run_state = "W5S";
        update->redraw = 1;
    }
}

void art_replan_begin_subject2(art_replan_update_struct *update)
{
    art_replan_begin_initial(update);
    art_launch_subject2 = 1u;
}

uint8 art_replan_begin_return_home(const art_replan_context_struct *context,
                                   float initial_pose_x_cm,
                                   float initial_pose_y_cm,
                                   art_replan_update_struct *update)
{
    map_scan_stats_struct stats;

    art_replan_update_reset(update);
    if((0 == context) || (0 == context->snapshot) ||
       (0 == context->snapshot_valid) || (0u == *context->snapshot_valid))
    {
        return 0u;
    }
    map_scan_stats(context->snapshot, &stats);
    if((1u != stats.car_count) || (0u != stats.box_count) ||
       (0u != stats.target_count))
    {
        return 0u;
    }
    art_replan_start_return(context, &stats,
                            initial_pose_x_cm, initial_pose_y_cm, update);
    return (ART_REPLAN_IDLE != art_replan_phase) ? 1u : 0u;
}

void art_replan_tick(const art_replan_context_struct *context,
                     uint8 art_source_enabled,
                     art_replan_update_struct *update)
{
    const map_source_struct *stable_source;
    uint8 center_ready;

    art_replan_update_reset(update);

    if(((ART_REPLAN_IDLE == art_replan_phase) ||
        (ART_REPLAN_PRE_PUSH_CENTER == art_replan_phase)) &&
       (0 != art_source_enabled) &&
       (0 != art_replan_handle_host_completion(update)))
    {
        return;
    }

    if((ART_REPLAN_IDLE == art_replan_phase) &&
       (0 != art_source_enabled) &&
       (0 != executor_art_pre_push_pending()))
    {
        art_replan_begin(ART_REPLAN_PRE_PUSH_CENTER, update);
    }
    else if((ART_REPLAN_IDLE == art_replan_phase) &&
            (0 != art_source_enabled) &&
            (0 != executor_art_sync_pending()))
    {
        art_replan_begin(ART_REPLAN_SEGMENT, update);
    }

    if(ART_REPLAN_IDLE == art_replan_phase)
    {
        return;
    }

    if(ART_REPLAN_PRE_PUSH_CENTER == art_replan_phase)
    {
        if(0u != art_pre_push_box_request_active)
        {
            art_replan_tick_pre_push_box(update);
        }
        else
        {
            center_ready = art_replan_collect_requested_center();
            art_replan_tick_pre_push_center(context, center_ready, update);
        }
        return;
    }

    if((ART_REPLAN_INITIAL_CENTER == art_replan_phase) ||
       (ART_REPLAN_SEGMENT_CENTER == art_replan_phase))
    {
        art_replan_phase_enum center_phase = art_replan_phase;

        if(0 == art_replan_collect_requested_center())
        {
            if((time_ms() - art_wait_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
            {
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
                if(0 == art_requested_center_use_map_cell(context->snapshot))
                {
                    art_replan_center_timeout_error(EXEC_ERROR_ART_CENTER,
                                                    "E:Ctr", update);
                    return;
                }
                art_replan_solve_snapshot_after_center(context, center_phase, update);
#else
                art_replan_center_timeout_error(EXEC_ERROR_ART_CENTER,
                                                "E:Ctr", update);
#endif
                return;
            }
            if(0 != update)
            {
                update->run_state = (ART_REPLAN_INITIAL_CENTER == center_phase) ? "ICtr" : "RCtr";
            }
            return;
        }
        art_replan_solve_snapshot_after_center(context, center_phase, update);
        return;
    }

    if(ART_REPLAN_WAIT_LAUNCH == art_replan_phase)
    {
        if((time_ms() - art_launch_delay_start_ms) >= ART_LAUNCH_DELAY_MS)
        {
            art_replan_begin_wait_center(update);
        }
        else
        {
            if(0 != update)
            {
                update->run_state = "W5S";
            }
        }
        return;
    }

    if(ART_REPLAN_WAIT_CENTER == art_replan_phase)
    {
        art_replan_tick_wait_center(update);
        return;
    }

    if(ART_REPLAN_LAUNCH_MOVE == art_replan_phase)
    {
        art_replan_tick_launch_move(update);
        return;
    }

    if(ART_REPLAN_RETURN_GRID == art_replan_phase)
    {
        if(EXEC_STATE_DONE == executor_get_state())
        {
            art_replan_begin_return_center(ART_REPLAN_RETURN_CENTER, update);
        }
        else if(EXEC_STATE_ERROR == executor_get_state())
        {
            art_replan_cancel();
            if(0 != update)
            {
                update->run_state = "RetErr";
                update->redraw = 1;
            }
        }
        else if(0 != update)
        {
            update->run_state = "RetGrid";
        }
        return;
    }

    if((ART_REPLAN_RETURN_CENTER == art_replan_phase) ||
       (ART_REPLAN_RETURN_VERIFY == art_replan_phase))
    {
        art_replan_tick_return_center(update);
        return;
    }

    if((ART_REPLAN_RETURN_ALIGN_Y == art_replan_phase) ||
       (ART_REPLAN_RETURN_MOVE_X == art_replan_phase))
    {
        art_replan_tick_return_axis(update);
        return;
    }

    if((time_ms() - art_wait_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        // ART 超时不停车，重启等待继续获取最新地图。
        art_replan_begin(art_replan_phase, update);
        if(0 != update)
        {
            update->run_state = (ART_REPLAN_INITIAL == art_replan_phase) ? "WMAP" : "ART Retry";
            update->redraw = 1;
        }
        return;
    }

    stable_source = 0;
    if(0 != art_get_stable_map(&stable_source))
    {
        art_handle_stable_map(context, stable_source, update);
    }
}

void art_replan_get_debug_status(art_replan_debug_status_struct *status)
{
    if(0 == status)
    {
        return;
    }

    status->phase = (uint8)art_replan_phase;
    status->stable_count = art_map_tracker.stable_count;
    status->candidate_valid = art_map_tracker.candidate_valid;
    status->confirmed_box_count = confirmed_box_count;
    status->confirmed_target_count = confirmed_target_count;
    status->confirmed_counts_valid = confirmed_counts_valid;
}
