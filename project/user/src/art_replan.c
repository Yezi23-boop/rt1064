#include "zf_common_headfile.h"
#include "art_replan.h"
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
    ART_REPLAN_INITIAL,          /**< 首次建图/求解：拿到稳定 ART 地图后直接启动执行器。 */
    ART_REPLAN_SEGMENT,          /**< waypoint 段末同步：每到一个点都按 ART 最新地图做确认。 */
} art_replan_phase_enum;

static art_replan_phase_enum art_replan_phase = ART_REPLAN_IDLE; // 主循环写入和读取；决定稳定帧成功后的启动策略。
static char art_candidate_rows[MAP_ROWS][MAP_COLS + 1];          // 当前候选稳定帧快照；不增加 UART 环形缓冲容量。
static uint8 art_candidate_valid = 0;                            // 1 表示 `art_candidate_rows` 已保存一帧可比较地图。
static uint8 art_stable_count = 0;                               // 连续一致的新完整帧计数，达到 EXEC_ART_STABLE_FRAMES 才求解。
static uint32 art_last_seen_frame = 0;                            // 已处理到的 OpenART 帧号；用于丢弃等待前的旧帧。
static uint32 art_wait_start_ms = 0;                              // 本轮 ART 等待起点，单位 ms；用于统一初始/段末超时。
static uint8 art_launch_pending = 0;                              // 初始 ART 求解成功后置 1，必须 K3 确认才允许 executor_start。
static uint32 art_launch_delay_start_ms = 0;                      // 5 秒延迟起点。
static uint8 confirmed_box_count = 0;                             // 上一次被 ART 认定为“同步完成”的箱子数基线。
static uint8 confirmed_target_count = 0;                          // 上一次被 ART 认定为“同步完成”的目标数基线。
static uint8 confirmed_counts_valid = 0;                           // 1 表示上面的 B/T 基线有效，可用于判断是否发生了消除。
static uint32 art_last_player_center_sample = 0;                   // 本轮段末同步已处理到的视觉中心样本序号。
static uint8 art_center_sampling_was_active = 0;                   // 1 表示上一轮已经处在段末中心采样窗口。

static void art_replan_update_reset(art_replan_update_struct *update)
{
    if(0 == update)
    {
        return;
    }
    update->redraw = 0;
    update->enter_execute = 0;
    update->reset_playback_step = 0;
    update->playback = ART_REPLAN_PLAYBACK_KEEP;
    update->run_state = 0;
}

static void art_replan_wait_fresh_frame(void)
{
    // 切入等待态时，先丢掉半帧和旧缓冲。
    // 这样后续只有“等待之后新产生的完整帧”才会参与稳定性判断。
    openart_uart_discard_pending();
    art_candidate_valid = 0;
    art_stable_count = 0;
    art_last_seen_frame = openart_uart_get_frame_count();
    art_last_player_center_sample = openart_get_player_center(0, 0, 0);
}

static void art_replan_begin(art_replan_phase_enum phase, art_replan_update_struct *update)
{
    art_replan_phase = phase;
    art_replan_wait_fresh_frame();
    art_wait_start_ms = time_ms();
    if(0 != update)
    {
        // 初始阶段强调“等 ART 建图”，段末阶段强调“等 ART 同步”。
        update->run_state = (ART_REPLAN_INITIAL == phase) ? "Wait ART" : "ART Sync";
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

    if((0 == source) || (0 == frame))
    {
        return 0;
    }

    if(frame == art_last_seen_frame)
    {
        return 0;
    }
    art_last_seen_frame = frame;

    // OpenART 在页面刷新瞬间会抖动，所以这里只接受“连续完整帧一致”的地图。
    // 稳定计数按帧推进，不按主循环次数推进。
    if((0 != art_candidate_valid) && (0 != map_rows_equal(art_candidate_rows, source)))
    {
        if(art_stable_count < EXEC_ART_STABLE_FRAMES)
        {
            art_stable_count++;
        }
    }
    else
    {
        map_copy_rows(art_candidate_rows, source);
        art_candidate_valid = 1;
        art_stable_count = 1;
    }

    if(art_stable_count >= EXEC_ART_STABLE_FRAMES)
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

static uint8 art_stats_valid_for_solve(const map_scan_stats_struct *stats)
{
    // BFS 只接受可解的虚拟地图快照：必须只有一个车，且箱子数等于目标数。
    // 不满足时继续等新稳定帧，不拿可疑识别结果硬算路径。
    if(1u != stats->car_count)
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
    // 记录当前稳定帧的 B/T 数量，作为下一次段末同步的对照基线。
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
                                      art_replan_update_struct *update)
{
    uint8 single_step = (RUN_MODE_STEP == context->run_mode) ? 1u : 0u;

    *context->start_row = stats->car_row;
    *context->start_col = stats->car_col;
    executor_start(context->result->waypoints, context->result->waypoint_count,
                   *context->start_row, *context->start_col, single_step, 1u);
    if(0 != update)
    {
        update->run_state = (0 != single_step) ? "Paused" : "Running";
    }
}

static void art_replan_collect_player_center(void)
{
    uint16 center_col_q;
    uint16 center_row_q;
    uint8 center_valid;
    uint32 sample_count;
    uint8 sampling_active = executor_art_center_sampling_active();

    if(0 == sampling_active)
    {
        art_center_sampling_was_active = 0;
        return;
    }
    if(0 == art_center_sampling_was_active)
    {
        art_last_player_center_sample = openart_get_player_center(0, 0, 0);
        art_center_sampling_was_active = 1;
        return;
    }
    sample_count = openart_get_player_center(&center_col_q, &center_row_q, &center_valid);
    if((sample_count == art_last_player_center_sample) || (0 == center_valid))
    {
        return;
    }
    art_last_player_center_sample = sample_count;
    (void)executor_apply_art_player_center(center_col_q, center_row_q, sample_count);
}

static void art_replan_wait_launch(const art_replan_context_struct *context,
                                   const map_scan_stats_struct *stats,
                                   art_replan_update_struct *update)
{
    *context->start_row = stats->car_row;
    *context->start_col = stats->car_col;
    art_replan_cancel();
    art_launch_pending = 1;
    if(0 != update)
    {
        update->run_state = "Ready K3";
        update->redraw = 1;
    }
}

static void art_handle_stable_map(const art_replan_context_struct *context,
                                  const map_source_struct *source,
                                  art_replan_update_struct *update)
{
    map_scan_stats_struct stats;
    uint32 start_ms;
    art_replan_phase_enum phase = art_replan_phase;
    executor_art_center_result_enum center_result = EXEC_ART_CENTER_NONE;

    art_replan_save_snapshot(context, source);
    map_scan_stats(context->snapshot, &stats);

    if((1u == stats.car_count) && (0 != art_stats_done(&stats)))
    {
        *context->start_row = stats.car_row;
        *context->start_col = stats.car_col;
        art_update_confirmed_counts(&stats);
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
        printf("ART_DONE frame=%lu C=%d,%d\r\n",
            (unsigned long)openart_uart_get_frame_count(),
            stats.car_row,
            stats.car_col);
        return;
    }

    if(0 == art_stats_valid_for_solve(&stats))
    {
        // 地图还没稳定到可解，重新进入稳定性等待，不在这张图上继续推进。
        art_replan_restart_stability();
        if(0 != update)
        {
            update->run_state = "ART Retry";
            update->redraw = 1;
        }
        return;
    }

    if(ART_REPLAN_INITIAL == phase)
    {
        // 初次求解只建立基线，不拿 B/T 变化做成败判断。
        art_update_confirmed_counts(&stats);
    }
    else if(ART_REPLAN_SEGMENT == phase)
    {
        char sync_action = executor_get_art_sync_action();

        center_result = executor_commit_art_player_center();
        printf("ART_CENTER result=%d action=%c\r\n", center_result, sync_action);

        if(0 == art_action_is_push(sync_action))
        {
            art_update_confirmed_counts(&stats);
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
            art_update_confirmed_counts(&stats);
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
            printf("ART_PUSH_NOT_CONFIRMED action=%c frame=%lu B=%d/%d T=%d/%d C=%d,%d\r\n",
                sync_action,
                (unsigned long)openart_uart_get_frame_count(),
                stats.box_count,
                confirmed_box_count,
                stats.target_count,
                confirmed_target_count,
                stats.car_row,
                stats.car_col);
        }
    }

    // 求解可能占用较长主循环时间；求解前后清空未解析字节，避免旧画面积压进下一轮同步。
    openart_uart_discard_pending();
    start_ms = time_ms();
    if(0 != solve_map(context->snapshot, context->result))
    {
        *context->elapsed_ms = time_ms() - start_ms;
        openart_uart_discard_pending();
        if(0 != update)
        {
            update->reset_playback_step = 1;
            update->playback = ART_REPLAN_PLAYBACK_PAUSED;
        }
        if(ART_REPLAN_INITIAL == phase)
        {
            // 初始阶段求解成功后直接进入执行。
            art_replan_start_executor(context, &stats, update);
            art_replan_cancel();
        }
        else
        {
            // 段末重解算来自执行器的同步请求，稳定帧通过后直接续跑，不再额外停一层。
            art_replan_start_executor(context, &stats, update);
            art_replan_cancel();
            if(0 != update)
            {
                update->redraw = 1;
            }
        }
        printf("ART_REPLAN_OK phase=%d tasks=%d actions=%d waypoints=%d time=%lu\r\n",
            phase,
            context->result->task_count,
            context->result->action_count,
            context->result->waypoint_count,
            (unsigned long)*context->elapsed_ms);
        if((ART_REPLAN_INITIAL == phase) && (0 != update))
        {
            update->enter_execute = 1;
            update->redraw = 1;
        }
    }
    else
    {
        *context->elapsed_ms = time_ms() - start_ms;
        art_replan_wait_fresh_frame();
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
    art_candidate_valid = 0;
    art_stable_count = 0;
    art_last_seen_frame = 0;
    art_wait_start_ms = 0;
    art_launch_pending = 0;
    art_launch_delay_start_ms = 0;
}

void art_replan_begin_initial(art_replan_update_struct *update)
{
    art_replan_update_reset(update);
    art_replan_phase = ART_REPLAN_WAIT_LAUNCH;
    art_launch_delay_start_ms = time_ms();
    confirmed_box_count = 0;
    confirmed_target_count = 0;
    confirmed_counts_valid = 0;
    if(0 != update)
    {
        update->run_state = "Wait 5s";
        update->redraw = 1;
    }
}

void art_replan_tick(const art_replan_context_struct *context,
                     uint8 art_source_enabled,
                     art_replan_update_struct *update)
{
    const map_source_struct *stable_source;

    art_replan_update_reset(update);

    if((ART_REPLAN_IDLE == art_replan_phase) &&
       (0 != art_source_enabled) &&
       (0 != executor_art_sync_pending()))
    {
        art_replan_begin(ART_REPLAN_SEGMENT, update);
    }

    if(ART_REPLAN_IDLE == art_replan_phase)
    {
        art_replan_collect_player_center();
        return;
    }

    art_replan_collect_player_center();

    if(ART_REPLAN_WAIT_LAUNCH == art_replan_phase)
    {
        if((time_ms() - art_launch_delay_start_ms) >= ART_LAUNCH_DELAY_MS)
        {
            art_replan_begin(ART_REPLAN_INITIAL, update);
        }
        else
        {
            if(0 != update)
            {
                update->run_state = "Wait 5s";
            }
        }
        return;
    }

    if((time_ms() - art_wait_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        // ART 超时不停车，重启等待继续获取最新地图。
        art_replan_begin(art_replan_phase, update);
        if(0 != update)
        {
            update->run_state = "ART Retry";
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

void art_replan_confirm_launch(const art_replan_context_struct *context,
                               art_replan_update_struct *update)
{
    map_scan_stats_struct stats;

    art_replan_update_reset(update);
    if(0 == art_launch_pending)
    {
        return;
    }

    map_scan_stats(context->snapshot, &stats);
    art_launch_pending = 0;
    // K3 确认只消费一次待启动标志；如果用户随后再按 K3，就交回菜单的暂停/继续逻辑处理。
    art_replan_start_executor(context, &stats, update);
    if(0 != update)
    {
        update->redraw = 1;
    }
}

uint8 art_replan_launch_pending(void)
{
    return art_launch_pending;
}

void art_replan_get_debug_status(art_replan_debug_status_struct *status)
{
    if(0 == status)
    {
        return;
    }

    status->phase = (uint8)art_replan_phase;
    status->stable_count = art_stable_count;
    status->candidate_valid = art_candidate_valid;
    status->confirmed_box_count = confirmed_box_count;
    status->confirmed_target_count = confirmed_target_count;
    status->confirmed_counts_valid = confirmed_counts_valid;
    status->launch_pending = art_launch_pending;
}
