#include "zf_common_headfile.h"
#include "art_replan.h"
#include "drive_config.h"
#include "executor.h"
#include "map_utils.h"
#include "openart_uart.h"
#include "solver.h"
#include "timebase.h"

typedef enum
{
    ART_REPLAN_IDLE = 0,   /**< 未等待 ART 稳定帧，tick 只处理 executor 段末请求。 */
    ART_REPLAN_INITIAL,    /**< 初始 ART 求解阶段，成功后必须等待 K3 人工确认发车。 */
    ART_REPLAN_SEGMENT,    /**< 段末低频重定位阶段，成功后直接续跑 executor。 */
} art_replan_phase_enum;

static art_replan_phase_enum art_replan_phase = ART_REPLAN_IDLE; // 主循环写入和读取；决定稳定帧成功后的启动策略。
static char art_candidate_rows[MAP_ROWS][MAP_COLS + 1];          // 当前候选稳定帧快照；不增加 UART 环形缓冲容量。
static uint8 art_candidate_valid = 0;                            // 1 表示 `art_candidate_rows` 已保存一帧可比较地图。
static uint8 art_stable_count = 0;                               // 连续一致的新完整帧计数，达到 EXEC_ART_STABLE_FRAMES 才求解。
static uint32 art_last_seen_frame = 0;                            // 已处理到的 OpenART 帧号；用于丢弃等待前的旧帧。
static uint32 art_wait_start_ms = 0;                              // 本轮 ART 等待起点，单位 ms；用于统一初始/段末超时。
static executor_error_enum art_timeout_error = EXEC_ERROR_ART_TIMEOUT; // 等待过程中若先遇到坏图，超时后映射为更具体错误码。
static uint8 art_launch_pending = 0;                              // 初始 ART 求解成功后置 1，必须 K3 确认才允许 executor_start。

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
    // 进入等待态时丢掉半帧和旧缓冲，只把之后完成的 OpenART 帧作为候选。
    // 这样可以避免上一页残留数据触发“刚进入执行页就用旧地图重算”。
    openart_uart_discard_pending();
    art_candidate_valid = 0;
    art_stable_count = 0;
    art_last_seen_frame = openart_uart_get_frame_count();
}

static void art_replan_begin(art_replan_phase_enum phase, art_replan_update_struct *update)
{
    art_replan_phase = phase;
    art_replan_wait_fresh_frame();
    art_wait_start_ms = time_ms();
    art_timeout_error = EXEC_ERROR_ART_TIMEOUT;
    if(0 != update)
    {
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

    // OpenART 屏幕再识别可能在页面刷新瞬间抖动；只有连续完整帧完全一致，
    // 才把地图交给 BFS。稳定计数按“完成帧”推进，而不是按主循环轮询次数推进。
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
    // BFS 只能处理“一个车、箱子数等于目标数、数量不超过数组容量”的快照。
    // 不满足时继续等新稳定帧，而不是在可疑识别结果上规划车辆动作。
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

    art_replan_save_snapshot(context, source);
    map_scan_stats(context->snapshot, &stats);

    if((1u == stats.car_count) && (0 != art_stats_done(&stats)))
    {
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
        printf("ART_DONE frame=%lu C=%d,%d\r\n",
            (unsigned long)openart_uart_get_frame_count(),
            stats.car_row,
            stats.car_col);
        return;
    }

    if(0 == art_stats_valid_for_solve(&stats))
    {
        art_timeout_error = EXEC_ERROR_ART_SYNC;
        // 错帧本身不能作为下一轮稳定候选，否则同一个坏识别会被反复求解失败。
        art_replan_restart_stability();
        if(0 != update)
        {
            update->run_state = "Bad ART";
            update->redraw = 1;
        }
        printf("ART_SYNC_BAD frame=%lu C=%d B=%d T=%d\r\n",
            (unsigned long)openart_uart_get_frame_count(),
            stats.car_count,
            stats.box_count,
            stats.target_count);
        return;
    }

    // 求解可能占用较长主循环时间；求解前后清空未解析字节，保证下一次同步等待的是
    // 求解完成后的新屏幕状态，而不是求解期间积压的旧画面。
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
            // 初次 ART 求解成功只进入待发车状态，必须由 K3 确认，避免识别到误帧后自动启动。
            art_replan_wait_launch(context, &stats, update);
        }
        else
        {
            // 分段重解算来自执行器的同步请求，车辆已处在执行流程中，稳定帧通过后直接续跑。
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
        }
    }
    else
    {
        *context->elapsed_ms = time_ms() - start_ms;
        art_replan_wait_fresh_frame();
        art_timeout_error = EXEC_ERROR_ART_PLAN;
        if(0 != update)
        {
            update->playback = ART_REPLAN_PLAYBACK_FAIL;
            update->run_state = "Plan Fail";
            update->redraw = 1;
        }
        printf("ART_REPLAN_FAIL phase=%d %s time=%lu\r\n",
            phase,
            context->result->message,
            (unsigned long)*context->elapsed_ms);
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
}

void art_replan_begin_initial(art_replan_update_struct *update)
{
    art_replan_update_reset(update);
    art_replan_begin(ART_REPLAN_INITIAL, update);
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
        return;
    }

    if((time_ms() - art_wait_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        // 超时错误码保留最近一次失败原因：一直没有稳定帧是 TIMEOUT，稳定坏帧则升级为 SYNC/PLAN。
        art_replan_cancel();
        executor_set_error(art_timeout_error);
        if(0 != update)
        {
            update->run_state = "ART Timeout";
            update->redraw = 1;
        }
        printf("ART_SYNC_TIMEOUT err=%d\r\n", art_timeout_error);
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
