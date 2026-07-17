#include "zf_common_headfile.h"
#include "art_observation.h"
#include "competition_flow.h"
#include "drive_config.h"
#include "executor.h"
#include "map_utils.h"
#include "openart_uart.h"
#include "solver.h"
#include "subject3.h"
#include "subject3_logic.h"
#include "timebase.h"

typedef enum
{
    SUBJECT3_CENTER_NONE = 0,
    SUBJECT3_CENTER_RESUME_SUBJECT2,
    SUBJECT3_CENTER_RESELECT_BLAST
} subject3_center_next_enum;

static subject3_state_enum subject3_state = SUBJECT3_IDLE;
static subject3_state_enum subject3_recovery_state = SUBJECT3_IDLE;
static solve_result_struct bomb_candidate_result;
static solve_result_struct recovered_task_result;
static solve_result_struct best_bomb_result;
static subject3_candidate_score_struct best_score;
static uint8 best_candidate_valid;
static uint16 select_bomb_cell;
static uint16 select_wall_cell;
static char candidate_rows[MAP_ROWS][MAP_COLS + 1];
static map_source_struct candidate_source;
static char before_rows[MAP_ROWS][MAP_COLS + 1];
static map_source_struct before_source;
static char confirmed_rows[MAP_ROWS][MAP_COLS + 1];
static map_source_struct confirmed_source;
static uint16 selected_bomb_cell = INVALID_STATE;
static uint16 selected_wall_cell = INVALID_STATE;
static map_stability_tracker_struct blast_tracker;
static uint32 blast_wait_start_ms;
static uint8 bomb_execution_interrupted;
static art_center_batch_struct center_batch;
static uint32 center_wait_start_ms;
static subject3_center_next_enum center_next;

static void subject3_update_reset(subject2_update_struct *update)
{
    if(0 != update)
    {
        memset(update, 0, sizeof(*update));
    }
}

static void subject3_bind_rows(map_source_struct *source,
                               char rows[MAP_ROWS][MAP_COLS + 1],
                               const char *name)
{
    uint8 row;

    source->name = name;
    for(row = 0u; row < MAP_ROWS; row++)
    {
        source->rows[row] = rows[row];
    }
}

static void subject3_fail(competition_fatal_reason_enum reason,
                          executor_error_enum error,
                          const char *state_text,
                          subject2_update_struct *update)
{
    subject3_recovery_state = subject3_state;
    /* 保留当前路径，菜单层需先读取错误瞬间的格点和格内偏移再统一清理。 */
    executor_set_error(error);
    competition_flow_latch_fatal(reason);
    subject3_state = SUBJECT3_ERROR;
    if(0 != update)
    {
        update->run_state = (0 != state_text) ? state_text :
                            competition_flow_fatal_text();
        update->redraw = 1u;
    }
}

static uint8 subject3_pose_offset_from_center(
    const map_source_struct *source,
    uint16 col_q,
    uint16 row_q,
    float *pose_x_cm,
    float *pose_y_cm)
{
    uint8 car_row;
    uint8 car_col;

    if((0 == pose_x_cm) || (0 == pose_y_cm) ||
       (0u == map_validate_player_center(source, col_q, row_q,
                                         &car_row, &car_col)))
    {
        return 0u;
    }
    *pose_x_cm = ((float)((int32)col_q -
                          (int32)(car_col * 100u + 50u)) *
                       GRID_SIZE_CM) / 100.0f;
    *pose_y_cm = -((float)((int32)row_q -
                           (int32)(car_row * 100u + 50u)) *
                        GRID_SIZE_CM) / 100.0f;
    return 1u;
}

static void subject3_begin_select(void)
{
    best_candidate_valid = 0u;
    select_bomb_cell = 0u;
    select_wall_cell = 0u;
    clear_result(&best_bomb_result);
    subject3_state = SUBJECT3_SELECT_BLAST;
}

static uint8 subject3_next_pair(const map_source_struct *source,
                                uint16 *bomb_cell,
                                uint16 *wall_cell)
{
    while(select_bomb_cell < MAP_CELLS)
    {
        if('X' != source->rows[map_cell_row(select_bomb_cell)]
                              [map_cell_col(select_bomb_cell)])
        {
            select_bomb_cell++;
            select_wall_cell = 0u;
            continue;
        }
        while(select_wall_cell < MAP_CELLS)
        {
            uint16 candidate = select_wall_cell++;
            uint8 row = map_cell_row(candidate);
            uint8 col = map_cell_col(candidate);

            if((0u < row) && (row < (MAP_ROWS - 1u)) &&
               (0u < col) && (col < (MAP_COLS - 1u)) &&
               ('#' == source->rows[row][col]))
            {
                *bomb_cell = select_bomb_cell;
                *wall_cell = candidate;
                return 1u;
            }
        }
        select_bomb_cell++;
        select_wall_cell = 0u;
    }
    return 0u;
}

static void subject3_start_bomb_executor(
    const subject2_context_struct *context,
    subject2_update_struct *update)
{
    map_scan_stats_struct stats;
    float pose_x_cm;
    float pose_y_cm;

    map_source_snapshot(&before_source, before_rows, context->snapshot);
    memcpy(context->result, &best_bomb_result, sizeof(*context->result));
    map_scan_stats(context->snapshot, &stats);
    if((1u != stats.car_count) || (0u == context->result->waypoint_count))
    {
        subject3_fail(COMPETITION_FATAL_MAP, EXEC_ERROR_MAP,
                      "F:Map", update);
        return;
    }
    subject2_get_pose_offset(&pose_x_cm, &pose_y_cm);
    *context->start_row = stats.car_row;
    *context->start_col = stats.car_col;
    executor_start(context->result->waypoints,
                   context->result->waypoint_count,
                   stats.car_row, stats.car_col,
                   pose_x_cm, pose_y_cm,
                   (RUN_MODE_STEP == context->run_mode) ? 1u : 0u,
                   1u);
    selected_bomb_cell = best_score.bomb_cell;
    selected_wall_cell = best_score.blast_wall_cell;
    bomb_execution_interrupted = 0u;
    art_center_batch_reset(&center_batch);
    center_wait_start_ms = 0u;
    subject3_state = SUBJECT3_EXECUTE_BOMB;
    if(0 != update)
    {
        update->run_state = "S3Bomb";
        update->redraw = 1u;
    }
}

static void subject3_tick_select(const subject2_context_struct *context,
                                 subject2_update_struct *update)
{
    uint16 bomb_cell;
    uint16 wall_cell;
    uint16 push_count;
    uint16 turn_count;
    uint16 final_car_cell;
    subject3_candidate_score_struct score;

    if((0 == context) || (0 == context->snapshot) ||
       (0u == subject3_next_pair(context->snapshot,
                                 &bomb_cell, &wall_cell)))
    {
        if(0u == best_candidate_valid)
        {
            subject2_reject_blast_fallback(update);
            subject3_state = SUBJECT3_RUN_SUBJECT2;
            return;
        }
        subject3_start_bomb_executor(context, update);
        return;
    }

    if((0u != solve_bomb_path(context->snapshot, bomb_cell, wall_cell,
                              &bomb_candidate_result, &push_count,
                              &turn_count, &final_car_cell)) &&
       (0u != subject3_build_expected_map(
                  context->snapshot, bomb_cell, wall_cell, final_car_cell,
                  candidate_rows, &candidate_source)) &&
       (0u != subject2_retry_blocked_plan(&candidate_source,
                                          &recovered_task_result)))
    {
        score.bomb_cell = bomb_cell;
        score.blast_wall_cell = wall_cell;
        score.final_car_cell = final_car_cell;
        score.action_count = (uint16)(bomb_candidate_result.action_count +
                                      recovered_task_result.action_count);
        score.push_count = push_count;
        score.turn_count = turn_count;
        if(0u != subject3_candidate_is_better(
                      &score, &best_score, best_candidate_valid))
        {
            best_score = score;
            memcpy(&best_bomb_result, &bomb_candidate_result,
                   sizeof(best_bomb_result));
            best_candidate_valid = 1u;
        }
    }
    if(0 != update)
    {
        update->run_state = "S3Plan";
    }
}

static void subject3_begin_confirm(uint8 interrupted,
                                   subject2_update_struct *update)
{
    executor_stop();
    bomb_execution_interrupted = interrupted;
    map_stability_tracker_reset(&blast_tracker,
                                openart_uart_get_frame_count());
    blast_wait_start_ms = time_ms();
    subject3_state = SUBJECT3_CONFIRM_BLAST;
    if(0 != update)
    {
        update->run_state = "S3Wait";
        update->redraw = 1u;
    }
}

static uint8 subject3_finish_confirmed_map(
    const subject2_context_struct *context,
    float pose_x_cm,
    float pose_y_cm,
    subject2_update_struct *update)
{
    if(SUBJECT3_CENTER_RESUME_SUBJECT2 == center_next)
    {
        if(0u == subject2_resume_after_blast(context, &confirmed_source,
                                             pose_x_cm, pose_y_cm, update))
        {
            return 0u;
        }
        subject3_state = SUBJECT3_RUN_SUBJECT2;
        return 1u;
    }
    if(SUBJECT3_CENTER_RESELECT_BLAST == center_next)
    {
        if(0u == subject2_refresh_blocked_map(context, &confirmed_source,
                                              pose_x_cm, pose_y_cm))
        {
            return 0u;
        }
        subject3_begin_select();
        if(0 != update)
        {
            update->run_state = "S3Plan";
            update->redraw = 1u;
        }
        return 1u;
    }
    return 0u;
}

static void subject3_begin_center(subject3_center_next_enum next,
                                  subject2_update_struct *update)
{
    center_next = next;
    art_center_batch_reset(&center_batch);
    executor_reset_art_player_center_samples();
    openart_request_player_center();
    center_wait_start_ms = time_ms();
    subject3_state = SUBJECT3_CENTER;
    if(0 != update)
    {
        update->run_state = "S3Ctr";
        update->redraw = 1u;
    }
}

static void subject3_accept_stable_map(
    const subject2_context_struct *context,
    subject3_center_next_enum next,
    subject2_update_struct *update)
{
    uint16 col_q;
    uint16 row_q;
    uint8 valid;
    uint32 center_frame;
    float pose_x_cm;
    float pose_y_cm;

    map_source_snapshot(&confirmed_source, confirmed_rows,
                        &candidate_source);
    center_frame = openart_get_player_center(&col_q, &row_q, &valid);
    center_next = next;
    if((center_frame == blast_tracker.last_frame) && (0u != valid) &&
       (0u != subject3_pose_offset_from_center(
                  &confirmed_source, col_q, row_q,
                  &pose_x_cm, &pose_y_cm)))
    {
        if(0u != subject3_finish_confirmed_map(
                      context, pose_x_cm, pose_y_cm, update))
        {
            return;
        }
        subject3_fail(COMPETITION_FATAL_MAP, EXEC_ERROR_ART_SYNC,
                      "F:Map", update);
        return;
    }
    subject3_begin_center(next, update);
}

static void subject3_tick_confirm(const subject2_context_struct *context,
                                  subject2_update_struct *update)
{
    const map_source_struct *source = openart_map_get();
    map_stability_result_enum result;

    result = map_stability_tracker_push(
        &blast_tracker, openart_uart_get_frame_count(), source,
        SUBJECT3_BLAST_STABLE_FRAMES);
    if(MAP_STABILITY_READY == result)
    {
        subject3_bind_rows(&candidate_source, blast_tracker.candidate_rows,
                           "S3 stable");
        if(0u != subject3_blast_map_matches(
                      &before_source, &candidate_source,
                      selected_bomb_cell, selected_wall_cell))
        {
            subject3_accept_stable_map(
                context, SUBJECT3_CENTER_RESUME_SUBJECT2, update);
            return;
        }
        if((0u != bomb_execution_interrupted) &&
           (0u != subject3_unexploded_map_matches(
                      &before_source, &candidate_source,
                      selected_bomb_cell)))
        {
            subject3_accept_stable_map(
                context, SUBJECT3_CENTER_RESELECT_BLAST, update);
            return;
        }
        map_stability_tracker_reset_candidate(&blast_tracker);
    }
    if((time_ms() - blast_wait_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        subject3_fail(COMPETITION_FATAL_ART1, EXEC_ERROR_ART_SYNC,
                      "F:ART1", update);
        return;
    }
    if(0 != update)
    {
        update->run_state = "S3Wait";
    }
}

static void subject3_tick_center(const subject2_context_struct *context,
                                 subject2_update_struct *update)
{
    const map_source_struct *paired_source;
    uint16 col_q;
    uint16 row_q;
    float pose_x_cm;
    float pose_y_cm;

    if(0u == art_center_batch_collect(&center_batch))
    {
        if((time_ms() - center_wait_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
        {
            subject3_fail(COMPETITION_FATAL_ART1, EXEC_ERROR_ART_CENTER,
                          "F:ART1", update);
        }
        else if(0 != update)
        {
            update->run_state = "S3Ctr";
        }
        return;
    }

    paired_source = openart_get_requested_center_map();
    if((0 == paired_source) ||
       (0u == map_rows_equal(confirmed_rows, paired_source)))
    {
        map_stability_tracker_reset(&blast_tracker,
                                    openart_uart_get_frame_count());
        blast_wait_start_ms = time_ms();
        subject3_state = SUBJECT3_CONFIRM_BLAST;
        if(0 != update)
        {
            update->run_state = "S3Wait";
            update->redraw = 1u;
        }
        return;
    }
    if((0u == art_center_batch_get_median(&center_batch, &col_q, &row_q)) ||
       (0u == subject3_pose_offset_from_center(
                  paired_source, col_q, row_q,
                  &pose_x_cm, &pose_y_cm)) ||
       (0u == subject3_finish_confirmed_map(
                  context, pose_x_cm, pose_y_cm, update)))
    {
        subject3_fail(COMPETITION_FATAL_ART1, EXEC_ERROR_ART_CENTER,
                      "F:ART1", update);
    }
}

static void subject3_tick_executor_center(subject2_update_struct *update)
{
    const map_source_struct *source;
    uint16 col_q;
    uint16 row_q;
    uint8 car_row;
    uint8 car_col;
    executor_art_center_result_enum center_result;

    if(0u == center_wait_start_ms)
    {
        art_center_batch_reset(&center_batch);
        executor_reset_art_player_center_samples();
        openart_request_player_center();
        center_wait_start_ms = time_ms();
    }
    if(0u == art_center_batch_collect(&center_batch))
    {
        if((time_ms() - center_wait_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
        {
            subject3_fail(COMPETITION_FATAL_ART1, EXEC_ERROR_ART_CENTER,
                          "F:ART1", update);
        }
        return;
    }

    source = openart_get_requested_center_map();
    if((0u == art_center_batch_get_median(&center_batch, &col_q, &row_q)) ||
       (0u == subject3_unexploded_map_matches(
                  &before_source, source, selected_bomb_cell)) ||
       ('X' != source->rows[map_cell_row(selected_bomb_cell)]
                           [map_cell_col(selected_bomb_cell)]) ||
       (0u == map_validate_player_center(source, col_q, row_q,
                                         &car_row, &car_col)))
    {
        subject3_fail(COMPETITION_FATAL_ART1, EXEC_ERROR_ART_CENTER,
                      "F:ART1", update);
        return;
    }
    (void)art_center_batch_apply_to_executor(&center_batch);
    center_result = executor_commit_art_player_center(car_row, car_col);
    if((EXEC_ART_CENTER_APPLIED != center_result) &&
       (EXEC_ART_CENTER_IGNORED != center_result))
    {
        subject3_fail(COMPETITION_FATAL_ART1, EXEC_ERROR_ART_CENTER,
                      "F:ART1", update);
        return;
    }
    if(0u == executor_continue_after_pre_push_center())
    {
        subject3_fail(COMPETITION_FATAL_DRIVE, EXEC_ERROR_ART_CENTER,
                      "F:Drive", update);
        return;
    }
    art_center_batch_reset(&center_batch);
    center_wait_start_ms = 0u;
}

static void subject3_tick_execute(subject2_update_struct *update)
{
    executor_state_enum state = executor_get_state();

    if(0u != executor_art_pre_push_pending())
    {
        subject3_tick_executor_center(update);
        return;
    }
    if(EXEC_STATE_DONE == state)
    {
        subject3_begin_confirm(0u, update);
        return;
    }
    if(EXEC_STATE_ERROR == state)
    {
        subject3_begin_confirm(1u, update);
        return;
    }
    if(0 != update)
    {
        update->run_state = "S3Bomb";
    }
}

void subject3_begin(const subject2_context_struct *context,
                    float initial_pose_x_cm,
                    float initial_pose_y_cm,
                    subject2_update_struct *update)
{
    subject2_context_struct subject2_context;

    subject3_cancel();
    subject3_update_reset(update);
    if(0 == context)
    {
        subject3_fail(COMPETITION_FATAL_MAP, EXEC_ERROR_MAP,
                      "F:Map", update);
        return;
    }
    subject2_context = *context;
    subject2_context.allow_blast_fallback = 1u;
    subject2_begin(&subject2_context, initial_pose_x_cm,
                   initial_pose_y_cm, update);
    subject3_state = (SUBJECT2_ERROR == subject2_get_state()) ?
                     SUBJECT3_ERROR : SUBJECT3_RUN_SUBJECT2;
}

void subject3_tick(const subject2_context_struct *context,
                   subject2_update_struct *update)
{
    subject2_context_struct subject2_context;

    subject3_update_reset(update);
    if((0 == context) || (SUBJECT3_IDLE == subject3_state) ||
       (SUBJECT3_ERROR == subject3_state))
    {
        return;
    }
    subject2_context = *context;
    subject2_context.allow_blast_fallback = 1u;
    switch(subject3_state)
    {
        case SUBJECT3_RUN_SUBJECT2:
            subject2_tick(&subject2_context, update);
            if(SUBJECT2_BLOCK_NONE != subject2_get_block_reason())
            {
                subject3_begin_select();
                if(0 != update)
                {
                    update->run_state = "S3Plan";
                    update->redraw = 1u;
                }
            }
            break;
        case SUBJECT3_SELECT_BLAST:
            subject3_tick_select(&subject2_context, update);
            break;
        case SUBJECT3_EXECUTE_BOMB:
            subject3_tick_execute(update);
            break;
        case SUBJECT3_CONFIRM_BLAST:
            subject3_tick_confirm(&subject2_context, update);
            break;
        case SUBJECT3_CENTER:
            subject3_tick_center(&subject2_context, update);
            break;
        default:
            break;
    }
}

uint8 subject3_manual_recover(subject2_update_struct *update)
{
    subject3_update_reset(update);
    if(SUBJECT3_ERROR != subject3_state)
    {
        return subject2_manual_recover(update);
    }
    if(SUBJECT3_RUN_SUBJECT2 == subject3_recovery_state)
    {
        if(0u == subject2_manual_recover(update))
        {
            return 0u;
        }
        subject3_state = SUBJECT3_RUN_SUBJECT2;
        return 1u;
    }
    if(SUBJECT3_SELECT_BLAST == subject3_recovery_state)
    {
        subject3_begin_select();
        if(0 != update) update->run_state = "S3Plan";
        return 1u;
    }
    if((SUBJECT3_EXECUTE_BOMB == subject3_recovery_state) ||
       (SUBJECT3_CONFIRM_BLAST == subject3_recovery_state))
    {
        subject3_begin_confirm(1u, update);
        return 1u;
    }
    if(SUBJECT3_CENTER == subject3_recovery_state)
    {
        subject3_begin_center(center_next, update);
        return 1u;
    }
    return 0u;
}

void subject3_cancel(void)
{
    subject2_cancel();
    if(EXEC_STATE_IDLE != executor_get_state())
    {
        executor_stop();
    }
    subject3_state = SUBJECT3_IDLE;
    subject3_recovery_state = SUBJECT3_IDLE;
    best_candidate_valid = 0u;
    selected_bomb_cell = INVALID_STATE;
    selected_wall_cell = INVALID_STATE;
    blast_wait_start_ms = 0u;
    bomb_execution_interrupted = 0u;
    art_center_batch_reset(&center_batch);
    center_wait_start_ms = 0u;
    center_next = SUBJECT3_CENTER_NONE;
}

subject3_state_enum subject3_get_state(void)
{
    return subject3_state;
}
