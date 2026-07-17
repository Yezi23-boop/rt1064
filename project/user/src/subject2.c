#include "zf_common_headfile.h"
#include "art_observation.h"
#include "competition_flow.h"
#include "drive_config.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "executor.h"
#include "map_utils.h"
#include "motion_math.h"
#include "openart_uart.h"
#include "solver.h"
#include "subject2.h"
#include "timebase.h"
#include "vision_uart.h"

typedef enum
{
    SUBJECT2_SCAN_SYNC_WAIT_MAP = 0,
    SUBJECT2_SCAN_SYNC_WAIT_CENTER
} subject2_scan_sync_phase_enum;

typedef enum
{
    SUBJECT2_RECOVERY_NONE = 0,
    SUBJECT2_RECOVERY_MAP,
    SUBJECT2_RECOVERY_CENTER,
    SUBJECT2_RECOVERY_OBSERVE,
    SUBJECT2_RECOVERY_YAW,
    SUBJECT2_RECOVERY_TRACK,
    SUBJECT2_RECOVERY_PLAN,
    SUBJECT2_RECOVERY_CLASS,
    SUBJECT2_RECOVERY_MOTION
} subject2_recovery_reason_enum;

typedef enum
{
    SUBJECT2_CONFIRM_STRICT = 0,
    SUBJECT2_CONFIRM_HOST_CHANGE,
    SUBJECT2_CONFIRM_TASK_END
} subject2_confirm_mode_enum;

static subject2_state_enum subject2_state = SUBJECT2_IDLE;
static subject2_object_struct box_objects[MAX_BOXES];
static subject2_object_struct target_objects[MAX_BOXES];
static uint8 box_object_count;
static uint8 target_object_count;
static subject2_observation_plan_struct current_observation;
static subject2_classifier_struct classifier;
static uint16 current_vision_request_id;
static uint32 classify_start_ms;
static art_center_batch_struct center_batch;
static uint32 pre_push_center_start_ms;
static uint8 pre_push_box_request_active;
static uint8 pre_push_box_preparation_started;
static art_box_observation_session_struct pre_push_box_session;
static uint8 pre_push_box_retry_count;
static uint8 pre_push_box_retry_moving;
static uint8 pre_push_box_retry_settling;
static uint32 pre_push_box_retry_settle_start_ms;
static float current_pose_offset_x_cm;
static float current_pose_offset_y_cm;
static uint8 navigation_start_row;
static uint8 navigation_start_col;
static uint8 validation_retry_count;
static uint8 active_class = SUBJECT2_INVALID_CLASS;
static uint8 active_box_valid;
static uint16 active_box_cell = INVALID_STATE;
static uint16 active_target_cell = INVALID_STATE;
static uint8 last_recognition_valid;
static uint8 last_recognition_is_target;
static uint8 last_recognition_class = SUBJECT2_INVALID_CLASS;
static solve_result_struct push_candidate_result;
static uint16 task_start_boxes[MAX_BOXES];
static uint8 task_start_box_count;
static uint8 task_start_target_count;
static uint8 host_completion_waiting_boundary;
static uint32 host_completion_start_ms;
static uint8 host_path_preserved;
static uint32 host_monitor_last_frame;
static char transit_overlap_rows[MAP_ROWS][MAP_COLS + 1];
static map_source_struct transit_overlap_source;
static uint16 transit_overlap_cell = INVALID_STATE;
static uint8 transit_overlap_valid;
static uint8 retry_active_only;
static uint8 replan_center_for_return;
static uint32 confirm_wait_start_ms;
static map_stability_tracker_struct confirm_tracker;
static subject2_confirm_mode_enum confirm_mode;
static uint8 confirm_retry_count;
static uint8 confirm_map_seen;
static uint32 confirm_start_frame;
static uint32 center_request_start_ms;
static uint32 center_adjust_start_ms;
static uint32 scan_fast_center_after_frame;
static uint8 scan_center_request_started;
static uint32 turn_start_ms;
static uint32 turn_stable_start_ms;
static uint8 turn_stable_active;
static uint32 vision_mode_start_ms;
static uint32 vision_mode_last_send_ms;
static vision_mode_enum active_vision_mode;
static uint8 view_backoff_active;
static uint8 view_backoff_recognition_accepted;
static float view_backoff_origin_x_cm;
static float view_backoff_origin_y_cm;
static uint32 view_motion_start_ms;
static float launch_yaw_deg;
static uint8 post_observe_art_yaw_bias_valid;
static float post_observe_art_yaw_bias_deg;
static uint8 post_observe_yaw_done;
static uint32 post_observe_yaw_start_ms;
static subject2_scan_sync_phase_enum scan_sync_phase;
static uint8 scan_plan_snapshot_pending;
static char scan_sync_rows[MAP_ROWS][MAP_COLS + 1];
static map_source_struct scan_sync_source;
static uint8 scan_sync_source_valid;
static uint32 center_request_frame;
static uint32 pre_push_request_frame;
static subject2_recovery_reason_enum recovery_reason;
static subject2_state_enum recovery_origin_state;
static uint8 recovery_count;
static uint16 recovery_box_cell;
static uint16 recovery_target_cell;
static uint8 recovery_observation_bit;
static subject2_block_reason_enum blocked_reason;
static subject2_state_enum blocked_resume_state;

static void subject2_accept_confirmed_map(const subject2_context_struct *context,
                                          const map_source_struct *source,
                                          subject2_update_struct *update);
static void subject2_begin_confirm_map(subject2_confirm_mode_enum mode,
                                       subject2_update_struct *update);
static const map_source_struct *subject2_effective_map(
    const subject2_context_struct *context,
    const map_source_struct *source,
    uint8 allow_infer);
static uint8 subject2_try_fast_scan_center(
    const subject2_context_struct *context,
    uint16 center_col_q,
    uint16 center_row_q,
    uint8 center_valid,
    uint8 is_box_scan,
    subject2_update_struct *update);
static void subject2_begin_recovery(subject2_recovery_reason_enum reason,
                                    competition_fatal_reason_enum fatal_reason,
                                    executor_error_enum final_error,
                                    subject2_update_struct *update);

static void subject2_update_reset(subject2_update_struct *update)
{
    if(0 != update)
    {
        memset(update, 0, sizeof(*update));
    }
}

static float subject2_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8 subject2_get_center_median(uint16 *col_q, uint16 *row_q)
{
    return art_center_batch_get_median(&center_batch, col_q, row_q);
}
static uint8 cells_equal(const uint16 *left, const uint16 *right, uint8 count)
{
    uint8 index;

    for(index = 0u; index < count; index++)
    {
        if(left[index] != right[index])
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8 subject2_map_objects_unchanged(const map_source_struct *source)
{
    uint16 boxes[MAX_BOXES];
    uint16 targets[MAX_BOXES];
    uint16 expected_boxes[MAX_BOXES];
    uint16 expected_targets[MAX_BOXES];
    uint8 box_count = 0u;
    uint8 target_count = 0u;
    uint8 index;

    if((0 == source) ||
       (0 == subject2_collect_cells(source, 'B', boxes, &box_count)) ||
       (0 == subject2_collect_cells(source, 'T', targets, &target_count)) ||
       (box_count != box_object_count) ||
       (target_count != target_object_count))
    {
        return 0u;
    }
    for(index = 0u; index < box_object_count; index++)
    {
        expected_boxes[index] = box_objects[index].cell;
    }
    for(index = 0u; index < target_object_count; index++)
    {
        expected_targets[index] = target_objects[index].cell;
    }
    return ((0u != cells_equal(boxes, expected_boxes, box_count)) &&
            (0u != cells_equal(targets, expected_targets, target_count))) ? 1u : 0u;
}

static void subject2_fail(executor_error_enum error,
                          const char *state_text,
                          subject2_update_struct *update)
{
    competition_fatal_reason_enum reason = COMPETITION_FATAL_CLASS;

    (void)state_text;
    if((EXEC_ERROR_ART_TIMEOUT == error) || (EXEC_ERROR_ART_SYNC == error) ||
       (EXEC_ERROR_ART_CENTER == error))
    {
        reason = COMPETITION_FATAL_ART1;
    }
    else if(EXEC_ERROR_SUBJECT2_TRACK == error)
    {
        reason = COMPETITION_FATAL_TRACK;
    }
    else if((EXEC_ERROR_SUBJECT2_PLAN == error) || (EXEC_ERROR_ART_PLAN == error) ||
            (EXEC_ERROR_MAP == error))
    {
        reason = COMPETITION_FATAL_PLAN;
    }
    else if(EXEC_ERROR_SUBJECT2_YAW == error)
    {
        reason = COMPETITION_FATAL_DRIVE;
    }
    vision_uart_cancel();
    if(EXEC_STATE_ERROR != executor_get_state())
    {
        executor_set_error(error);
    }
    competition_flow_latch_fatal(reason);
    subject2_state = SUBJECT2_ERROR;
    if(0 != update)
    {
        update->run_state = competition_flow_fatal_text();
        update->redraw = 1u;
    }
}

static void subject2_wait_for_blast(subject2_block_reason_enum reason,
                                    subject2_state_enum resume_state,
                                    subject2_update_struct *update)
{
    blocked_reason = reason;
    blocked_resume_state = resume_state;
    subject2_state = SUBJECT2_WAIT_BLAST;
    if(0 != update)
    {
        update->blast_requested = 1u;
        update->block_reason = reason;
        update->run_state = "S3Plan";
        update->redraw = 1u;
    }
}

static void subject2_begin_vision_mode(vision_mode_enum mode,
                                       subject2_state_enum state)
{
    active_vision_mode = mode;
    vision_mode_start_ms = time_ms();
    vision_mode_last_send_ms = vision_mode_start_ms;
    vision_uart_set_mode(mode);
    subject2_state = state;
}

static void subject2_begin_scan_map_sync(subject2_update_struct *update)
{
    executor_stop();
    vision_uart_cancel();
    scan_sync_source_valid = 0u;
    map_stability_tracker_reset(&confirm_tracker,
                                openart_uart_get_frame_count());
    center_request_start_ms = time_ms();
    scan_sync_phase = SUBJECT2_SCAN_SYNC_WAIT_MAP;
    subject2_state = SUBJECT2_SCAN_MAP_SYNC;
    if(0 != update)
    {
        update->run_state = "VSync";
        update->redraw = 1u;
    }
}

static void subject2_tick_vision_mode(subject2_update_struct *update)
{
    uint32 now_ms = time_ms();
    const char *state_text = (VISION_MODE_BOX == active_vision_mode) ?
                             "BScan" : "TScan";

    if(0u != vision_uart_mode_ready(active_vision_mode))
    {
        subject2_state = (VISION_MODE_BOX == active_vision_mode) ?
                         SUBJECT2_SCAN_BOX_PLAN : SUBJECT2_SCAN_TARGET_PLAN;
    }
    else if((now_ms - vision_mode_start_ms) >= SUBJECT2_VISION_READY_TIMEOUT_MS)
    {
        subject2_begin_recovery(SUBJECT2_RECOVERY_CLASS,
                                COMPETITION_FATAL_ART2,
                                EXEC_ERROR_SUBJECT2_CLASS, update);
        return;
    }
    else if((now_ms - vision_mode_last_send_ms) >= SUBJECT2_VISION_READY_RETRY_MS)
    {
        vision_uart_set_mode(active_vision_mode);
        vision_mode_last_send_ms = now_ms;
    }
    if(0 != update)
    {
        update->run_state = state_text;
    }
}

static subject2_object_struct *current_objects(void)
{
    return ((SUBJECT2_SCAN_BOX_PLAN == subject2_state) ||
            (SUBJECT2_SCAN_BOX_MOVE == subject2_state) ||
            (SUBJECT2_SCAN_BOX_TURN == subject2_state) ||
            (SUBJECT2_SCAN_BOX_CENTER == subject2_state) ||
            (SUBJECT2_SCAN_BOX_ADJUST == subject2_state) ||
            (SUBJECT2_SCAN_BOX_CLASSIFY == subject2_state) ||
            (SUBJECT2_SCAN_BOX_BACKOFF == subject2_state) ||
            (SUBJECT2_SCAN_BOX_BACKOFF_RETURN == subject2_state)) ?
           box_objects : target_objects;
}

static uint8 current_object_count(void)
{
    return (box_objects == current_objects()) ? box_object_count : target_object_count;
}

static uint8 all_objects_recognized(const subject2_object_struct *objects, uint8 count)
{
    uint8 index;

    for(index = 0u; index < count; index++)
    {
        if(0u == objects[index].recognized)
        {
            return 0u;
        }
    }
    return 1u;
}

static void subject2_mark_observation_failed(subject2_update_struct *update)
{
    subject2_object_struct *objects = current_objects();

    objects[current_observation.object_index].tried_observation_mask |=
        current_observation.observation_bit;
    subject2_state = (objects == box_objects) ?
                     SUBJECT2_SCAN_BOX_PLAN : SUBJECT2_SCAN_TARGET_PLAN;
    if(0 != update)
    {
        update->run_state = "VRetry";
        update->redraw = 1u;
    }
}

static uint8 subject2_collect_center(void)
{
    return art_center_batch_collect(&center_batch);
}

static void subject2_retry_center_request(const char *state_text,
                                          subject2_update_struct *update)
{
    set_motion(0.0f, 0.0f);
    art_center_batch_reset(&center_batch);
    executor_reset_art_player_center_samples();
    openart_request_player_center();
    center_request_frame = openart_uart_get_frame_count();
    scan_center_request_started = 1u;
    if(0 != update)
    {
        update->run_state = state_text;
        update->redraw = 1u;
    }
}

static uint8 subject2_observation_map_matches(const map_source_struct *source)
{
    uint8 car_row;
    uint8 car_col;

    return ((0 != source) &&
            (0 != map_find_car(source, &car_row, &car_col, 0)) &&
            (0 != subject2_map_objects_unchanged(source)) &&
            (car_row == current_observation.row) &&
            (car_col == current_observation.col)) ? 1u : 0u;
}

static uint8 subject2_apply_center_from_cell(const subject2_context_struct *context,
                                              const map_source_struct *source,
                                              uint8 source_car_row,
                                              uint8 source_car_col,
                                              uint8 reference_row,
                                              uint8 reference_col,
                                              uint16 col_q,
                                              uint16 row_q,
                                              float *applied_x_cm,
                                              float *applied_y_cm)
{
    char old_car_value;
    char reference_value;
    float offset_x_cm;
    float offset_y_cm;

    if((0 == context) || (0 == source))
    {
        return 0u;
    }

    if((col_q >= (MAP_COLS * 100u)) || (row_q >= (MAP_ROWS * 100u)))
    {
        return 0u;
    }
    offset_x_cm = ((float)((int32)col_q - (int32)(reference_col * 100u + 50u)) /
                   100.0f) * GRID_SIZE_CM;
    offset_y_cm = -((float)((int32)row_q - (int32)(reference_row * 100u + 50u)) /
                    100.0f) * GRID_SIZE_CM;
    if((subject2_abs_float(offset_x_cm) > EXEC_ART_CENTER_ABNORMAL_CM) ||
       (subject2_abs_float(offset_y_cm) > EXEC_ART_CENTER_ABNORMAL_CM))
    {
        return 0u;
    }

    current_pose_offset_x_cm = offset_x_cm;
    current_pose_offset_y_cm = offset_y_cm;
    navigation_start_row = reference_row;
    navigation_start_col = reference_col;
    *context->start_row = reference_row;
    *context->start_col = reference_col;
    map_source_snapshot(context->snapshot, context->snapshot_rows, source);
    if((source_car_row != reference_row) || (source_car_col != reference_col))
    {
        old_car_value = context->snapshot_rows[source_car_row][source_car_col];
        context->snapshot_rows[source_car_row][source_car_col] =
            ('+' == old_car_value) ? 'T' : '.';
        reference_value = context->snapshot_rows[reference_row][reference_col];
        context->snapshot_rows[reference_row][reference_col] =
            ('T' == reference_value) ? '+' : 'C';
    }
    *context->snapshot_valid = 1u;
    if(0 != applied_x_cm)
    {
        *applied_x_cm = offset_x_cm;
    }
    if(0 != applied_y_cm)
    {
        *applied_y_cm = offset_y_cm;
    }
    return 1u;
}

static uint8 subject2_apply_center(const subject2_context_struct *context,
                                   const map_source_struct *source,
                                   uint8 require_observation_match,
                                   float *applied_x_cm,
                                   float *applied_y_cm)
{
    uint8 car_row;
    uint8 car_col;
    uint16 col_q;
    uint16 row_q;

    if((0 == source) ||
       (0u == subject2_get_center_median(&col_q, &row_q)) ||
       (0 == map_find_car(source, &car_row, &car_col, 0)))
    {
        return 0u;
    }
    if((0u != require_observation_match) &&
       (0 == subject2_observation_map_matches(source)))
    {
        return 0u;
    }
    return subject2_apply_center_from_cell(context, source,
                                           car_row, car_col,
                                           car_row, car_col,
                                           col_q, row_q,
                                           applied_x_cm, applied_y_cm);
}

static uint8 subject2_fresh_grid_is_usable(uint32 after_frame,
                                           uint8 require_observation,
                                           uint8 require_pre_push_wait,
                                           uint8 require_pre_push_box)
{
    const map_source_struct *source = openart_map_get();
    map_scan_stats_struct stats;
    uint8 wait_row;
    uint8 wait_col;
    char box_value;

    if((openart_uart_get_frame_count() <= after_frame) || (0 == source) ||
       (0u == drive_control_is_healthy()) ||
       (EXEC_STATE_ERROR == executor_get_state()))
    {
        return 0u;
    }
    map_scan_stats(source, &stats);
    if((1u != stats.car_count) || (stats.box_count != stats.target_count))
    {
        return 0u;
    }
    if((0u != require_observation) &&
       (0u == subject2_observation_map_matches(source)))
    {
        return 0u;
    }
    if(0u != require_pre_push_wait)
    {
        if((0u == executor_get_pre_push_wait_cell(&wait_row, &wait_col)) ||
           (stats.car_row != wait_row) || (stats.car_col != wait_col))
        {
            return 0u;
        }
    }
    if(0u != require_pre_push_box)
    {
        if((pre_push_box_session.box_row >= MAP_ROWS) ||
           (pre_push_box_session.box_col >= MAP_COLS))
        {
            return 0u;
        }
        box_value = source->rows[pre_push_box_session.box_row]
                                [pre_push_box_session.box_col];
        if(('B' != box_value) && (MAP_BOX_ON_TARGET != box_value))
        {
            return 0u;
        }
    }
    return 1u;
}

static void subject2_reset_recovery(void)
{
    recovery_reason = SUBJECT2_RECOVERY_NONE;
    recovery_count = 0u;
    recovery_box_cell = INVALID_STATE;
    recovery_target_cell = INVALID_STATE;
    recovery_observation_bit = 0u;
}

static competition_fatal_reason_enum subject2_recovery_fatal_reason(void)
{
    switch(recovery_reason)
    {
        case SUBJECT2_RECOVERY_CENTER:
        case SUBJECT2_RECOVERY_OBSERVE:
            return COMPETITION_FATAL_ART1;
        case SUBJECT2_RECOVERY_YAW:
        case SUBJECT2_RECOVERY_MOTION:
            return COMPETITION_FATAL_DRIVE;
        case SUBJECT2_RECOVERY_TRACK:
            return COMPETITION_FATAL_TRACK;
        case SUBJECT2_RECOVERY_PLAN:
            return COMPETITION_FATAL_PLAN;
        case SUBJECT2_RECOVERY_CLASS:
            return COMPETITION_FATAL_ART2;
        default:
            return COMPETITION_FATAL_MAP;
    }
}

static executor_error_enum subject2_recovery_final_error(void)
{
    switch(recovery_reason)
    {
        case SUBJECT2_RECOVERY_CENTER:
        case SUBJECT2_RECOVERY_OBSERVE:
            return EXEC_ERROR_ART_CENTER;
        case SUBJECT2_RECOVERY_YAW:
        case SUBJECT2_RECOVERY_MOTION:
            return EXEC_ERROR_SUBJECT2_YAW;
        case SUBJECT2_RECOVERY_TRACK:
            return EXEC_ERROR_SUBJECT2_TRACK;
        case SUBJECT2_RECOVERY_PLAN:
            return EXEC_ERROR_SUBJECT2_PLAN;
        case SUBJECT2_RECOVERY_CLASS:
            return EXEC_ERROR_SUBJECT2_CLASS;
        default:
            return EXEC_ERROR_ART_SYNC;
    }
}

static void subject2_begin_recovery(subject2_recovery_reason_enum reason,
                                    competition_fatal_reason_enum fatal_reason,
                                    executor_error_enum final_error,
                                    subject2_update_struct *update)
{
    subject2_state_enum origin = subject2_state;
    uint8 observation_bit = current_observation.observation_bit;

    if((SUBJECT2_SCAN_MAP_SYNC == subject2_state) &&
       (SUBJECT2_RECOVERY_NONE != recovery_reason))
    {
        origin = recovery_origin_state;
        observation_bit = recovery_observation_bit;
    }
    if((reason == recovery_reason) &&
       ((origin == recovery_origin_state) ||
        (SUBJECT2_RECOVERY_PLAN == reason)) &&
       (active_box_cell == recovery_box_cell) &&
       (active_target_cell == recovery_target_cell) &&
       (observation_bit == recovery_observation_bit))
    {
        recovery_count++;
    }
    else
    {
        recovery_reason = reason;
        recovery_origin_state = origin;
        recovery_count = 1u;
        recovery_box_cell = active_box_cell;
        recovery_target_cell = active_target_cell;
        recovery_observation_bit = observation_bit;
    }

    if((0u == drive_control_is_healthy()) ||
       (recovery_count > RECOVERY_MAX_RETRIES))
    {
        if(0u == drive_control_is_healthy())
        {
            competition_flow_latch_fatal(COMPETITION_FATAL_DRIVE);
        }
        else
        {
            competition_flow_latch_fatal(fatal_reason);
        }
        vision_uart_cancel();
        if(EXEC_STATE_ERROR != executor_get_state())
        {
            executor_set_error(final_error);
        }
        subject2_state = SUBJECT2_ERROR;
        if(0 != update)
        {
            update->run_state = competition_flow_fatal_text();
            update->redraw = 1u;
        }
        return;
    }
    subject2_begin_scan_map_sync(update);
}

#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
static uint8 subject2_apply_map_cell_center(const subject2_context_struct *context)
{
    const map_source_struct *source;
    uint8 car_row;
    uint8 car_col;

    if((0 == context) || (0 == context->snapshot) ||
       (0 == context->snapshot_valid) || (0u == *context->snapshot_valid))
    {
        return 0u;
    }
    source = context->snapshot;
    if(0 == map_find_car(source, &car_row, &car_col, 0))
    {
        return 0u;
    }
    current_pose_offset_x_cm = 0.0f;
    current_pose_offset_y_cm = 0.0f;
    navigation_start_row = car_row;
    navigation_start_col = car_col;
    *context->start_row = car_row;
    *context->start_col = car_col;
    return 1u;
}
#endif

static void subject2_begin_classification(subject2_state_enum classify_state,
                                          subject2_update_struct *update)
{
    subject2_classifier_reset(&classifier);
    current_vision_request_id = vision_uart_request_classification();
    classify_start_ms = time_ms();
    subject2_state = classify_state;
    if(0 != update)
    {
        update->run_state = "VWait";
        update->redraw = 1u;
    }
}

static void subject2_begin_view_backoff(subject2_update_struct *update)
{
    subject2_object_struct *objects = current_objects();
    const drive_pose_struct *pose = drive_pose_get();
    uint16 object_cell = objects[current_observation.object_index].cell;
    int16 away_row = (int16)current_observation.row -
                     (int16)map_cell_row(object_cell);
    int16 away_col = (int16)current_observation.col -
                     (int16)map_cell_col(object_cell);
    float target_x_cm;
    float target_y_cm;

    if(1 != (subject2_abs_float((float)away_row) +
             subject2_abs_float((float)away_col)))
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:Back", update);
        return;
    }

    view_backoff_origin_x_cm = pose->x_cm;
    view_backoff_origin_y_cm = pose->y_cm;
    target_x_cm = view_backoff_origin_x_cm +
                  (float)away_col * SUBJECT2_VIEW_BACKOFF_CM;
    target_y_cm = view_backoff_origin_y_cm -
                  (float)away_row * SUBJECT2_VIEW_BACKOFF_CM;
    if(0 == executor_start_position_correction(target_x_cm, target_y_cm))
    {
        if(EXEC_STATE_ERROR == executor_get_state())
        {
            subject2_fail(executor_get_error(), "E:Back", update);
        }
        else
        {
            subject2_begin_recovery(SUBJECT2_RECOVERY_MOTION,
                                    COMPETITION_FATAL_DRIVE,
                                    EXEC_ERROR_SUBJECT2_CLASS, update);
        }
        return;
    }

    vision_uart_cancel();
    view_backoff_active = 1u;
    view_backoff_recognition_accepted = 0u;
    view_motion_start_ms = time_ms();
    subject2_state = (objects == box_objects) ?
                     SUBJECT2_SCAN_BOX_BACKOFF : SUBJECT2_SCAN_TARGET_BACKOFF;
    if(0 != update)
    {
        update->run_state = "VBack";
        update->redraw = 1u;
    }
}

static void subject2_begin_view_return(uint8 recognition_accepted,
                                       subject2_update_struct *update)
{
    uint8 box_scan = (box_objects == current_objects()) ? 1u : 0u;

    if(0 == executor_start_position_correction(view_backoff_origin_x_cm,
                                               view_backoff_origin_y_cm))
    {
        if(EXEC_STATE_ERROR == executor_get_state())
        {
            subject2_fail(executor_get_error(), "E:Back", update);
        }
        else
        {
            subject2_begin_recovery(SUBJECT2_RECOVERY_MOTION,
                                    COMPETITION_FATAL_DRIVE,
                                    EXEC_ERROR_SUBJECT2_CLASS, update);
        }
        return;
    }
    view_backoff_recognition_accepted = recognition_accepted;
    view_motion_start_ms = time_ms();
    subject2_state = (0u != box_scan) ?
                     SUBJECT2_SCAN_BOX_BACKOFF_RETURN :
                     SUBJECT2_SCAN_TARGET_BACKOFF_RETURN;
    if(0 != update)
    {
        update->run_state = "VHome";
        update->redraw = 1u;
    }
}

static void subject2_begin_scan_center(subject2_update_struct *update)
{
    uint8 box_scan = (box_objects == current_objects()) ? 1u : 0u;

    set_motion(0.0f, 0.0f);
    art_center_batch_reset(&center_batch);
    center_request_start_ms = time_ms();
    center_request_frame = openart_uart_get_frame_count();
    scan_fast_center_after_frame = openart_get_player_center(0, 0, 0);
    scan_center_request_started = 0u;
    subject2_state = (0u != box_scan) ?
                     SUBJECT2_SCAN_BOX_CENTER : SUBJECT2_SCAN_TARGET_CENTER;
    if(0 != update)
    {
        update->run_state = "VCtr";
        update->redraw = 1u;
    }
}

static void subject2_begin_turn(subject2_update_struct *update)
{
    uint8 box_scan = (box_objects == current_objects()) ? 1u : 0u;

    set_motion(0.0f, 0.0f);
    set_target_yaw(current_observation.target_yaw_deg);
    turn_start_ms = time_ms();
    turn_stable_start_ms = 0u;
    turn_stable_active = 0u;
    subject2_state = (0u != box_scan) ?
                     SUBJECT2_SCAN_BOX_TURN : SUBJECT2_SCAN_TARGET_TURN;
    if(0 != update)
    {
        update->run_state = "VTurn";
        update->redraw = 1u;
    }
}

static void subject2_tick_turn(subject2_update_struct *update)
{
    const control_status_struct *status = get_control_status();
    uint32 now_ms = time_ms();
    subject2_state_enum classify_state =
        (SUBJECT2_SCAN_BOX_TURN == subject2_state) ?
        SUBJECT2_SCAN_BOX_CLASSIFY : SUBJECT2_SCAN_TARGET_CLASSIFY;

    if(subject2_abs_float(status->yaw_error) <= SUBJECT2_TURN_TOLERANCE_DEG)
    {
        if(0u == turn_stable_active)
        {
            turn_stable_active = 1u;
            turn_stable_start_ms = now_ms;
        }
        else if((now_ms - turn_stable_start_ms) >= SUBJECT2_TURN_STABLE_MS)
        {
            subject2_begin_classification(classify_state, update);
            return;
        }
    }
    else
    {
        turn_stable_active = 0u;
    }

    if((now_ms - turn_start_ms) >= SUBJECT2_TURN_TIMEOUT_MS)
    {
        if((0u != drive_control_is_healthy()) &&
           (subject2_abs_float(status->yaw_error) <= 2.0f))
        {
            subject2_begin_classification(classify_state, update);
            if(0 != update) update->run_state = "YawKeep";
        }
        else if(0u != drive_control_is_healthy())
        {
            subject2_mark_observation_failed(update);
        }
        else
        {
            subject2_fail(EXEC_ERROR_SUBJECT2_YAW, "E:Yaw", update);
        }
    }
    else if(0 != update)
    {
        update->run_state = "VTurn";
    }
}

static void subject2_tick_center(const subject2_context_struct *context,
                                 subject2_update_struct *update)
{
    const map_source_struct *source;
    uint8 is_box_scan = (SUBJECT2_SCAN_BOX_CENTER == subject2_state) ? 1u : 0u;
    uint8 car_row;
    uint8 car_col;
    uint16 center_col_q;
    uint16 center_row_q;
    uint8 center_applied = 0u;

    if(0u == scan_center_request_started)
    {
        uint8 periodic_center_valid;
        uint32 periodic_center_frame = openart_get_player_center(
            &center_col_q, &center_row_q, &periodic_center_valid);

        if(periodic_center_frame != scan_fast_center_after_frame)
        {
            if(0u != subject2_try_fast_scan_center(context,
                                                    center_col_q,
                                                    center_row_q,
                                                    periodic_center_valid,
                                                    is_box_scan,
                                                    update))
            {
                return;
            }
            openart_request_player_center();
            center_request_frame = openart_uart_get_frame_count();
            scan_center_request_started = 1u;
        }
        else if((time_ms() - center_request_start_ms) >=
                SUBJECT2_FAST_CENTER_WAIT_MS)
        {
            openart_request_player_center();
            center_request_frame = openart_uart_get_frame_count();
            scan_center_request_started = 1u;
        }
        else
        {
            if(0 != update)
            {
                update->run_state = "VCtr";
            }
            return;
        }
    }

    if(0 == subject2_collect_center())
    {
        if((time_ms() - center_request_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
        {
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
            if(0u != subject2_fresh_grid_is_usable(center_request_frame,
                                                    1u, 0u, 0u))
            {
                subject2_begin_turn(update);
                if(0 != update) update->run_state = "CtrSkip";
            }
            else
            {
                subject2_begin_recovery(SUBJECT2_RECOVERY_CENTER,
                                        COMPETITION_FATAL_ART1,
                                        EXEC_ERROR_ART_CENTER, update);
            }
#else
            subject2_fail(EXEC_ERROR_ART_CENTER, "E:CTmo", update);
#endif
            return;
        }
        if(0 != update)
        {
            update->run_state = "VCtr";
        }
        return;
    }
    (void)subject2_get_center_median(&center_col_q, &center_row_q);
    source = openart_get_requested_center_map();
    if(0 == map_validate_player_center(source,
                                       center_col_q, center_row_q,
                                       &car_row, &car_col))
    {
        subject2_retry_center_request("VCtr", update);
        return;
    }
    if(0 != subject2_map_objects_unchanged(source))
    {
        if((car_row == current_observation.row) &&
           (car_col == current_observation.col))
        {
            if(0 == subject2_apply_center(context, source, 1u, 0, 0))
            {
                subject2_begin_recovery(SUBJECT2_RECOVERY_CENTER,
                                        COMPETITION_FATAL_ART1,
                                        EXEC_ERROR_ART_CENTER, update);
                return;
            }
            center_applied = 1u;
        }
        else
        {
            if(0 == subject2_apply_center(context, source, 0u, 0, 0))
            {
                subject2_begin_recovery(SUBJECT2_RECOVERY_CENTER,
                                        COMPETITION_FATAL_ART1,
                                        EXEC_ERROR_ART_CENTER, update);
                return;
            }
            subject2_state = (0u != is_box_scan) ?
                             SUBJECT2_SCAN_BOX_PLAN : SUBJECT2_SCAN_TARGET_PLAN;
            scan_plan_snapshot_pending = 1u;
            if(0 != update)
            {
                update->run_state = "VReplan";
                update->redraw = 1u;
            }
            return;
        }
    }
    if((0u == center_applied) &&
       (0 == subject2_map_objects_unchanged(source)))
    {
        subject2_begin_scan_map_sync(update);
        return;
    }
    if((0u == center_applied) &&
       (0 == subject2_apply_center(context, source, 1u, 0, 0)))
    {
        subject2_begin_recovery(SUBJECT2_RECOVERY_CENTER,
                                COMPETITION_FATAL_ART1,
                                EXEC_ERROR_ART_CENTER, update);
        return;
    }

    if(0 == executor_start_position_correction_with_pose_reset(
                 current_pose_offset_x_cm,
                 current_pose_offset_y_cm,
                 0.0f, 0.0f))
    {
        if(EXEC_STATE_ERROR == executor_get_state())
        {
            subject2_fail(executor_get_error(), "E:CBsy", update);
        }
        else
        {
            subject2_begin_recovery(SUBJECT2_RECOVERY_CENTER,
                                    COMPETITION_FATAL_ART1,
                                    EXEC_ERROR_ART_CENTER, update);
        }
        return;
    }

    center_adjust_start_ms = time_ms();
    subject2_state = (0u != is_box_scan) ?
                     SUBJECT2_SCAN_BOX_ADJUST : SUBJECT2_SCAN_TARGET_ADJUST;
    if(0 != update)
    {
        update->run_state = "VAdj";
        update->redraw = 1u;
    }
}

static void subject2_tick_center_adjust(subject2_update_struct *update)
{
    const drive_pose_struct *pose;

    if(EXEC_STATE_DONE == executor_get_state())
    {
        pose = drive_pose_get();
        current_pose_offset_x_cm = pose->x_cm;
        current_pose_offset_y_cm = pose->y_cm;
        center_adjust_start_ms = 0u;
        subject2_begin_turn(update);
        return;
    }
    if(EXEC_STATE_ERROR == executor_get_state())
    {
        if(EXEC_ERROR_ART_CENTER == executor_get_error())
        {
            subject2_begin_recovery(SUBJECT2_RECOVERY_CENTER,
                                    COMPETITION_FATAL_ART1,
                                    EXEC_ERROR_ART_CENTER, update);
        }
        else
        {
            subject2_fail(executor_get_error(), "E:CExe", update);
        }
        return;
    }
    if((time_ms() - center_adjust_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        pose = drive_pose_get();
        if((0u != drive_control_is_healthy()) &&
           (subject2_abs_float(pose->x_cm) <= SUBJECT2_FAST_CENTER_TOLERANCE_CM) &&
           (subject2_abs_float(pose->y_cm) <= SUBJECT2_FAST_CENTER_TOLERANCE_CM))
        {
            executor_stop();
            current_pose_offset_x_cm = pose->x_cm;
            current_pose_offset_y_cm = pose->y_cm;
            subject2_begin_turn(update);
        }
        else
        {
            subject2_begin_recovery(SUBJECT2_RECOVERY_CENTER,
                                    COMPETITION_FATAL_ART1,
                                    EXEC_ERROR_ART_CENTER, update);
        }
        return;
    }
    if(0 != update)
    {
        update->run_state = "VAdj";
    }
}

static void subject2_advance_after_recognition(subject2_update_struct *update)
{
    if(box_objects == current_objects())
    {
        if(0 != all_objects_recognized(box_objects, box_object_count))
        {
            subject2_begin_vision_mode(VISION_MODE_TARGET,
                                       SUBJECT2_SCAN_TARGET_MODE);
        }
        else
        {
            subject2_state = SUBJECT2_SCAN_BOX_PLAN;
        }
    }
    else
    {
        subject2_state = (0 != all_objects_recognized(target_objects,
                                                       target_object_count)) ?
                         SUBJECT2_VALIDATE_BINDINGS : SUBJECT2_SCAN_TARGET_PLAN;
    }
    if(0 != update)
    {
        update->run_state = (SUBJECT2_SCAN_TARGET_MODE == subject2_state) ?
                            "TScan" :
                            ((SUBJECT2_VALIDATE_BINDINGS == subject2_state) ?
                             "Bind" : "VPos");
        update->redraw = 1u;
    }
}

static void subject2_tick_classify(subject2_update_struct *update)
{
    vision_sample_struct sample;
    subject2_object_struct *objects = current_objects();
    uint8 confirmed_class = SUBJECT2_INVALID_CLASS;
    uint8 confirmed = 0u;

    while(0 != vision_uart_get_sample(&sample))
    {
        if(sample.request_id != current_vision_request_id)
        {
            continue;
        }
        if(0 != subject2_classifier_push(&classifier,
                                         sample.class_id,
                                         sample.confidence_q,
                                         SUBJECT2_CLASS_CONFIDENCE_Q,
                                         SUBJECT2_CLASS_STABLE_SAMPLES,
                                         &confirmed_class))
        {
            confirmed = 1u;
            break;
        }
    }
    if(0u != confirmed)
    {
        vision_uart_ack(current_vision_request_id);
        objects[current_observation.object_index].class_id = confirmed_class;
        objects[current_observation.object_index].recognized = 1u;
        last_recognition_valid = 1u;
        last_recognition_is_target = (objects == target_objects) ? 1u : 0u;
        last_recognition_class = confirmed_class;
        if(0u != view_backoff_active)
        {
            subject2_begin_view_return(1u, update);
        }
        else
        {
            subject2_advance_after_recognition(update);
        }
        return;
    }
    if((time_ms() - classify_start_ms) >= SUBJECT2_VIEW_TIMEOUT_MS)
    {
        if(0u == view_backoff_active)
        {
            subject2_begin_view_backoff(update);
        }
        else
        {
            vision_uart_cancel();
            subject2_begin_view_return(0u, update);
        }
        return;
    }
    if(0 != update)
    {
        update->run_state = "VWait";
    }
}

static void subject2_tick_view_backoff(subject2_update_struct *update)
{
    uint8 box_scan = (SUBJECT2_SCAN_BOX_BACKOFF == subject2_state) ? 1u : 0u;

    if(EXEC_STATE_DONE == executor_get_state())
    {
        subject2_begin_classification(
            (0u != box_scan) ? SUBJECT2_SCAN_BOX_CLASSIFY :
                               SUBJECT2_SCAN_TARGET_CLASSIFY,
            update);
        if(0 != update)
        {
            update->run_state = "VFar";
        }
        return;
    }
    if(EXEC_STATE_ERROR == executor_get_state())
    {
        subject2_fail(executor_get_error(), "E:Back", update);
        return;
    }
    if((time_ms() - view_motion_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        subject2_begin_recovery(SUBJECT2_RECOVERY_MOTION,
                                COMPETITION_FATAL_DRIVE,
                                EXEC_ERROR_SUBJECT2_CLASS, update);
        return;
    }
    if(0 != update)
    {
        update->run_state = "VBack";
    }
}

static void subject2_tick_view_return(subject2_update_struct *update)
{
    const drive_pose_struct *pose;
    uint8 recognition_accepted;

    if(EXEC_STATE_DONE == executor_get_state())
    {
        pose = drive_pose_get();
        current_pose_offset_x_cm = pose->x_cm;
        current_pose_offset_y_cm = pose->y_cm;
        recognition_accepted = view_backoff_recognition_accepted;
        view_backoff_active = 0u;
        view_backoff_recognition_accepted = 0u;
        view_motion_start_ms = 0u;
        if(0u != recognition_accepted)
        {
            subject2_advance_after_recognition(update);
        }
        else
        {
            subject2_mark_observation_failed(update);
        }
        return;
    }
    if(EXEC_STATE_ERROR == executor_get_state())
    {
        subject2_fail(executor_get_error(), "E:Back", update);
        return;
    }
    if((time_ms() - view_motion_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        subject2_begin_recovery(SUBJECT2_RECOVERY_MOTION,
                                COMPETITION_FATAL_DRIVE,
                                EXEC_ERROR_SUBJECT2_CLASS, update);
        return;
    }
    if(0 != update)
    {
        update->run_state = "VHome";
    }
}

static void subject2_tick_plan(const subject2_context_struct *context,
                               subject2_update_struct *update)
{
    const map_source_struct *source;
    const control_status_struct *control_status;
    subject2_object_struct *objects = current_objects();
    uint8 count = current_object_count();
    map_scan_stats_struct stats;

    source = ((0u != scan_plan_snapshot_pending) &&
              (0u != *context->snapshot_valid)) ?
             context->snapshot : openart_map_get();

    if(0 == source)
    {
        subject2_begin_recovery(SUBJECT2_RECOVERY_MAP,
                                COMPETITION_FATAL_MAP,
                                EXEC_ERROR_ART_SYNC, update);
        return;
    }
    if(0 == subject2_map_objects_unchanged(source))
    {
        subject2_begin_scan_map_sync(update);
        return;
    }
    if(source != context->snapshot)
    {
        map_source_snapshot(context->snapshot, context->snapshot_rows, source);
        *context->snapshot_valid = 1u;
    }
    scan_plan_snapshot_pending = 0u;
#if SUBJECT2_LAST_TARGET_ELIMINATION_ENABLE
    if(objects == target_objects)
    {
        uint8 inferred_target_index;
        uint8 inferred_class;

        if(0u != subject2_infer_last_target_class(
                      box_objects, box_object_count,
                      target_objects, target_object_count,
                      &inferred_target_index, &inferred_class))
        {
            target_objects[inferred_target_index].class_id = inferred_class;
            target_objects[inferred_target_index].recognized = 1u;
            subject2_state = SUBJECT2_VALIDATE_BINDINGS;
            if(0 != update)
            {
                update->run_state = "Bind";
                update->redraw = 1u;
            }
            return;
        }
    }
#endif
    control_status = get_control_status();
    if(0 == subject2_select_observation(context->snapshot, objects, count,
                                         control_status->current_yaw,
                                         &current_observation, context->result))
    {
        if(0u != context->allow_blast_fallback)
        {
            subject2_wait_for_blast(
                (objects == box_objects) ? SUBJECT2_BLOCK_OBSERVE_BOX :
                                           SUBJECT2_BLOCK_OBSERVE_TARGET,
                (objects == box_objects) ? SUBJECT2_SCAN_BOX_PLAN :
                                           SUBJECT2_SCAN_TARGET_PLAN,
                update);
            return;
        }
        subject2_begin_recovery(SUBJECT2_RECOVERY_PLAN,
                                COMPETITION_FATAL_PLAN,
                                EXEC_ERROR_SUBJECT2_PLAN, update);
        return;
    }
    view_backoff_active = 0u;
    view_backoff_recognition_accepted = 0u;
    map_scan_stats(context->snapshot, &stats);
    navigation_start_row = stats.car_row;
    navigation_start_col = stats.car_col;
    *context->start_row = stats.car_row;
    *context->start_col = stats.car_col;
    if(0u == context->result->waypoint_count)
    {
        subject2_begin_scan_center(update);
        return;
    }

    executor_start(context->result->waypoints,
                   context->result->waypoint_count,
                   stats.car_row,
                   stats.car_col,
                   current_pose_offset_x_cm,
                   current_pose_offset_y_cm,
                   (RUN_MODE_STEP == context->run_mode) ? 1u : 0u,
                   0u);
    subject2_state = (objects == box_objects) ?
                     SUBJECT2_SCAN_BOX_MOVE : SUBJECT2_SCAN_TARGET_MOVE;
    if(0 != update)
    {
        update->run_state = "VPos";
        update->redraw = 1u;
    }
}

static void subject2_tick_move(subject2_update_struct *update)
{
    if(EXEC_STATE_DONE == executor_get_state())
    {
        subject2_begin_scan_center(update);
    }
    else if(EXEC_STATE_ERROR == executor_get_state())
    {
        subject2_fail(executor_get_error(), "E:Move", update);
    }
    else if(0 != update)
    {
        update->run_state = "VPos";
    }
}

static void invalidate_mismatched_classes(uint8 *need_boxes, uint8 *need_targets)
{
    subject2_invalidate_mismatched_classes(box_objects, box_object_count,
                                           target_objects, target_object_count,
                                           need_boxes, need_targets);
}

static void subject2_tick_validate(subject2_update_struct *update)
{
    uint8 need_boxes;
    uint8 need_targets;

    if(0 != subject2_object_class_counts_match(box_objects, box_object_count,
                                                target_objects, target_object_count))
    {
        set_motion(0.0f, 0.0f);
        set_target_yaw(launch_yaw_deg);
        turn_start_ms = time_ms();
        turn_stable_start_ms = 0u;
        turn_stable_active = 0u;
        subject2_state = SUBJECT2_RESTORE_HEADING;
        if(0 != update)
        {
            update->run_state = "HYaw";
            update->redraw = 1u;
        }
        return;
    }
    if(0u != validation_retry_count)
    {
        subject2_begin_recovery(SUBJECT2_RECOVERY_CLASS,
                                COMPETITION_FATAL_CLASS,
                                EXEC_ERROR_SUBJECT2_CLASS, update);
        return;
    }
    invalidate_mismatched_classes(&need_boxes, &need_targets);
    validation_retry_count++;
    if(0u != need_boxes)
    {
        subject2_begin_vision_mode(VISION_MODE_BOX,
                                   SUBJECT2_SCAN_BOX_MODE);
    }
    else if(0u != need_targets)
    {
        subject2_begin_vision_mode(VISION_MODE_TARGET,
                                   SUBJECT2_SCAN_TARGET_MODE);
    }
    else
    {
        subject2_begin_recovery(SUBJECT2_RECOVERY_CLASS,
                                COMPETITION_FATAL_CLASS,
                                EXEC_ERROR_SUBJECT2_CLASS, update);
        return;
    }
    if(0 != update)
    {
        update->run_state = "VRetry";
        update->redraw = 1u;
    }
}

static uint8 subject2_try_fast_scan_center(
    const subject2_context_struct *context,
    uint16 center_col_q,
    uint16 center_row_q,
    uint8 center_valid,
    uint8 is_box_scan,
    subject2_update_struct *update)
{
    const map_source_struct *source = openart_map_get();
    uint8 car_row;
    uint8 car_col;
    float offset_x_cm;
    float offset_y_cm;

    if((0u == map_find_car(source, &car_row, &car_col, 0)) ||
       (0u == subject2_map_objects_unchanged(source)) ||
       (car_row != current_observation.row) ||
       (car_col != current_observation.col))
    {
        return 0u;
    }

    if(0u != center_valid)
    {
        if(0u == map_validate_player_center(source,
                                             center_col_q, center_row_q,
                                             &car_row, &car_col))
        {
            return 0u;
        }
        offset_x_cm = ((float)((int32)center_col_q -
                               (int32)(car_col * 100u + 50u)) /
                       100.0f) * GRID_SIZE_CM;
        offset_y_cm = -((float)((int32)center_row_q -
                                (int32)(car_row * 100u + 50u)) /
                        100.0f) * GRID_SIZE_CM;
        if((subject2_abs_float(offset_x_cm) > SUBJECT2_FAST_CENTER_TOLERANCE_CM) ||
           (subject2_abs_float(offset_y_cm) > SUBJECT2_FAST_CENTER_TOLERANCE_CM))
        {
            return 0u;
        }
    }
    else
    {
        /* 周期帧没有精确中心时，唯一 C 已匹配观察格，按格中心重建导航原点。 */
        center_col_q = (uint16)(car_col * 100u + 50u);
        center_row_q = (uint16)(car_row * 100u + 50u);
    }
    if(0u == subject2_apply_center_from_cell(context, source,
                                              car_row, car_col,
                                              car_row, car_col,
                                              center_col_q, center_row_q,
                                              0, 0))
    {
        return 0u;
    }
    if(0u == executor_start_position_correction_with_pose_reset(
                  current_pose_offset_x_cm,
                  current_pose_offset_y_cm,
                  current_pose_offset_x_cm,
                  current_pose_offset_y_cm))
    {
        if(EXEC_STATE_ERROR == executor_get_state())
        {
            subject2_fail(executor_get_error(), "E:CBsy", update);
        }
        else
        {
            subject2_begin_recovery(SUBJECT2_RECOVERY_CENTER,
                                    COMPETITION_FATAL_ART1,
                                    EXEC_ERROR_ART_CENTER, update);
        }
        return 1u;
    }

    center_adjust_start_ms = time_ms();
    subject2_state = (0u != is_box_scan) ?
                     SUBJECT2_SCAN_BOX_ADJUST : SUBJECT2_SCAN_TARGET_ADJUST;
    if(0 != update)
    {
        update->run_state = "VAdj";
        update->redraw = 1u;
    }
    return 1u;
}

static void subject2_enter_select_push(subject2_update_struct *update)
{
    subject2_state = SUBJECT2_SELECT_PUSH;
    if(0 != update)
    {
        update->run_state = "Bind";
        update->redraw = 1u;
    }
}

static void subject2_finish_post_observe_yaw(uint8 rebase,
                                             subject2_update_struct *update)
{
    if(0u != rebase)
    {
        drive_control_lock_yaw_and_reset_pose();
        launch_yaw_deg = get_control_status()->target_yaw;
    }
    subject2_enter_select_push(update);
}

static void subject2_begin_post_observe_yaw(subject2_update_struct *update)
{
    if(0u != post_observe_yaw_done)
    {
        subject2_enter_select_push(update);
        return;
    }

    post_observe_yaw_done = 1u;
    if(0u == post_observe_art_yaw_bias_valid)
    {
        subject2_enter_select_push(update);
        return;
    }
    set_motion(0.0f, 0.0f);
    art_center_batch_reset(&center_batch);
    openart_request_player_center();
    post_observe_yaw_start_ms = time_ms();
    subject2_state = SUBJECT2_POST_OBSERVE_YAW_SAMPLE;
    if(0 != update)
    {
        update->run_state = "VYaw";
        update->redraw = 1u;
    }
}

static void subject2_tick_post_observe_yaw_sample(subject2_update_struct *update)
{
    const map_source_struct *source;
    uint16 center_col_q;
    uint16 center_row_q;
    uint8 car_row;
    uint8 car_col;
    float measured_yaw_deg;
    float corrected_yaw_deg;
    float correction_deg;
    float correction_abs_deg;

    if((time_ms() - post_observe_yaw_start_ms) >=
       SUBJECT2_POST_OBSERVE_YAW_TIMEOUT_MS)
    {
        subject2_finish_post_observe_yaw(0u, update);
        return;
    }
    if(0u == art_center_batch_collect(&center_batch))
    {
        if(0 != update)
        {
            update->run_state = "VYaw";
        }
        return;
    }

    source = openart_get_requested_center_map();
    if((0u == art_center_batch_get_median(&center_batch,
                                          &center_col_q, &center_row_q)) ||
       (0u == map_validate_player_center(source,
                                         center_col_q, center_row_q,
                                         &car_row, &car_col)) ||
       (0u == art_center_batch_get_yaw_deg(&center_batch,
                                           &measured_yaw_deg, 0)))
    {
        subject2_finish_post_observe_yaw(0u, update);
        return;
    }
    (void)car_row;
    (void)car_col;

    corrected_yaw_deg = measured_yaw_deg + post_observe_art_yaw_bias_deg;
    correction_deg = shortest_angle_error(ART_LAUNCH_EXPECTED_YAW_DEG,
                                           corrected_yaw_deg);
    correction_abs_deg = subject2_abs_float(correction_deg);
    if(correction_abs_deg <= SUBJECT2_POST_OBSERVE_YAW_IGNORE_DEG)
    {
        subject2_finish_post_observe_yaw(1u, update);
        return;
    }
    if(correction_abs_deg > SUBJECT2_POST_OBSERVE_YAW_MAX_CORRECT_DEG)
    {
        subject2_finish_post_observe_yaw(0u, update);
        return;
    }

    drive_control_start_relative_yaw_correction(correction_deg);
    turn_start_ms = time_ms();
    turn_stable_start_ms = 0u;
    turn_stable_active = 0u;
    subject2_state = SUBJECT2_POST_OBSERVE_YAW_FIX;
    if(0 != update)
    {
        update->run_state = "VFix";
        update->redraw = 1u;
    }
}

static void subject2_tick_post_observe_yaw_fix(subject2_update_struct *update)
{
    const control_status_struct *status = get_control_status();
    uint32 now_ms = time_ms();

    if(subject2_abs_float(status->yaw_error) <= SUBJECT2_TURN_TOLERANCE_DEG)
    {
        if(0u == turn_stable_active)
        {
            turn_stable_active = 1u;
            turn_stable_start_ms = now_ms;
        }
        else if((now_ms - turn_stable_start_ms) >= SUBJECT2_TURN_STABLE_MS)
        {
            turn_stable_active = 0u;
            subject2_finish_post_observe_yaw(1u, update);
            return;
        }
    }
    else
    {
        turn_stable_active = 0u;
    }

    if((now_ms - turn_start_ms) >= SUBJECT2_TURN_TIMEOUT_MS)
    {
        subject2_finish_post_observe_yaw(1u, update);
    }
    else if(0 != update)
    {
        update->run_state = "VFix";
    }
}

static void subject2_tick_restore_heading(subject2_update_struct *update)
{
    const control_status_struct *status = get_control_status();
    uint32 now_ms = time_ms();

    if(subject2_abs_float(status->yaw_error) <= SUBJECT2_TURN_TOLERANCE_DEG)
    {
        if(0u == turn_stable_active)
        {
            turn_stable_active = 1u;
            turn_stable_start_ms = now_ms;
        }
        else if((now_ms - turn_stable_start_ms) >= SUBJECT2_TURN_STABLE_MS)
        {
            turn_stable_active = 0u;
            subject2_begin_post_observe_yaw(update);
            return;
        }
    }
    else
    {
        turn_stable_active = 0u;
    }

    if((now_ms - turn_start_ms) >= SUBJECT2_TURN_TIMEOUT_MS)
    {
        if(0u != drive_control_is_healthy())
        {
            drive_control_lock_yaw_and_reset_pose();
            launch_yaw_deg = get_control_status()->target_yaw;
            subject2_enter_select_push(update);
        }
        else
        {
            subject2_fail(EXEC_ERROR_SUBJECT2_YAW, "E:HYaw", update);
        }
    }
    else if(0 != update)
    {
        update->run_state = "HYaw";
    }
}

static void subject2_tick_select_push(const subject2_context_struct *context,
                                      subject2_update_struct *update)
{
    map_scan_stats_struct stats;
    subject2_push_plan_struct push_plan;

    if(0u == subject2_select_push_plan(context->snapshot,
                                       box_objects, box_object_count,
                                       target_objects, target_object_count,
                                       retry_active_only,
                                       active_box_valid,
                                       active_box_cell,
                                       active_target_cell,
                                       &push_plan,
                                       &push_candidate_result))
    {
        if(0u != context->allow_blast_fallback)
        {
            subject2_wait_for_blast(SUBJECT2_BLOCK_PUSH,
                                    SUBJECT2_SELECT_PUSH, update);
            return;
        }
        subject2_begin_recovery(SUBJECT2_RECOVERY_PLAN,
                                COMPETITION_FATAL_PLAN,
                                EXEC_ERROR_SUBJECT2_PLAN, update);
        return;
    }

    memcpy(context->result, &push_candidate_result, sizeof(*context->result));
    active_class = push_plan.class_id;
    active_box_cell = push_plan.box_cell;
    active_target_cell = push_plan.target_cell;
    active_box_valid = 1u;
    retry_active_only = 0u;
    if(0 == subject2_collect_cells(context->snapshot, 'B',
                                   task_start_boxes, &task_start_box_count))
    {
        subject2_begin_recovery(SUBJECT2_RECOVERY_TRACK,
                                COMPETITION_FATAL_TRACK,
                                EXEC_ERROR_SUBJECT2_TRACK, update);
        return;
    }
    task_start_target_count = task_start_box_count;
    map_scan_stats(context->snapshot, &stats);
    *context->start_row = stats.car_row;
    *context->start_col = stats.car_col;
    executor_start(context->result->waypoints,
                   context->result->waypoint_count,
                   stats.car_row,
                   stats.car_col,
                   current_pose_offset_x_cm,
                   current_pose_offset_y_cm,
                   (RUN_MODE_STEP == context->run_mode) ? 1u : 0u,
                   1u);
    subject2_reset_recovery();
    subject2_state = SUBJECT2_EXECUTE_PUSH;
    if(0 != update)
    {
        update->run_state = "S2Push";
        update->redraw = 1u;
    }
}

static void subject2_begin_pre_push_center(subject2_update_struct *update)
{
    uint8 box_row;
    uint8 box_col;

    pre_push_box_request_active = 0u;
    pre_push_box_preparation_started = 0u;
    pre_push_box_retry_count = 0u;
    pre_push_box_retry_moving = 0u;
    pre_push_box_retry_settling = 0u;
    pre_push_box_retry_settle_start_ms = 0u;
    pre_push_center_start_ms = time_ms();
    pre_push_request_frame = openart_uart_get_frame_count();
    subject2_state = SUBJECT2_PRE_PUSH_CENTER;
    if(0 != executor_get_pre_push_box_request(&box_row, &box_col))
    {
        art_box_observation_session_request(&pre_push_box_session,
                                            box_row, box_col);
        pre_push_box_request_active = 1u;
    }
    else
    {
        art_center_batch_reset(&center_batch);
        executor_reset_art_player_center_samples();
        openart_request_player_center();
    }
    if(0 != update)
    {
        update->run_state = (0u != pre_push_box_request_active) ? "BCtr" : "PCtr";
        update->redraw = 1u;
    }
}

static void subject2_tick_pre_push_box(subject2_update_struct *update)
{
    executor_art_box_prep_result_enum prep_result;

    if(0u != pre_push_box_retry_moving)
    {
        if(0u != executor_pre_push_box_preparation_active())
        {
            if(0 != update) update->run_state = "BRetry";
            return;
        }
        if(EXEC_STATE_ERROR == executor_get_state())
        {
            if(EXEC_ERROR_ART_CENTER == executor_get_error())
            {
                subject2_begin_recovery(SUBJECT2_RECOVERY_OBSERVE,
                                        COMPETITION_FATAL_ART1,
                                        EXEC_ERROR_ART_CENTER, update);
            }
            else
            {
                subject2_fail(executor_get_error(), "E:BTim", update);
            }
            return;
        }
        if(0u == pre_push_box_retry_settling)
        {
            pre_push_box_retry_settling = 1u;
            pre_push_box_retry_settle_start_ms = time_ms();
        }
        if((time_ms() - pre_push_box_retry_settle_start_ms) <
           ART_BOX_OBSERVE_RETRY_SETTLE_MS)
        {
            if(0 != update) update->run_state = "BSet";
            return;
        }

        pre_push_box_retry_moving = 0u;
        pre_push_box_retry_settling = 0u;
        art_box_observation_session_request(&pre_push_box_session,
                                            pre_push_box_session.box_row,
                                            pre_push_box_session.box_col);
        pre_push_center_start_ms = time_ms();
        if(0 != update)
        {
            update->run_state = "BCtr";
            update->redraw = 1u;
        }
        return;
    }

    if(0u != pre_push_box_preparation_started)
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
            if(EXEC_ERROR_ART_CENTER == executor_get_error())
            {
                subject2_begin_recovery(SUBJECT2_RECOVERY_OBSERVE,
                                        COMPETITION_FATAL_ART1,
                                        EXEC_ERROR_ART_CENTER, update);
            }
            else
            {
                subject2_fail(executor_get_error(), "E:BTim", update);
            }
            return;
        }

        pre_push_center_start_ms = 0u;
        pre_push_box_request_active = 0u;
        pre_push_box_preparation_started = 0u;
        subject2_state = SUBJECT2_EXECUTE_PUSH;
        if(0 != update)
        {
            update->run_state = "S2Push";
            update->redraw = 1u;
        }
        return;
    }

    if(0u == art_box_observation_session_collect(&pre_push_box_session))
    {
        if((time_ms() - pre_push_center_start_ms) >= ART_BOX_OBSERVE_WAIT_MS)
        {
            if((0u != subject2_fresh_grid_is_usable(pre_push_request_frame,
                                                     0u, 1u, 1u)) &&
               (0u != executor_continue_after_pre_push_center()))
            {
                art_box_observation_session_reset(&pre_push_box_session);
                pre_push_center_start_ms = 0u;
                pre_push_box_request_active = 0u;
                subject2_state = SUBJECT2_EXECUTE_PUSH;
                if(0 != update)
                {
                    update->run_state = "GridPush";
                    update->redraw = 1u;
                }
            }
            else
            {
                subject2_begin_recovery(SUBJECT2_RECOVERY_OBSERVE,
                                        COMPETITION_FATAL_ART1,
                                        EXEC_ERROR_ART_CENTER, update);
            }
        }
        else if(0 != update)
        {
            update->run_state = "BCtr";
        }
        return;
    }

    prep_result = executor_start_pre_push_box_preparation();
    if(EXEC_ART_BOX_PREP_GEOMETRY_ERROR == prep_result)
    {
        art_box_observation_session_reset(&pre_push_box_session);
        subject2_begin_recovery(SUBJECT2_RECOVERY_OBSERVE,
                                COMPETITION_FATAL_ART1,
                                EXEC_ERROR_ART_CENTER, update);
        return;
    }
    if(EXEC_ART_BOX_PREP_STARTED != prep_result)
    {
        art_box_observation_session_reset(&pre_push_box_session);
        if(0 == executor_continue_after_pre_push_center())
        {
            subject2_fail(EXEC_ERROR_ART_CENTER, "E:CPsh", update);
            return;
        }
        pre_push_center_start_ms = 0u;
        pre_push_box_request_active = 0u;
        pre_push_box_preparation_started = 0u;
        subject2_state = SUBJECT2_EXECUTE_PUSH;
        if(0 != update)
        {
            update->run_state = "S2Push";
            update->redraw = 1u;
        }
        return;
    }

    art_box_observation_session_reset(&pre_push_box_session);
    pre_push_box_preparation_started = 1u;
    if(0 != update)
    {
        update->run_state = executor_pre_push_box_state_name();
        update->redraw = 1u;
    }
}

static void subject2_tick_pre_push_center(const subject2_context_struct *context,
                                          subject2_update_struct *update)
{
    const map_source_struct *source;
    executor_art_center_result_enum result;
    uint8 ready = 0u;
    uint8 car_row;
    uint8 car_col;
    uint16 center_col_q;
    uint16 center_row_q;

    if(0u != pre_push_box_request_active)
    {
        subject2_tick_pre_push_box(update);
        return;
    }

    if(0u != art_center_batch_collect(&center_batch))
    {
        ready = art_center_batch_apply_to_executor(&center_batch);
    }
    if(0u == ready)
    {
        if((time_ms() - pre_push_center_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
        {
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
            if((0u == subject2_fresh_grid_is_usable(pre_push_request_frame,
                                                     0u, 1u, 0u)) ||
               (0 == executor_continue_after_pre_push_center()))
            {
                subject2_begin_recovery(SUBJECT2_RECOVERY_CENTER,
                                        COMPETITION_FATAL_ART1,
                                        EXEC_ERROR_ART_CENTER, update);
                return;
            }
            pre_push_center_start_ms = 0u;
            subject2_state = SUBJECT2_EXECUTE_PUSH;
            if(0 != update)
            {
                update->run_state = "CtrSkip";
                update->redraw = 1u;
            }
#else
            subject2_fail(EXEC_ERROR_ART_CENTER, "E:CPsh", update);
#endif
            return;
        }
        if(0 != update) update->run_state = "PCtr";
        return;
    }
    source = openart_get_requested_center_map();
    if((0 == art_center_batch_get_median(&center_batch,
                                         &center_col_q, &center_row_q)) ||
       (0 == map_validate_player_center(source,
                                        center_col_q, center_row_q,
                                        &car_row, &car_col)))
    {
        subject2_retry_center_request("PCtr", update);
        return;
    }
    if(0 == subject2_map_objects_unchanged(source))
    {
        subject2_begin_confirm_map(SUBJECT2_CONFIRM_STRICT, update);
        return;
    }
    result = executor_commit_art_player_center(car_row, car_col);
    if((EXEC_ART_CENTER_APPLIED != result) &&
       (EXEC_ART_CENTER_IGNORED != result))
    {
        if((EXEC_ART_CENTER_REJECTED == result) ||
           (EXEC_ART_CENTER_ABNORMAL == result))
        {
            subject2_accept_confirmed_map(context, source, update);
            return;
        }
        subject2_fail(EXEC_ERROR_ART_CENTER, "E:CPsh", update);
        return;
    }
    if(0 != executor_center_requires_push_alignment())
    {
        if(0 == executor_start_pre_push_alignment(car_row, car_col))
        {
            subject2_fail(EXEC_ERROR_ART_CENTER, "E:CPsh", update);
            return;
        }
    }
    else if(0 == executor_continue_after_pre_push_center())
    {
        subject2_fail(EXEC_ERROR_ART_CENTER, "E:CPsh", update);
        return;
    }
    pre_push_center_start_ms = 0u;
    subject2_state = SUBJECT2_EXECUTE_PUSH;
    if(0 != update)
    {
        update->run_state = "S2Push";
        update->redraw = 1u;
    }
}

static void subject2_begin_confirm_map(subject2_confirm_mode_enum mode,
                                       subject2_update_struct *update)
{
    uint32 frame_count = openart_uart_get_frame_count();

    art_box_observation_session_reset(&pre_push_box_session);
    map_stability_tracker_reset(&confirm_tracker, frame_count);
    confirm_wait_start_ms = time_ms();
    confirm_start_frame = frame_count;
    confirm_mode = mode;
    confirm_retry_count = 0u;
    confirm_map_seen = 0u;
    if(SUBJECT2_CONFIRM_HOST_CHANGE != mode)
    {
        host_path_preserved = 0u;
    }
    subject2_state = SUBJECT2_CONFIRM_MAP;
    if(0 != update)
    {
        update->run_state = "ART Wait";
        update->redraw = 1u;
    }
}

static const map_source_struct *subject2_effective_map(
    const subject2_context_struct *context,
    const map_source_struct *source,
    uint8 allow_infer)
{
    map_scan_stats_struct stats;
    uint8 overlap_row;
    uint8 overlap_col;

    if((0 == context) || (0 == source))
    {
        return source;
    }
    map_scan_stats(source, &stats);
    if(0u != transit_overlap_valid)
    {
        overlap_row = map_cell_row(transit_overlap_cell);
        overlap_col = map_cell_col(transit_overlap_cell);
        if(('T' == source->rows[overlap_row][overlap_col]) &&
           ((uint8)(stats.box_count + 1u) == stats.target_count))
        {
            map_source_snapshot(&transit_overlap_source,
                                transit_overlap_rows, source);
            transit_overlap_rows[overlap_row][overlap_col] =
                MAP_BOX_ON_TARGET;
            return &transit_overlap_source;
        }
        if((stats.box_count == stats.target_count) ||
           (stats.target_count < task_start_target_count))
        {
            transit_overlap_valid = 0u;
            transit_overlap_cell = INVALID_STATE;
        }
    }
    if((0u == allow_infer) ||
       ((uint8)(stats.box_count + 1u) != task_start_box_count) ||
       (stats.target_count != task_start_target_count))
    {
        return source;
    }
    if(0u == subject2_normalize_transit_box_overlap(
                  source, context->result,
                  executor_get_current_step(),
                  active_box_cell, active_target_cell,
                  executor_get_art_sync_action(),
                  transit_overlap_rows, &transit_overlap_source,
                  &transit_overlap_cell))
    {
        return source;
    }
    transit_overlap_valid = 1u;
    return &transit_overlap_source;
}

static uint8 subject2_handle_host_completion(subject2_update_struct *update)
{
    const map_source_struct *source = openart_map_get();
    map_scan_stats_struct stats;
    uint32 frame_count = openart_uart_get_frame_count();
    uint8 box_reduction;
    uint8 target_reduction;

    if(0u != host_completion_waiting_boundary)
    {
        if(0u != executor_stop_after_current_push_ready())
        {
            host_completion_waiting_boundary = 0u;
            host_completion_start_ms = 0u;
            host_path_preserved = 1u;
            subject2_begin_confirm_map(SUBJECT2_CONFIRM_HOST_CHANGE, update);
            if(0 != update)
            {
                update->run_state = "Host Sync";
                update->redraw = 1u;
            }
        }
        else if(EXEC_STATE_RUNNING != executor_get_state())
        {
            host_completion_waiting_boundary = 0u;
            host_completion_start_ms = 0u;
            return 0u;
        }
        else if((time_ms() - host_completion_start_ms) >=
                RECOVERY_RESYNC_TIMEOUT_MS)
        {
            if(0u == executor_force_current_push_stop())
            {
                subject2_begin_recovery(SUBJECT2_RECOVERY_MOTION,
                                        COMPETITION_FATAL_DRIVE,
                                        EXEC_ERROR_SUBJECT2_PLAN, update);
                return 1u;
            }
            host_completion_waiting_boundary = 0u;
            host_completion_start_ms = 0u;
            host_path_preserved = 1u;
            subject2_begin_confirm_map(SUBJECT2_CONFIRM_HOST_CHANGE, update);
            if(0 != update)
            {
                update->run_state = "Host Sync";
                update->redraw = 1u;
            }
        }
        else if(0 != update)
        {
            update->run_state = "Host Pend";
            update->redraw = 1u;
        }
        return 1u;
    }

    if((0u == frame_count) || (frame_count == host_monitor_last_frame))
    {
        return 0u;
    }
    host_monitor_last_frame = frame_count;

    map_scan_stats(source, &stats);
    if((1u != stats.car_count) ||
       (stats.box_count >= task_start_box_count) ||
       (stats.target_count >= task_start_target_count))
    {
        return 0u;
    }

    box_reduction = (uint8)(task_start_box_count - stats.box_count);
    target_reduction = (uint8)(task_start_target_count - stats.target_count);
    if(box_reduction != target_reduction)
    {
        return 0u;
    }

    if(0u != executor_request_stop_after_current_push())
    {
        host_completion_waiting_boundary = 1u;
        host_completion_start_ms = time_ms();
        if(0 != update)
        {
            update->run_state = "Host Pend";
            update->redraw = 1u;
        }
        return 1u;
    }

    if(0u != executor_art_sync_pending())
    {
        subject2_begin_confirm_map(SUBJECT2_CONFIRM_TASK_END, update);
    }
    else
    {
        executor_stop();
        subject2_begin_confirm_map(SUBJECT2_CONFIRM_STRICT, update);
    }
    return 1u;
}

static void subject2_begin_replan_center(uint8 for_return,
                                         subject2_update_struct *update)
{
    replan_center_for_return = for_return;
    art_center_batch_reset(&center_batch);
    center_request_start_ms = time_ms();
    openart_request_player_center();
    subject2_state = SUBJECT2_REPLAN_CENTER;
    if(0 != update)
    {
        update->run_state = for_return ? "S2Ret" : "RCtr";
        update->redraw = 1u;
    }
}

static void subject2_accept_confirmed_map(const subject2_context_struct *context,
                                          const map_source_struct *source,
                                          subject2_update_struct *update)
{
    map_scan_stats_struct stats;
    subject2_object_struct next_boxes[MAX_BOXES];
    subject2_object_struct next_targets[MAX_BOXES];
    subject2_sync_update_struct sync_update;
    subject2_sync_result_enum sync_result;
    uint8 next_box_count = box_object_count;
    uint8 next_target_count = target_object_count;

    host_path_preserved = 0u;
    confirm_mode = SUBJECT2_CONFIRM_STRICT;
    confirm_retry_count = 0u;
    confirm_map_seen = 0u;
    map_scan_stats(source, &stats);
    if(1u != stats.car_count)
    {
        subject2_begin_recovery(SUBJECT2_RECOVERY_TRACK,
                                COMPETITION_FATAL_TRACK,
                                EXEC_ERROR_SUBJECT2_TRACK, update);
        return;
    }
    memcpy(next_boxes, box_objects, sizeof(next_boxes));
    memcpy(next_targets, target_objects, sizeof(next_targets));
    sync_result = subject2_reconcile_object_lists(
        source,
        next_boxes, &next_box_count,
        next_targets, &next_target_count,
        active_box_cell, active_target_cell, &sync_update);
    if((SUBJECT2_SYNC_AMBIGUOUS == sync_result) ||
       ((stats.box_count > task_start_box_count) ||
        (stats.target_count > task_start_target_count)))
    {
        subject2_begin_recovery(SUBJECT2_RECOVERY_TRACK,
                                COMPETITION_FATAL_TRACK,
                                EXEC_ERROR_SUBJECT2_TRACK, update);
        return;
    }

    memcpy(box_objects, next_boxes, sizeof(box_objects));
    memcpy(target_objects, next_targets, sizeof(target_objects));
    box_object_count = next_box_count;
    target_object_count = next_target_count;
    active_box_valid = sync_update.active_box_valid;
    active_box_cell = sync_update.active_box_cell;
    if((0u != sync_update.active_target_removed) ||
       (stats.box_count < task_start_box_count) ||
       (stats.target_count < task_start_target_count))
    {
        retry_active_only = 0u;
        active_target_cell = INVALID_STATE;
    }
    else
    {
        retry_active_only = 1u;
    }
    map_source_snapshot(context->snapshot, context->snapshot_rows, source);
    *context->snapshot_valid = 1u;
    if(SUBJECT2_RECOVERY_PLAN != recovery_reason)
    {
        subject2_reset_recovery();
    }
    executor_stop();
    if(SUBJECT2_SYNC_RESCAN == sync_result)
    {
        subject2_begin_scan_map_sync(update);
        return;
    }
    subject2_begin_replan_center((0u == stats.box_count) ? 1u : 0u, update);
}

static uint8 subject2_complete_active_task_locally(
    const subject2_context_struct *context,
    subject2_update_struct *update)
{
    char predicted_rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct predicted_source;
    subject2_object_struct next_boxes[MAX_BOXES];
    subject2_object_struct next_targets[MAX_BOXES];
    subject2_sync_update_struct sync_update;
    subject2_sync_result_enum sync_result;
    map_scan_stats_struct stats;
    const waypoint_struct *last_waypoint;
    uint8 next_box_count = box_object_count;
    uint8 next_target_count = target_object_count;
    uint8 old_car_row;
    uint8 old_car_col;
    uint8 box_row;
    uint8 box_col;
    uint8 target_row;
    uint8 target_col;
    char value;

    if((0 == context) || (0 == context->snapshot) ||
       (0 == context->snapshot_rows) || (0 == context->snapshot_valid) ||
       (0 == context->result) || (0 == context->start_row) ||
       (0 == context->start_col) || (0u == *context->snapshot_valid) ||
       (0u == context->result->waypoint_count) ||
       (MAP_CELLS <= active_box_cell) ||
       (MAP_CELLS <= active_target_cell))
    {
        return 0u;
    }

    last_waypoint = &context->result->waypoints[
        context->result->waypoint_count - 1u];
    if((last_waypoint->row >= MAP_ROWS) ||
       (last_waypoint->col >= MAP_COLS))
    {
        return 0u;
    }

    map_source_snapshot(&predicted_source, predicted_rows, context->snapshot);
    if(0 == map_find_car(&predicted_source, &old_car_row, &old_car_col, 0))
    {
        return 0u;
    }
    predicted_rows[old_car_row][old_car_col] =
        ('+' == predicted_rows[old_car_row][old_car_col]) ? 'T' : '.';

    box_row = map_cell_row(active_box_cell);
    box_col = map_cell_col(active_box_cell);
    target_row = map_cell_row(active_target_cell);
    target_col = map_cell_col(active_target_cell);
    value = predicted_rows[box_row][box_col];
    if(('B' != value) && (MAP_BOX_ON_TARGET != value))
    {
        return 0u;
    }
    predicted_rows[box_row][box_col] =
        (MAP_BOX_ON_TARGET == value) ? 'T' : '.';

    if(active_target_cell == active_box_cell)
    {
        predicted_rows[target_row][target_col] = '.';
    }
    else
    {
        value = predicted_rows[target_row][target_col];
        if(('T' != value) && ('+' != value) &&
           (MAP_BOX_ON_TARGET != value))
        {
            return 0u;
        }
        predicted_rows[target_row][target_col] =
            ('+' == value) ? 'C' : '.';
    }

    value = predicted_rows[last_waypoint->row][last_waypoint->col];
    if(('T' == value) || ('+' == value))
    {
        predicted_rows[last_waypoint->row][last_waypoint->col] = '+';
    }
    else if(('.' == value) || ('C' == value))
    {
        predicted_rows[last_waypoint->row][last_waypoint->col] = 'C';
    }
    else
    {
        return 0u;
    }

    map_scan_stats(&predicted_source, &stats);
    if((1u != stats.car_count) ||
       (stats.box_count != stats.target_count) ||
       ((uint8)(stats.box_count + 1u) != task_start_box_count) ||
       ((uint8)(stats.target_count + 1u) != task_start_target_count))
    {
        return 0u;
    }

    memcpy(next_boxes, box_objects, sizeof(next_boxes));
    memcpy(next_targets, target_objects, sizeof(next_targets));
    sync_result = subject2_reconcile_object_lists(
        &predicted_source,
        next_boxes, &next_box_count,
        next_targets, &next_target_count,
        active_box_cell, active_target_cell, &sync_update);
    if((SUBJECT2_SYNC_OK != sync_result) ||
       (0u == sync_update.active_target_removed))
    {
        return 0u;
    }

    memcpy(box_objects, next_boxes, sizeof(box_objects));
    memcpy(target_objects, next_targets, sizeof(target_objects));
    box_object_count = next_box_count;
    target_object_count = next_target_count;
    map_source_snapshot(context->snapshot, context->snapshot_rows,
                        &predicted_source);
    *context->snapshot_valid = 1u;
    *context->start_row = last_waypoint->row;
    *context->start_col = last_waypoint->col;
    navigation_start_row = last_waypoint->row;
    navigation_start_col = last_waypoint->col;
    current_pose_offset_x_cm = 0.0f;
    current_pose_offset_y_cm = 0.0f;
    active_class = SUBJECT2_INVALID_CLASS;
    active_box_valid = 0u;
    active_box_cell = INVALID_STATE;
    active_target_cell = INVALID_STATE;
    retry_active_only = 0u;
    transit_overlap_valid = 0u;
    transit_overlap_cell = INVALID_STATE;
    host_path_preserved = 0u;
    confirm_mode = SUBJECT2_CONFIRM_STRICT;
    confirm_retry_count = 0u;
    confirm_map_seen = 0u;
    executor_stop();
    subject2_reset_recovery();

    if((0u == box_object_count) && (0u == target_object_count))
    {
        subject2_state = SUBJECT2_RETURN_REQUESTED;
        if(0 != update)
        {
            update->return_requested = 1u;
            update->return_pose_x_cm = 0.0f;
            update->return_pose_y_cm = 0.0f;
            update->run_state = "MCU Go";
            update->redraw = 1u;
        }
    }
    else
    {
        subject2_state = SUBJECT2_SELECT_PUSH;
        if(0 != update)
        {
            update->run_state = "MCU Go";
            update->redraw = 1u;
        }
    }
    return 1u;
}

static void subject2_tick_confirm_map(const subject2_context_struct *context,
                                      subject2_update_struct *update)
{
    const map_source_struct *source;
    map_scan_stats_struct stats;
    uint32 frame_count = openart_uart_get_frame_count();
    uint32 timeout_ms = (SUBJECT2_CONFIRM_STRICT == confirm_mode) ?
                        EXEC_ART_SYNC_TIMEOUT_MS :
                        RECOVERY_RESYNC_TIMEOUT_MS;
    map_stability_result_enum stability;

    source = openart_map_get();
    if(0 != source)
    {
        source = subject2_effective_map(context, source, 1u);
        map_scan_stats(source, &stats);
        /* 双 C 或数量变化仍是上位机发布了新场景的证据，不能盲续；
         * 没有任何车证据的空帧则不应阻塞 MCU 路线兜底。 */
        if((frame_count != confirm_start_frame) &&
           (0u != stats.car_count))
        {
            confirm_map_seen = 1u;
        }
        if((1u != stats.car_count) ||
           (stats.box_count != stats.target_count) ||
           (stats.box_count > task_start_box_count) ||
           (stats.target_count > task_start_target_count))
        {
            map_stability_tracker_reset_candidate(&confirm_tracker);
        }
        else
        {
            stability = map_stability_tracker_push(
                &confirm_tracker, frame_count, source,
                EXEC_ART_STABLE_FRAMES);
            if(MAP_STABILITY_READY == stability)
            {
                subject2_accept_confirmed_map(context, source, update);
                return;
            }
        }
    }

    if((time_ms() - confirm_wait_start_ms) < timeout_ms)
    {
        if(0 != update) update->run_state = "ART Wait";
        return;
    }

    if((SUBJECT2_CONFIRM_STRICT != confirm_mode) &&
       (0u == confirm_retry_count))
    {
        confirm_retry_count = 1u;
        confirm_wait_start_ms = time_ms();
        confirm_start_frame = frame_count;
        map_stability_tracker_reset(&confirm_tracker, frame_count);
        if(0 != update)
        {
            update->run_state = "ART Retry";
            update->redraw = 1u;
        }
        return;
    }

    if((0u == confirm_map_seen) &&
       (SUBJECT2_CONFIRM_HOST_CHANGE == confirm_mode) &&
       (0u != host_path_preserved) &&
       (0u != executor_resume_after_current_push_stop()))
    {
        host_path_preserved = 0u;
        confirm_mode = SUBJECT2_CONFIRM_STRICT;
        confirm_retry_count = 0u;
        subject2_state = SUBJECT2_EXECUTE_PUSH;
        if(0 != update)
        {
            update->run_state = "MCU Go";
            update->redraw = 1u;
        }
        return;
    }
    if((0u == confirm_map_seen) &&
       (SUBJECT2_CONFIRM_TASK_END == confirm_mode) &&
       (0u != subject2_complete_active_task_locally(context, update)))
    {
        return;
    }

    host_path_preserved = 0u;
    confirm_mode = SUBJECT2_CONFIRM_STRICT;
    confirm_retry_count = 0u;
    subject2_begin_recovery(SUBJECT2_RECOVERY_MAP,
                            COMPETITION_FATAL_ART1,
                            EXEC_ERROR_ART_TIMEOUT, update);
}

static void subject2_finish_scan_map_sync(
    const subject2_context_struct *context,
    const map_source_struct *source,
    subject2_update_struct *update)
{
    subject2_object_struct next_boxes[MAX_BOXES];
    subject2_object_struct next_targets[MAX_BOXES];
    subject2_sync_update_struct sync_update;
    subject2_sync_result_enum sync_result;
    uint8 next_box_count = box_object_count;
    uint8 next_target_count = target_object_count;

    memcpy(next_boxes, box_objects, sizeof(next_boxes));
    memcpy(next_targets, target_objects, sizeof(next_targets));
    sync_result = subject2_reconcile_object_lists(
        source,
        next_boxes, &next_box_count,
        next_targets, &next_target_count,
        active_box_cell, active_target_cell, &sync_update);
    if(SUBJECT2_SYNC_AMBIGUOUS == sync_result)
    {
        subject2_begin_recovery(SUBJECT2_RECOVERY_TRACK,
                                COMPETITION_FATAL_TRACK,
                                EXEC_ERROR_SUBJECT2_TRACK, update);
        return;
    }
    if(0 == subject2_apply_center(context, source, 0u, 0, 0))
    {
        subject2_begin_recovery(SUBJECT2_RECOVERY_CENTER,
                                COMPETITION_FATAL_ART1,
                                EXEC_ERROR_ART_CENTER, update);
        return;
    }
    if(SUBJECT2_RECOVERY_PLAN != recovery_reason)
    {
        subject2_reset_recovery();
    }
    scan_plan_snapshot_pending = 1u;

    memcpy(box_objects, next_boxes, sizeof(box_objects));
    memcpy(target_objects, next_targets, sizeof(target_objects));
    box_object_count = next_box_count;
    target_object_count = next_target_count;
    active_box_valid = sync_update.active_box_valid;
    active_box_cell = sync_update.active_box_cell;
    if(0u != sync_update.active_target_removed)
    {
        retry_active_only = 0u;
        active_target_cell = INVALID_STATE;
    }

    if((0u == box_object_count) && (0u == target_object_count))
    {
        subject2_state = SUBJECT2_RETURN_REQUESTED;
        if(0 != update)
        {
            update->return_requested = 1u;
            update->return_pose_x_cm = current_pose_offset_x_cm;
            update->return_pose_y_cm = current_pose_offset_y_cm;
            update->run_state = "S2Ret";
            update->redraw = 1u;
        }
        return;
    }
    if(0u != sync_update.need_box_scan)
    {
        subject2_begin_vision_mode(VISION_MODE_BOX, SUBJECT2_SCAN_BOX_MODE);
        if(0 != update)
        {
            update->run_state = "BScan";
            update->redraw = 1u;
        }
        return;
    }
    if(0u != sync_update.need_target_scan)
    {
        subject2_begin_vision_mode(VISION_MODE_TARGET, SUBJECT2_SCAN_TARGET_MODE);
        if(0 != update)
        {
            update->run_state = "TScan";
            update->redraw = 1u;
        }
        return;
    }

    subject2_state = SUBJECT2_VALIDATE_BINDINGS;
    if(0 != update)
    {
        update->run_state = "Bind";
        update->redraw = 1u;
    }
}

static void subject2_tick_scan_map_sync(const subject2_context_struct *context,
                                        subject2_update_struct *update)
{
    const map_source_struct *source;
    map_scan_stats_struct stats;
    uint32 frame_count;
    uint16 center_col_q;
    uint16 center_row_q;
    uint8 car_row;
    uint8 car_col;
    map_stability_result_enum stability;

    if(SUBJECT2_SCAN_SYNC_WAIT_MAP == scan_sync_phase)
    {
        if((time_ms() - center_request_start_ms) >= RECOVERY_RESYNC_TIMEOUT_MS)
        {
            subject2_begin_recovery(
                (SUBJECT2_RECOVERY_NONE == recovery_reason) ?
                SUBJECT2_RECOVERY_MAP : recovery_reason,
                subject2_recovery_fatal_reason(),
                subject2_recovery_final_error(), update);
            return;
        }
        frame_count = openart_uart_get_frame_count();
        source = openart_map_get();
        if(0 == source)
        {
            return;
        }
        source = subject2_effective_map(context, source, 0u);
        map_scan_stats(source, &stats);
        if((1u != stats.car_count) || (stats.box_count != stats.target_count))
        {
            map_stability_tracker_reset_candidate(&confirm_tracker);
            if(0 != update) update->run_state = "VSync";
            return;
        }
        stability = map_stability_tracker_push(&confirm_tracker, frame_count,
                                               source, EXEC_ART_STABLE_FRAMES);
        if(MAP_STABILITY_READY != stability)
        {
            if(0 != update) update->run_state = "VSync";
            return;
        }
        map_source_snapshot(&scan_sync_source, scan_sync_rows, source);
        scan_sync_source_valid = 1u;
        art_center_batch_reset(&center_batch);
        openart_request_player_center();
        scan_sync_phase = SUBJECT2_SCAN_SYNC_WAIT_CENTER;
        if(0 != update)
        {
            update->run_state = "VSync";
        }
        return;
    }

    if(0 == subject2_collect_center())
    {
        if((time_ms() - center_request_start_ms) >= RECOVERY_RESYNC_TIMEOUT_MS)
        {
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
            uint8 index;

            source = (0u != scan_sync_source_valid) ? &scan_sync_source : 0;
            if(0 == source)
            {
                subject2_fail(EXEC_ERROR_ART_CENTER, "E:CTmo", update);
                return;
            }
            map_scan_stats(source, &stats);
            if(1u != stats.car_count)
            {
                subject2_fail(EXEC_ERROR_ART_CENTER, "E:CTmo", update);
                return;
            }
            center_batch.count = ART_CENTER_SAMPLE_COUNT;
            for(index = 0u; index < ART_CENTER_SAMPLE_COUNT; index++)
            {
                center_batch.col_q[index] = (uint16)(stats.car_col * 100u + 50u);
                center_batch.row_q[index] = (uint16)(stats.car_row * 100u + 50u);
            }
            subject2_finish_scan_map_sync(context, source, update);
#else
            subject2_fail(EXEC_ERROR_ART_CENTER, "E:CTmo", update);
#endif
        }
        else if(0 != update)
        {
            update->run_state = "VSync";
        }
        return;
    }

    (void)subject2_get_center_median(&center_col_q, &center_row_q);
    source = openart_get_requested_center_map();
    source = subject2_effective_map(context, source, 0u);
    if(0 == map_validate_player_center(source,
                                       center_col_q, center_row_q,
                                       &car_row, &car_col))
    {
        subject2_retry_center_request("VSync", update);
        return;
    }
    subject2_finish_scan_map_sync(context, source, update);
}

static void subject2_tick_replan_center(const subject2_context_struct *context,
                                        subject2_update_struct *update)
{
    const map_source_struct *source;
    uint16 center_col_q;
    uint16 center_row_q;
    uint8 car_row;
    uint8 car_col;

    if(0 == subject2_collect_center())
    {
        if((time_ms() - center_request_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
        {
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
            if(0 == subject2_apply_map_cell_center(context))
            {
                subject2_fail(EXEC_ERROR_ART_CENTER, "E:CRpl", update);
                return;
            }
#else
            subject2_fail(EXEC_ERROR_ART_CENTER, "E:CRpl", update);
            return;
#endif
        }
        else
        {
            if(0 != update) update->run_state = replan_center_for_return ? "S2Ret" : "RCtr";
            return;
        }
    }
    else
    {
        (void)subject2_get_center_median(&center_col_q, &center_row_q);
        source = openart_get_requested_center_map();
        source = subject2_effective_map(context, source, 0u);
        if(0 == map_validate_player_center(source,
                                           center_col_q, center_row_q,
                                           &car_row, &car_col))
        {
            subject2_retry_center_request(
                replan_center_for_return ? "S2Ret" : "RCtr", update);
            return;
        }
        if(0 == subject2_map_objects_unchanged(source))
        {
            subject2_begin_confirm_map(SUBJECT2_CONFIRM_STRICT, update);
            return;
        }
        if(0 == subject2_apply_center(context, source, 0u, 0, 0))
        {
            subject2_fail(EXEC_ERROR_ART_CENTER, "E:CRpl", update);
            return;
        }
    }
    if(0u != replan_center_for_return)
    {
        subject2_state = SUBJECT2_RETURN_REQUESTED;
        if(0 != update)
        {
            update->return_requested = 1u;
            update->return_pose_x_cm = current_pose_offset_x_cm;
            update->return_pose_y_cm = current_pose_offset_y_cm;
            update->run_state = "S2Ret";
            update->redraw = 1u;
        }
    }
    else
    {
        subject2_state = SUBJECT2_SELECT_PUSH;
        if(0 != update)
        {
            update->run_state = "Bind";
            update->redraw = 1u;
        }
    }
}

static void subject2_tick_execute_push(subject2_update_struct *update)
{
    if(0 != executor_art_pre_push_pending())
    {
        subject2_begin_pre_push_center(update);
    }
    else if(0 != executor_art_sync_pending())
    {
        subject2_begin_confirm_map(SUBJECT2_CONFIRM_TASK_END, update);
    }
    else if(EXEC_STATE_ERROR == executor_get_state())
    {
        if(EXEC_ERROR_ART_CENTER == executor_get_error())
        {
            subject2_begin_recovery(SUBJECT2_RECOVERY_OBSERVE,
                                    COMPETITION_FATAL_ART1,
                                    EXEC_ERROR_ART_CENTER, update);
        }
        else
        {
            subject2_fail(EXEC_ERROR_SUBJECT2_PLAN, "E:Plan", update);
        }
    }
    else if(0 != update)
    {
        update->run_state = "S2Push";
    }
}

void subject2_begin(const subject2_context_struct *context,
                    float initial_pose_x_cm,
                    float initial_pose_y_cm,
                    subject2_update_struct *update)
{
    map_scan_stats_struct stats;

    subject2_update_reset(update);
    subject2_cancel();
    if((0 == context) || (0 == context->snapshot) ||
       (0 == context->snapshot_rows) || (0 == context->snapshot_valid) ||
       (0 == context->result) || (0 == context->start_row) ||
       (0 == context->start_col) || (0u == *context->snapshot_valid) ||
       (0 == subject2_collect_objects(context->snapshot, 'B',
                                      box_objects, &box_object_count)) ||
       (0 == subject2_collect_objects(context->snapshot, 'T',
                                      target_objects, &target_object_count)) ||
       (box_object_count != target_object_count))
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:Map", update);
        return;
    }

    subject2_classifier_reset(&classifier);
    current_pose_offset_x_cm = initial_pose_x_cm;
    current_pose_offset_y_cm = initial_pose_y_cm;
    validation_retry_count = 0u;
    active_class = SUBJECT2_INVALID_CLASS;
    active_box_valid = 0u;
    active_box_cell = INVALID_STATE;
    active_target_cell = INVALID_STATE;
    last_recognition_valid = 0u;
    last_recognition_is_target = 0u;
    last_recognition_class = SUBJECT2_INVALID_CLASS;
    retry_active_only = 0u;
    transit_overlap_valid = 0u;
    transit_overlap_cell = INVALID_STATE;
    replan_center_for_return = 0u;
    launch_yaw_deg = context->launch_yaw_deg;
    post_observe_art_yaw_bias_valid = context->art_yaw_bias_valid;
    post_observe_art_yaw_bias_deg = context->art_yaw_bias_deg;
    post_observe_yaw_done = 0u;
    post_observe_yaw_start_ms = 0u;
    map_stability_tracker_reset(&confirm_tracker, 0u);
    confirm_wait_start_ms = 0u;
    scan_sync_phase = SUBJECT2_SCAN_SYNC_WAIT_MAP;
    scan_plan_snapshot_pending = 0u;
    map_scan_stats(context->snapshot, &stats);
    navigation_start_row = stats.car_row;
    navigation_start_col = stats.car_col;
    subject2_begin_vision_mode(VISION_MODE_BOX, SUBJECT2_SCAN_BOX_MODE);
    if(0 != update)
    {
        update->run_state = "BScan";
        update->enter_execute = 1u;
        update->redraw = 1u;
    }
}

void subject2_tick(const subject2_context_struct *context,
                   subject2_update_struct *update)
{
    subject2_update_reset(update);
    if((0 == context) || (SUBJECT2_IDLE == subject2_state) ||
       (SUBJECT2_ERROR == subject2_state) || (SUBJECT2_DONE == subject2_state))
    {
        return;
    }

    if(((SUBJECT2_EXECUTE_PUSH == subject2_state) ||
        (SUBJECT2_PRE_PUSH_CENTER == subject2_state)) &&
       (0 != subject2_handle_host_completion(update)))
    {
        return;
    }

    switch(subject2_state)
    {
        case SUBJECT2_SCAN_BOX_MODE:
        case SUBJECT2_SCAN_TARGET_MODE:
            subject2_tick_vision_mode(update);
            break;
        case SUBJECT2_SCAN_BOX_PLAN:
        case SUBJECT2_SCAN_TARGET_PLAN:
            subject2_tick_plan(context, update);
            break;
        case SUBJECT2_SCAN_BOX_MOVE:
        case SUBJECT2_SCAN_TARGET_MOVE:
            subject2_tick_move(update);
            break;
        case SUBJECT2_SCAN_BOX_TURN:
        case SUBJECT2_SCAN_TARGET_TURN:
            subject2_tick_turn(update);
            break;
        case SUBJECT2_SCAN_BOX_CENTER:
        case SUBJECT2_SCAN_TARGET_CENTER:
            subject2_tick_center(context, update);
            break;
        case SUBJECT2_SCAN_BOX_ADJUST:
        case SUBJECT2_SCAN_TARGET_ADJUST:
            subject2_tick_center_adjust(update);
            break;
        case SUBJECT2_SCAN_BOX_CLASSIFY:
        case SUBJECT2_SCAN_TARGET_CLASSIFY:
            subject2_tick_classify(update);
            break;
        case SUBJECT2_SCAN_BOX_BACKOFF:
        case SUBJECT2_SCAN_TARGET_BACKOFF:
            subject2_tick_view_backoff(update);
            break;
        case SUBJECT2_SCAN_BOX_BACKOFF_RETURN:
        case SUBJECT2_SCAN_TARGET_BACKOFF_RETURN:
            subject2_tick_view_return(update);
            break;
        case SUBJECT2_SCAN_MAP_SYNC:
            subject2_tick_scan_map_sync(context, update);
            break;
        case SUBJECT2_VALIDATE_BINDINGS:
            subject2_tick_validate(update);
            break;
        case SUBJECT2_RESTORE_HEADING:
            subject2_tick_restore_heading(update);
            break;
        case SUBJECT2_POST_OBSERVE_YAW_SAMPLE:
            subject2_tick_post_observe_yaw_sample(update);
            break;
        case SUBJECT2_POST_OBSERVE_YAW_FIX:
            subject2_tick_post_observe_yaw_fix(update);
            break;
        case SUBJECT2_SELECT_PUSH:
            subject2_tick_select_push(context, update);
            break;
        case SUBJECT2_EXECUTE_PUSH:
            subject2_tick_execute_push(update);
            break;
        case SUBJECT2_PRE_PUSH_CENTER:
            subject2_tick_pre_push_center(context, update);
            break;
        case SUBJECT2_CONFIRM_MAP:
            subject2_tick_confirm_map(context, update);
            break;
        case SUBJECT2_REPLAN_CENTER:
            subject2_tick_replan_center(context, update);
            break;
        case SUBJECT2_WAIT_BLAST:
            if(0 != update)
            {
                update->run_state = "S3Plan";
            }
            break;
        case SUBJECT2_RETURN_REQUESTED:
            if(0 != update)
            {
                update->return_requested = 1u;
                update->return_pose_x_cm = current_pose_offset_x_cm;
                update->return_pose_y_cm = current_pose_offset_y_cm;
                update->run_state = "S2Ret";
            }
            break;
        default:
            break;
    }
}

subject2_block_reason_enum subject2_get_block_reason(void)
{
    return (SUBJECT2_WAIT_BLAST == subject2_state) ?
           blocked_reason : SUBJECT2_BLOCK_NONE;
}

uint8 subject2_retry_blocked_plan(const map_source_struct *source,
                                  solve_result_struct *result)
{
    subject2_observation_plan_struct observation;
    subject2_push_plan_struct push_plan;

    if((SUBJECT2_WAIT_BLAST != subject2_state) ||
       (SUBJECT2_BLOCK_NONE == blocked_reason) ||
       (0 == source) || (0 == result))
    {
        return 0u;
    }
    clear_result(result);
    if(SUBJECT2_BLOCK_OBSERVE_BOX == blocked_reason)
    {
        return subject2_select_observation(
            source, box_objects, box_object_count,
            get_control_status()->current_yaw, &observation, result);
    }
    if(SUBJECT2_BLOCK_OBSERVE_TARGET == blocked_reason)
    {
        return subject2_select_observation(
            source, target_objects, target_object_count,
            get_control_status()->current_yaw, &observation, result);
    }
    if(SUBJECT2_BLOCK_PUSH == blocked_reason)
    {
        return subject2_select_push_plan(
            source, box_objects, box_object_count,
            target_objects, target_object_count,
            retry_active_only, active_box_valid,
            active_box_cell, active_target_cell,
            &push_plan, result);
    }
    return 0u;
}

uint8 subject2_resume_after_blast(const subject2_context_struct *context,
                                  const map_source_struct *source,
                                  float pose_x_cm,
                                  float pose_y_cm,
                                  subject2_update_struct *update)
{
    map_scan_stats_struct stats;

    subject2_update_reset(update);
    if((SUBJECT2_WAIT_BLAST != subject2_state) ||
       (SUBJECT2_BLOCK_NONE == blocked_reason) ||
       (0 == context) || (0 == context->snapshot) ||
       (0 == context->snapshot_rows) || (0 == context->snapshot_valid) ||
       (0 == context->start_row) || (0 == context->start_col) ||
       (0 == source))
    {
        return 0u;
    }
    map_scan_stats(source, &stats);
    if(1u != stats.car_count)
    {
        return 0u;
    }

    map_source_snapshot(context->snapshot, context->snapshot_rows, source);
    *context->snapshot_valid = 1u;
    *context->start_row = stats.car_row;
    *context->start_col = stats.car_col;
    current_pose_offset_x_cm = pose_x_cm;
    current_pose_offset_y_cm = pose_y_cm;
    scan_plan_snapshot_pending = 0u;
    subject2_state = blocked_resume_state;
    blocked_reason = SUBJECT2_BLOCK_NONE;
    blocked_resume_state = SUBJECT2_IDLE;
    if(0 != update)
    {
        update->run_state = "S3Run";
        update->redraw = 1u;
    }
    return 1u;
}

uint8 subject2_refresh_blocked_map(const subject2_context_struct *context,
                                   const map_source_struct *source,
                                   float pose_x_cm,
                                   float pose_y_cm)
{
    map_scan_stats_struct stats;

    if((SUBJECT2_WAIT_BLAST != subject2_state) ||
       (SUBJECT2_BLOCK_NONE == blocked_reason) ||
       (0 == context) || (0 == context->snapshot) ||
       (0 == context->snapshot_rows) || (0 == context->snapshot_valid) ||
       (0 == context->start_row) || (0 == context->start_col) ||
       (0 == source))
    {
        return 0u;
    }
    map_scan_stats(source, &stats);
    if(1u != stats.car_count)
    {
        return 0u;
    }
    map_source_snapshot(context->snapshot, context->snapshot_rows, source);
    *context->snapshot_valid = 1u;
    *context->start_row = stats.car_row;
    *context->start_col = stats.car_col;
    current_pose_offset_x_cm = pose_x_cm;
    current_pose_offset_y_cm = pose_y_cm;
    return 1u;
}

void subject2_get_pose_offset(float *pose_x_cm, float *pose_y_cm)
{
    if(0 != pose_x_cm)
    {
        *pose_x_cm = current_pose_offset_x_cm;
    }
    if(0 != pose_y_cm)
    {
        *pose_y_cm = current_pose_offset_y_cm;
    }
}

void subject2_reject_blast_fallback(subject2_update_struct *update)
{
    subject2_update_reset(update);
    if(SUBJECT2_WAIT_BLAST != subject2_state)
    {
        return;
    }
    subject2_state = blocked_resume_state;
    blocked_reason = SUBJECT2_BLOCK_NONE;
    blocked_resume_state = SUBJECT2_IDLE;
    subject2_begin_recovery(SUBJECT2_RECOVERY_PLAN,
                            COMPETITION_FATAL_PLAN,
                            EXEC_ERROR_SUBJECT2_PLAN, update);
}

uint8 subject2_manual_recover(subject2_update_struct *update)
{
    subject2_update_reset(update);
    if(SUBJECT2_ERROR != subject2_state)
    {
        return 0u;
    }
    executor_stop();
    vision_uart_cancel();
    subject2_reset_recovery();
    subject2_begin_scan_map_sync(update);
    return 1u;
}

void subject2_cancel(void)
{
    subject2_state = SUBJECT2_IDLE;
    host_completion_waiting_boundary = 0u;
    host_completion_start_ms = 0u;
    host_path_preserved = 0u;
    host_monitor_last_frame = 0u;
    box_object_count = 0u;
    target_object_count = 0u;
    current_vision_request_id = 0u;
    classify_start_ms = 0u;
    art_center_batch_reset(&center_batch);
    pre_push_center_start_ms = 0u;
    pre_push_box_request_active = 0u;
    pre_push_box_preparation_started = 0u;
    art_box_observation_session_reset(&pre_push_box_session);
    pre_push_box_retry_count = 0u;
    pre_push_box_retry_moving = 0u;
    pre_push_box_retry_settling = 0u;
    pre_push_box_retry_settle_start_ms = 0u;
    validation_retry_count = 0u;
    active_class = SUBJECT2_INVALID_CLASS;
    active_box_valid = 0u;
    active_box_cell = INVALID_STATE;
    active_target_cell = INVALID_STATE;
    last_recognition_valid = 0u;
    last_recognition_is_target = 0u;
    last_recognition_class = SUBJECT2_INVALID_CLASS;
    retry_active_only = 0u;
    transit_overlap_valid = 0u;
    transit_overlap_cell = INVALID_STATE;
    replan_center_for_return = 0u;
    map_stability_tracker_reset(&confirm_tracker, 0u);
    confirm_wait_start_ms = 0u;
    confirm_mode = SUBJECT2_CONFIRM_STRICT;
    confirm_retry_count = 0u;
    confirm_map_seen = 0u;
    confirm_start_frame = 0u;
    center_request_start_ms = 0u;
    center_adjust_start_ms = 0u;
    scan_fast_center_after_frame = 0u;
    scan_center_request_started = 0u;
    turn_start_ms = 0u;
    turn_stable_start_ms = 0u;
    turn_stable_active = 0u;
    vision_mode_start_ms = 0u;
    vision_mode_last_send_ms = 0u;
    active_vision_mode = VISION_MODE_NONE;
    view_backoff_active = 0u;
    view_backoff_recognition_accepted = 0u;
    view_backoff_origin_x_cm = 0.0f;
    view_backoff_origin_y_cm = 0.0f;
    view_motion_start_ms = 0u;
    launch_yaw_deg = 0.0f;
    post_observe_art_yaw_bias_valid = 0u;
    post_observe_art_yaw_bias_deg = 0.0f;
    post_observe_yaw_done = 0u;
    post_observe_yaw_start_ms = 0u;
    scan_sync_phase = SUBJECT2_SCAN_SYNC_WAIT_MAP;
    scan_plan_snapshot_pending = 0u;
    scan_sync_source_valid = 0u;
    center_request_frame = 0u;
    pre_push_request_frame = 0u;
    blocked_reason = SUBJECT2_BLOCK_NONE;
    blocked_resume_state = SUBJECT2_IDLE;
    subject2_reset_recovery();
}

subject2_state_enum subject2_get_state(void)
{
    return subject2_state;
}

uint8 subject2_get_active_class(void)
{
    return active_class;
}

uint8 subject2_get_last_recognition(uint8 *is_target, uint8 *class_id)
{
    if((0 == is_target) || (0 == class_id))
    {
        return 0u;
    }
    *is_target = 0u;
    *class_id = SUBJECT2_INVALID_CLASS;
    if(0u == last_recognition_valid)
    {
        return 0u;
    }
    *is_target = last_recognition_is_target;
    *class_id = last_recognition_class;
    return 1u;
}
