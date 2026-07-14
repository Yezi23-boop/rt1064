#include "zf_common_headfile.h"
#include "drive_config.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "executor.h"
#include "map_utils.h"
#include "openart_uart.h"
#include "solver.h"
#include "subject2.h"
#include "timebase.h"
#include "vision_uart.h"

static subject2_state_enum subject2_state = SUBJECT2_IDLE;
static subject2_object_struct box_objects[MAX_BOXES];
static subject2_object_struct target_objects[MAX_BOXES];
static subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT];
static uint8 box_object_count;
static uint8 target_object_count;
static subject2_observation_plan_struct current_observation;
static subject2_classifier_struct classifier;
static uint16 current_vision_request_id;
static uint32 classify_start_ms;
static uint16 center_col_samples[ART_CENTER_SAMPLE_COUNT];
static uint16 center_row_samples[ART_CENTER_SAMPLE_COUNT];
static uint8 center_sample_count;
static uint32 pre_push_center_start_ms;
static uint8 pre_push_box_request_active;
static uint8 pre_push_box_preparation_started;
static float current_pose_offset_x_cm;
static float current_pose_offset_y_cm;
static uint8 navigation_start_row;
static uint8 navigation_start_col;
static uint8 validation_retry_count;
static uint8 active_class = SUBJECT2_INVALID_CLASS;
static uint8 last_recognition_valid;
static uint8 last_recognition_is_target;
static uint8 last_recognition_class = SUBJECT2_INVALID_CLASS;
static solve_result_struct push_candidate_result;
static uint16 task_start_boxes[MAX_BOXES];
static uint8 task_start_box_count;
static uint8 task_start_target_count;
static uint8 retry_active_only;
static uint8 replan_center_for_return;
static uint32 confirm_last_frame;
static char confirm_candidate_rows[MAP_ROWS][MAP_COLS + 1];
static uint8 confirm_candidate_valid;
static uint8 confirm_stable_count;
static uint32 center_request_start_ms;
static uint32 center_adjust_start_ms;
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

static void subject2_accept_confirmed_map(const subject2_context_struct *context,
                                          const map_source_struct *source,
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

static uint16 subject2_median_u16(const uint16 *values)
{
    uint16 sorted[ART_CENTER_SAMPLE_COUNT];
    uint16 key;
    uint8 i;
    uint8 j;

    for(i = 0u; i < ART_CENTER_SAMPLE_COUNT; i++)
    {
        sorted[i] = values[i];
    }
    for(i = 1u; i < ART_CENTER_SAMPLE_COUNT; i++)
    {
        key = sorted[i];
        j = i;
        while((0u < j) && (sorted[j - 1u] > key))
        {
            sorted[j] = sorted[j - 1u];
            j--;
        }
        sorted[j] = key;
    }
    return sorted[ART_CENTER_SAMPLE_COUNT / 2u];
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
    vision_uart_cancel();
    executor_set_error(error);
    subject2_state = SUBJECT2_ERROR;
    if(0 != update)
    {
        update->run_state = state_text;
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
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:VMod", update);
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
    uint16 col_q;
    uint16 row_q;
    uint8 sample_index;

    while(0u != (sample_index = openart_get_requested_center_sample(&col_q, &row_q)))
    {
        (void)sample_index;
        if(center_sample_count < ART_CENTER_SAMPLE_COUNT)
        {
            center_col_samples[center_sample_count] = col_q;
            center_row_samples[center_sample_count] = row_q;
            center_sample_count++;
        }
    }
    return (center_sample_count >= ART_CENTER_SAMPLE_COUNT) ? 1u : 0u;
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

static uint8 subject2_cells_are_neighbors(uint8 first_row, uint8 first_col,
                                          uint8 second_row, uint8 second_col)
{
    int16 row_delta = (int16)first_row - (int16)second_row;
    int16 col_delta = (int16)first_col - (int16)second_col;

    return ((row_delta >= -1) && (row_delta <= 1) &&
            (col_delta >= -1) && (col_delta <= 1)) ? 1u : 0u;
}

static uint8 subject2_apply_center_from_cell(const subject2_context_struct *context,
                                             const map_source_struct *source,
                                             uint8 source_car_row,
                                             uint8 source_car_col,
                                             uint8 reference_row,
                                             uint8 reference_col,
                                             float *applied_x_cm,
                                             float *applied_y_cm)
{
    const drive_pose_struct *pose;
    uint16 col_q;
    uint16 row_q;
    char old_car_value;
    char reference_value;
    float offset_x_cm;
    float offset_y_cm;

    if((0 == context) || (0 == source))
    {
        return 0u;
    }

    col_q = subject2_median_u16(center_col_samples);
    row_q = subject2_median_u16(center_row_samples);
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

    pose = drive_pose_get();
    drive_pose_reset(offset_x_cm, offset_y_cm, pose->yaw_deg);
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
                                   uint8 require_observation_match,
                                   float *applied_x_cm,
                                   float *applied_y_cm)
{
    const map_source_struct *source = openart_get_requested_center_map();
    uint8 car_row;
    uint8 car_col;

    if((0 == source) || (0 == map_find_car(source, &car_row, &car_col, 0)))
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
                                           applied_x_cm, applied_y_cm);
}

#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
static uint8 subject2_apply_map_cell_center(const subject2_context_struct *context)
{
    const map_source_struct *source = openart_map_get();
    const drive_pose_struct *pose;
    uint8 car_row;
    uint8 car_col;

    if((0 == source) || (0 == map_find_car(source, &car_row, &car_col, 0)))
    {
        return 0u;
    }
    pose = drive_pose_get();
    drive_pose_reset(0.0f, 0.0f, pose->yaw_deg);
    current_pose_offset_x_cm = 0.0f;
    current_pose_offset_y_cm = 0.0f;
    navigation_start_row = car_row;
    navigation_start_col = car_col;
    *context->start_row = car_row;
    *context->start_col = car_col;
    map_source_snapshot(context->snapshot, context->snapshot_rows, source);
    *context->snapshot_valid = 1u;
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
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:Back", update);
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
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:Back", update);
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
    center_sample_count = 0u;
    center_request_start_ms = time_ms();
    openart_request_player_center();
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
        subject2_fail(EXEC_ERROR_SUBJECT2_YAW, "E:Yaw", update);
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

    if(0 == subject2_collect_center())
    {
        if((time_ms() - center_request_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
        {
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
            subject2_begin_turn(update);
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
    source = openart_get_requested_center_map();
    if((0 != source) &&
       (0 != map_find_car(source, &car_row, &car_col, 0)) &&
       (0 != subject2_map_objects_unchanged(source)))
    {
        center_col_q = subject2_median_u16(center_col_samples);
        center_row_q = subject2_median_u16(center_row_samples);
        if((center_col_q < (MAP_COLS * 100u)) &&
           (center_row_q < (MAP_ROWS * 100u)) &&
           ((center_col_q / 100u) == current_observation.col) &&
           ((center_row_q / 100u) == current_observation.row) &&
           (0 != subject2_cells_are_neighbors(car_row, car_col,
                                              current_observation.row,
                                              current_observation.col)))
        {
            if(0 == subject2_apply_center_from_cell(context, source,
                                                    car_row, car_col,
                                                    current_observation.row,
                                                    current_observation.col,
                                                    0, 0))
            {
                subject2_fail(EXEC_ERROR_ART_CENTER, "E:CRef", update);
                return;
            }
            center_applied = 1u;
        }
        else if(((car_row != current_observation.row) ||
                 (car_col != current_observation.col)) &&
                ((center_col_q / 100u) == car_col) &&
                ((center_row_q / 100u) == car_row))
        {
            if(0 == subject2_apply_center(context, 0u, 0, 0))
            {
                subject2_fail(EXEC_ERROR_ART_CENTER, "E:CRef", update);
                return;
            }
            subject2_state = (0u != is_box_scan) ?
                             SUBJECT2_SCAN_BOX_PLAN : SUBJECT2_SCAN_TARGET_PLAN;
            if(0 != update)
            {
                update->run_state = "VReplan";
                update->redraw = 1u;
            }
            return;
        }
    }
    if((0u == center_applied) &&
       (0 == subject2_observation_map_matches(source)))
    {
        if((time_ms() - center_request_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
        {
            subject2_fail(EXEC_ERROR_ART_CENTER, "E:CMap", update);
        }
        else if(0 != update)
        {
            update->run_state = "VCtr";
        }
        return;
    }
    if((0u == center_applied) &&
       (0 == subject2_apply_center(context, 1u, 0, 0)))
    {
        subject2_fail(EXEC_ERROR_ART_CENTER, "E:CRef", update);
        return;
    }

    if(0 == executor_start_position_correction(0.0f, 0.0f))
    {
        subject2_fail(EXEC_ERROR_ART_CENTER, "E:CBsy", update);
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
        subject2_fail(EXEC_ERROR_ART_CENTER, "E:CExe", update);
        return;
    }
    if((time_ms() - center_adjust_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
    {
        subject2_fail(EXEC_ERROR_ART_CENTER, "E:CTim", update);
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
    uint8 bound;

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
        bound = (objects == box_objects) ?
            subject2_bind_box(bindings, confirmed_class,
                              objects[current_observation.object_index].cell) :
            subject2_bind_target(bindings, confirmed_class,
                                 objects[current_observation.object_index].cell);
        if(0u == bound)
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
    if((EXEC_STATE_ERROR == executor_get_state()) ||
       ((time_ms() - view_motion_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS))
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:Back", update);
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
    if((EXEC_STATE_ERROR == executor_get_state()) ||
       ((time_ms() - view_motion_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS))
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:Back", update);
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
    const map_source_struct *source = openart_map_get();
    subject2_object_struct *objects = current_objects();
    uint8 count = current_object_count();
    map_scan_stats_struct stats;

    if((0 == source) || (0 == subject2_map_objects_unchanged(source)))
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:Map", update);
        return;
    }
    map_source_snapshot(context->snapshot, context->snapshot_rows, source);
    *context->snapshot_valid = 1u;
    if(0 == subject2_select_observation(context->snapshot, objects, count,
                                        &current_observation, context->result))
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:VPos", update);
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
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:Move", update);
    }
    else if(0 != update)
    {
        update->run_state = "VPos";
    }
}

static void clear_mismatched_bindings(uint8 *need_boxes, uint8 *need_targets)
{
    uint8 class_id;
    uint8 index;

    *need_boxes = 0u;
    *need_targets = 0u;
    for(class_id = 0u; class_id < SUBJECT2_CLASS_COUNT; class_id++)
    {
        if((0u != bindings[class_id].box_valid) &&
           (0u == bindings[class_id].target_valid))
        {
            for(index = 0u; index < box_object_count; index++)
            {
                if(box_objects[index].class_id == class_id)
                {
                    box_objects[index].recognized = 0u;
                    box_objects[index].class_id = SUBJECT2_INVALID_CLASS;
                    box_objects[index].tried_observation_mask = 0u;
                    break;
                }
            }
            bindings[class_id].box_valid = 0u;
            bindings[class_id].box_cell = INVALID_STATE;
            *need_boxes = 1u;
        }
        else if((0u == bindings[class_id].box_valid) &&
                (0u != bindings[class_id].target_valid))
        {
            for(index = 0u; index < target_object_count; index++)
            {
                if(target_objects[index].class_id == class_id)
                {
                    target_objects[index].recognized = 0u;
                    target_objects[index].class_id = SUBJECT2_INVALID_CLASS;
                    target_objects[index].tried_observation_mask = 0u;
                    break;
                }
            }
            bindings[class_id].target_valid = 0u;
            bindings[class_id].target_cell = INVALID_STATE;
            *need_targets = 1u;
        }
    }
}

static void subject2_tick_validate(subject2_update_struct *update)
{
    uint8 need_boxes;
    uint8 need_targets;

    if(0 != subject2_binding_sets_match(bindings))
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
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:BSet", update);
        return;
    }
    clear_mismatched_bindings(&need_boxes, &need_targets);
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
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:BSet", update);
        return;
    }
    if(0 != update)
    {
        update->run_state = "VRetry";
        update->redraw = 1u;
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
            subject2_state = SUBJECT2_SELECT_PUSH;
            if(0 != update)
            {
                update->run_state = "Bind";
                update->redraw = 1u;
            }
            return;
        }
    }
    else
    {
        turn_stable_active = 0u;
    }

    if((now_ms - turn_start_ms) >= SUBJECT2_TURN_TIMEOUT_MS)
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_YAW, "E:HYaw", update);
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
    uint16 best_actions = 0xFFFFu;
    uint8 class_id;
    uint8 selected = SUBJECT2_INVALID_CLASS;

    for(class_id = 0u; class_id < SUBJECT2_CLASS_COUNT; class_id++)
    {
        if((0u == bindings[class_id].box_valid) ||
           (0u == bindings[class_id].target_valid) ||
           (0u != bindings[class_id].completed) ||
           ((0u != retry_active_only) && (class_id != active_class)))
        {
            continue;
        }
        if(0 != solve_bound_box_path(context->snapshot,
                                     bindings[class_id].box_cell,
                                     bindings[class_id].target_cell,
                                     &push_candidate_result))
        {
            if((SUBJECT2_INVALID_CLASS == selected) ||
               (push_candidate_result.action_count < best_actions))
            {
                selected = class_id;
                best_actions = push_candidate_result.action_count;
                memcpy(context->result, &push_candidate_result,
                       sizeof(*context->result));
            }
        }
    }
    if(SUBJECT2_INVALID_CLASS == selected)
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_PLAN, "E:Plan", update);
        return;
    }

    active_class = selected;
    retry_active_only = 0u;
    if(0 == subject2_collect_cells(context->snapshot, 'B',
                                   task_start_boxes, &task_start_box_count))
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_TRACK, "E:Track", update);
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
    pre_push_center_start_ms = time_ms();
    subject2_state = SUBJECT2_PRE_PUSH_CENTER;
    if(0 != executor_get_pre_push_box_request(&box_row, &box_col))
    {
        executor_reset_art_box_observation_samples();
        openart_request_observation(box_row, box_col);
        pre_push_box_request_active = 1u;
    }
    else
    {
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
    openart_observation_sample_struct sample;
    executor_art_box_prep_result_enum prep_result;
    uint8 ready = 0u;

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
            subject2_fail(EXEC_ERROR_ART_CENTER, "E:BTim", update);
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

    while(0u != openart_get_observation_sample(&sample))
    {
        if(0u != executor_apply_art_box_observation(sample.car_col_q,
                                                    sample.car_row_q,
                                                    sample.box_col_q,
                                                    sample.box_row_q))
        {
            ready = 1u;
        }
    }
    if(0u == ready)
    {
        if((time_ms() - pre_push_center_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
        {
            subject2_fail(EXEC_ERROR_ART_CENTER, "E:BObs", update);
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
        subject2_fail(EXEC_ERROR_ART_CENTER, "E:BGeo", update);
        return;
    }

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
    uint16 col_q;
    uint16 row_q;
    uint8 sample_index;
    uint8 ready = 0u;
    uint8 car_row;
    uint8 car_col;

    if(0u != pre_push_box_request_active)
    {
        subject2_tick_pre_push_box(update);
        return;
    }

    while(0u != (sample_index = openart_get_requested_center_sample(&col_q, &row_q)))
    {
        if(0 != executor_apply_art_player_center(col_q, row_q, sample_index))
        {
            ready = 1u;
        }
    }
    if(0u == ready)
    {
        if((time_ms() - pre_push_center_start_ms) >= EXEC_ART_SYNC_TIMEOUT_MS)
        {
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
            if(0 == executor_continue_after_pre_push_center())
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
#else
            subject2_fail(EXEC_ERROR_ART_CENTER, "E:CPsh", update);
#endif
            return;
        }
        if(0 != update) update->run_state = "PCtr";
        return;
    }
    source = openart_map_get();
    if((0 == source) || (0 == map_find_car(source, &car_row, &car_col, 0)))
    {
        subject2_fail(EXEC_ERROR_ART_CENTER, "E:CPsh", update);
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

static void subject2_begin_confirm_map(subject2_update_struct *update)
{
    confirm_last_frame = openart_uart_get_frame_count();
    confirm_candidate_valid = 0u;
    confirm_stable_count = 0u;
    subject2_state = SUBJECT2_CONFIRM_MAP;
    if(0 != update)
    {
        update->run_state = "ART Wait";
        update->redraw = 1u;
    }
}

static uint8 subject2_handle_host_completion(subject2_update_struct *update)
{
    const map_source_struct *source = openart_map_get();
    map_scan_stats_struct stats;
    uint8 box_reduction;
    uint8 target_reduction;

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

    executor_stop();
    subject2_begin_confirm_map(update);
    return 1u;
}

static void subject2_begin_replan_center(uint8 for_return,
                                         subject2_update_struct *update)
{
    replan_center_for_return = for_return;
    center_sample_count = 0u;
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
    uint16 new_boxes[MAX_BOXES];
    uint8 new_box_count = 0u;
    subject2_track_result_enum track_result;

    map_scan_stats(source, &stats);
    if(1u != stats.car_count)
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_TRACK, "E:Track", update);
        return;
    }
    if((stats.box_count + 1u == task_start_box_count) &&
       (stats.target_count + 1u == task_start_target_count))
    {
        bindings[active_class].completed = 1u;
        map_source_snapshot(context->snapshot, context->snapshot_rows, source);
        *context->snapshot_valid = 1u;
        executor_stop();
        subject2_begin_replan_center((0u == stats.box_count) ? 1u : 0u, update);
        return;
    }
    if((stats.box_count == task_start_box_count) &&
       (stats.target_count == task_start_target_count) &&
       (0 != subject2_collect_cells(source, 'B', new_boxes, &new_box_count)))
    {
        track_result = subject2_track_active_box(bindings, active_class,
                                                 task_start_boxes,
                                                 task_start_box_count,
                                                 new_boxes,
                                                 new_box_count);
        if(SUBJECT2_TRACK_AMBIGUOUS == track_result)
        {
            subject2_fail(EXEC_ERROR_SUBJECT2_TRACK, "E:Track", update);
            return;
        }
        retry_active_only = 1u;
        map_source_snapshot(context->snapshot, context->snapshot_rows, source);
        *context->snapshot_valid = 1u;
        executor_stop();
        subject2_begin_replan_center(0u, update);
        return;
    }
    subject2_fail(EXEC_ERROR_SUBJECT2_TRACK, "E:Track", update);
}

static void subject2_tick_confirm_map(const subject2_context_struct *context,
                                      subject2_update_struct *update)
{
    const map_source_struct *source;
    uint32 frame_count = openart_uart_get_frame_count();

    if(frame_count == confirm_last_frame)
    {
        if(0 != update) update->run_state = "ART Wait";
        return;
    }
    confirm_last_frame = frame_count;
    source = openart_map_get();
    if(0 == source)
    {
        return;
    }
    if((0u == confirm_candidate_valid) ||
       (0u == map_rows_equal(confirm_candidate_rows, source)))
    {
        map_copy_rows(confirm_candidate_rows, source);
        confirm_candidate_valid = 1u;
        confirm_stable_count = 1u;
    }
    else if(confirm_stable_count < EXEC_ART_STABLE_FRAMES)
    {
        confirm_stable_count++;
    }
    if(confirm_stable_count >= EXEC_ART_STABLE_FRAMES)
    {
        subject2_accept_confirmed_map(context, source, update);
    }
}

static void subject2_tick_replan_center(const subject2_context_struct *context,
                                        subject2_update_struct *update)
{
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
    else if(0 == subject2_apply_center(context, 0u, 0, 0))
    {
        subject2_fail(EXEC_ERROR_ART_CENTER, "E:CRpl", update);
        return;
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
        subject2_begin_confirm_map(update);
    }
    else if(EXEC_STATE_ERROR == executor_get_state())
    {
        if(EXEC_ERROR_ART_CENTER == executor_get_error())
        {
            subject2_fail(EXEC_ERROR_ART_CENTER, "E:CPsh", update);
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

    subject2_bindings_clear(bindings);
    subject2_classifier_reset(&classifier);
    current_pose_offset_x_cm = initial_pose_x_cm;
    current_pose_offset_y_cm = initial_pose_y_cm;
    validation_retry_count = 0u;
    active_class = SUBJECT2_INVALID_CLASS;
    last_recognition_valid = 0u;
    last_recognition_is_target = 0u;
    last_recognition_class = SUBJECT2_INVALID_CLASS;
    retry_active_only = 0u;
    replan_center_for_return = 0u;
    launch_yaw_deg = context->launch_yaw_deg;
    confirm_candidate_valid = 0u;
    confirm_stable_count = 0u;
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
        case SUBJECT2_VALIDATE_BINDINGS:
            subject2_tick_validate(update);
            break;
        case SUBJECT2_RESTORE_HEADING:
            subject2_tick_restore_heading(update);
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

void subject2_cancel(void)
{
    subject2_state = SUBJECT2_IDLE;
    box_object_count = 0u;
    target_object_count = 0u;
    current_vision_request_id = 0u;
    classify_start_ms = 0u;
    center_sample_count = 0u;
    pre_push_center_start_ms = 0u;
    pre_push_box_request_active = 0u;
    pre_push_box_preparation_started = 0u;
    validation_retry_count = 0u;
    active_class = SUBJECT2_INVALID_CLASS;
    last_recognition_valid = 0u;
    last_recognition_is_target = 0u;
    last_recognition_class = SUBJECT2_INVALID_CLASS;
    retry_active_only = 0u;
    replan_center_for_return = 0u;
    confirm_candidate_valid = 0u;
    confirm_stable_count = 0u;
    center_request_start_ms = 0u;
    center_adjust_start_ms = 0u;
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
