#include "zf_common_headfile.h"
#include "drive_config.h"
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
static float current_pose_offset_x_cm;
static float current_pose_offset_y_cm;
static uint8 navigation_start_row;
static uint8 navigation_start_col;
static uint8 validation_retry_count;
static uint8 active_class = SUBJECT2_INVALID_CLASS;

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

static subject2_object_struct *current_objects(void)
{
    return ((SUBJECT2_SCAN_BOX_PLAN == subject2_state) ||
            (SUBJECT2_SCAN_BOX_MOVE == subject2_state) ||
            (SUBJECT2_SCAN_BOX_CENTER == subject2_state) ||
            (SUBJECT2_SCAN_BOX_CLASSIFY == subject2_state)) ?
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

static void subject2_begin_center(subject2_state_enum center_state,
                                  const char *state_text,
                                  subject2_update_struct *update)
{
    center_sample_count = 0u;
    openart_request_player_center();
    subject2_state = center_state;
    if(0 != update)
    {
        update->run_state = state_text;
        update->redraw = 1u;
    }
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

static uint8 subject2_apply_observation_center(const subject2_context_struct *context)
{
    const map_source_struct *source = openart_map_get();
    const drive_pose_struct *pose;
    uint16 col_q;
    uint16 row_q;
    uint8 car_row;
    uint8 car_col;
    float offset_x_cm;
    float offset_y_cm;

    if((0 == source) || (0 == subject2_map_objects_unchanged(source)) ||
       (0 == map_find_car(source, &car_row, &car_col, 0)) ||
       (car_row != current_observation.row) ||
       (car_col != current_observation.col))
    {
        return 0u;
    }

    col_q = subject2_median_u16(center_col_samples);
    row_q = subject2_median_u16(center_row_samples);
    if((col_q >= (MAP_COLS * 100u)) || (row_q >= (MAP_ROWS * 100u)))
    {
        return 0u;
    }
    offset_x_cm = ((float)((int32)col_q - (int32)(car_col * 100u + 50u)) /
                   100.0f) * GRID_SIZE_CM;
    offset_y_cm = -((float)((int32)row_q - (int32)(car_row * 100u + 50u)) /
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
    navigation_start_row = car_row;
    navigation_start_col = car_col;
    *context->start_row = car_row;
    *context->start_col = car_col;
    map_source_snapshot(context->snapshot, context->snapshot_rows, source);
    *context->snapshot_valid = 1u;
    return 1u;
}

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

static void subject2_tick_center(const subject2_context_struct *context,
                                 subject2_update_struct *update)
{
    subject2_state_enum classify_state =
        (SUBJECT2_SCAN_BOX_CENTER == subject2_state) ?
        SUBJECT2_SCAN_BOX_CLASSIFY : SUBJECT2_SCAN_TARGET_CLASSIFY;

    if(0 == subject2_collect_center())
    {
        if(0 != update)
        {
            update->run_state = "VCtr";
        }
        return;
    }
    if(0 == subject2_apply_observation_center(context))
    {
        subject2_mark_observation_failed(update);
        return;
    }
    subject2_begin_classification(classify_state, update);
}

static void subject2_advance_after_recognition(subject2_update_struct *update)
{
    if(SUBJECT2_SCAN_BOX_CLASSIFY == subject2_state)
    {
        if(0 != all_objects_recognized(box_objects, box_object_count))
        {
            vision_uart_set_mode(VISION_MODE_TARGET);
            subject2_state = SUBJECT2_SCAN_TARGET_MODE;
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
            vision_uart_cancel();
            subject2_mark_observation_failed(update);
            return;
        }
        vision_uart_ack(current_vision_request_id);
        objects[current_observation.object_index].class_id = confirmed_class;
        objects[current_observation.object_index].recognized = 1u;
        subject2_advance_after_recognition(update);
        return;
    }
    if((time_ms() - classify_start_ms) >= SUBJECT2_VIEW_TIMEOUT_MS)
    {
        vision_uart_cancel();
        subject2_mark_observation_failed(update);
        return;
    }
    if(0 != update)
    {
        update->run_state = "VWait";
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
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:Class", update);
        return;
    }
    map_scan_stats(context->snapshot, &stats);
    navigation_start_row = stats.car_row;
    navigation_start_col = stats.car_col;
    *context->start_row = stats.car_row;
    *context->start_col = stats.car_col;
    if(0u == context->result->waypoint_count)
    {
        subject2_begin_center((objects == box_objects) ?
                              SUBJECT2_SCAN_BOX_CENTER : SUBJECT2_SCAN_TARGET_CENTER,
                              "VCtr", update);
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
        subject2_begin_center((SUBJECT2_SCAN_BOX_MOVE == subject2_state) ?
                              SUBJECT2_SCAN_BOX_CENTER : SUBJECT2_SCAN_TARGET_CENTER,
                              "VCtr", update);
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
        subject2_state = SUBJECT2_SELECT_PUSH;
        if(0 != update)
        {
            update->run_state = "Bind";
            update->redraw = 1u;
        }
        return;
    }
    if(0u != validation_retry_count)
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:Class", update);
        return;
    }
    clear_mismatched_bindings(&need_boxes, &need_targets);
    validation_retry_count++;
    if(0u != need_boxes)
    {
        vision_uart_set_mode(VISION_MODE_BOX);
        subject2_state = SUBJECT2_SCAN_BOX_MODE;
    }
    else if(0u != need_targets)
    {
        vision_uart_set_mode(VISION_MODE_TARGET);
        subject2_state = SUBJECT2_SCAN_TARGET_MODE;
    }
    else
    {
        subject2_fail(EXEC_ERROR_SUBJECT2_CLASS, "E:Class", update);
        return;
    }
    if(0 != update)
    {
        update->run_state = "VRetry";
        update->redraw = 1u;
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
    map_scan_stats(context->snapshot, &stats);
    navigation_start_row = stats.car_row;
    navigation_start_col = stats.car_col;
    vision_uart_set_mode(VISION_MODE_BOX);
    subject2_state = SUBJECT2_SCAN_BOX_MODE;
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

    switch(subject2_state)
    {
        case SUBJECT2_SCAN_BOX_MODE:
            if(0 != vision_uart_mode_ready(VISION_MODE_BOX))
            {
                subject2_state = SUBJECT2_SCAN_BOX_PLAN;
            }
            if(0 != update) update->run_state = "BScan";
            break;
        case SUBJECT2_SCAN_TARGET_MODE:
            if(0 != vision_uart_mode_ready(VISION_MODE_TARGET))
            {
                subject2_state = SUBJECT2_SCAN_TARGET_PLAN;
            }
            if(0 != update) update->run_state = "TScan";
            break;
        case SUBJECT2_SCAN_BOX_PLAN:
        case SUBJECT2_SCAN_TARGET_PLAN:
            subject2_tick_plan(context, update);
            break;
        case SUBJECT2_SCAN_BOX_MOVE:
        case SUBJECT2_SCAN_TARGET_MOVE:
            subject2_tick_move(update);
            break;
        case SUBJECT2_SCAN_BOX_CENTER:
        case SUBJECT2_SCAN_TARGET_CENTER:
            subject2_tick_center(context, update);
            break;
        case SUBJECT2_SCAN_BOX_CLASSIFY:
        case SUBJECT2_SCAN_TARGET_CLASSIFY:
            subject2_tick_classify(update);
            break;
        case SUBJECT2_VALIDATE_BINDINGS:
            subject2_tick_validate(update);
            break;
        case SUBJECT2_SELECT_PUSH:
            if(0 != update) update->run_state = "Bind";
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
    validation_retry_count = 0u;
    active_class = SUBJECT2_INVALID_CLASS;
}

subject2_state_enum subject2_get_state(void)
{
    return subject2_state;
}

uint8 subject2_get_active_class(void)
{
    return active_class;
}
