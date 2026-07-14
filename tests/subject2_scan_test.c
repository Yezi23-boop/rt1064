#include <stdio.h>
#include <string.h>
#include "drive_control.h"
#include "drive_pose.h"
#include "executor.h"
#include "map_utils.h"
#include "openart_uart.h"
#include "subject2.h"
#include "vision_uart.h"

#ifndef ART_CENTER_TIMEOUT_FALLBACK_ENABLE
#define ART_CENTER_TIMEOUT_FALLBACK_ENABLE (1)
#endif

static uint32 fake_time_ms;
static executor_state_enum fake_executor_state;
static executor_error_enum fake_executor_error;
static drive_pose_struct fake_pose;
static control_status_struct fake_control_status;
static uint16 set_motion_count;
static uint16 set_target_yaw_count;
static float last_target_yaw;
static uint8 fake_ready_box;
static uint8 fake_ready_target;
static vision_mode_enum last_mode;
static uint16 vision_mode_send_count;
static uint16 vision_request_id;
static vision_sample_struct vision_samples[8];
static uint8 vision_sample_count;
static uint8 vision_sample_read;
static uint16 vision_ack_count;
static uint16 vision_cancel_count;
static uint16 center_col_q[ART_CENTER_SAMPLE_COUNT];
static uint16 center_row_q[ART_CENTER_SAMPLE_COUNT];
static uint8 center_count;
static uint8 center_read;
static uint16 center_request_count;
static openart_observation_sample_struct observation_samples[3];
static uint8 observation_count;
static uint8 observation_read;
static uint16 observation_request_count;
static uint8 observation_request_row;
static uint8 observation_request_col;
static uint16 pose_reset_count;
static uint16 executor_start_count;
static uint8 executor_start_single_step;
static uint16 correction_start_count;
static float correction_target_x;
static float correction_target_y;
static uint8 correction_start_allowed;
static uint8 executor_start_art_sync;
static uint8 fake_pre_push_pending;
static uint8 fake_pre_push_box_request;
static uint8 fake_pre_push_box_active;
static uint8 fake_error_on_box_active_query;
static uint8 fake_pre_push_box_row;
static uint8 fake_pre_push_box_col;
static const char *fake_pre_push_box_state;
static executor_art_box_prep_result_enum fake_pre_push_box_result;
static uint8 fake_art_box_sample_count;
static uint16 start_pre_push_box_count;
static uint8 fake_center_requires_push_alignment;
static executor_art_center_result_enum fake_center_result;
static uint8 fake_sync_pending;
static uint16 continue_pre_push_count;
static uint16 start_pre_push_alignment_count;
static uint32 fake_frame_count;
static uint8 executor_target_row;
static uint8 executor_target_col;
static char live_rows[MAP_ROWS][MAP_COLS + 1];
static map_source_struct live_source;
static char paired_center_rows[MAP_ROWS][MAP_COLS + 1];
static map_source_struct paired_center_source;
static uint8 paired_center_valid;
static char snapshot_rows[MAP_ROWS][MAP_COLS + 1];
static map_source_struct snapshot;
static solve_result_struct result;
static uint8 snapshot_valid;
static uint32 elapsed_ms;
static uint8 start_row;
static uint8 start_col;

uint32 time_ms(void) { return fake_time_ms; }
const drive_pose_struct *drive_pose_get(void) { return &fake_pose; }
void drive_pose_reset(float x, float y, float yaw)
{
    pose_reset_count++;
    fake_pose.x_cm = x;
    fake_pose.y_cm = y;
    fake_pose.yaw_deg = yaw;
}
void stop_motion(void) { }
const control_status_struct *get_control_status(void)
{
    return &fake_control_status;
}
void set_motion(float vx, float vy)
{
    (void)vx;
    (void)vy;
    set_motion_count++;
}
void set_target_yaw(float yaw)
{
    set_target_yaw_count++;
    last_target_yaw = yaw;
}

void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 row, uint8 col, float x, float y,
                    uint8 single_step, uint8 art_sync)
{
    (void)row; (void)col; (void)x; (void)y; (void)single_step;
    executor_start_count++;
    executor_start_single_step = single_step;
    executor_start_art_sync = art_sync;
    fake_executor_state = EXEC_STATE_RUNNING;
    if(0u != count)
    {
        executor_target_row = waypoints[count - 1u].row;
        executor_target_col = waypoints[count - 1u].col;
    }
}
void executor_stop(void)
{
    fake_executor_state = EXEC_STATE_IDLE;
    fake_executor_error = EXEC_ERROR_NONE;
    fake_pre_push_pending = 0u;
    fake_sync_pending = 0u;
}
uint8 executor_start_position_correction(float target_x, float target_y)
{
    if(0u == correction_start_allowed) return 0u;
    correction_start_count++;
    correction_target_x = target_x;
    correction_target_y = target_y;
    fake_executor_state = EXEC_STATE_RUNNING;
    return 1u;
}
executor_state_enum executor_get_state(void) { return fake_executor_state; }
executor_error_enum executor_get_error(void) { return fake_executor_error; }
void executor_set_error(executor_error_enum error)
{
    fake_executor_error = error;
    fake_executor_state = EXEC_STATE_ERROR;
}
void executor_reset_art_player_center_samples(void) { }
uint8 executor_apply_art_player_center(uint16 col, uint16 row, uint32 sample)
{
    (void)col; (void)row;
    return (sample >= ART_CENTER_SAMPLE_COUNT) ? 1u : 0u;
}
executor_art_center_result_enum executor_commit_art_player_center(uint8 row, uint8 col)
{
    (void)row; (void)col;
    return fake_center_result;
}
uint8 executor_art_pre_push_pending(void) { return fake_pre_push_pending; }
uint8 executor_get_pre_push_box_request(uint8 *box_row, uint8 *box_col)
{
    if((0u == fake_pre_push_pending) || (0u == fake_pre_push_box_request)) return 0u;
    *box_row = fake_pre_push_box_row;
    *box_col = fake_pre_push_box_col;
    return 1u;
}
void executor_reset_art_box_observation_samples(void)
{
    fake_art_box_sample_count = 0u;
}
uint8 executor_apply_art_box_observation(uint16 car_col_q, uint16 car_row_q,
                                         uint16 box_col_q, uint16 box_row_q)
{
    (void)car_col_q; (void)car_row_q; (void)box_col_q; (void)box_row_q;
    if(fake_art_box_sample_count < 3u) fake_art_box_sample_count++;
    return (fake_art_box_sample_count >= 3u) ? 1u : 0u;
}
executor_art_box_prep_result_enum executor_start_pre_push_box_preparation(void)
{
    start_pre_push_box_count++;
    if(EXEC_ART_BOX_PREP_STARTED == fake_pre_push_box_result)
    {
        fake_pre_push_pending = 0u;
        fake_pre_push_box_active = 1u;
    }
    return fake_pre_push_box_result;
}
uint8 executor_pre_push_box_preparation_active(void)
{
    if(0u != fake_error_on_box_active_query)
    {
        fake_error_on_box_active_query = 0u;
        fake_pre_push_box_active = 0u;
        fake_executor_error = EXEC_ERROR_ART_CENTER;
        fake_executor_state = EXEC_STATE_ERROR;
    }
    return fake_pre_push_box_active;
}
const char *executor_pre_push_box_state_name(void)
{
    return fake_pre_push_box_state;
}
uint8 executor_center_requires_push_alignment(void)
{
    return fake_center_requires_push_alignment;
}
uint8 executor_art_sync_pending(void) { return fake_sync_pending; }
uint8 executor_continue_after_pre_push_center(void)
{
    if(0u == fake_pre_push_pending) return 0u;
    fake_pre_push_pending = 0u;
    continue_pre_push_count++;
    return 1u;
}
uint8 executor_start_pre_push_alignment(uint8 row, uint8 col)
{
    (void)row;
    (void)col;
    if(0u == fake_pre_push_pending) return 0u;
    fake_pre_push_pending = 0u;
    start_pre_push_alignment_count++;
    return 1u;
}

void openart_request_player_center(void)
{
    center_request_count++;
    center_count = 0u;
    center_read = 0u;
    paired_center_valid = 0u;
}
uint8 openart_get_requested_center_sample(uint16 *col_q, uint16 *row_q)
{
    if(center_read >= center_count)
    {
        return 0u;
    }
    *col_q = center_col_q[center_read];
    *row_q = center_row_q[center_read];
    center_read++;
    return center_read;
}
void openart_request_observation(uint8 box_row, uint8 box_col)
{
    observation_request_row = box_row;
    observation_request_col = box_col;
    observation_request_count++;
    observation_count = 0u;
    observation_read = 0u;
}
uint8 openart_get_observation_sample(openart_observation_sample_struct *sample)
{
    if((0 == sample) || (observation_read >= observation_count)) return 0u;
    *sample = observation_samples[observation_read++];
    return 1u;
}
const map_source_struct *openart_map_get(void) { return &live_source; }
const map_source_struct *openart_get_requested_center_map(void)
{
    return (0u != paired_center_valid) ? &paired_center_source : 0;
}
uint32 openart_uart_get_frame_count(void) { return fake_frame_count; }

void vision_uart_set_mode(vision_mode_enum mode)
{
    last_mode = mode;
    vision_mode_send_count++;
}
uint8 vision_uart_mode_ready(vision_mode_enum mode)
{
    return ((VISION_MODE_BOX == mode) ? fake_ready_box : fake_ready_target);
}
uint16 vision_uart_request_classification(void)
{
    vision_request_id++;
    vision_sample_count = 0u;
    vision_sample_read = 0u;
    return vision_request_id;
}
uint8 vision_uart_get_sample(vision_sample_struct *sample)
{
    if(vision_sample_read >= vision_sample_count)
    {
        return 0u;
    }
    *sample = vision_samples[vision_sample_read++];
    return 1u;
}
void vision_uart_ack(uint16 request_id)
{
    if(request_id == vision_request_id)
    {
        vision_ack_count++;
    }
}
void vision_uart_cancel(void)
{
    vision_cancel_count++;
    vision_sample_count = 0u;
    vision_sample_read = 0u;
}

static void build_map(void)
{
    uint8 row;
    uint8 col;

    subject2_cancel();
    fake_time_ms = 0u;
    fake_executor_state = EXEC_STATE_IDLE;
    fake_executor_error = EXEC_ERROR_NONE;
    memset(&fake_pose, 0, sizeof(fake_pose));
    memset(&fake_control_status, 0, sizeof(fake_control_status));
    set_motion_count = 0u;
    set_target_yaw_count = 0u;
    last_target_yaw = 0.0f;
    fake_ready_box = 0u;
    fake_ready_target = 0u;
    last_mode = VISION_MODE_NONE;
    vision_mode_send_count = 0u;
    vision_request_id = 0u;
    vision_sample_count = 0u;
    vision_sample_read = 0u;
    vision_ack_count = 0u;
    vision_cancel_count = 0u;
    center_count = 0u;
    center_read = 0u;
    center_request_count = 0u;
    observation_count = 0u;
    observation_read = 0u;
    observation_request_count = 0u;
    observation_request_row = 0u;
    observation_request_col = 0u;
    pose_reset_count = 0u;
    executor_start_count = 0u;
    executor_start_single_step = 0u;
    correction_start_count = 0u;
    correction_target_x = 0.0f;
    correction_target_y = 0.0f;
    correction_start_allowed = 1u;
    executor_start_art_sync = 0u;
    fake_pre_push_pending = 0u;
    fake_pre_push_box_request = 1u;
    fake_pre_push_box_active = 0u;
    fake_error_on_box_active_query = 0u;
    fake_pre_push_box_row = 5u;
    fake_pre_push_box_col = 6u;
    fake_pre_push_box_state = "BGap";
    fake_pre_push_box_result = EXEC_ART_BOX_PREP_STARTED;
    fake_art_box_sample_count = 0u;
    start_pre_push_box_count = 0u;
    fake_center_requires_push_alignment = 1u;
    fake_center_result = EXEC_ART_CENTER_APPLIED;
    fake_sync_pending = 0u;
    continue_pre_push_count = 0u;
    start_pre_push_alignment_count = 0u;
    fake_frame_count = 0u;
    for(row = 0u; row < MAP_ROWS; row++)
    {
        for(col = 0u; col < MAP_COLS; col++)
        {
            live_rows[row][col] = ((0u == row) || ((MAP_ROWS - 1u) == row) ||
                                   (0u == col) || ((MAP_COLS - 1u) == col)) ? '#' : '.';
        }
        live_rows[row][MAP_COLS] = '\0';
        live_source.rows[row] = live_rows[row];
        paired_center_source.rows[row] = paired_center_rows[row];
        snapshot.rows[row] = snapshot_rows[row];
    }
    live_source.name = "subject2-live";
    paired_center_source.name = "subject2-center";
    paired_center_valid = 0u;
    snapshot.name = "subject2-snapshot";
    live_rows[5][5] = 'C';
    live_rows[5][6] = 'B';
    live_rows[5][9] = 'T';
    map_source_snapshot(&snapshot, snapshot_rows, &live_source);
    snapshot_valid = 1u;
    start_row = 5u;
    start_col = 5u;
}

static void init_context(subject2_context_struct *context)
{
    memset(context, 0, sizeof(*context));
    context->result = &result;
    context->snapshot = &snapshot;
    context->snapshot_rows = snapshot_rows;
    context->snapshot_valid = &snapshot_valid;
    context->elapsed_ms = &elapsed_ms;
    context->start_row = &start_row;
    context->start_col = &start_col;
    context->run_mode = RUN_MODE_RUN;
    context->launch_yaw_deg = 23.0f;
}

static void feed_center(uint16 col_q, uint16 row_q)
{
    uint8 index;
    for(index = 0u; index < ART_CENTER_SAMPLE_COUNT; index++)
    {
        center_col_q[index] = (uint16)(col_q + index);
        center_row_q[index] = row_q;
    }
    center_count = ART_CENTER_SAMPLE_COUNT;
    center_read = 0u;
    map_source_snapshot(&paired_center_source, paired_center_rows, &live_source);
    paired_center_valid = 1u;
}

static void feed_observation(uint16 car_col_q, uint16 car_row_q,
                             uint16 box_col_q, uint16 box_row_q)
{
    uint8 index;

    for(index = 0u; index < 3u; index++)
    {
        observation_samples[index].car_col_q = (uint16)(car_col_q + index);
        observation_samples[index].car_row_q = car_row_q;
        observation_samples[index].box_col_q = (uint16)(box_col_q + index);
        observation_samples[index].box_row_q = box_row_q;
    }
    observation_count = 3u;
    observation_read = 0u;
}

static void feed_class(uint8 class_id)
{
    uint8 index;
    for(index = 0u; index < SUBJECT2_CLASS_STABLE_SAMPLES; index++)
    {
        vision_samples[index].request_id = vision_request_id;
        vision_samples[index].sample_id = (uint16)(index + 1u);
        vision_samples[index].class_id = class_id;
        vision_samples[index].confidence_q = 900u;
    }
    vision_sample_count = SUBJECT2_CLASS_STABLE_SAMPLES;
    vision_sample_read = 0u;
}

static void move_live_car(uint8 row, uint8 col)
{
    uint8 current_row;
    uint8 current_col;

    if(0 != map_find_car(&live_source, &current_row, &current_col, 0))
    {
        live_rows[current_row][current_col] = '.';
    }
    live_rows[row][col] = 'C';
}

static void complete_observation_turn(const subject2_context_struct *context,
                                      subject2_update_struct *update)
{
    fake_control_status.yaw_error = 0.0f;
    subject2_tick(context, update);
    fake_time_ms += SUBJECT2_TURN_STABLE_MS;
    subject2_tick(context, update);
}

static uint8 complete_center_adjust(const subject2_context_struct *context,
                                    subject2_update_struct *update,
                                    subject2_state_enum adjust_state,
                                    subject2_state_enum turn_state)
{
    if(adjust_state != subject2_get_state())
    {
        return 0u;
    }
    fake_pose.x_cm = 0.0f;
    fake_pose.y_cm = 0.0f;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(context, update);
    return (turn_state == subject2_get_state()) ? 1u : 0u;
}

static uint8 scan_to_heading_restore(subject2_context_struct *context,
                                     subject2_update_struct *update)
{
    uint8 recognition_is_target = 0u;
    uint8 recognition_class = SUBJECT2_INVALID_CLASS;

    subject2_begin(context, 0.0f, 0.0f, update);
    if((SUBJECT2_SCAN_BOX_MODE != subject2_get_state()) ||
       (VISION_MODE_BOX != last_mode)) return 0u;

    fake_ready_box = 1u;
    subject2_tick(context, update);
    subject2_tick(context, update);
    if((SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) ||
       (1u != center_request_count) ||
       (0u != set_target_yaw_count)) return 0u;

    feed_center(550u, 550u);
    subject2_tick(context, update);
    if((SUBJECT2_SCAN_BOX_ADJUST != subject2_get_state()) ||
       (1u != pose_reset_count) || (0u != vision_request_id) ||
       (0u != observation_request_count) ||
       (1u != correction_start_count)) return 0u;
    if(0u == complete_center_adjust(context, update,
                                    SUBJECT2_SCAN_BOX_ADJUST,
                                    SUBJECT2_SCAN_BOX_TURN)) return 0u;
    if(90.0f != last_target_yaw) return 0u;
    complete_observation_turn(context, update);
    if((SUBJECT2_SCAN_BOX_CLASSIFY != subject2_get_state()) ||
       (1u != vision_request_id)) return 0u;

    feed_class(4u);
    subject2_tick(context, update);
    if((SUBJECT2_SCAN_TARGET_MODE != subject2_get_state()) ||
       (VISION_MODE_TARGET != last_mode) || (1u != vision_ack_count) ||
       (0u == subject2_get_last_recognition(&recognition_is_target,
                                            &recognition_class)) ||
       (0u != recognition_is_target) ||
       (4u != recognition_class)) return 0u;

    fake_ready_target = 1u;
    subject2_tick(context, update);
    subject2_tick(context, update);
    if((SUBJECT2_SCAN_TARGET_MOVE != subject2_get_state()) ||
       (1u != executor_start_count) || (0u != executor_start_art_sync)) return 0u;

    move_live_car(executor_target_row, executor_target_col);
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(context, update);
    if((SUBJECT2_SCAN_TARGET_CENTER != subject2_get_state()) ||
       (2u != center_request_count)) return 0u;

    feed_center((uint16)(executor_target_col * 100u + 60u),
                (uint16)(executor_target_row * 100u + 50u));
    subject2_tick(context, update);
    if((SUBJECT2_SCAN_TARGET_ADJUST != subject2_get_state()) ||
       (2u != pose_reset_count) || (2u != correction_start_count) ||
       (1u != vision_request_id)) return 0u;
    fake_pose.x_cm = 0.0f;
    fake_pose.y_cm = 0.0f;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(context, update);
    if(SUBJECT2_SCAN_TARGET_TURN != subject2_get_state()) return 0u;
    complete_observation_turn(context, update);
    if((SUBJECT2_SCAN_TARGET_CLASSIFY != subject2_get_state()) ||
       (2u != vision_request_id)) return 0u;

    feed_class(4u);
    subject2_tick(context, update);
    if((SUBJECT2_VALIDATE_BINDINGS != subject2_get_state()) ||
       (0u == subject2_get_last_recognition(&recognition_is_target,
                                            &recognition_class)) ||
       (0u == recognition_is_target) ||
       (4u != recognition_class)) return 0u;
    subject2_tick(context, update);
    return ((0 == strcmp(update->run_state, "HYaw")) &&
            (SUBJECT2_RESTORE_HEADING == subject2_get_state()) &&
            (23.0f == last_target_yaw) &&
            (2u == vision_ack_count)) ? 1u : 0u;
}

static uint8 complete_heading_restore(const subject2_context_struct *context,
                                      subject2_update_struct *update)
{
    if(SUBJECT2_RESTORE_HEADING != subject2_get_state()) return 0u;
    fake_control_status.yaw_error = 0.0f;
    subject2_tick(context, update);
    fake_time_ms += SUBJECT2_TURN_STABLE_MS;
    subject2_tick(context, update);
    return (SUBJECT2_SELECT_PUSH == subject2_get_state()) ? 1u : 0u;
}

static uint8 scan_to_select_push(subject2_context_struct *context,
                                 subject2_update_struct *update)
{
    return ((0u != scan_to_heading_restore(context, update)) &&
            (0u != complete_heading_restore(context, update))) ? 1u : 0u;
}

static uint8 run_scan(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 center_before;
    uint16 observation_before;

    init_context(&context);
    if(0 == scan_to_select_push(&context, &update)) return 0u;

    subject2_tick(&context, &update);
    if((SUBJECT2_EXECUTE_PUSH != subject2_get_state()) ||
       (2u != executor_start_count) ||
       (1u != executor_start_art_sync) ||
       (4u != subject2_get_active_class())) return 0u;

    center_before = center_request_count;
    observation_before = observation_request_count;
    fake_pre_push_pending = 1u;
    subject2_tick(&context, &update);
    if((SUBJECT2_PRE_PUSH_CENTER != subject2_get_state()) ||
       (center_before != center_request_count) ||
       (observation_before + 1u != observation_request_count) ||
       (5u != observation_request_row) || (6u != observation_request_col) ||
       (0 != strcmp(update.run_state, "BCtr"))) return 0u;

    feed_observation(550u, 550u, 650u, 550u);
    subject2_tick(&context, &update);
    if((SUBJECT2_PRE_PUSH_CENTER != subject2_get_state()) ||
       (1u != start_pre_push_box_count) ||
       (0 != strcmp(update.run_state, "BGap"))) return 0u;
    fake_pre_push_box_active = 0u;
    subject2_tick(&context, &update);
    if((SUBJECT2_EXECUTE_PUSH != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "S2Push"))) return 0u;

    fake_sync_pending = 1u;
    subject2_tick(&context, &update);
    if(SUBJECT2_CONFIRM_MAP != subject2_get_state()) return 0u;

    live_rows[5][6] = '.';
    live_rows[5][9] = '.';
    move_live_car(executor_target_row, executor_target_col);
    fake_frame_count++;
    subject2_tick(&context, &update);
    if((SUBJECT2_REPLAN_CENTER != subject2_get_state()) ||
       (center_before + 1u != center_request_count)) return 0u;

    feed_center((uint16)(executor_target_col * 100u + 50u),
                (uint16)(executor_target_row * 100u + 50u));
    subject2_tick(&context, &update);
    return ((SUBJECT2_RETURN_REQUESTED == subject2_get_state()) &&
            (0u != update.return_requested)) ? 1u : 0u;
}

static uint8 host_completion_before_task_end_enters_confirm(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == scan_to_select_push(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    if(SUBJECT2_EXECUTE_PUSH != subject2_get_state()) return 0u;

    live_rows[5][6] = '.';
    live_rows[5][9] = '.';
    fake_frame_count++;
    subject2_tick(&context, &update);
    if((SUBJECT2_CONFIRM_MAP != subject2_get_state()) ||
       (EXEC_STATE_IDLE != fake_executor_state) ||
       (0 != strcmp(update.run_state, "ART Wait"))) return 0u;

    fake_frame_count++;
    subject2_tick(&context, &update);
    return ((SUBJECT2_REPLAN_CENTER == subject2_get_state()) &&
            (3u == center_request_count) &&
            ('.' == snapshot.rows[5][6]) &&
            ('.' == snapshot.rows[5][9])) ? 1u : 0u;
}

static uint8 pre_push_box_observation_timeout_stops(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0 == scan_to_select_push(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    if(SUBJECT2_EXECUTE_PUSH != subject2_get_state()) return 0u;

    fake_pre_push_pending = 1u;
    subject2_tick(&context, &update);
    if((SUBJECT2_PRE_PUSH_CENTER != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "BCtr"))) return 0u;

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);

    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (0u == continue_pre_push_count) &&
            (EXEC_ERROR_ART_CENTER == fake_executor_error) &&
            (0 == strcmp(update.run_state, "E:BObs"))) ? 1u : 0u;
}

static uint8 turn_center_continues_without_push_alignment(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0 == scan_to_select_push(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    if(SUBJECT2_EXECUTE_PUSH != subject2_get_state()) return 0u;

    fake_pre_push_pending = 1u;
    fake_pre_push_box_request = 0u;
    fake_center_requires_push_alignment = 0u;
    subject2_tick(&context, &update);
    if(SUBJECT2_PRE_PUSH_CENTER != subject2_get_state()) return 0u;

    feed_center((uint16)(executor_target_col * 100u + 50u),
                (uint16)(executor_target_row * 100u + 50u));
    subject2_tick(&context, &update);

    return ((SUBJECT2_EXECUTE_PUSH == subject2_get_state()) &&
            (1u == continue_pre_push_count) &&
            (0u == start_pre_push_alignment_count)) ? 1u : 0u;
}

static uint8 invalid_pre_push_box_geometry_stops(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0 == scan_to_select_push(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    if(SUBJECT2_EXECUTE_PUSH != subject2_get_state()) return 0u;

    fake_pre_push_pending = 1u;
    subject2_tick(&context, &update);
    if(SUBJECT2_PRE_PUSH_CENTER != subject2_get_state()) return 0u;

    fake_pre_push_box_result = EXEC_ART_BOX_PREP_GEOMETRY_ERROR;
    feed_observation(550u, 550u, 650u, 550u);
    subject2_tick(&context, &update);

    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "E:BGeo")) &&
            (EXEC_STATE_ERROR == fake_executor_state) &&
            (EXEC_ERROR_ART_CENTER == fake_executor_error)) ? 1u : 0u;
}

static uint8 pre_push_box_motion_error_has_timeout_reason(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0 == scan_to_select_push(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    if(SUBJECT2_EXECUTE_PUSH != subject2_get_state()) return 0u;

    fake_pre_push_pending = 1u;
    subject2_tick(&context, &update);
    feed_observation(550u, 550u, 650u, 550u);
    subject2_tick(&context, &update);
    if((SUBJECT2_PRE_PUSH_CENTER != subject2_get_state()) ||
       (0u == fake_pre_push_box_active)) return 0u;

    fake_pre_push_box_active = 0u;
    fake_executor_error = EXEC_ERROR_ART_CENTER;
    fake_executor_state = EXEC_STATE_ERROR;
    subject2_tick(&context, &update);
    if((SUBJECT2_ERROR != subject2_get_state()) ||
       (EXEC_ERROR_ART_CENTER != fake_executor_error) ||
       (0 != strcmp(update.run_state, "E:BTim"))) return 0u;

    build_map();
    init_context(&context);
    if(0 == scan_to_select_push(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    fake_pre_push_pending = 1u;
    subject2_tick(&context, &update);
    feed_observation(550u, 550u, 650u, 550u);
    subject2_tick(&context, &update);
    fake_error_on_box_active_query = 1u;
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "E:BTim"))) ? 1u : 0u;
}

static uint8 push_retry_tracks_active_box(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint8 car_row;
    uint8 car_col;
    uint16 starts_before;
    uint16 yaw_sets_before;

    build_map();
    init_context(&context);
    if(0 == scan_to_select_push(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    if(SUBJECT2_EXECUTE_PUSH != subject2_get_state()) return 0u;
    starts_before = executor_start_count;
    yaw_sets_before = set_target_yaw_count;

    fake_sync_pending = 1u;
    subject2_tick(&context, &update);
    if(SUBJECT2_CONFIRM_MAP != subject2_get_state()) return 0u;
    live_rows[5][6] = '.';
    live_rows[5][7] = 'B';
    fake_frame_count++;
    subject2_tick(&context, &update);
    if(SUBJECT2_REPLAN_CENTER != subject2_get_state()) return 0u;

    if(0 == map_find_car(&live_source, &car_row, &car_col, 0)) return 0u;
    feed_center((uint16)(car_col * 100u + 50u),
                (uint16)(car_row * 100u + 50u));
    subject2_tick(&context, &update);
    if(SUBJECT2_SELECT_PUSH != subject2_get_state()) return 0u;
    fake_sync_pending = 0u;
    subject2_tick(&context, &update);

    return ((SUBJECT2_EXECUTE_PUSH == subject2_get_state()) &&
            (4u == subject2_get_active_class()) &&
            (yaw_sets_before == set_target_yaw_count) &&
            (executor_start_count == (uint16)(starts_before + 1u))) ? 1u : 0u;
}

static uint8 classification_timeout_retries(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 correction_count_before_backoff;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) return 0u;
    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    if(0u == complete_center_adjust(&context, &update,
                                    SUBJECT2_SCAN_BOX_ADJUST,
                                    SUBJECT2_SCAN_BOX_TURN)) return 0u;
    complete_observation_turn(&context, &update);
    if(SUBJECT2_SCAN_BOX_CLASSIFY != subject2_get_state()) return 0u;

    correction_count_before_backoff = correction_start_count;
    fake_time_ms += SUBJECT2_VIEW_TIMEOUT_MS - 1u;
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_CLASSIFY != subject2_get_state()) ||
       (correction_count_before_backoff != correction_start_count)) return 0u;

    fake_time_ms += 1u;
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_BACKOFF != subject2_get_state()) ||
       (correction_count_before_backoff + 1u != correction_start_count) ||
       (-SUBJECT2_VIEW_BACKOFF_CM != correction_target_x) ||
       (0.0f != correction_target_y) ||
       (0 != strcmp(update.run_state, "VBack"))) return 0u;

    fake_pose.x_cm = correction_target_x;
    fake_pose.y_cm = correction_target_y;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_CLASSIFY != subject2_get_state()) ||
       (2u != vision_request_id)) return 0u;

    fake_time_ms += SUBJECT2_VIEW_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_BACKOFF_RETURN != subject2_get_state()) ||
       (correction_count_before_backoff + 2u != correction_start_count) ||
       (0.0f != correction_target_x) || (0.0f != correction_target_y) ||
       (0 != strcmp(update.run_state, "VHome"))) return 0u;

    fake_pose.x_cm = 0.0f;
    fake_pose.y_cm = 0.0f;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_PLAN == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "VRetry"))) ? 1u : 0u;
}

static uint8 classification_backoff_success_returns_to_origin(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint8 recognition_is_target = 1u;
    uint8 recognition_class = SUBJECT2_INVALID_CLASS;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    if(0u == complete_center_adjust(&context, &update,
                                    SUBJECT2_SCAN_BOX_ADJUST,
                                    SUBJECT2_SCAN_BOX_TURN)) return 0u;
    complete_observation_turn(&context, &update);

    fake_time_ms += SUBJECT2_VIEW_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_BACKOFF != subject2_get_state()) return 0u;

    fake_pose.x_cm = correction_target_x;
    fake_pose.y_cm = correction_target_y;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_CLASSIFY != subject2_get_state()) ||
       (2u != vision_request_id)) return 0u;

    feed_class(4u);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_BACKOFF_RETURN != subject2_get_state()) ||
       (0.0f != correction_target_x) || (0.0f != correction_target_y) ||
       (1u != vision_ack_count)) return 0u;

    fake_pose.x_cm = 0.0f;
    fake_pose.y_cm = 0.0f;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_TARGET_MODE == subject2_get_state()) &&
            (0u != subject2_get_last_recognition(&recognition_is_target,
                                                 &recognition_class)) &&
            (0u == recognition_is_target) && (4u == recognition_class)) ? 1u : 0u;
}

static uint8 backoff_target_matches_observation(uint8 car_row, uint8 car_col,
                                                uint8 box_row, uint8 box_col,
                                                float expected_x,
                                                float expected_y)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    live_rows[5][5] = '.';
    live_rows[5][6] = '.';
    live_rows[car_row][car_col] = 'C';
    live_rows[box_row][box_col] = 'B';
    map_source_snapshot(&snapshot, snapshot_rows, &live_source);
    start_row = car_row;
    start_col = car_col;
    init_context(&context);

    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) return 0u;
    feed_center((uint16)(car_col * 100u + 50u),
                (uint16)(car_row * 100u + 50u));
    subject2_tick(&context, &update);
    if(0u == complete_center_adjust(&context, &update,
                                    SUBJECT2_SCAN_BOX_ADJUST,
                                    SUBJECT2_SCAN_BOX_TURN)) return 0u;
    complete_observation_turn(&context, &update);
    fake_time_ms += SUBJECT2_VIEW_TIMEOUT_MS;
    subject2_tick(&context, &update);

    return ((SUBJECT2_SCAN_BOX_BACKOFF == subject2_get_state()) &&
            (expected_x == correction_target_x) &&
            (expected_y == correction_target_y)) ? 1u : 0u;
}

static uint8 classification_backoff_uses_all_four_directions(void)
{
    return ((0u != backoff_target_matches_observation(5u, 5u, 5u, 6u,
                                                       -SUBJECT2_VIEW_BACKOFF_CM, 0.0f)) &&
            (0u != backoff_target_matches_observation(5u, 7u, 5u, 6u,
                                                        SUBJECT2_VIEW_BACKOFF_CM, 0.0f)) &&
            (0u != backoff_target_matches_observation(4u, 6u, 5u, 6u,
                                                       0.0f, SUBJECT2_VIEW_BACKOFF_CM)) &&
            (0u != backoff_target_matches_observation(6u, 6u, 5u, 6u,
                                                       0.0f, -SUBJECT2_VIEW_BACKOFF_CM))) ? 1u : 0u;
}

static uint8 begin_box_classification(subject2_context_struct *context,
                                      subject2_update_struct *update)
{
    build_map();
    init_context(context);
    subject2_begin(context, 0.0f, 0.0f, update);
    fake_ready_box = 1u;
    subject2_tick(context, update);
    subject2_tick(context, update);
    if(SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) return 0u;
    feed_center(550u, 550u);
    subject2_tick(context, update);
    if(0u == complete_center_adjust(context, update,
                                    SUBJECT2_SCAN_BOX_ADJUST,
                                    SUBJECT2_SCAN_BOX_TURN)) return 0u;
    complete_observation_turn(context, update);
    return (SUBJECT2_SCAN_BOX_CLASSIFY == subject2_get_state()) ? 1u : 0u;
}

static uint8 backoff_motion_error_stops(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    if(0u == begin_box_classification(&context, &update)) return 0u;
    fake_time_ms += SUBJECT2_VIEW_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_BACKOFF != subject2_get_state()) return 0u;

    fake_executor_state = EXEC_STATE_ERROR;
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_ERROR_SUBJECT2_CLASS == fake_executor_error) &&
            (0 == strcmp(update.run_state, "E:Back"))) ? 1u : 0u;
}

static uint8 backoff_return_timeout_stops(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    if(0u == begin_box_classification(&context, &update)) return 0u;
    fake_time_ms += SUBJECT2_VIEW_TIMEOUT_MS;
    subject2_tick(&context, &update);
    fake_pose.x_cm = correction_target_x;
    fake_pose.y_cm = correction_target_y;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    feed_class(4u);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_BACKOFF_RETURN != subject2_get_state()) return 0u;

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_ERROR_SUBJECT2_CLASS == fake_executor_error) &&
            (0 == strcmp(update.run_state, "E:Back"))) ? 1u : 0u;
}

static uint8 complete_current_box_observation(subject2_context_struct *context,
                                              subject2_update_struct *update)
{
    uint8 car_row;
    uint8 car_col;

    if(SUBJECT2_SCAN_BOX_MOVE == subject2_get_state())
    {
        move_live_car(executor_target_row, executor_target_col);
        fake_executor_state = EXEC_STATE_DONE;
        subject2_tick(context, update);
    }
    if((SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) ||
       (0 == map_find_car(&live_source, &car_row, &car_col, 0))) return 0u;
    feed_center((uint16)(car_col * 100u + 50u),
                (uint16)(car_row * 100u + 50u));
    subject2_tick(context, update);
    if(0u == complete_center_adjust(context, update,
                                    SUBJECT2_SCAN_BOX_ADJUST,
                                    SUBJECT2_SCAN_BOX_TURN)) return 0u;
    complete_observation_turn(context, update);
    return (SUBJECT2_SCAN_BOX_CLASSIFY == subject2_get_state()) ? 1u : 0u;
}

static uint8 begin_duplicate_box_backoff(subject2_context_struct *context,
                                         subject2_update_struct *update)
{
    build_map();
    live_rows[7][6] = 'B';
    live_rows[7][9] = 'T';
    map_source_snapshot(&snapshot, snapshot_rows, &live_source);
    init_context(context);
    subject2_begin(context, 0.0f, 0.0f, update);
    fake_ready_box = 1u;
    subject2_tick(context, update);
    subject2_tick(context, update);
    if(0u == complete_current_box_observation(context, update)) return 0u;
    feed_class(4u);
    subject2_tick(context, update);
    if(SUBJECT2_SCAN_BOX_PLAN != subject2_get_state()) return 0u;

    subject2_tick(context, update);
    if(0u == complete_current_box_observation(context, update)) return 0u;
    feed_class(4u);
    subject2_tick(context, update);
    return (SUBJECT2_SCAN_BOX_BACKOFF == subject2_get_state()) ? 1u : 0u;
}

static uint8 duplicate_class_backoff_can_recover(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    if(0u == begin_duplicate_box_backoff(&context, &update)) return 0u;
    fake_pose.x_cm = correction_target_x;
    fake_pose.y_cm = correction_target_y;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    feed_class(5u);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_BACKOFF_RETURN != subject2_get_state()) return 0u;
    fake_pose.x_cm = 0.0f;
    fake_pose.y_cm = 0.0f;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_TARGET_MODE == subject2_get_state()) &&
            (2u == vision_ack_count)) ? 1u : 0u;
}

static uint8 duplicate_class_twice_changes_observation(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint8 failed_row;
    uint8 failed_col;

    if(0u == begin_duplicate_box_backoff(&context, &update)) return 0u;
    failed_row = executor_target_row;
    failed_col = executor_target_col;
    fake_pose.x_cm = correction_target_x;
    fake_pose.y_cm = correction_target_y;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    feed_class(4u);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_BACKOFF_RETURN != subject2_get_state()) return 0u;
    fake_pose.x_cm = 0.0f;
    fake_pose.y_cm = 0.0f;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_PLAN != subject2_get_state()) return 0u;
    subject2_tick(&context, &update);

    return ((SUBJECT2_SCAN_BOX_MOVE == subject2_get_state()) &&
            ((failed_row != executor_target_row) ||
             (failed_col != executor_target_col))) ? 1u : 0u;
}

static uint8 target_classification_uses_same_backoff_flow(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint8 car_row;
    uint8 car_col;

    if(0u == begin_box_classification(&context, &update)) return 0u;
    feed_class(4u);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_TARGET_MODE != subject2_get_state()) return 0u;
    fake_ready_target = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_TARGET_MOVE == subject2_get_state())
    {
        move_live_car(executor_target_row, executor_target_col);
        fake_executor_state = EXEC_STATE_DONE;
        subject2_tick(&context, &update);
    }
    if((SUBJECT2_SCAN_TARGET_CENTER != subject2_get_state()) ||
       (0 == map_find_car(&live_source, &car_row, &car_col, 0))) return 0u;
    feed_center((uint16)(car_col * 100u + 50u),
                (uint16)(car_row * 100u + 50u));
    subject2_tick(&context, &update);
    if(0u == complete_center_adjust(&context, &update,
                                    SUBJECT2_SCAN_TARGET_ADJUST,
                                    SUBJECT2_SCAN_TARGET_TURN)) return 0u;
    complete_observation_turn(&context, &update);

    fake_time_ms += SUBJECT2_VIEW_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_TARGET_BACKOFF != subject2_get_state()) return 0u;
    fake_pose.x_cm = correction_target_x;
    fake_pose.y_cm = correction_target_y;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    feed_class(4u);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_TARGET_BACKOFF_RETURN != subject2_get_state()) return 0u;
    fake_pose.x_cm = 0.0f;
    fake_pose.y_cm = 0.0f;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    if(SUBJECT2_VALIDATE_BINDINGS != subject2_get_state()) return 0u;
    subject2_tick(&context, &update);
    return complete_heading_restore(&context, &update);
}

static uint8 heading_restore_requires_new_continuous_window(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == scan_to_heading_restore(&context, &update)) return 0u;

    fake_control_status.yaw_error = 0.0f;
    subject2_tick(&context, &update);
    fake_time_ms += 80u;
    subject2_tick(&context, &update);
    fake_control_status.yaw_error = SUBJECT2_TURN_TOLERANCE_DEG + 1.0f;
    subject2_tick(&context, &update);
    fake_control_status.yaw_error = 0.0f;
    subject2_tick(&context, &update);
    fake_time_ms += 99u;
    subject2_tick(&context, &update);
    if(SUBJECT2_RESTORE_HEADING != subject2_get_state()) return 0u;
    fake_time_ms += 1u;
    subject2_tick(&context, &update);
    return (SUBJECT2_SELECT_PUSH == subject2_get_state()) ? 1u : 0u;
}

static uint8 heading_restore_timeout_stops(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == scan_to_heading_restore(&context, &update)) return 0u;

    fake_control_status.yaw_error = 2.0f;
    fake_time_ms += SUBJECT2_TURN_TIMEOUT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_STATE_ERROR == fake_executor_state) &&
            (EXEC_ERROR_SUBJECT2_YAW == fake_executor_error) &&
            (0 == strcmp(update.run_state, "E:HYaw"))) ? 1u : 0u;
}

static uint8 map_change_stops_scan(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    live_rows[5][6] = '.';
    live_rows[5][7] = 'B';
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_STATE_ERROR == fake_executor_state)) ? 1u : 0u;
}

static uint8 scan_center_timeout_follows_policy(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) return 0u;
    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    return ((SUBJECT2_SCAN_BOX_TURN == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0 == strcmp(update.run_state, "VTurn"))) ? 1u : 0u;
#else
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_STATE_ERROR == fake_executor_state) &&
            (EXEC_ERROR_ART_CENTER == fake_executor_error) &&
            (0 == strcmp(update.run_state, "E:CTmo"))) ? 1u : 0u;
#endif
}

static uint8 replan_center_timeout_follows_policy(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0 == scan_to_select_push(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    if(SUBJECT2_EXECUTE_PUSH != subject2_get_state()) return 0u;

    fake_sync_pending = 1u;
    subject2_tick(&context, &update);
    if(SUBJECT2_CONFIRM_MAP != subject2_get_state()) return 0u;
    live_rows[5][6] = '.';
    live_rows[5][7] = 'B';
    fake_frame_count++;
    subject2_tick(&context, &update);
    if(SUBJECT2_REPLAN_CENTER != subject2_get_state()) return 0u;

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    return ((SUBJECT2_SELECT_PUSH == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "Bind")) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0.0f == fake_pose.x_cm) && (0.0f == fake_pose.y_cm)) ? 1u : 0u;
#else
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "E:CRpl"))) ? 1u : 0u;
#endif
}

static uint8 turn_requires_continuous_100ms(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) return 0u;
    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    if(0u == complete_center_adjust(&context, &update,
                                    SUBJECT2_SCAN_BOX_ADJUST,
                                    SUBJECT2_SCAN_BOX_TURN)) return 0u;
    if((SUBJECT2_SCAN_BOX_TURN != subject2_get_state()) ||
       (90.0f != last_target_yaw)) return 0u;

    fake_control_status.yaw_error = SUBJECT2_TURN_TOLERANCE_DEG * 0.5f;
    subject2_tick(&context, &update);
    fake_time_ms += 80u;
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_TURN != subject2_get_state()) return 0u;

    fake_control_status.yaw_error = SUBJECT2_TURN_TOLERANCE_DEG + 1.0f;
    subject2_tick(&context, &update);
    fake_control_status.yaw_error = SUBJECT2_TURN_TOLERANCE_DEG * 0.5f;
    subject2_tick(&context, &update);
    fake_time_ms += 99u;
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_TURN != subject2_get_state()) return 0u;

    fake_time_ms += 1u;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_CLASSIFY == subject2_get_state()) &&
            (1u == center_request_count) &&
            (0u == observation_request_count)) ? 1u : 0u;
}

static uint8 turn_timeout_stops(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) return 0u;
    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    if(0u == complete_center_adjust(&context, &update,
                                    SUBJECT2_SCAN_BOX_ADJUST,
                                    SUBJECT2_SCAN_BOX_TURN)) return 0u;

    fake_control_status.yaw_error = 5.0f;
    fake_time_ms = SUBJECT2_TURN_TIMEOUT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_ERROR_SUBJECT2_YAW == fake_executor_error) &&
            (0 == strcmp(update.run_state, "E:Yaw"))) ? 1u : 0u;
}

static uint8 vision_ready_retries_then_times_out(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    if(1u != vision_mode_send_count) return 0u;

    fake_time_ms = SUBJECT2_VISION_READY_RETRY_MS;
    subject2_tick(&context, &update);
    if(2u != vision_mode_send_count) return 0u;

    fake_time_ms = SUBJECT2_VISION_READY_TIMEOUT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_ERROR_SUBJECT2_CLASS == fake_executor_error) &&
            (0 == strcmp(update.run_state, "E:VMod"))) ? 1u : 0u;
}

static uint8 begin_box_center(subject2_context_struct *context,
                              subject2_update_struct *update)
{
    subject2_begin(context, 0.0f, 0.0f, update);
    fake_ready_box = 1u;
    subject2_tick(context, update);
    subject2_tick(context, update);
    return (SUBJECT2_SCAN_BOX_CENTER == subject2_get_state()) ? 1u : 0u;
}

static uint8 center_offset_adjusts_before_turn_and_classify(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;

    feed_center(560u, 550u);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_ADJUST != subject2_get_state()) ||
       (1u != correction_start_count) ||
       (0.0f != correction_target_x) ||
       (0.0f != correction_target_y) ||
       (1u != center_request_count) ||
       (fake_pose.x_cm < 2.3f) ||
       (fake_pose.x_cm > 2.5f) ||
       (0u != vision_request_id)) return 0u;

    fake_pose.x_cm = 0.0f;
    fake_pose.y_cm = 0.0f;
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_TURN != subject2_get_state()) ||
       (1u != center_request_count) ||
       (0u != vision_request_id)) return 0u;
    complete_observation_turn(&context, &update);
    return ((SUBJECT2_SCAN_BOX_CLASSIFY == subject2_get_state()) &&
            (1u == vision_request_id)) ? 1u : 0u;
}

static uint8 small_center_offset_still_enters_adjustment(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;

    feed_center(551u, 550u);
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_ADJUST == subject2_get_state()) &&
            (1u == correction_start_count) &&
            (0.0f == correction_target_x) &&
            (0.0f == correction_target_y) &&
            (0u == vision_request_id)) ? 1u : 0u;
}

static uint8 center_adjustment_timeout_stops(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;

    feed_center(560u, 550u);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_ADJUST != subject2_get_state()) return 0u;

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_ERROR_ART_CENTER == fake_executor_error) &&
            (0 == strcmp(update.run_state, "E:CTim"))) ? 1u : 0u;
}

static uint8 center_adjustment_busy_has_reason(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;

    correction_start_allowed = 0u;
    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "E:CBsy"))) ? 1u : 0u;
}

static uint8 center_adjustment_executor_error_has_reason(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;

    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_ADJUST != subject2_get_state()) return 0u;
    fake_executor_state = EXEC_STATE_ERROR;
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "E:CExe"))) ? 1u : 0u;
}

static uint8 center_adjacent_map_uses_median(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;
    move_live_car(4u, 5u);
    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_ADJUST == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "VAdj")) &&
            (1u == correction_start_count) &&
            (5u == start_row) && (5u == start_col)) ? 1u : 0u;
}

static uint8 center_uses_paired_map_after_global_multi_car(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;

    feed_center(550u, 550u);
    live_rows[4][5] = 'C';
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_ADJUST == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "VAdj")) &&
            (1u == correction_start_count)) ? 1u : 0u;
}

static uint8 center_map_mismatch_times_out_with_reason(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;
    move_live_car(3u, 5u);
    feed_center(550u, 550u);
    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_ERROR_ART_CENTER == fake_executor_error) &&
            (0 == strcmp(update.run_state, "E:CMap"))) ? 1u : 0u;
}

static uint8 paired_center_on_target_replans_observation(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;

    live_rows[5][5] = '.';
    live_rows[5][9] = '+';
    feed_center(950u, 550u);
    subject2_tick(&context, &update);

    return ((SUBJECT2_SCAN_BOX_PLAN == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "VReplan")) &&
            (1u == pose_reset_count) &&
            (5u == start_row) && (9u == start_col) &&
            (0u == correction_start_count)) ? 1u : 0u;
}

static uint8 center_abnormal_stops(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;
    feed_center(1550u, 550u);
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_ERROR_ART_CENTER == fake_executor_error) &&
            (0u == correction_start_count) &&
            (0 == strcmp(update.run_state, "E:CRef"))) ? 1u : 0u;
}

static uint8 classification_request_is_not_resent(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;
    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    if(0u == complete_center_adjust(&context, &update,
                                    SUBJECT2_SCAN_BOX_ADJUST,
                                    SUBJECT2_SCAN_BOX_TURN)) return 0u;
    complete_observation_turn(&context, &update);
    if((SUBJECT2_SCAN_BOX_CLASSIFY != subject2_get_state()) ||
       (1u != vision_request_id)) return 0u;

    fake_time_ms += SUBJECT2_VIEW_TIMEOUT_MS - 1u;
    subject2_tick(&context, &update);
    if((1u != vision_request_id) || (0u != vision_cancel_count)) return 0u;
    fake_time_ms += 1u;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_BACKOFF == subject2_get_state()) &&
            (1u == vision_request_id) &&
            (1u == vision_cancel_count)) ? 1u : 0u;
}

static uint8 step_mode_only_pauses_navigation(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    move_live_car(5u, 3u);
    map_source_snapshot(&snapshot, snapshot_rows, &live_source);
    init_context(&context);
    context.run_mode = RUN_MODE_STEP;
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_MOVE != subject2_get_state()) ||
       (1u != executor_start_single_step)) return 0u;

    move_live_car(executor_target_row, executor_target_col);
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) return 0u;
    feed_center((uint16)(executor_target_col * 100u + 50u),
                (uint16)(executor_target_row * 100u + 50u));
    subject2_tick(&context, &update);
    if(0u == complete_center_adjust(&context, &update,
                                    SUBJECT2_SCAN_BOX_ADJUST,
                                    SUBJECT2_SCAN_BOX_TURN)) return 0u;
    complete_observation_turn(&context, &update);
    return (SUBJECT2_SCAN_BOX_CLASSIFY == subject2_get_state()) ? 1u : 0u;
}

int main(void)
{
    uint8 passed = 1u;

    build_map();
    passed &= run_scan();
    printf("subject2-scan-state         %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= classification_timeout_retries();
    printf("subject2-scan-timeout       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= classification_backoff_success_returns_to_origin();
    printf("subject2-scan-backoff       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= classification_backoff_uses_all_four_directions();
    printf("subject2-backoff-direction  %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= backoff_motion_error_stops();
    printf("subject2-backoff-error      %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= backoff_return_timeout_stops();
    printf("subject2-backoff-return-tmo %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= duplicate_class_backoff_can_recover();
    printf("subject2-backoff-conflict   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= duplicate_class_twice_changes_observation();
    printf("subject2-backoff-reobserve  %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= target_classification_uses_same_backoff_flow();
    printf("subject2-target-backoff     %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= heading_restore_requires_new_continuous_window();
    printf("subject2-home-yaw-stable    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= heading_restore_timeout_stops();
    printf("subject2-home-yaw-timeout   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= map_change_stops_scan();
    printf("subject2-scan-map-change    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= host_completion_before_task_end_enters_confirm();
    printf("subject2-host-completion    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= push_retry_tracks_active_box();
    printf("subject2-push-retry         %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= scan_center_timeout_follows_policy();
    printf("subject2-center-timeout     %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= replan_center_timeout_follows_policy();
    printf("subject2-replan-timeout     %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= turn_requires_continuous_100ms();
    printf("subject2-turn-stable        %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= turn_timeout_stops();
    printf("subject2-turn-timeout       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= vision_ready_retries_then_times_out();
    printf("subject2-ready-timeout      %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_offset_adjusts_before_turn_and_classify();
    printf("subject2-center-pose        %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= small_center_offset_still_enters_adjustment();
    printf("subject2-center-small       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_adjustment_timeout_stops();
    printf("subject2-center-adjust-tmo  %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_adjustment_busy_has_reason();
    printf("subject2-center-adjust-busy %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_adjustment_executor_error_has_reason();
    printf("subject2-center-adjust-exec %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_adjacent_map_uses_median();
    printf("subject2-center-adjacent    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_uses_paired_map_after_global_multi_car();
    printf("subject2-center-paired-map  %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_map_mismatch_times_out_with_reason();
    printf("subject2-center-map-tmo     %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= paired_center_on_target_replans_observation();
    printf("subject2-center-target-plan %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_abnormal_stops();
    printf("subject2-center-abnormal    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= classification_request_is_not_resent();
    printf("subject2-request-once       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= step_mode_only_pauses_navigation();
    printf("subject2-step-observe       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= pre_push_box_observation_timeout_stops();
    printf("subject2-box-observe-tmo    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= turn_center_continues_without_push_alignment();
    printf("subject2-turn-center        %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= invalid_pre_push_box_geometry_stops();
    printf("subject2-box-geometry       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= pre_push_box_motion_error_has_timeout_reason();
    printf("subject2-box-motion-error   %s\n", (0u != passed) ? "PASS" : "FAIL");
    return (0u != passed) ? 0 : 1;
}
