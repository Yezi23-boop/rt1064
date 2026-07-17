#include <stdio.h>
#include <string.h>
#include <math.h>
#include "drive_control.h"
#include "drive_pose.h"
#include "executor.h"
#include "map_utils.h"
#include "openart_uart.h"
#include "subject2.h"
#include "subject3.h"
#include "vision_uart.h"
#include "competition_flow.h"

#ifndef ART_CENTER_TIMEOUT_FALLBACK_ENABLE
#define ART_CENTER_TIMEOUT_FALLBACK_ENABLE (1)
#endif

static uint32 fake_time_ms;
static executor_state_enum fake_executor_state;
static executor_error_enum fake_executor_error;
static uint8 fake_push_boundary_ready;
static uint8 fake_push_boundary_requested;
static uint16 push_boundary_request_count;
static uint16 force_push_stop_count;
static uint16 resume_push_stop_count;
static drive_pose_struct fake_pose;
static control_status_struct fake_control_status;
static uint16 set_motion_count;
static uint16 set_target_yaw_count;
static float last_target_yaw;
static uint16 relative_yaw_correction_count;
static float relative_yaw_correction_deg;
static uint16 yaw_rebase_count;
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
static uint16 center_yaw_q[ART_CENTER_SAMPLE_COUNT];
static uint8 center_yaw_valid[ART_CENTER_SAMPLE_COUNT];
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
static uint16 start_pre_push_box_retry_count;
static uint8 fake_center_requires_push_alignment;
static executor_art_center_result_enum fake_center_result;
static uint8 fake_executor_center_median_valid;
static uint16 fake_executor_center_median_col_q;
static uint16 fake_executor_center_median_row_q;
static uint8 fake_sync_pending;
static uint16 fake_current_step;
static char fake_art_sync_action;
static uint16 continue_pre_push_count;
static uint16 start_pre_push_alignment_count;
static uint32 fake_frame_count;
static uint16 periodic_center_col_q;
static uint16 periodic_center_row_q;
static uint8 periodic_center_valid;
static uint8 periodic_center_auto_invalid_frame;
static uint16 periodic_center_read_count;
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

static uint8 complete_current_box_observation(
    subject2_context_struct *context,
    subject2_update_struct *update);
static uint8 complete_current_target_observation(
    subject2_context_struct *context,
    subject2_update_struct *update);

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
uint8 drive_control_is_healthy(void) { return 1u; }
drive_health_fault_enum drive_control_get_health_fault(void)
{
    return DRIVE_HEALTH_NONE;
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
    fake_control_status.target_yaw = yaw;
}
void drive_control_start_relative_yaw_correction(float delta_deg)
{
    relative_yaw_correction_count++;
    relative_yaw_correction_deg = delta_deg;
    fake_control_status.target_yaw = fake_control_status.current_yaw + delta_deg;
    fake_control_status.yaw_error = delta_deg;
}
void drive_control_lock_yaw_and_reset_pose(void)
{
    yaw_rebase_count++;
    fake_control_status.target_yaw = fake_control_status.current_yaw;
    fake_control_status.yaw_error = 0.0f;
    drive_pose_reset(0.0f, 0.0f, 0.0f);
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
    fake_push_boundary_ready = 0u;
    fake_push_boundary_requested = 0u;
}
uint8 executor_request_stop_after_current_push(void)
{
    if((EXEC_STATE_RUNNING != fake_executor_state) ||
       (0u != fake_pre_push_pending) || (0u != fake_sync_pending))
    {
        return 0u;
    }
    push_boundary_request_count++;
    fake_push_boundary_requested = 1u;
    return 1u;
}
uint8 executor_stop_after_current_push_ready(void)
{
    return fake_push_boundary_ready;
}
uint8 executor_force_current_push_stop(void)
{
    if(0u == fake_push_boundary_requested) return 0u;
    force_push_stop_count++;
    fake_push_boundary_ready = 1u;
    return 1u;
}
uint8 executor_resume_after_current_push_stop(void)
{
    if(0u == fake_push_boundary_ready) return 0u;
    resume_push_stop_count++;
    fake_push_boundary_ready = 0u;
    fake_push_boundary_requested = 0u;
    fake_executor_state = EXEC_STATE_RUNNING;
    return 1u;
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
uint8 executor_start_position_correction_with_pose_reset(
    float initial_x, float initial_y,
    float target_x, float target_y)
{
    if(0u == correction_start_allowed) return 0u;
    drive_pose_reset(initial_x, initial_y, fake_pose.yaw_deg);
    return executor_start_position_correction(target_x, target_y);
}
executor_state_enum executor_get_state(void) { return fake_executor_state; }
executor_error_enum executor_get_error(void) { return fake_executor_error; }
void executor_set_error(executor_error_enum error)
{
    fake_executor_error = error;
    fake_executor_state = EXEC_STATE_ERROR;
}
void executor_reset_art_player_center_samples(void)
{
    fake_executor_center_median_valid = 0u;
}
uint8 executor_apply_art_player_center(uint16 col, uint16 row, uint32 sample)
{
    (void)col;
    (void)row;
    if(sample >= ART_CENTER_SAMPLE_COUNT)
    {
        fake_executor_center_median_col_q =
            center_col_q[ART_CENTER_SAMPLE_COUNT / 2u];
        fake_executor_center_median_row_q =
            center_row_q[ART_CENTER_SAMPLE_COUNT / 2u];
        fake_executor_center_median_valid = 1u;
        return 1u;
    }
    return 0u;
}
uint8 executor_get_art_player_center_median(uint16 *col_q, uint16 *row_q)
{
    if((0u == fake_executor_center_median_valid) ||
       (0 == col_q) || (0 == row_q))
    {
        return 0u;
    }
    *col_q = fake_executor_center_median_col_q;
    *row_q = fake_executor_center_median_row_q;
    return 1u;
}
executor_art_center_result_enum executor_commit_art_player_center(uint8 row, uint8 col)
{
    (void)row; (void)col;
    return fake_center_result;
}
uint8 executor_art_pre_push_pending(void) { return fake_pre_push_pending; }
uint8 executor_get_pre_push_wait_cell(uint8 *row, uint8 *col)
{
    if((0u == fake_pre_push_pending) || (0 == row) || (0 == col)) return 0u;
    return map_find_car(&live_source, row, col, 0);
}
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
uint8 executor_start_pre_push_box_retry_nudge(float distance_cm)
{
    if((0u == fake_pre_push_pending) ||
       (distance_cm != ART_BOX_OBSERVE_RETRY_MOVE_CM)) return 0u;
    start_pre_push_box_retry_count++;
    fake_pre_push_box_active = 1u;
    fake_pre_push_box_state = "BRetry";
    return 1u;
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
uint16 executor_get_current_step(void) { return fake_current_step; }
char executor_get_art_sync_action(void) { return fake_art_sync_action; }
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
    if((0u == center_count) || (0u != center_read))
    {
        center_count = 0u;
        center_read = 0u;
        paired_center_valid = 0u;
    }
}
uint8 openart_get_requested_center_sample(uint16 *col_q, uint16 *row_q,
                                          uint16 *yaw_q, uint8 *yaw_valid)
{
    if(center_read >= center_count)
    {
        return 0u;
    }
    *col_q = center_col_q[center_read];
    *row_q = center_row_q[center_read];
    *yaw_q = center_yaw_q[center_read];
    *yaw_valid = center_yaw_valid[center_read];
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
uint32 openart_get_player_center(uint16 *col_q, uint16 *row_q, uint8 *valid)
{
    if((0u != periodic_center_auto_invalid_frame) &&
       (0u == periodic_center_valid) &&
       (0u != periodic_center_read_count))
    {
        fake_frame_count++;
    }
    periodic_center_read_count++;
    if(0 != col_q) *col_q = periodic_center_col_q;
    if(0 != row_q) *row_q = periodic_center_row_q;
    if(0 != valid) *valid = periodic_center_valid;
    return fake_frame_count;
}
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
    competition_flow_start(COMPETITION_MODE_SUBJECT2_DEBUG);
    (void)competition_flow_take_action();
    fake_time_ms = 0u;
    fake_executor_state = EXEC_STATE_IDLE;
    fake_executor_error = EXEC_ERROR_NONE;
    fake_push_boundary_ready = 0u;
    fake_push_boundary_requested = 0u;
    push_boundary_request_count = 0u;
    force_push_stop_count = 0u;
    resume_push_stop_count = 0u;
    memset(&fake_pose, 0, sizeof(fake_pose));
    memset(&fake_control_status, 0, sizeof(fake_control_status));
    set_motion_count = 0u;
    set_target_yaw_count = 0u;
    last_target_yaw = 0.0f;
    relative_yaw_correction_count = 0u;
    relative_yaw_correction_deg = 0.0f;
    yaw_rebase_count = 0u;
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
    memset(center_yaw_q, 0, sizeof(center_yaw_q));
    memset(center_yaw_valid, 0, sizeof(center_yaw_valid));
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
    start_pre_push_box_retry_count = 0u;
    fake_center_requires_push_alignment = 1u;
    fake_center_result = EXEC_ART_CENTER_APPLIED;
    fake_executor_center_median_valid = 0u;
    fake_executor_center_median_col_q = 0u;
    fake_executor_center_median_row_q = 0u;
    fake_sync_pending = 0u;
    fake_current_step = 0u;
    fake_art_sync_action = '\0';
    continue_pre_push_count = 0u;
    start_pre_push_alignment_count = 0u;
    fake_frame_count = 0u;
    periodic_center_col_q = 0u;
    periodic_center_row_q = 0u;
    periodic_center_valid = 0u;
    periodic_center_auto_invalid_frame = 0u;
    periodic_center_read_count = 0u;
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
    uint8 old_row;
    uint8 old_col;
    uint8 center_row = (uint8)(row_q / 100u);
    uint8 center_col = (uint8)(col_q / 100u);
    char destination;

    for(index = 0u; index < ART_CENTER_SAMPLE_COUNT; index++)
    {
        center_col_q[index] = (uint16)(col_q + index);
        center_row_q[index] = row_q;
        center_yaw_q[index] = 0u;
        center_yaw_valid[index] = 0u;
    }
    center_count = ART_CENTER_SAMPLE_COUNT;
    center_read = 0u;
    periodic_center_col_q = (uint16)(center_col * 100u + 70u);
    periodic_center_row_q = (uint16)(center_row * 100u + 50u);
    periodic_center_valid = 1u;
    fake_frame_count++;
    map_source_snapshot(&paired_center_source, paired_center_rows, &live_source);
    if((0u != map_find_car(&paired_center_source, &old_row, &old_col, 0)) &&
       (center_row < MAP_ROWS) && (center_col < MAP_COLS) &&
       (('.' == paired_center_rows[center_row][center_col]) ||
        ('T' == paired_center_rows[center_row][center_col]) ||
        ('C' == paired_center_rows[center_row][center_col]) ||
        ('+' == paired_center_rows[center_row][center_col])))
    {
        destination = paired_center_rows[center_row][center_col];
        paired_center_rows[old_row][old_col] =
            ('+' == paired_center_rows[old_row][old_col]) ? 'T' : '.';
        paired_center_rows[center_row][center_col] =
            (('T' == destination) || ('+' == destination)) ? '+' : 'C';
    }
    paired_center_valid = 1u;
}

static void feed_center_yaw(uint16 col_q, uint16 row_q, float yaw_deg)
{
    uint8 index;
    uint16 yaw_q = (uint16)(yaw_deg * 100.0f + 0.5f);

    feed_center(col_q, row_q);
    for(index = 0u; index < ART_CENTER_SAMPLE_COUNT; index++)
    {
        center_yaw_q[index] = yaw_q;
        center_yaw_valid[index] = 1u;
    }
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
       (0u != center_request_count) ||
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
       (1u != center_request_count)) return 0u;

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
    if((SUBJECT2_EXECUTE_PUSH != subject2_get_state()) ||
       (0u != push_boundary_request_count)) return 0u;
    fake_frame_count++;
    subject2_tick(&context, &update);
    if((SUBJECT2_EXECUTE_PUSH != subject2_get_state()) ||
       (EXEC_STATE_RUNNING != fake_executor_state) ||
       (0u == push_boundary_request_count) ||
       (0 != strcmp(update.run_state, "Host Pend"))) return 0u;

    fake_push_boundary_ready = 1u;
    subject2_tick(&context, &update);
    if((SUBJECT2_CONFIRM_MAP != subject2_get_state()) ||
       (EXEC_STATE_RUNNING != fake_executor_state) ||
       (0u != force_push_stop_count) ||
       (0 != strcmp(update.run_state, "Host Sync"))) return 0u;

    fake_frame_count++;
    subject2_tick(&context, &update);
    return ((SUBJECT2_REPLAN_CENTER == subject2_get_state()) &&
            (3u == center_request_count) &&
            ('.' == snapshot.rows[5][6]) &&
            ('.' == snapshot.rows[5][9])) ? 1u : 0u;
}

static uint8 host_completion_timeout_resumes_preserved_path(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 request_count_after_resume;

    build_map();
    init_context(&context);
    if(0u == scan_to_select_push(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    if(SUBJECT2_EXECUTE_PUSH != subject2_get_state()) return 0u;

    live_rows[5][6] = '.';
    live_rows[5][9] = '.';
    fake_frame_count++;
    subject2_tick(&context, &update);
    if((SUBJECT2_EXECUTE_PUSH != subject2_get_state()) ||
       (0u != push_boundary_request_count)) return 0u;
    fake_frame_count++;
    subject2_tick(&context, &update);
    if((SUBJECT2_EXECUTE_PUSH != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "Host Pend"))) return 0u;

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if((SUBJECT2_CONFIRM_MAP != subject2_get_state()) ||
       (1u != force_push_stop_count) ||
       (0 != strcmp(update.run_state, "Host Sync"))) return 0u;

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if((SUBJECT2_CONFIRM_MAP != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "ART Retry"))) return 0u;

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if((SUBJECT2_EXECUTE_PUSH != subject2_get_state()) ||
       (1u != resume_push_stop_count) ||
       (EXEC_STATE_RUNNING != fake_executor_state) ||
       (0 != strcmp(update.run_state, "MCU Go"))) return 0u;

    request_count_after_resume = push_boundary_request_count;
    subject2_tick(&context, &update);
    return ((SUBJECT2_EXECUTE_PUSH == subject2_get_state()) &&
            (request_count_after_resume == push_boundary_request_count) &&
            (0 == strcmp(update.run_state, "S2Push"))) ? 1u : 0u;
}

static uint8 task_end_timeout_completes_last_box_locally(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    const waypoint_struct *last_waypoint;

    build_map();
    init_context(&context);
    if(0u == scan_to_select_push(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    if((SUBJECT2_EXECUTE_PUSH != subject2_get_state()) ||
       (0u == result.waypoint_count)) return 0u;
    last_waypoint = &result.waypoints[result.waypoint_count - 1u];

    fake_sync_pending = 1u;
    subject2_tick(&context, &update);
    if(SUBJECT2_CONFIRM_MAP != subject2_get_state()) return 0u;

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if((SUBJECT2_CONFIRM_MAP != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "ART Retry"))) return 0u;

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_RETURN_REQUESTED == subject2_get_state()) &&
            (0u != update.return_requested) &&
            ('.' == snapshot.rows[5][9]) &&
            (('C' == snapshot.rows[last_waypoint->row][last_waypoint->col]) ||
             ('+' == snapshot.rows[last_waypoint->row][last_waypoint->col])) &&
            (0 == strcmp(update.run_state, "MCU Go"))) ? 1u : 0u;
}

static uint8 task_end_timeout_plans_remaining_box(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    map_scan_stats_struct stats;

    build_map();
    live_rows[7][6] = 'B';
    live_rows[7][9] = 'T';
    map_source_snapshot(&snapshot, snapshot_rows, &live_source);
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);

    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(0u == complete_current_box_observation(&context, &update)) return 0u;
    feed_class(4u);
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(0u == complete_current_box_observation(&context, &update)) return 0u;
    feed_class(4u);
    subject2_tick(&context, &update);

    fake_ready_target = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(0u == complete_current_target_observation(&context, &update)) return 0u;
    feed_class(4u);
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(0u == complete_current_target_observation(&context, &update)) return 0u;
    feed_class(4u);
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(0u == complete_heading_restore(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    if(SUBJECT2_EXECUTE_PUSH != subject2_get_state()) return 0u;

    fake_sync_pending = 1u;
    subject2_tick(&context, &update);
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if((SUBJECT2_SELECT_PUSH != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "MCU Go"))) return 0u;
    map_scan_stats(&snapshot, &stats);
    if((1u != stats.box_count) || (1u != stats.target_count)) return 0u;

    fake_sync_pending = 0u;
    subject2_tick(&context, &update);
    return (SUBJECT2_EXECUTE_PUSH == subject2_get_state()) ? 1u : 0u;
}

static uint8 confirm_invalid_map_allows_mcu_resume(void)
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
    fake_frame_count++;
    subject2_tick(&context, &update);
    fake_push_boundary_ready = 1u;
    subject2_tick(&context, &update);
    if(SUBJECT2_CONFIRM_MAP != subject2_get_state()) return 0u;

    live_rows[4][4] = 'C';
    fake_frame_count++;
    subject2_tick(&context, &update);
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);

    return ((SUBJECT2_EXECUTE_PUSH == subject2_get_state()) &&
            (1u == resume_push_stop_count) &&
            (0 == strcmp(update.run_state, "MCU Go"))) ? 1u : 0u;
}

static uint8 subject3_push_block_retries_original_planner(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    solve_result_struct recovered;
    char opened_rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct opened;

    build_map();
    init_context(&context);
    context.allow_blast_fallback = 1u;
    if(0u == scan_to_select_push(&context, &update)) return 0u;

    snapshot_rows[4][6] = '#';
    snapshot_rows[5][7] = '#';
    snapshot_rows[6][6] = '#';
    subject2_tick(&context, &update);
    if((SUBJECT2_WAIT_BLAST != subject2_get_state()) ||
       (SUBJECT2_BLOCK_PUSH != subject2_get_block_reason()) ||
       (0u == update.blast_requested))
    {
        return 0u;
    }

    map_source_snapshot(&opened, opened_rows, &snapshot);
    opened_rows[5][7] = '.';
    return subject2_retry_blocked_plan(&opened, &recovered);
}

static uint8 confirm_map_ignores_transient_invalid_frames(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == scan_to_select_push(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    if(SUBJECT2_EXECUTE_PUSH != subject2_get_state()) return 0u;

    fake_sync_pending = 1u;
    subject2_tick(&context, &update);
    if(SUBJECT2_CONFIRM_MAP != subject2_get_state()) return 0u;

    live_rows[5][6] = '.';
    fake_frame_count++;
    subject2_tick(&context, &update);
    fake_frame_count++;
    subject2_tick(&context, &update);
    if((SUBJECT2_CONFIRM_MAP != subject2_get_state()) ||
       (EXEC_STATE_ERROR == fake_executor_state)) return 0u;

    live_rows[5][6] = 'B';
    live_rows[5][4] = 'C';
    fake_frame_count++;
    subject2_tick(&context, &update);
    fake_frame_count++;
    subject2_tick(&context, &update);
    if((SUBJECT2_CONFIRM_MAP != subject2_get_state()) ||
       (EXEC_STATE_ERROR == fake_executor_state)) return 0u;

    live_rows[5][4] = '.';
    live_rows[5][6] = '.';
    live_rows[5][9] = '.';
    fake_frame_count++;
    subject2_tick(&context, &update);
    fake_frame_count++;
    subject2_tick(&context, &update);
    return ((SUBJECT2_REPLAN_CENTER == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state)) ? 1u : 0u;
}

static uint8 pre_push_box_observation_timeout_uses_fresh_grid(void)
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

    fake_frame_count++;
    fake_time_ms += ART_BOX_OBSERVE_WAIT_MS;
    subject2_tick(&context, &update);

    return ((SUBJECT2_EXECUTE_PUSH == subject2_get_state()) &&
            (1u == continue_pre_push_count) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0 == strcmp(update.run_state, "GridPush"))) ? 1u : 0u;
}

static uint8 pre_push_box_requests_only_after_wait(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 request_before;

    build_map();
    init_context(&context);
    if(0 == scan_to_select_push(&context, &update)) return 0u;
    subject2_tick(&context, &update);
    if(SUBJECT2_EXECUTE_PUSH != subject2_get_state()) return 0u;
    request_before = observation_request_count;
    subject2_tick(&context, &update);
    if(request_before != observation_request_count) return 0u;

    fake_pre_push_pending = 1u;
    subject2_tick(&context, &update);
    if((request_before + 1u != observation_request_count) ||
       (SUBJECT2_PRE_PUSH_CENTER != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "BCtr"))) return 0u;
    feed_observation(550u, 550u, 650u, 550u);
    subject2_tick(&context, &update);
    return ((request_before + 1u == observation_request_count) &&
            (SUBJECT2_PRE_PUSH_CENTER == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "BGap"))) ? 1u : 0u;
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

    move_live_car(executor_target_row, executor_target_col);
    feed_center((uint16)(executor_target_col * 100u + 50u),
                (uint16)(executor_target_row * 100u + 50u));
    live_rows[4][4] = 'C';
    subject2_tick(&context, &update);

    return ((SUBJECT2_EXECUTE_PUSH == subject2_get_state()) &&
            (1u == continue_pre_push_count) &&
            (0u == start_pre_push_alignment_count)) ? 1u : 0u;
}

static uint8 invalid_pre_push_box_geometry_resyncs(void)
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

    return ((SUBJECT2_SCAN_MAP_SYNC == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "VSync")) &&
            (0u == continue_pre_push_count) &&
            (EXEC_STATE_ERROR != fake_executor_state)) ? 1u : 0u;
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
    if((SUBJECT2_SCAN_MAP_SYNC != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "VSync"))) return 0u;

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
    return ((SUBJECT2_SCAN_MAP_SYNC == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "VSync"))) ? 1u : 0u;
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
    fake_executor_error = EXEC_ERROR_MAP;
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_ERROR_MAP == fake_executor_error) &&
            (COMPETITION_FATAL_PLAN == competition_flow_get_fatal_reason()) &&
            (0 == strcmp(update.run_state, "F:Plan"))) ? 1u : 0u;
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
    return ((SUBJECT2_SCAN_MAP_SYNC == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0 == strcmp(update.run_state, "VSync"))) ? 1u : 0u;
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

static uint8 complete_current_target_observation(subject2_context_struct *context,
                                                 subject2_update_struct *update)
{
    uint8 car_row;
    uint8 car_col;

    if(SUBJECT2_SCAN_TARGET_MOVE == subject2_get_state())
    {
        move_live_car(executor_target_row, executor_target_col);
        fake_executor_state = EXEC_STATE_DONE;
        subject2_tick(context, update);
    }
    if((SUBJECT2_SCAN_TARGET_CENTER != subject2_get_state()) ||
       (0 == map_find_car(&live_source, &car_row, &car_col, 0))) return 0u;
    feed_center((uint16)(car_col * 100u + 50u),
                (uint16)(car_row * 100u + 50u));
    subject2_tick(context, update);
    if(0u == complete_center_adjust(context, update,
                                    SUBJECT2_SCAN_TARGET_ADJUST,
                                    SUBJECT2_SCAN_TARGET_TURN)) return 0u;
    complete_observation_turn(context, update);
    return (SUBJECT2_SCAN_TARGET_CLASSIFY == subject2_get_state()) ? 1u : 0u;
}

static uint8 duplicate_classes_are_accepted(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    live_rows[7][6] = 'B';
    live_rows[7][9] = 'T';
    map_source_snapshot(&snapshot, snapshot_rows, &live_source);
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(0u == complete_current_box_observation(&context, &update)) return 0u;
    feed_class(4u);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_PLAN != subject2_get_state()) return 0u;

    subject2_tick(&context, &update);
    if(0u == complete_current_box_observation(&context, &update)) return 0u;
    feed_class(4u);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_TARGET_MODE != subject2_get_state()) ||
       (2u != vision_ack_count)) return 0u;

    fake_ready_target = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if(0u == complete_current_target_observation(&context, &update)) return 0u;
    feed_class(4u);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_TARGET_PLAN != subject2_get_state()) return 0u;

    subject2_tick(&context, &update);
    if(0u == complete_current_target_observation(&context, &update)) return 0u;
    feed_class(4u);
    subject2_tick(&context, &update);
    if((SUBJECT2_VALIDATE_BINDINGS != subject2_get_state()) ||
       (4u != vision_ack_count)) return 0u;
    subject2_tick(&context, &update);
    return ((SUBJECT2_RESTORE_HEADING == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "HYaw"))) ? 1u : 0u;
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

static uint8 subject3_observation_block_starts_bomb_path(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 tick;

    build_map();
    live_rows[5][5] = '.';
    live_rows[5][6] = '.';
    live_rows[5][9] = '#';
    live_rows[5][4] = 'X';
    live_rows[5][7] = '#';
    live_rows[4][8] = '#';
    live_rows[5][8] = 'B';
    live_rows[6][8] = '#';
    live_rows[5][10] = 'T';
    move_live_car(5u, 2u);
    map_source_snapshot(&snapshot, snapshot_rows, &live_source);
    init_context(&context);

    subject3_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject3_tick(&context, &update);
    subject3_tick(&context, &update);
    if(SUBJECT3_SELECT_BLAST != subject3_get_state()) return 0u;

    for(tick = 0u; (tick < (MAP_CELLS * 2u)) &&
                    (SUBJECT3_SELECT_BLAST == subject3_get_state()); tick++)
    {
        subject3_tick(&context, &update);
    }
    if((SUBJECT3_EXECUTE_BOMB != subject3_get_state()) ||
       (0u == executor_start_count) || (0u == executor_start_art_sync) ||
       (0u == result.waypoint_count))
    {
        subject3_cancel();
        return 0u;
    }
    for(tick = 0u; tick < result.waypoint_count; tick++)
    {
        if(0u != result.waypoints[tick].center_correct_before)
        {
            subject3_cancel();
            return 1u;
        }
    }
    subject3_cancel();
    return 0u;
}

static uint8 subject3_center_rejects_changed_bomb_map(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 tick;

    build_map();
    live_rows[5][5] = '.';
    live_rows[5][6] = '.';
    live_rows[5][9] = '#';
    live_rows[5][4] = 'X';
    live_rows[5][7] = '#';
    live_rows[4][8] = '#';
    live_rows[5][8] = 'B';
    live_rows[6][8] = '#';
    live_rows[5][10] = 'T';
    move_live_car(5u, 2u);
    map_source_snapshot(&snapshot, snapshot_rows, &live_source);
    init_context(&context);

    subject3_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject3_tick(&context, &update);
    subject3_tick(&context, &update);
    for(tick = 0u; (tick < (MAP_CELLS * 2u)) &&
                    (SUBJECT3_SELECT_BLAST == subject3_get_state()); tick++)
    {
        subject3_tick(&context, &update);
    }
    if(SUBJECT3_EXECUTE_BOMB != subject3_get_state()) return 0u;

    fake_pre_push_pending = 1u;
    subject3_tick(&context, &update);
    live_rows[3][12] = '#';
    feed_center(250u, 550u);
    subject3_tick(&context, &update);
    return ((SUBJECT3_ERROR == subject3_get_state()) &&
            (COMPETITION_FATAL_ART1 == competition_flow_get_fatal_reason()) &&
            (0u == continue_pre_push_count)) ? 1u : 0u;
}

#if SUBJECT2_LAST_TARGET_ELIMINATION_ENABLE
static void build_three_object_map(void)
{
    build_map();
    live_rows[7][6] = 'B';
    live_rows[9][6] = 'B';
    live_rows[7][9] = 'T';
    live_rows[9][9] = 'T';
    map_source_snapshot(&snapshot, snapshot_rows, &live_source);
}

static uint8 scan_three_boxes(subject2_context_struct *context,
                              subject2_update_struct *update,
                              const uint8 classes[3])
{
    uint8 index;

    subject2_begin(context, 0.0f, 0.0f, update);
    fake_ready_box = 1u;
    subject2_tick(context, update);
    subject2_tick(context, update);
    for(index = 0u; index < 3u; index++)
    {
        if(0u == complete_current_box_observation(context, update)) return 0u;
        feed_class(classes[index]);
        subject2_tick(context, update);
        if(index < 2u)
        {
            if(SUBJECT2_SCAN_BOX_PLAN != subject2_get_state()) return 0u;
            subject2_tick(context, update);
        }
    }
    return (SUBJECT2_SCAN_TARGET_MODE == subject2_get_state()) ? 1u : 0u;
}

static uint8 scan_two_targets(subject2_context_struct *context,
                              subject2_update_struct *update,
                              const uint8 classes[2])
{
    uint8 index;

    fake_ready_target = 1u;
    subject2_tick(context, update);
    subject2_tick(context, update);
    for(index = 0u; index < 2u; index++)
    {
        if(0u == complete_current_target_observation(context, update)) return 0u;
        feed_class(classes[index]);
        subject2_tick(context, update);
        if(index < 1u)
        {
            if(SUBJECT2_SCAN_TARGET_PLAN != subject2_get_state()) return 0u;
            subject2_tick(context, update);
        }
    }
    return (SUBJECT2_SCAN_TARGET_PLAN == subject2_get_state()) ? 1u : 0u;
}

static uint8 last_target_elimination_enters_bind_without_observation(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 center_requests_before;
    uint16 executor_starts_before;
    uint16 vision_requests_before;
    uint8 recognition_is_target;
    uint8 recognition_class;

    if(0u == begin_box_classification(&context, &update)) return 0u;
    feed_class(4u);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_TARGET_MODE != subject2_get_state()) return 0u;

    center_requests_before = center_request_count;
    executor_starts_before = executor_start_count;
    vision_requests_before = vision_request_id;
    fake_ready_target = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if((SUBJECT2_VALIDATE_BINDINGS != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "Bind")) ||
       (center_requests_before != center_request_count) ||
       (executor_starts_before != executor_start_count) ||
       (vision_requests_before != vision_request_id) ||
       (0u == subject2_get_last_recognition(&recognition_is_target,
                                            &recognition_class)) ||
       (0u != recognition_is_target) || (4u != recognition_class))
    {
        return 0u;
    }

    subject2_tick(&context, &update);
    return ((SUBJECT2_RESTORE_HEADING == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "HYaw"))) ? 1u : 0u;
}

static uint8 duplicate_last_target_elimination_enters_bind(void)
{
    static const uint8 box_classes[3] = {2u, 2u, 5u};
    static const uint8 target_classes[2] = {2u, 5u};
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 center_requests_before;
    uint16 executor_starts_before;
    uint16 vision_requests_before;

    build_three_object_map();
    init_context(&context);
    if((0u == scan_three_boxes(&context, &update, box_classes)) ||
       (0u == scan_two_targets(&context, &update, target_classes)))
    {
        return 0u;
    }
    center_requests_before = center_request_count;
    executor_starts_before = executor_start_count;
    vision_requests_before = vision_request_id;
    subject2_tick(&context, &update);
    if((SUBJECT2_VALIDATE_BINDINGS != subject2_get_state()) ||
       (center_requests_before != center_request_count) ||
       (executor_starts_before != executor_start_count) ||
       (vision_requests_before != vision_request_id))
    {
        return 0u;
    }
    subject2_tick(&context, &update);
    return (SUBJECT2_RESTORE_HEADING == subject2_get_state()) ? 1u : 0u;
}

static uint8 invalid_target_counts_fall_back_to_rescan(void)
{
    static const uint8 box_classes[3] = {2u, 5u, 7u};
    static const uint8 target_classes[2] = {2u, 2u};
    subject2_context_struct context;
    subject2_update_struct update;

    build_three_object_map();
    init_context(&context);
    if((0u == scan_three_boxes(&context, &update, box_classes)) ||
       (0u == scan_two_targets(&context, &update, target_classes)))
    {
        return 0u;
    }
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_TARGET_MOVE != subject2_get_state()) &&
       (SUBJECT2_SCAN_TARGET_CENTER != subject2_get_state()))
    {
        return 0u;
    }
    if(0u == complete_current_target_observation(&context, &update)) return 0u;
    feed_class(7u);
    subject2_tick(&context, &update);
    if(SUBJECT2_VALIDATE_BINDINGS != subject2_get_state()) return 0u;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_MODE == subject2_get_state()) &&
            (VISION_MODE_BOX == last_mode) &&
            (0 == strcmp(update.run_state, "VRetry"))) ? 1u : 0u;
}
#endif

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

static uint8 heading_restore_timeout_keeps_original_target(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == scan_to_heading_restore(&context, &update)) return 0u;

    fake_control_status.yaw_error = 2.0f;
    fake_time_ms += SUBJECT2_TURN_TIMEOUT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SELECT_PUSH == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0u == yaw_rebase_count) &&
            (fabsf(last_target_yaw - 23.0f) < 0.01f) &&
            (0 == strcmp(update.run_state, "YawKeep"))) ? 1u : 0u;
}

static uint8 enter_post_observe_yaw(subject2_context_struct *context,
                                    subject2_update_struct *update,
                                    float bias_deg)
{
    build_map();
    init_context(context);
    context->art_yaw_bias_valid = 1u;
    context->art_yaw_bias_deg = bias_deg;
    if(0u == scan_to_heading_restore(context, update)) return 0u;
    fake_control_status.yaw_error = 0.0f;
    subject2_tick(context, update);
    fake_time_ms += SUBJECT2_TURN_STABLE_MS;
    subject2_tick(context, update);
    return ((SUBJECT2_POST_OBSERVE_YAW_SAMPLE == subject2_get_state()) &&
            (0 == strcmp(update->run_state, "VYaw"))) ? 1u : 0u;
}

static uint8 post_observe_yaw_thresholds_and_bias(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 request_count_after_sample;

    if(0u == enter_post_observe_yaw(&context, &update, 1.0f)) return 0u;
    feed_center_yaw(550u, 550u, 179.5f);
    subject2_tick(&context, &update);
    if((SUBJECT2_SELECT_PUSH != subject2_get_state()) ||
       (1u != yaw_rebase_count) ||
       (0u != relative_yaw_correction_count)) return 0u;
    request_count_after_sample = center_request_count;
    subject2_tick(&context, &update);
    if((SUBJECT2_EXECUTE_PUSH != subject2_get_state()) ||
       (request_count_after_sample != center_request_count)) return 0u;

    if(0u == enter_post_observe_yaw(&context, &update, 0.0f)) return 0u;
    feed_center_yaw(550u, 550u, 176.0f);
    subject2_tick(&context, &update);
    if((SUBJECT2_POST_OBSERVE_YAW_FIX != subject2_get_state()) ||
       (1u != relative_yaw_correction_count) ||
       (fabsf(relative_yaw_correction_deg - 4.0f) > 0.01f))
    {
        return 0u;
    }
    fake_control_status.yaw_error = 0.0f;
    subject2_tick(&context, &update);
    fake_time_ms += SUBJECT2_TURN_STABLE_MS;
    subject2_tick(&context, &update);
    if((SUBJECT2_SELECT_PUSH != subject2_get_state()) ||
       (1u != yaw_rebase_count)) return 0u;

    if(0u == enter_post_observe_yaw(&context, &update, 0.0f)) return 0u;
    feed_center_yaw(550u, 550u, 174.0f);
    subject2_tick(&context, &update);
    if((SUBJECT2_POST_OBSERVE_YAW_FIX != subject2_get_state()) ||
       (fabsf(relative_yaw_correction_deg - 6.0f) > 0.01f))
    {
        return 0u;
    }

    if(0u == enter_post_observe_yaw(&context, &update, 0.0f)) return 0u;
    feed_center_yaw(550u, 550u, 173.9f);
    subject2_tick(&context, &update);
    return ((SUBJECT2_SELECT_PUSH == subject2_get_state()) &&
            (0u == relative_yaw_correction_count) &&
            (0u == yaw_rebase_count)) ? 1u : 0u;
}

static uint8 post_observe_yaw_fallbacks(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 request_before;

    build_map();
    init_context(&context);
    if(0u == scan_to_heading_restore(&context, &update)) return 0u;
    request_before = center_request_count;
    if(0u == complete_heading_restore(&context, &update)) return 0u;
    if(request_before != center_request_count) return 0u;

    if(0u == enter_post_observe_yaw(&context, &update, 0.0f)) return 0u;
    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    if((SUBJECT2_SELECT_PUSH != subject2_get_state()) ||
       (0u != yaw_rebase_count)) return 0u;

    if(0u == enter_post_observe_yaw(&context, &update, 0.0f)) return 0u;
    feed_center_yaw(550u, 550u, 180.0f);
    paired_center_valid = 0u;
    subject2_tick(&context, &update);
    if((SUBJECT2_SELECT_PUSH != subject2_get_state()) ||
       (0u != yaw_rebase_count)) return 0u;

    if(0u == enter_post_observe_yaw(&context, &update, 0.0f)) return 0u;
    fake_time_ms += SUBJECT2_POST_OBSERVE_YAW_TIMEOUT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SELECT_PUSH == subject2_get_state()) &&
            (0u == relative_yaw_correction_count) &&
            (0u == yaw_rebase_count)) ? 1u : 0u;
}

static uint8 post_observe_yaw_fix_timeout_keeps_original_target(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    if(0u == enter_post_observe_yaw(&context, &update, 0.0f)) return 0u;
    feed_center_yaw(550u, 550u, 176.0f);
    subject2_tick(&context, &update);
    if(SUBJECT2_POST_OBSERVE_YAW_FIX != subject2_get_state()) return 0u;
    fake_time_ms += SUBJECT2_TURN_TIMEOUT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SELECT_PUSH == subject2_get_state()) &&
            (0u == yaw_rebase_count) &&
            (fabsf(last_target_yaw - 23.0f) < 0.01f) &&
            (0 == strcmp(update.run_state, "VFixTmo"))) ? 1u : 0u;
}

static uint8 map_change_syncs_scan(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 center_before;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    live_rows[5][6] = '.';
    live_rows[5][7] = 'B';
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_MAP_SYNC != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "VSync"))) return 0u;

    center_before = center_request_count;
    fake_frame_count++;
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_MAP_SYNC != subject2_get_state()) ||
       (center_before + 1u != center_request_count)) return 0u;

    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_MODE == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0 == strcmp(update.run_state, "BScan"))) ? 1u : 0u;
}

static uint8 scan_map_sync_all_done_requests_return(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    live_rows[5][6] = '.';
    live_rows[5][9] = '.';
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_MAP_SYNC != subject2_get_state()) return 0u;

    fake_frame_count++;
    subject2_tick(&context, &update);
    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    return ((SUBJECT2_RETURN_REQUESTED == subject2_get_state()) &&
            (0u != update.return_requested) &&
            (0 == strcmp(update.run_state, "S2Ret"))) ? 1u : 0u;
}

static uint8 scan_map_sync_wait_map_times_out(void)
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
    if(SUBJECT2_SCAN_MAP_SYNC != subject2_get_state()) return 0u;

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_MAP_SYNC != subject2_get_state()) ||
       (EXEC_STATE_ERROR == fake_executor_state)) return 0u;
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_MAP_SYNC != subject2_get_state()) ||
       (EXEC_STATE_ERROR == fake_executor_state)) return 0u;
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if((SUBJECT2_ERROR != subject2_get_state()) ||
       (0u == competition_flow_is_fatal()) ||
       (COMPETITION_FATAL_MAP != competition_flow_get_fatal_reason()) ||
       (0 != strcmp(update.run_state, "F:Map"))) return 0u;

    if(0u == subject2_manual_recover(&update)) return 0u;
    competition_flow_clear_fatal();
    if((SUBJECT2_SCAN_MAP_SYNC != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "VSync"))) return 0u;
    subject2_tick(&context, &update);
    return (SUBJECT2_SCAN_MAP_SYNC == subject2_get_state()) ? 1u : 0u;
}

static uint8 scan_map_sync_recovers_after_invalid_frames(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 center_before;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    live_rows[5][6] = '.';
    live_rows[5][7] = 'B';
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_MAP_SYNC != subject2_get_state()) return 0u;

    center_before = center_request_count;
    live_rows[5][9] = '.';
    fake_frame_count++;
    subject2_tick(&context, &update);
    fake_frame_count++;
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_MAP_SYNC != subject2_get_state()) ||
       (EXEC_STATE_ERROR == fake_executor_state) ||
       (center_before != center_request_count) ||
       (0 != strcmp(update.run_state, "VSync"))) return 0u;

    live_rows[5][9] = 'T';
    fake_frame_count++;
    subject2_tick(&context, &update);
    if(center_before + 1u != center_request_count) return 0u;
    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_MODE == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state)) ? 1u : 0u;
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
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    fake_time_ms += SUBJECT2_FAST_CENTER_WAIT_MS;
    subject2_tick(&context, &update);
    fake_frame_count++;
#endif
    fake_time_ms = EXEC_ART_SYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    return ((SUBJECT2_SCAN_BOX_TURN == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0 == strcmp(update.run_state, "CtrSkip"))) ? 1u : 0u;
#else
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_STATE_ERROR == fake_executor_state) &&
            (EXEC_ERROR_ART_CENTER == fake_executor_error) &&
            (0 == strcmp(update.run_state, "F:ART1"))) ? 1u : 0u;
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

#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    live_rows[4][4] = 'C';
    fake_frame_count++;
#endif
    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    return ((SUBJECT2_SELECT_PUSH == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "Bind")) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0.0f == fake_pose.x_cm) && (0.0f == fake_pose.y_cm)) ? 1u : 0u;
#else
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "F:ART1"))) ? 1u : 0u;
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

static uint8 turn_timeout_changes_observation_direction(void)
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
    return ((SUBJECT2_SCAN_BOX_PLAN == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0 == strcmp(update.run_state, "VRetry"))) ? 1u : 0u;
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
    return ((SUBJECT2_SCAN_MAP_SYNC == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0 == strcmp(update.run_state, "VSync"))) ? 1u : 0u;
}

static uint8 scan_sync_center_timeout_uses_accepted_map(void)
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
    if(SUBJECT2_SCAN_MAP_SYNC != subject2_get_state()) return 0u;

    fake_frame_count++;
    subject2_tick(&context, &update);
    if(center_request_count != 1u) return 0u;

    live_rows[4][4] = 'C';
    fake_frame_count++;
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);

#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    return ((SUBJECT2_SCAN_BOX_MODE == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0u == competition_flow_is_fatal()) &&
            (0 == strcmp(update.run_state, "BScan"))) ? 1u : 0u;
#else
    return (SUBJECT2_ERROR == subject2_get_state()) ? 1u : 0u;
#endif
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

static uint8 scan_center_stale_map_enters_sync(void)
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
    return ((SUBJECT2_SCAN_MAP_SYNC == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0 == strcmp(update.run_state, "VSync"))) ? 1u : 0u;
#else
    return (SUBJECT2_ERROR == subject2_get_state()) ? 1u : 0u;
#endif
}

static uint8 fresh_periodic_center_within_2cm_skips_batch(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    periodic_center_col_q = 550u;
    periodic_center_row_q = 550u;
    periodic_center_valid = 1u;
    fake_frame_count = 1u;
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) ||
       (0u != center_request_count)) return 0u;

    periodic_center_col_q = 560u;
    fake_frame_count++;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_ADJUST == subject2_get_state()) &&
            (0u == center_request_count) &&
            (1u == correction_start_count) &&
            (correction_target_x > 1.99f) &&
            (correction_target_x < 2.01f) &&
            (correction_target_y == 0.0f)) ? 1u : 0u;
}

static uint8 fresh_periodic_center_over_2cm_uses_batch(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    periodic_center_col_q = 550u;
    periodic_center_row_q = 550u;
    periodic_center_valid = 1u;
    fake_frame_count = 1u;
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);

    periodic_center_col_q = 561u;
    fake_frame_count++;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_CENTER == subject2_get_state()) &&
            (1u == center_request_count) &&
            (0u == correction_start_count)) ? 1u : 0u;
}

static uint8 fresh_periodic_map_matching_c_skips_batch(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    periodic_center_auto_invalid_frame = 0u;
    periodic_center_valid = 0u;
    fake_frame_count = 1u;
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) ||
       (0u != center_request_count)) return 0u;

    fake_frame_count++;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_ADJUST == subject2_get_state()) &&
            (0u == center_request_count) &&
            (1u == correction_start_count) &&
            (0.0f == correction_target_x) &&
            (0.0f == correction_target_y)) ? 1u : 0u;
}

static uint8 fresh_periodic_map_wrong_c_uses_batch(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    periodic_center_auto_invalid_frame = 0u;
    periodic_center_valid = 0u;
    fake_frame_count = 1u;
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);

    live_rows[5][5] = '.';
    live_rows[5][6] = 'C';
    fake_frame_count++;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_CENTER == subject2_get_state()) &&
            (1u == center_request_count) &&
            (0u == correction_start_count)) ? 1u : 0u;
}

static uint8 old_invalid_periodic_center_can_recover_fast(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    periodic_center_auto_invalid_frame = 0u;
    periodic_center_valid = 0u;
    fake_frame_count = 1u;
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) ||
       (0u != center_request_count)) return 0u;

    periodic_center_col_q = 560u;
    periodic_center_row_q = 550u;
    periodic_center_valid = 1u;
    fake_frame_count++;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_ADJUST == subject2_get_state()) &&
            (0u == center_request_count) &&
            (1u == correction_start_count)) ? 1u : 0u;
}

static uint8 periodic_center_old_frame_waits_then_uses_batch(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    periodic_center_col_q = 550u;
    periodic_center_row_q = 550u;
    periodic_center_valid = 1u;
    fake_frame_count = 1u;
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if((0u != center_request_count) ||
       (0u != correction_start_count)) return 0u;

    fake_time_ms += SUBJECT2_FAST_CENTER_WAIT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_CENTER == subject2_get_state()) &&
            (1u == center_request_count) &&
            (0u == correction_start_count)) ? 1u : 0u;
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

static uint8 center_adjustment_small_timeout_continues(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;

    feed_center(558u, 550u);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_ADJUST != subject2_get_state()) return 0u;

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
    return ((SUBJECT2_SCAN_BOX_TURN == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0 == strcmp(update.run_state, "VTurn"))) ? 1u : 0u;
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
    return ((SUBJECT2_SCAN_MAP_SYNC == subject2_get_state()) &&
            (EXEC_STATE_ERROR != fake_executor_state) &&
            (0 == strcmp(update.run_state, "VSync"))) ? 1u : 0u;
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
    fake_executor_error = EXEC_ERROR_MAP;
    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_ERROR_MAP == fake_executor_error) &&
            (0 == strcmp(update.run_state, "F:Plan"))) ? 1u : 0u;
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

static uint8 center_map_boundary_replans_from_median(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;
    move_live_car(3u, 5u);
    feed_center(550u, 450u);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_PLAN != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "VReplan")) ||
       (4u != start_row) || (5u != start_col) ||
       (0u != pose_reset_count) ||
       (0u != correction_start_count)) return 0u;

    subject2_tick(&context, &update);
    return ((SUBJECT2_ERROR != subject2_get_state()) &&
            (4u == start_row) && (5u == start_col)) ? 1u : 0u;
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
            (0u == pose_reset_count) &&
            (5u == start_row) && (9u == start_col) &&
            (0u == correction_start_count)) ? 1u : 0u;
}

static uint8 center_cross_cell_retries_until_total_timeout(void)
{
    subject2_context_struct context;
    subject2_update_struct update;
    uint16 request_before;

    build_map();
    init_context(&context);
    if(0u == begin_box_center(&context, &update)) return 0u;
    request_before = center_request_count;
    feed_center(1550u, 550u);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) ||
       (request_before + 2u != center_request_count) ||
       (0u != correction_start_count) ||
       (0 != strcmp(update.run_state, "VCtr"))) return 0u;

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    subject2_tick(&context, &update);
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    return ((SUBJECT2_SCAN_MAP_SYNC == subject2_get_state()) &&
            (0 == strcmp(update.run_state, "VSync"))) ? 1u : 0u;
#else
    return ((SUBJECT2_ERROR == subject2_get_state()) &&
            (EXEC_ERROR_ART_CENTER == fake_executor_error) &&
            (0 == strcmp(update.run_state, "F:ART1"))) ? 1u : 0u;
#endif
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

#if SUBJECT2_SCAN_ELIMINATION_ONLY
    passed &= last_target_elimination_enters_bind_without_observation();
    printf("subject2-last-target-elim   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= duplicate_last_target_elimination_enters_bind();
    printf("subject2-last-target-dupe   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= invalid_target_counts_fall_back_to_rescan();
    printf("subject2-last-target-rescan %s\n", (0u != passed) ? "PASS" : "FAIL");
#else
    build_map();
    passed &= run_scan();
    printf("subject2-scan-state         %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= subject3_push_block_retries_original_planner();
    printf("subject3-push-block         %s\n", (0u != passed) ? "PASS" : "FAIL");
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
    passed &= duplicate_classes_are_accepted();
    printf("subject2-duplicate-classes  %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= target_classification_uses_same_backoff_flow();
    printf("subject2-target-backoff     %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= subject3_observation_block_starts_bomb_path();
    printf("subject3-observe-bomb       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= subject3_center_rejects_changed_bomb_map();
    printf("subject3-center-map-check   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= heading_restore_requires_new_continuous_window();
    printf("subject2-home-yaw-stable    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= heading_restore_timeout_keeps_original_target();
    printf("subject2-home-yaw-timeout   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= post_observe_yaw_thresholds_and_bias();
    printf("subject2-post-yaw-threshold %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= post_observe_yaw_fallbacks();
    printf("subject2-post-yaw-fallback  %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= post_observe_yaw_fix_timeout_keeps_original_target();
    printf("subject2-post-yaw-timeout   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= map_change_syncs_scan();
    printf("subject2-scan-map-change    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= scan_map_sync_all_done_requests_return();
    printf("subject2-scan-map-done      %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= scan_map_sync_wait_map_times_out();
    printf("subject2-scan-map-timeout   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= scan_map_sync_recovers_after_invalid_frames();
    printf("subject2-scan-map-recover   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= scan_sync_center_timeout_uses_accepted_map();
    printf("subject2-sync-frozen-map    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= host_completion_before_task_end_enters_confirm();
    printf("subject2-host-completion    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= host_completion_timeout_resumes_preserved_path();
    printf("subject2-host-timeout-go    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= task_end_timeout_completes_last_box_locally();
    printf("subject2-task-timeout-go    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= task_end_timeout_plans_remaining_box();
    printf("subject2-task-timeout-next  %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= confirm_invalid_map_allows_mcu_resume();
    printf("subject2-confirm-conflict   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= confirm_map_ignores_transient_invalid_frames();
    printf("subject2-confirm-transient  %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= push_retry_tracks_active_box();
    printf("subject2-push-retry         %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= scan_center_timeout_follows_policy();
    printf("subject2-center-timeout     %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= scan_center_stale_map_enters_sync();
    printf("subject2-center-stale-map   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= replan_center_timeout_follows_policy();
    printf("subject2-replan-timeout     %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= turn_requires_continuous_100ms();
    printf("subject2-turn-stable        %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= turn_timeout_changes_observation_direction();
    printf("subject2-turn-timeout       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= vision_ready_retries_then_times_out();
    printf("subject2-ready-timeout      %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_offset_adjusts_before_turn_and_classify();
    printf("subject2-center-pose        %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= fresh_periodic_center_within_2cm_skips_batch();
    printf("subject2-center-fast-2cm    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= fresh_periodic_center_over_2cm_uses_batch();
    printf("subject2-center-fast-over   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= fresh_periodic_map_matching_c_skips_batch();
    printf("subject2-center-fast-grid   %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= fresh_periodic_map_wrong_c_uses_batch();
    printf("subject2-center-grid-mismatch %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= old_invalid_periodic_center_can_recover_fast();
    printf("subject2-center-fast-recover %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= periodic_center_old_frame_waits_then_uses_batch();
    printf("subject2-center-fast-stale  %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= small_center_offset_still_enters_adjustment();
    printf("subject2-center-small       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_adjustment_small_timeout_continues();
    printf("subject2-center-adjust-tmo  %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_adjustment_busy_has_reason();
    printf("subject2-center-adjust-busy %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_adjustment_executor_error_has_reason();
    printf("subject2-center-adjust-exec %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_adjacent_map_uses_median();
    printf("subject2-center-adjacent    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_uses_paired_map_after_global_multi_car();
    printf("subject2-center-paired-map  %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_map_boundary_replans_from_median();
    printf("subject2-center-map-median  %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= paired_center_on_target_replans_observation();
    printf("subject2-center-target-plan %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= center_cross_cell_retries_until_total_timeout();
    printf("subject2-center-cross-retry %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= classification_request_is_not_resent();
    printf("subject2-request-once       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= step_mode_only_pauses_navigation();
    printf("subject2-step-observe       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= pre_push_box_observation_timeout_uses_fresh_grid();
    printf("subject2-box-observe-tmo    %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= pre_push_box_requests_only_after_wait();
    printf("subject2-box-after-stop     %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= turn_center_continues_without_push_alignment();
    printf("subject2-turn-center        %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= invalid_pre_push_box_geometry_resyncs();
    printf("subject2-box-geometry       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= pre_push_box_motion_error_has_timeout_reason();
    printf("subject2-box-motion-error   %s\n", (0u != passed) ? "PASS" : "FAIL");
#endif
    return (0u != passed) ? 0 : 1;
}
