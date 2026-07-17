#include "art_replan.h"
#include "art_observation.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "executor.h"
#include "map_utils.h"
#include "motion_math.h"
#include "openart_uart.h"
#include "solver.h"
#include "competition_flow.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32 fake_time_ms;
static uint32 fake_frame_count;
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
static uint16 solve_count;
static uint16 executor_start_count;
static uint16 position_correction_start_count;
static float position_correction_target_x_cm;
static float position_correction_target_y_cm;
static uint16 executor_error_count;
static float executor_initial_x_cm;
static float executor_initial_y_cm;
static uint8 fake_sync_pending;
static uint8 fake_pre_push_pending;
static uint8 fake_pre_push_box_request;
static uint8 fake_pre_push_box_active;
static uint8 fake_error_on_box_active_query;
static uint8 fake_pre_push_box_row;
static uint8 fake_pre_push_box_col;
static const char *fake_pre_push_box_state = "BGap";
static executor_art_box_prep_result_enum fake_pre_push_box_result;
static uint16 start_pre_push_box_count;
static uint16 start_pre_push_box_retry_count;
static uint8 fake_art_box_sample_count;
static uint8 fake_center_requires_push_alignment;
static uint16 continue_pre_push_count;
static uint16 start_pre_push_alignment_count;
static uint8 last_commit_row;
static uint8 last_commit_col;
static char fake_sync_action;
static executor_state_enum fake_executor_state;
static executor_error_enum fake_executor_error;
static uint8 fake_push_boundary_ready;
static uint16 push_boundary_request_count;
static uint16 push_boundary_force_count;
static uint16 push_boundary_resume_count;
static uint16 continue_art_sync_count;
static drive_pose_struct fake_pose;
static control_status_struct fake_control_status;
static uint16 yaw_correction_start_count;
static float yaw_correction_delta_deg;
static uint16 yaw_rebase_count;
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
static char error_return_rows[MAP_ROWS][MAP_COLS + 1];
static map_source_struct error_return_source;
static uint8 error_return_source_valid;

static void feed_center(uint16 col0, uint16 row0,
                        uint16 col1, uint16 row1,
                        uint16 col2, uint16 row2);

uint32 time_ms(void) { return fake_time_ms; }
const drive_pose_struct *drive_pose_get(void) { return &fake_pose; }
void drive_pose_reset(float x, float y, float yaw)
{
    fake_pose.x_cm = x;
    fake_pose.y_cm = y;
    fake_pose.yaw_deg = yaw;
}
void stop_motion(void) { }
void reset_motion_segment(void) { }
void set_motion(float vx, float vy) { (void)vx; (void)vy; }
const control_status_struct *get_control_status(void) { return &fake_control_status; }
uint8 drive_control_is_healthy(void) { return 1u; }
drive_health_fault_enum drive_control_get_health_fault(void)
{
    return DRIVE_HEALTH_NONE;
}
void drive_control_start_relative_yaw_correction(float delta_deg)
{
    yaw_correction_start_count++;
    yaw_correction_delta_deg = delta_deg;
    fake_control_status.yaw_error = delta_deg;
}
void drive_control_lock_yaw_and_reset_pose(void)
{
    yaw_rebase_count++;
    fake_control_status.target_yaw = fake_control_status.current_yaw;
    fake_control_status.yaw_error = 0.0f;
    memset(&fake_pose, 0, sizeof(fake_pose));
}
void openart_request_player_center(void)
{
    center_request_count++;
    center_count = 0;
    center_read = 0;
    paired_center_valid = 0u;
}
void openart_request_observation(uint8 box_row, uint8 box_col)
{
    observation_request_count++;
    observation_request_row = box_row;
    observation_request_col = box_col;
    observation_count = 0u;
    observation_read = 0u;
}
uint8 openart_get_observation_sample(openart_observation_sample_struct *sample)
{
    if((0 == sample) || (observation_read >= observation_count)) return 0u;
    *sample = observation_samples[observation_read++];
    return 1u;
}
uint8 openart_get_requested_center_sample(uint16 *col_q, uint16 *row_q,
                                          uint16 *yaw_q, uint8 *yaw_valid)
{
    if(center_read >= center_count) return 0;
    *col_q = center_col_q[center_read];
    *row_q = center_row_q[center_read];
    *yaw_q = center_yaw_q[center_read];
    *yaw_valid = center_yaw_valid[center_read];
    center_read++;
    return center_read;
}
const map_source_struct *openart_map_get(void) { return &live_source; }
const map_source_struct *openart_get_requested_center_map(void)
{
    return (0u != paired_center_valid) ? &paired_center_source : 0;
}
uint32 openart_uart_get_frame_count(void) { return fake_frame_count; }
void openart_uart_discard_pending(void) { }

uint8 solve_map(const map_source_struct *source, solve_result_struct *out)
{
    map_scan_stats_struct stats;
    map_scan_stats(source, &stats);
    solve_count++;
    memset(out, 0, sizeof(*out));
    out->solved = 1;
    out->waypoint_count = 1;
    out->waypoints[0].row = stats.car_row;
    out->waypoints[0].col = (uint8)(stats.car_col + 1u);
    out->waypoints[0].action = 'r';
    return 1;
}
uint8 solve_navigation_path(const map_source_struct *source,
                            uint8 target_row,
                            uint8 target_col,
                            solve_result_struct *out)
{
    (void)source;
    memset(out, 0, sizeof(*out));
    out->solved = 1;
    out->waypoint_count = 1;
    out->waypoints[0].row = target_row;
    out->waypoints[0].col = target_col;
    out->waypoints[0].action = 'l';
    return 1;
}

void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 row, uint8 col, float x, float y,
                    uint8 single_step, uint8 art_sync)
{
    (void)waypoints; (void)count; (void)row; (void)col;
    (void)single_step; (void)art_sync;
    executor_start_count++;
    executor_initial_x_cm = x;
    executor_initial_y_cm = y;
    fake_executor_state = EXEC_STATE_RUNNING;
}
void executor_stop(void)
{
    fake_executor_state = EXEC_STATE_IDLE;
    fake_pre_push_pending = 0u;
    fake_sync_pending = 0u;
    fake_push_boundary_ready = 0u;
}
uint8 executor_request_stop_after_current_push(void)
{
    if((EXEC_STATE_RUNNING != fake_executor_state) ||
       (0u != fake_pre_push_pending) || (0u != fake_sync_pending))
    {
        return 0u;
    }
    push_boundary_request_count++;
    return 1u;
}
uint8 executor_stop_after_current_push_ready(void)
{
    return fake_push_boundary_ready;
}
uint8 executor_force_current_push_stop(void)
{
    push_boundary_force_count++;
    fake_push_boundary_ready = 1u;
    return 1u;
}
uint8 executor_resume_after_current_push_stop(void)
{
    if(0u == fake_push_boundary_ready) return 0u;
    fake_push_boundary_ready = 0u;
    push_boundary_resume_count++;
    return 1u;
}
uint8 executor_start_position_correction(float target_x_cm, float target_y_cm)
{
    if((EXEC_STATE_RUNNING == fake_executor_state) ||
       (EXEC_STATE_PAUSED == fake_executor_state)) return 0u;
    position_correction_start_count++;
    position_correction_target_x_cm = target_x_cm;
    position_correction_target_y_cm = target_y_cm;
    fake_executor_state = EXEC_STATE_RUNNING;
    return 1u;
}
uint8 executor_start_position_correction_with_pose_reset(
    float initial_x_cm, float initial_y_cm,
    float target_x_cm, float target_y_cm)
{
    (void)initial_x_cm;
    (void)initial_y_cm;
    return executor_start_position_correction(target_x_cm, target_y_cm);
}
void executor_finish_done(void) { fake_executor_state = EXEC_STATE_DONE; }
void executor_set_error(executor_error_enum error)
{
    fake_executor_error = error;
    executor_error_count++;
    fake_executor_state = EXEC_STATE_ERROR;
}
executor_state_enum executor_get_state(void) { return fake_executor_state; }
executor_error_enum executor_get_error(void) { return fake_executor_error; }
uint8 executor_art_pre_push_pending(void) { return fake_pre_push_pending; }
uint8 executor_get_pre_push_wait_cell(uint8 *row, uint8 *col)
{
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
char executor_get_art_sync_action(void) { return fake_sync_action; }
uint8 executor_continue_after_art_sync(void)
{
    fake_sync_pending = 0;
    continue_art_sync_count++;
    return 1;
}
uint16 executor_get_current_step(void) { return 0; }
const char *executor_state_name(void) { return "Running"; }
uint8 executor_apply_art_player_center(uint16 col, uint16 row, uint32 sample)
{
    (void)col; (void)row;
    return (sample >= ART_CENTER_SAMPLE_COUNT) ? 1u : 0u;
}

void executor_reset_art_player_center_samples(void)
{
}
executor_art_center_result_enum executor_commit_art_player_center(uint8 row, uint8 col)
{
    last_commit_row = row;
    last_commit_col = col;
    return EXEC_ART_CENTER_APPLIED;
}
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

static void build_map(void)
{
    uint8 row;
    uint8 col;
    for(row = 0; row < MAP_ROWS; row++)
    {
        for(col = 0; col < MAP_COLS; col++)
        {
            live_rows[row][col] = ((0u == row) || ((MAP_ROWS - 1u) == row) ||
                                   (0u == col) || ((MAP_COLS - 1u) == col)) ? '#' : '.';
        }
        live_rows[row][MAP_COLS] = '\0';
        live_source.rows[row] = live_rows[row];
    }
    live_rows[5][2] = 'C';
    live_rows[5][5] = 'B';
    live_rows[5][6] = 'T';
    live_source.name = "test";
}

static void feed_center(uint16 col0, uint16 row0,
                        uint16 col1, uint16 row1,
                        uint16 col2, uint16 row2)
{
    uint8 index;

    center_col_q[0] = col0; center_row_q[0] = row0;
    center_col_q[1] = col1; center_row_q[1] = row1;
    center_col_q[2] = col2; center_row_q[2] = row2;
    for(index = 3u; index < ART_CENTER_SAMPLE_COUNT; index++)
    {
        center_col_q[index] = col0;
        center_row_q[index] = row0;
    }
    memset(center_yaw_q, 0, sizeof(center_yaw_q));
    memset(center_yaw_valid, 0, sizeof(center_yaw_valid));
    center_count = ART_CENTER_SAMPLE_COUNT;
    center_read = 0;
    map_source_snapshot(&paired_center_source, paired_center_rows, &live_source);
    paired_center_valid = 1u;
}

static void init_error_return_context(art_replan_context_struct *context)
{
    memset(&result, 0, sizeof(result));
    memset(&snapshot, 0, sizeof(snapshot));
    memset(&error_return_source, 0, sizeof(error_return_source));
    memset(error_return_rows, 0, sizeof(error_return_rows));
    build_map();
    map_source_snapshot(&snapshot, snapshot_rows, &live_source);
    snapshot_valid = 1u;
    error_return_source_valid = 0u;
    context->result = &result;
    context->snapshot = &snapshot;
    context->snapshot_rows = snapshot_rows;
    context->snapshot_valid = &snapshot_valid;
    context->elapsed_ms = &elapsed_ms;
    context->start_row = &start_row;
    context->start_col = &start_col;
    context->error_return_snapshot = &error_return_source;
    context->error_return_snapshot_rows = error_return_rows;
    context->error_return_snapshot_valid = &error_return_source_valid;
    context->run_mode = RUN_MODE_RUN;
    art_replan_begin_initial(&(art_replan_update_struct){0});
    fake_time_ms = 5000u;
    art_replan_tick(context, 1u, &(art_replan_update_struct){0});
    feed_center(250u, 550u, 250u, 550u, 250u, 550u);
    art_replan_tick(context, 1u, &(art_replan_update_struct){0});
}

static uint8 error_return_preserves_obstacles(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;

    init_error_return_context(&context);
    live_rows[5][5] = MAP_BOX_ON_TARGET;
    map_source_snapshot(&snapshot, snapshot_rows, &live_source);
    snapshot_valid = 1u;
    if(0u == art_replan_begin_error_return(&context, 5u, 3u,
                                           1.0f, -1.0f, 1u, &update))
    {
        return 0u;
    }
    return ((MAP_BOX_ON_TARGET == error_return_rows[5][5]) &&
            ('C' == error_return_rows[5][3]) &&
            ('.' == error_return_rows[5][2]) &&
            (0 == strcmp(update.run_state, "RetGrid"))) ? 1u : 0u;
}

static uint8 error_return_rejects_box_start(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;

    init_error_return_context(&context);
    if(0u != art_replan_begin_error_return(&context, 5u, 5u,
                                           0.0f, 0.0f, 1u, &update))
    {
        return 0u;
    }
    return (0u == error_return_source_valid) ? 1u : 0u;
}

static void feed_center_yaw(uint16 col_q, uint16 row_q, float yaw_deg)
{
    uint8 index;
    uint16 yaw_q = (uint16)(yaw_deg * 100.0f + 0.5f);

    for(index = 0u; index < ART_CENTER_SAMPLE_COUNT; index++)
    {
        center_col_q[index] = col_q;
        center_row_q[index] = row_q;
        center_yaw_q[index] = yaw_q;
        center_yaw_valid[index] = 1u;
    }
    center_count = ART_CENTER_SAMPLE_COUNT;
    center_read = 0u;
    map_source_snapshot(&paired_center_source, paired_center_rows, &live_source);
    paired_center_valid = 1u;
}

static uint8 circular_yaw_filter_handles_wrap_and_outlier(void)
{
    art_center_batch_struct batch;
    float yaw_deg = 0.0f;
    uint8 accepted = 0u;

    art_center_batch_reset(&batch);
    batch.count = ART_CENTER_SAMPLE_COUNT;
    batch.yaw_q[0] = 35900u;
    batch.yaw_q[1] = 0u;
    batch.yaw_q[2] = 100u;
    batch.yaw_q[3] = 200u;
    batch.yaw_q[4] = 2000u;
    memset(batch.yaw_valid, 1, sizeof(batch.yaw_valid));

    if((0u == art_center_batch_get_yaw_deg(&batch, &yaw_deg, &accepted)) ||
       (4u != accepted) ||
       (fabsf(shortest_angle_error(0.5f, yaw_deg)) > 0.01f))
    {
        return 0u;
    }

    batch.yaw_valid[3] = 0u;
    return (0u == art_center_batch_get_yaw_deg(&batch, &yaw_deg, &accepted)) ? 1u : 0u;
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

static void init_pre_push_box_test(art_replan_context_struct *context)
{
    art_replan_cancel();
    competition_flow_start(COMPETITION_MODE_SUBJECT1_DEBUG);
    (void)competition_flow_take_action();
    memset(&result, 0, sizeof(result));
    memset(&snapshot, 0, sizeof(snapshot));
    build_map();
    context->result = &result;
    context->snapshot = &snapshot;
    context->snapshot_rows = snapshot_rows;
    context->snapshot_valid = &snapshot_valid;
    context->elapsed_ms = &elapsed_ms;
    context->start_row = &start_row;
    context->start_col = &start_col;
    context->run_mode = RUN_MODE_RUN;
    fake_time_ms = 0u;
    fake_executor_state = EXEC_STATE_RUNNING;
    fake_executor_error = EXEC_ERROR_NONE;
    fake_pre_push_pending = 1u;
    fake_pre_push_box_request = 1u;
    fake_pre_push_box_active = 0u;
    fake_error_on_box_active_query = 0u;
    fake_pre_push_box_row = 5u;
    fake_pre_push_box_col = 5u;
    fake_pre_push_box_state = "BGap";
    fake_pre_push_box_result = EXEC_ART_BOX_PREP_STARTED;
    fake_art_box_sample_count = 0u;
    observation_count = 0u;
    observation_read = 0u;
}

static uint8 pre_push_box_observation_flow(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint16 center_before = center_request_count;
    uint16 observation_before = observation_request_count;
    uint16 start_before = start_pre_push_box_count;

    init_pre_push_box_test(&context);
    art_replan_tick(&context, 1u, &update);
    if((observation_before + 1u != observation_request_count) ||
       (center_before != center_request_count) ||
       (5u != observation_request_row) || (5u != observation_request_col) ||
       (0 != strcmp(update.run_state, "BCtr"))) return 0u;

    feed_observation(450u, 550u, 550u, 550u);
    art_replan_tick(&context, 1u, &update);
    if((start_before + 1u != start_pre_push_box_count) ||
       (0 != strcmp(update.run_state, "BGap"))) return 0u;

    fake_pre_push_box_state = "BAlign";
    art_replan_tick(&context, 1u, &update);
    if(0 != strcmp(update.run_state, "BAlign")) return 0u;

    fake_pre_push_box_active = 0u;
    art_replan_tick(&context, 1u, &update);
    return (0 == strcmp(update.run_state, "Running")) ? 1u : 0u;
}

static uint8 pre_push_box_requests_only_after_wait(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint16 request_before;
    uint16 start_before;

    init_pre_push_box_test(&context);
    fake_pre_push_pending = 0u;
    request_before = observation_request_count;
    start_before = start_pre_push_box_count;
    art_replan_tick(&context, 1u, &update);
    if(request_before != observation_request_count) return 0u;

    fake_pre_push_pending = 1u;
    art_replan_tick(&context, 1u, &update);
    if((request_before + 1u != observation_request_count) ||
       (0 != strcmp(update.run_state, "BCtr"))) return 0u;
    feed_observation(450u, 550u, 550u, 550u);
    art_replan_tick(&context, 1u, &update);
    return ((request_before + 1u == observation_request_count) &&
            (start_before + 1u == start_pre_push_box_count) &&
            (0 == strcmp(update.run_state, "BGap"))) ? 1u : 0u;
}

static uint8 pre_push_box_failures_follow_policy(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint16 errors_before;
    uint16 continues_before;

    init_pre_push_box_test(&context);
    errors_before = executor_error_count;
    continues_before = continue_pre_push_count;
    art_replan_tick(&context, 1u, &update);
    fake_frame_count++;
    fake_time_ms += ART_BOX_OBSERVE_WAIT_MS;
    art_replan_tick(&context, 1u, &update);
    if((errors_before != executor_error_count) ||
       (continues_before + 1u != continue_pre_push_count) ||
       (0 != strcmp(update.run_state, "GridPush"))) return 0u;

    init_pre_push_box_test(&context);
    errors_before = executor_error_count;
    continues_before = continue_pre_push_count;
    fake_pre_push_box_result = EXEC_ART_BOX_PREP_GEOMETRY_ERROR;
    art_replan_tick(&context, 1u, &update);
    feed_observation(450u, 550u, 550u, 550u);
    art_replan_tick(&context, 1u, &update);
    if((errors_before != executor_error_count) ||
       (continues_before != continue_pre_push_count) ||
       (0 != strcmp(update.run_state, "ART Sync"))) return 0u;

    init_pre_push_box_test(&context);
    errors_before = executor_error_count;
    art_replan_tick(&context, 1u, &update);
    feed_observation(450u, 550u, 550u, 550u);
    art_replan_tick(&context, 1u, &update);
    fake_pre_push_box_active = 0u;
    fake_executor_error = EXEC_ERROR_ART_CENTER;
    fake_executor_state = EXEC_STATE_ERROR;
    art_replan_tick(&context, 1u, &update);
    if((errors_before != executor_error_count) ||
       (0 != strcmp(update.run_state, "ART Sync"))) return 0u;

    init_pre_push_box_test(&context);
    art_replan_tick(&context, 1u, &update);
    feed_observation(450u, 550u, 550u, 550u);
    art_replan_tick(&context, 1u, &update);
    fake_error_on_box_active_query = 1u;
    art_replan_tick(&context, 1u, &update);
    return (0 == strcmp(update.run_state, "ART Sync")) ? 1u : 0u;
}

static void publish_stable_map(const art_replan_context_struct *context,
                               art_replan_update_struct *update)
{
    uint8 frame;
    for(frame = 0; frame < (uint8)(EXEC_ART_STABLE_FRAMES + 1u); frame++)
    {
        fake_frame_count++;
        art_replan_tick(context, 1u, update);
    }
}

static uint8 run_test(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint16 solve_before;

    memset(&result, 0, sizeof(result));
    memset(&snapshot, 0, sizeof(snapshot));
    build_map();
    context.result = &result;
    context.snapshot = &snapshot;
    context.snapshot_rows = snapshot_rows;
    context.snapshot_valid = &snapshot_valid;
    context.elapsed_ms = &elapsed_ms;
    context.start_row = &start_row;
    context.start_col = &start_col;
    context.run_mode = RUN_MODE_RUN;

    memset(&update, 0xA5, sizeof(update));
    art_replan_begin_initial(&update);
    if((0u != update.classification_map_ready) || (0u != update.return_complete) ||
       (0.0f != update.initial_pose_x_cm) || (0.0f != update.initial_pose_y_cm)) return 0;
    fake_time_ms = 5000u;
    art_replan_tick(&context, 1u, &update);
    if((1u != center_request_count) || (0 != strcmp(update.run_state, "WCTR"))) return 0;

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS - 1u;
    art_replan_tick(&context, 1u, &update);
    if((1u != center_request_count) || (0u != executor_error_count)) return 0;

    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    art_replan_tick(&context, 1u, &update);
    publish_stable_map(&context, &update);
    if((2u != center_request_count) || (0 != strcmp(update.run_state, "ICtr"))) return 0;

    solve_before = solve_count;
    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS - 1u;
    art_replan_tick(&context, 1u, &update);
    if((solve_before != solve_count) || (0u != executor_error_count)) return 0;

    feed_center(260u, 550u, 259u, 550u, 261u, 550u);
    live_rows[5][2] = '.';
    live_rows[5][3] = 'C';
    art_replan_tick(&context, 1u, &update);
    if((1u != solve_count) || (1u != executor_start_count) ||
       (2u != start_col) || ('C' != snapshot_rows[5][2]) ||
       ('.' != snapshot_rows[5][3]) ||
       (fabsf(executor_initial_x_cm - 2.0f) > 0.01f) ||
       (fabsf(executor_initial_y_cm) > 0.01f)) return 0;
    live_rows[5][3] = '.';
    live_rows[5][2] = 'C';

    fake_sync_pending = 1;
    fake_sync_action = 'R';
    art_replan_tick(&context, 1u, &update);
    publish_stable_map(&context, &update);
    if((3u != center_request_count) || (0 != strcmp(update.run_state, "RCtr"))) return 0;

    solve_before = solve_count;
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS - 1u;
    art_replan_tick(&context, 1u, &update);
    if((solve_before != solve_count) || (0u != executor_error_count)) return 0;

    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    live_rows[5][2] = '.';
    live_rows[5][3] = 'C';
    art_replan_tick(&context, 1u, &update);
    if((2u != solve_count) || (2u != executor_start_count) ||
       (2u != start_col) || ('C' != snapshot_rows[5][2]) ||
       ('.' != snapshot_rows[5][3])) return 0;
    live_rows[5][3] = '.';
    live_rows[5][2] = 'C';

    live_rows[5][5] = '.';
    live_rows[5][6] = '.';
    fake_sync_pending = 1;
    fake_sync_action = 'R';
    art_replan_tick(&context, 1u, &update);
    publish_stable_map(&context, &update);
    if((4u != center_request_count) || (0 != strcmp(update.run_state, "RCtr"))) return 0;

    feed_center(250u, 550u, 249u, 550u, 251u, 550u);
    art_replan_tick(&context, 1u, &update);
    if(3u != executor_start_count) return 0;

    fake_executor_state = EXEC_STATE_DONE;
    art_replan_tick(&context, 1u, &update);
    if((5u != center_request_count) || (0 != strcmp(update.run_state, "RetCtr"))) return 0;

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS - 1u;
    art_replan_tick(&context, 1u, &update);
    if(0u != executor_error_count) return 0;

    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    art_replan_tick(&context, 1u, &update);
    if((6u != center_request_count) || (0 != strcmp(update.run_state, "RetChk"))) return 0;

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS - 1u;
    art_replan_tick(&context, 1u, &update);
    if(0u != executor_error_count) return 0;

    feed_center(250u, 550u, 249u, 550u, 251u, 550u);
    art_replan_tick(&context, 1u, &update);
    if((EXEC_STATE_DONE != fake_executor_state) || (0u == update.return_complete)) return 0u;

    live_rows[5][5] = 'B';
    live_rows[5][6] = 'T';
    fake_executor_state = EXEC_STATE_IDLE;
    solve_before = solve_count;
    {
        uint16 starts_before = executor_start_count;

        art_replan_begin_classification(&update);
        fake_time_ms += 5000u;
        art_replan_tick(&context, 1u, &update);
        if(0 != strcmp(update.run_state, "WCTR")) return 0u;

        feed_center(250u, 550u, 251u, 550u, 249u, 550u);
        art_replan_tick(&context, 1u, &update);
        publish_stable_map(&context, &update);
        if(0 != strcmp(update.run_state, "ICtr")) return 0u;

        feed_center(260u, 550u, 259u, 550u, 261u, 550u);
        art_replan_tick(&context, 1u, &update);
        if((0u == update.classification_map_ready) ||
           (solve_before != solve_count) ||
           (starts_before != executor_start_count) ||
           (fabsf(update.initial_pose_x_cm - 2.0f) > 0.01f) ||
           (fabsf(update.initial_pose_y_cm) > 0.01f)) return 0u;
    }

    art_replan_cancel();
    fake_pre_push_pending = 1u;
    fake_center_requires_push_alignment = 1u;
    fake_executor_state = EXEC_STATE_RUNNING;
    fake_time_ms += 100u;
    art_replan_tick(&context, 1u, &update);
    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    live_rows[5][2] = '.';
    live_rows[5][3] = 'C';
    art_replan_tick(&context, 1u, &update);
    if((0u != fake_pre_push_pending) ||
       (1u != start_pre_push_alignment_count) ||
       (0u != continue_pre_push_count) ||
       (5u != last_commit_row) || (2u != last_commit_col)) return 0u;
    live_rows[5][3] = '.';
    live_rows[5][2] = 'C';

    art_replan_cancel();
    fake_pre_push_pending = 1u;
    fake_executor_state = EXEC_STATE_RUNNING;
    fake_time_ms += 100u;
    art_replan_tick(&context, 1u, &update);
    if((0 != strcmp(update.run_state, "PCtr")) ||
       (0u != continue_pre_push_count)) return 0u;

    fake_frame_count++;
    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    if((0u != fake_pre_push_pending) ||
       (1u != continue_pre_push_count) ||
       (1u != start_pre_push_alignment_count) ||
       (0u != executor_error_count) ||
       (0 != strcmp(update.run_state, "CtrSkip"))) return 0u;
#else
    if((0u != executor_error_count) ||
       (0 != strcmp(update.run_state, "ART Sync"))) return 0u;
#endif

    return 1u;
}

static uint8 run_timeout_fallback_test(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint16 request_before = center_request_count;
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    uint16 solve_before = solve_count;
    uint16 start_before = executor_start_count;
#endif
    uint16 error_before = executor_error_count;
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    uint16 correction_before = position_correction_start_count;
#endif

#define TIMEOUT_REQUIRE(condition, checkpoint) \
    do { if(!(condition)) { printf("timeout-checkpoint-%u\n", (unsigned)(checkpoint)); return 0u; } } while(0)

    build_map();
    memset(&result, 0, sizeof(result));
    context.result = &result;
    context.snapshot = &snapshot;
    context.snapshot_rows = snapshot_rows;
    context.snapshot_valid = &snapshot_valid;
    context.elapsed_ms = &elapsed_ms;
    context.start_row = &start_row;
    context.start_col = &start_col;
    context.run_mode = RUN_MODE_RUN;
    fake_time_ms = 0u;
    fake_frame_count = 0u;
    fake_pose.x_cm = 0.0f;
    fake_pose.y_cm = 0.0f;
    fake_executor_state = EXEC_STATE_IDLE;
    fake_sync_pending = 0u;
    fake_pre_push_pending = 0u;
    art_replan_begin_initial(&update);
    fake_time_ms = 5000u;
    art_replan_tick(&context, 1u, &update);
    TIMEOUT_REQUIRE((request_before + 1u == center_request_count) &&
                    (0 == strcmp(update.run_state, "WCTR")), 1u);

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    TIMEOUT_REQUIRE((0 == strcmp(update.run_state, "LCH")) &&
                    (correction_before + 1u == position_correction_start_count) &&
                    (fabsf(position_correction_target_x_cm -
                           ART_LAUNCH_FALLBACK_MOVE_CM) <= 0.01f) &&
                    (fabsf(position_correction_target_y_cm) <= 0.01f), 2u);
    fake_executor_state = EXEC_STATE_DONE;
    art_replan_tick(&context, 1u, &update);
    TIMEOUT_REQUIRE(0 == strcmp(update.run_state, "WMAP"), 3u);

    publish_stable_map(&context, &update);
    TIMEOUT_REQUIRE((request_before + 2u == center_request_count) &&
                    (0 == strcmp(update.run_state, "ICtr")), 4u);
    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    TIMEOUT_REQUIRE((solve_before + 1u == solve_count) &&
                    (start_before + 1u == executor_start_count) &&
                    (fabsf(executor_initial_x_cm) <= 0.01f) &&
                    (fabsf(executor_initial_y_cm) <= 0.01f), 5u);

    fake_sync_pending = 1u;
    fake_sync_action = 'R';
    art_replan_tick(&context, 1u, &update);
    publish_stable_map(&context, &update);
    TIMEOUT_REQUIRE((request_before + 3u == center_request_count) &&
                    (0 == strcmp(update.run_state, "RCtr")), 6u);
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    TIMEOUT_REQUIRE((solve_before + 2u == solve_count) &&
                    (start_before + 2u == executor_start_count), 7u);

    live_rows[5][5] = '.';
    live_rows[5][6] = '.';
    fake_sync_pending = 1u;
    fake_sync_action = 'R';
    art_replan_tick(&context, 1u, &update);
    publish_stable_map(&context, &update);
    TIMEOUT_REQUIRE((request_before + 4u == center_request_count) &&
                    (0 == strcmp(update.run_state, "RCtr")), 8u);
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    fake_executor_state = EXEC_STATE_DONE;
    art_replan_tick(&context, 1u, &update);
    TIMEOUT_REQUIRE((request_before + 5u == center_request_count) &&
                    (0 == strcmp(update.run_state, "RetCtr")), 9u);

    live_rows[5][2] = '.';
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    TIMEOUT_REQUIRE((correction_before + 2u == position_correction_start_count) &&
                    (fabsf(position_correction_target_x_cm +
                           ART_LAUNCH_FALLBACK_MOVE_CM) <= 0.01f) &&
                    (fabsf(position_correction_target_y_cm) <= 0.01f), 10u);
    fake_executor_state = EXEC_STATE_DONE;
    art_replan_tick(&context, 1u, &update);
    TIMEOUT_REQUIRE((request_before + 6u == center_request_count) &&
                    (0 == strcmp(update.run_state, "RetChk")), 11u);
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    TIMEOUT_REQUIRE((request_before + 7u == center_request_count) &&
                    (EXEC_STATE_ERROR != fake_executor_state) &&
                    (0 == strcmp(update.run_state, "RetChk")), 12u);
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    TIMEOUT_REQUIRE((request_before + 8u == center_request_count) &&
                    (EXEC_STATE_ERROR != fake_executor_state) &&
                    (0 == strcmp(update.run_state, "RetChk")), 13u);
    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    TIMEOUT_REQUIRE((EXEC_STATE_ERROR == fake_executor_state) &&
                    (0u == update.return_complete) &&
                    (error_before + 1u == executor_error_count) &&
                    (0 == strcmp(update.run_state, "F:Return")), 14u);
    return 1u;
#else
    return ((error_before + 1u == executor_error_count) &&
            (0 == strcmp(update.run_state, "F:ART1"))) ? 1u : 0u;
#endif

#undef TIMEOUT_REQUIRE
}

static uint8 host_completion_before_task_end_replans(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    art_replan_debug_status_struct status;

    art_replan_cancel();
    fake_time_ms = 0u;
    fake_frame_count = 0u;
    center_count = 0u;
    center_read = 0u;
    center_request_count = 0u;
    solve_count = 0u;
    executor_start_count = 0u;
    executor_error_count = 0u;
    push_boundary_request_count = 0u;
    push_boundary_force_count = 0u;
    push_boundary_resume_count = 0u;
    continue_art_sync_count = 0u;
    fake_push_boundary_ready = 0u;
    fake_sync_pending = 0u;
    fake_pre_push_pending = 0u;
    fake_executor_state = EXEC_STATE_IDLE;
    memset(&fake_pose, 0, sizeof(fake_pose));
    memset(&result, 0, sizeof(result));
    memset(&snapshot, 0, sizeof(snapshot));
    build_map();
    live_rows[4][5] = 'B';
    live_rows[4][6] = 'T';

    context.result = &result;
    context.snapshot = &snapshot;
    context.snapshot_rows = snapshot_rows;
    context.snapshot_valid = &snapshot_valid;
    context.elapsed_ms = &elapsed_ms;
    context.start_row = &start_row;
    context.start_col = &start_col;
    context.run_mode = RUN_MODE_RUN;

    art_replan_begin_initial(&update);
    fake_time_ms = 5000u;
    art_replan_tick(&context, 1u, &update);
    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    art_replan_tick(&context, 1u, &update);
    publish_stable_map(&context, &update);
    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    art_replan_tick(&context, 1u, &update);
    if((1u != executor_start_count) ||
       (EXEC_STATE_RUNNING != fake_executor_state)) return 0u;

    live_rows[4][5] = '.';
    live_rows[4][6] = '.';
    fake_frame_count++;
    art_replan_tick(&context, 1u, &update);
    if((EXEC_STATE_RUNNING != fake_executor_state) ||
       (0u != push_boundary_request_count)) return 0u;
    fake_frame_count++;
    art_replan_tick(&context, 1u, &update);
    if((EXEC_STATE_RUNNING != fake_executor_state) ||
       (1u != push_boundary_request_count) ||
       (0 != strcmp(update.run_state, "Host Pend"))) return 0u;

    fake_push_boundary_ready = 1u;
    art_replan_tick(&context, 1u, &update);
    if((EXEC_STATE_RUNNING != fake_executor_state) ||
       (0 != strcmp(update.run_state, "Host Sync"))) return 0u;

    publish_stable_map(&context, &update);
    if(0 != strcmp(update.run_state, "RCtr")) return 0u;
    art_replan_get_debug_status(&status);
    if((2u != status.confirmed_box_count) ||
       (2u != status.confirmed_target_count)) return 0u;
    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    live_rows[4][5] = 'B';
    live_rows[4][6] = 'T';
    art_replan_tick(&context, 1u, &update);
    art_replan_get_debug_status(&status);

    if((2u != executor_start_count) ||
       (EXEC_STATE_RUNNING != fake_executor_state) ||
       (1u != status.confirmed_box_count) ||
       (1u != status.confirmed_target_count)) return 0u;

    fake_pre_push_pending = 1u;
    fake_pre_push_box_request = 1u;
    art_replan_tick(&context, 1u, &update);
    if(0 != strcmp(update.run_state, "BCtr")) return 0u;
    live_rows[4][5] = '.';
    live_rows[4][6] = '.';
    live_rows[5][5] = '.';
    live_rows[5][6] = '.';
    fake_frame_count++;
    art_replan_tick(&context, 1u, &update);
    if(0 != strcmp(update.run_state, "BCtr")) return 0u;
    fake_frame_count++;
    art_replan_tick(&context, 1u, &update);
    return ((EXEC_STATE_IDLE == fake_executor_state) &&
            (0 == strcmp(update.run_state, "Host Sync"))) ? 1u : 0u;
}

static uint8 start_two_box_art_path(art_replan_context_struct *context,
                                    art_replan_update_struct *update)
{
    art_replan_cancel();
    fake_time_ms = 0u;
    fake_frame_count = 0u;
    center_count = 0u;
    center_read = 0u;
    center_request_count = 0u;
    solve_count = 0u;
    executor_start_count = 0u;
    executor_error_count = 0u;
    push_boundary_request_count = 0u;
    push_boundary_force_count = 0u;
    push_boundary_resume_count = 0u;
    continue_art_sync_count = 0u;
    fake_push_boundary_ready = 0u;
    fake_sync_pending = 0u;
    fake_pre_push_pending = 0u;
    fake_executor_state = EXEC_STATE_IDLE;
    memset(&fake_pose, 0, sizeof(fake_pose));
    memset(&result, 0, sizeof(result));
    memset(&snapshot, 0, sizeof(snapshot));
    build_map();
    live_rows[4][5] = 'B';
    live_rows[4][6] = 'T';

    context->result = &result;
    context->snapshot = &snapshot;
    context->snapshot_rows = snapshot_rows;
    context->snapshot_valid = &snapshot_valid;
    context->elapsed_ms = &elapsed_ms;
    context->start_row = &start_row;
    context->start_col = &start_col;
    context->run_mode = RUN_MODE_RUN;

    art_replan_begin_initial(update);
    fake_time_ms = 5000u;
    art_replan_tick(context, 1u, update);
    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    art_replan_tick(context, 1u, update);
    publish_stable_map(context, update);
    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    art_replan_tick(context, 1u, update);
    return ((1u == executor_start_count) &&
            (EXEC_STATE_RUNNING == fake_executor_state)) ? 1u : 0u;
}

static uint8 host_sync_timeout_resumes_old_path(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint8 passed;

    if(0u == start_two_box_art_path(&context, &update)) return 0u;
    live_rows[4][5] = '.';
    live_rows[4][6] = '.';
    fake_frame_count++;
    art_replan_tick(&context, 1u, &update);
    if((EXEC_STATE_RUNNING != fake_executor_state) ||
       (0u != push_boundary_request_count)) return 0u;
    fake_frame_count++;
    art_replan_tick(&context, 1u, &update);
    if(0 != strcmp(update.run_state, "Host Pend")) return 0u;

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    if((1u != push_boundary_force_count) ||
       (0 != strcmp(update.run_state, "Host Sync"))) return 0u;

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    if((0u != push_boundary_resume_count) ||
       (0 != strcmp(update.run_state, "ART Retry"))) return 0u;

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    passed = ((1u == push_boundary_resume_count) &&
              (EXEC_STATE_RUNNING == fake_executor_state) &&
              (0 == strcmp(update.run_state, "MCU Go"))) ? 1u : 0u;
    art_replan_begin_initial(&update);
    art_replan_cancel();
    return passed;
}

static uint8 invalid_map_does_not_replace_snapshot(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint8 passed;

    if(0u == start_two_box_art_path(&context, &update)) return 0u;
    fake_sync_pending = 1u;
    art_replan_tick(&context, 1u, &update);
    live_rows[4][5] = '.';
    publish_stable_map(&context, &update);
    passed = (('B' == snapshot_rows[4][5]) &&
              ('T' == snapshot_rows[4][6])) ? 1u : 0u;
    art_replan_cancel();
    return passed;
}

static uint8 normal_art_sync_timeout_continues_old_path(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint8 passed;

    if(0u == start_two_box_art_path(&context, &update)) return 0u;
    fake_sync_pending = 1u;
    fake_sync_action = 'R';
    art_replan_tick(&context, 1u, &update);

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    if((0u != continue_art_sync_count) ||
       (0 != strcmp(update.run_state, "ART Retry"))) return 0u;

    fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    passed = ((1u == continue_art_sync_count) &&
              (0 == strcmp(update.run_state, "MCU Go"))) ? 1u : 0u;
    art_replan_begin_initial(&update);
    art_replan_cancel();
    return passed;
}

static uint8 cross_cell_center_retries_without_resetting_timeout(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint16 request_before;
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    uint16 continue_before;
#endif
    uint16 error_before;

    init_pre_push_box_test(&context);
    fake_pre_push_box_request = 0u;
    fake_center_requires_push_alignment = 0u;
    request_before = center_request_count;
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    continue_before = continue_pre_push_count;
#endif
    error_before = executor_error_count;

    art_replan_tick(&context, 1u, &update);
    if((request_before + 1u != center_request_count) ||
       (0 != strcmp(update.run_state, "PCtr"))) return 0u;

    feed_center(350u, 550u, 351u, 550u, 349u, 550u);
    art_replan_tick(&context, 1u, &update);
    if((request_before + 2u != center_request_count) ||
       (0 != strcmp(update.run_state, "PCtr")) ||
       (0u == fake_pre_push_pending)) return 0u;

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    return ((continue_before == continue_pre_push_count) &&
            (error_before == executor_error_count) &&
            (0 == strcmp(update.run_state, "ART Sync"))) ? 1u : 0u;
#else
    return ((error_before == executor_error_count) &&
            (0 == strcmp(update.run_state, "ART Sync"))) ? 1u : 0u;
#endif
}

static void init_launch_yaw_test(art_replan_context_struct *context)
{
    art_replan_cancel();
    art_replan_reset_competition_yaw();
    memset(&result, 0, sizeof(result));
    memset(&snapshot, 0, sizeof(snapshot));
    memset(&fake_control_status, 0, sizeof(fake_control_status));
    memset(&fake_pose, 0, sizeof(fake_pose));
    build_map();
    context->result = &result;
    context->snapshot = &snapshot;
    context->snapshot_rows = snapshot_rows;
    context->snapshot_valid = &snapshot_valid;
    context->elapsed_ms = &elapsed_ms;
    context->start_row = &start_row;
    context->start_col = &start_col;
    context->run_mode = RUN_MODE_RUN;
    fake_time_ms = 0u;
    fake_executor_state = EXEC_STATE_IDLE;
    fake_executor_error = EXEC_ERROR_NONE;
    center_count = 0u;
    center_read = 0u;
    yaw_correction_start_count = 0u;
    yaw_correction_delta_deg = 0.0f;
    yaw_rebase_count = 0u;
    position_correction_start_count = 0u;
}

static uint8 per_subject_launch_yaw_flow(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint16 request_before;
    float launch_yaw_bias_deg = 0.0f;

    init_launch_yaw_test(&context);
    request_before = center_request_count;
    art_replan_begin_initial(&update);
    fake_time_ms = 5000u;
    art_replan_tick(&context, 1u, &update);
    feed_center_yaw(85u, 525u, 177.0f);
    art_replan_tick(&context, 1u, &update);
    if((request_before + 1u != center_request_count) ||
       (0u != yaw_correction_start_count) ||
       (1u != yaw_rebase_count) ||
       (0 != strcmp(update.run_state, "LCH")) ||
       (0u == art_replan_get_launch_yaw_bias(&launch_yaw_bias_deg)) ||
       (fabsf(launch_yaw_bias_deg - 3.0f) > 0.01f)) return 0u;

    art_replan_begin_classification(&update);
    fake_time_ms += 5000u;
    art_replan_tick(&context, 1u, &update);
    feed_center_yaw(85u, 525u, 184.0f);
    art_replan_tick(&context, 1u, &update);
    if((1u != yaw_correction_start_count) ||
       (fabsf(yaw_correction_delta_deg + 7.0f) > 0.01f) ||
       (0 != strcmp(update.run_state, "YawFix"))) return 0u;

    fake_control_status.yaw_error = 0.0f;
    art_replan_tick(&context, 1u, &update);
    fake_time_ms += SUBJECT2_TURN_STABLE_MS;
    art_replan_tick(&context, 1u, &update);
    return ((2u == yaw_rebase_count) &&
            (2u == position_correction_start_count) &&
            (0 == strcmp(update.run_state, "LCH"))) ? 1u : 0u;
}

static uint8 large_launch_yaw_requires_matching_recheck(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint16 request_before;

    init_launch_yaw_test(&context);
    art_replan_begin_initial(&update);
    fake_time_ms = 5000u;
    art_replan_tick(&context, 1u, &update);
    feed_center_yaw(85u, 525u, 180.0f);
    art_replan_tick(&context, 1u, &update);

    art_replan_begin_classification(&update);
    fake_time_ms += 5000u;
    art_replan_tick(&context, 1u, &update);
    request_before = center_request_count;
    feed_center_yaw(85u, 525u, 195.0f);
    art_replan_tick(&context, 1u, &update);
    if((request_before + 1u != center_request_count) ||
       (0 != strcmp(update.run_state, "YawChk")) ||
       (0u != yaw_correction_start_count)) return 0u;

    feed_center_yaw(85u, 525u, 196.0f);
    art_replan_tick(&context, 1u, &update);
    return ((1u == yaw_correction_start_count) &&
            (fabsf(yaw_correction_delta_deg + 16.0f) <= 0.01f) &&
            (0 == strcmp(update.run_state, "YawFix"))) ? 1u : 0u;
}

static uint8 launch_yaw_fallbacks_and_turn_timeout(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;

    init_launch_yaw_test(&context);
    art_replan_begin_initial(&update);
    fake_time_ms = 5000u;
    art_replan_tick(&context, 1u, &update);
    feed_center_yaw(85u, 525u, 180.0f);
    art_replan_tick(&context, 1u, &update);
    art_replan_begin_classification(&update);
    fake_time_ms += 5000u;
    art_replan_tick(&context, 1u, &update);
    feed_center_yaw(95u, 525u, 190.0f);
    art_replan_tick(&context, 1u, &update);
    if((0u != yaw_correction_start_count) ||
       (2u != yaw_rebase_count) ||
       (0 != strcmp(update.run_state, "LCH"))) return 0u;

    init_launch_yaw_test(&context);
    art_replan_begin_initial(&update);
    fake_time_ms = 5000u;
    art_replan_tick(&context, 1u, &update);
    fake_time_ms += ART_LAUNCH_YAW_TIMEOUT_MS + 1u;
    feed_center_yaw(85u, 525u, 180.0f);
    art_replan_tick(&context, 1u, &update);
    if((0u != yaw_correction_start_count) ||
       (1u != yaw_rebase_count) ||
       (0 != strcmp(update.run_state, "LCH"))) return 0u;

    init_launch_yaw_test(&context);
    art_replan_begin_initial(&update);
    fake_time_ms = 5000u;
    art_replan_tick(&context, 1u, &update);
    feed_center_yaw(85u, 525u, 180.0f);
    art_replan_tick(&context, 1u, &update);
    art_replan_begin_classification(&update);
    fake_time_ms += 5000u;
    art_replan_tick(&context, 1u, &update);
    feed_center_yaw(85u, 525u, 187.0f);
    art_replan_tick(&context, 1u, &update);
    fake_time_ms += SUBJECT2_TURN_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    return ((EXEC_STATE_ERROR != fake_executor_state) &&
            (0u != yaw_rebase_count) &&
            (0 == strcmp(update.run_state, "LCH"))) ? 1u : 0u;
}

static uint8 map_recovery_escalates_and_manual_restart_waits(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint8 retry;

    art_replan_cancel();
    competition_flow_start(COMPETITION_MODE_SUBJECT1_DEBUG);
    (void)competition_flow_take_action();
    memset(&result, 0, sizeof(result));
    context.result = &result;
    context.snapshot = &snapshot;
    context.snapshot_rows = snapshot_rows;
    context.snapshot_valid = &snapshot_valid;
    context.elapsed_ms = &elapsed_ms;
    context.start_row = &start_row;
    context.start_col = &start_col;
    context.run_mode = RUN_MODE_RUN;
    fake_time_ms = 0u;
    fake_frame_count = 0u;
    fake_executor_state = EXEC_STATE_IDLE;
    fake_executor_error = EXEC_ERROR_NONE;

    if(0u == art_replan_manual_recover(&update)) return 0u;
    for(retry = 0u; retry <= RECOVERY_MAX_RETRIES; retry++)
    {
        fake_time_ms += RECOVERY_RESYNC_TIMEOUT_MS;
        art_replan_tick(&context, 1u, &update);
    }
    if((0u == competition_flow_is_fatal()) ||
       (COMPETITION_FATAL_MAP != competition_flow_get_fatal_reason()) ||
       (0 != strcmp(update.run_state, "F:Map"))) return 0u;

    if(0u == art_replan_manual_recover(&update)) return 0u;
    competition_flow_clear_fatal();
    if((0 != strcmp(update.run_state, "ART Sync")) &&
       (0 != strcmp(update.run_state, "WMAP"))) return 0u;
    art_replan_tick(&context, 1u, &update);
    return ((EXEC_STATE_RUNNING != fake_executor_state) &&
            (0u == competition_flow_is_fatal())) ? 1u : 0u;
}

int main(void)
{
    uint8 all_passed = 1u;
    uint8 passed;

    passed = circular_yaw_filter_handles_wrap_and_outlier();
    printf("center-yaw-circular-filter   %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;

    passed = run_test();
    printf("request-driven-art-state      %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = run_timeout_fallback_test();
    printf("request-center-timeout-policy %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = host_completion_before_task_end_replans();
    printf("host-completion-replan        %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = host_sync_timeout_resumes_old_path();
    printf("host-sync-timeout-resume      %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = invalid_map_does_not_replace_snapshot();
    printf("invalid-map-keeps-snapshot    %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = normal_art_sync_timeout_continues_old_path();
    printf("art-sync-timeout-continue     %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = cross_cell_center_retries_without_resetting_timeout();
    printf("center-cross-cell-retry       %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = pre_push_box_observation_flow();
    printf("pre-push-box-observation      %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = pre_push_box_requests_only_after_wait();
    printf("pre-push-box-after-stop       %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = pre_push_box_failures_follow_policy();
    printf("pre-push-box-failures         %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = per_subject_launch_yaw_flow();
    printf("per-subject-launch-yaw       %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = large_launch_yaw_requires_matching_recheck();
    printf("launch-yaw-recheck           %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = launch_yaw_fallbacks_and_turn_timeout();
    printf("launch-yaw-fallbacks         %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = map_recovery_escalates_and_manual_restart_waits();
    printf("map-fatal-manual-recovery     %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = error_return_preserves_obstacles();
    printf("error-return-obstacles        %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    passed = error_return_rejects_box_start();
    printf("error-return-box-start        %s\n", (0 != passed) ? "PASS" : "FAIL");
    all_passed &= passed;
    return (0 != all_passed) ? 0 : 1;
}
