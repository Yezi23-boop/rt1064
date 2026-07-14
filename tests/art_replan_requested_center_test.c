#include "art_replan.h"
#include "drive_pose.h"
#include "executor.h"
#include "map_utils.h"
#include "openart_uart.h"
#include "solver.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32 fake_time_ms;
static uint32 fake_frame_count;
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
static uint16 solve_count;
static uint16 executor_start_count;
static uint16 executor_error_count;
static float executor_initial_x_cm;
static float executor_initial_y_cm;
static uint8 fake_sync_pending;
static uint8 fake_pre_push_pending;
static uint8 fake_pre_push_box_request;
static uint8 fake_pre_push_box_prefetch_request;
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
static char fake_sync_action;
static executor_state_enum fake_executor_state;
static executor_error_enum fake_executor_error;
static drive_pose_struct fake_pose;
static char live_rows[MAP_ROWS][MAP_COLS + 1];
static map_source_struct live_source;
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
    fake_pose.x_cm = x;
    fake_pose.y_cm = y;
    fake_pose.yaw_deg = yaw;
}
void stop_motion(void) { }
void reset_motion_segment(void) { }
void set_motion(float vx, float vy) { (void)vx; (void)vy; }
float limit_float(float value, float min_value, float max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

void openart_request_player_center(void)
{
    center_request_count++;
    center_count = 0;
    center_read = 0;
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
uint8 openart_get_requested_center_sample(uint16 *col_q, uint16 *row_q)
{
    if(center_read >= center_count) return 0;
    *col_q = center_col_q[center_read];
    *row_q = center_row_q[center_read];
    center_read++;
    return center_read;
}
const map_source_struct *openart_map_get(void) { return &live_source; }
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
void executor_stop(void) { fake_executor_state = EXEC_STATE_IDLE; }
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
uint8 executor_get_pre_push_box_request(uint8 *box_row, uint8 *box_col)
{
    if((0u == fake_pre_push_pending) || (0u == fake_pre_push_box_request)) return 0u;
    *box_row = fake_pre_push_box_row;
    *box_col = fake_pre_push_box_col;
    return 1u;
}
uint8 executor_get_pre_push_box_prefetch_request(uint8 *box_row, uint8 *box_col)
{
    if(0u == fake_pre_push_box_prefetch_request) return 0u;
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
uint8 executor_continue_after_art_sync(void) { fake_sync_pending = 0; return 1; }
uint16 executor_get_current_step(void) { return 0; }
const char *executor_state_name(void) { return "Running"; }
uint8 executor_apply_art_player_center(uint16 col, uint16 row, uint32 sample)
{
    (void)col; (void)row;
    return (sample >= ART_CENTER_SAMPLE_COUNT) ? 1u : 0u;
}
executor_art_center_result_enum executor_commit_art_player_center(uint8 row, uint8 col)
{
    (void)row; (void)col;
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
    center_count = ART_CENTER_SAMPLE_COUNT;
    center_read = 0;
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
    fake_pre_push_box_prefetch_request = 0u;
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

static uint8 pre_push_box_prefetch_freshness(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint16 request_before;
    uint16 start_before;

    init_pre_push_box_test(&context);
    fake_pre_push_pending = 0u;
    fake_pre_push_box_prefetch_request = 1u;
    request_before = observation_request_count;
    start_before = start_pre_push_box_count;
    art_replan_tick(&context, 1u, &update);
    if(request_before + 1u != observation_request_count) return 0u;
    feed_observation(450u, 550u, 550u, 550u);
    art_replan_tick(&context, 1u, &update);

    fake_pre_push_pending = 1u;
    fake_pre_push_box_prefetch_request = 0u;
    fake_time_ms += ART_BOX_OBSERVE_SAMPLE_MAX_AGE_MS;
    art_replan_tick(&context, 1u, &update);
    if((request_before + 1u != observation_request_count) ||
       (start_before + 1u != start_pre_push_box_count) ||
       (0 != strcmp(update.run_state, "BGap"))) return 0u;

    init_pre_push_box_test(&context);
    fake_pre_push_pending = 0u;
    fake_pre_push_box_prefetch_request = 1u;
    request_before = observation_request_count;
    start_before = start_pre_push_box_count;
    art_replan_tick(&context, 1u, &update);
    feed_observation(450u, 550u, 550u, 550u);
    art_replan_tick(&context, 1u, &update);
    fake_time_ms += ART_BOX_OBSERVE_SAMPLE_MAX_AGE_MS + 1u;
    fake_pre_push_pending = 1u;
    fake_pre_push_box_prefetch_request = 0u;
    art_replan_tick(&context, 1u, &update);
    return ((request_before + 2u == observation_request_count) &&
            (start_before == start_pre_push_box_count) &&
            (0 == strcmp(update.run_state, "BCtr"))) ? 1u : 0u;
}

static uint8 pre_push_box_failures_follow_policy(void)
{
    art_replan_context_struct context;
    art_replan_update_struct update;
    uint16 errors_before;
    uint16 continues_before;

    init_pre_push_box_test(&context);
    errors_before = executor_error_count;
    art_replan_tick(&context, 1u, &update);
    fake_time_ms += ART_BOX_OBSERVE_WAIT_MS;
    art_replan_tick(&context, 1u, &update);
    if((errors_before != executor_error_count) ||
       (0u == start_pre_push_box_retry_count) ||
       (0 != strcmp(update.run_state, "BRetry"))) return 0u;
    fake_pre_push_box_active = 0u;
    art_replan_tick(&context, 1u, &update);
    fake_time_ms += ART_BOX_OBSERVE_RETRY_SETTLE_MS;
    art_replan_tick(&context, 1u, &update);
    if(0 != strcmp(update.run_state, "BCtr")) return 0u;
    fake_time_ms += ART_BOX_OBSERVE_WAIT_MS;
    art_replan_tick(&context, 1u, &update);
    if((errors_before + 1u != executor_error_count) ||
       (0 != strcmp(update.run_state, "E:BObs"))) return 0u;

    init_pre_push_box_test(&context);
    errors_before = executor_error_count;
    continues_before = continue_pre_push_count;
    fake_pre_push_box_result = EXEC_ART_BOX_PREP_GEOMETRY_ERROR;
    art_replan_tick(&context, 1u, &update);
    feed_observation(450u, 550u, 550u, 550u);
    art_replan_tick(&context, 1u, &update);
    if((errors_before != executor_error_count) ||
       (continues_before + 1u != continue_pre_push_count) ||
       (0 != strcmp(update.run_state, "Running"))) return 0u;

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
       (0 != strcmp(update.run_state, "E:BTim"))) return 0u;

    init_pre_push_box_test(&context);
    art_replan_tick(&context, 1u, &update);
    feed_observation(450u, 550u, 550u, 550u);
    art_replan_tick(&context, 1u, &update);
    fake_error_on_box_active_query = 1u;
    art_replan_tick(&context, 1u, &update);
    return (0 == strcmp(update.run_state, "E:BTim")) ? 1u : 0u;
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
    if((0u != update.subject2_map_ready) || (0u != update.return_complete) ||
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
    art_replan_tick(&context, 1u, &update);
    if((1u != solve_count) || (1u != executor_start_count) ||
       (fabsf(executor_initial_x_cm - 2.0f) > 0.01f) ||
       (fabsf(executor_initial_y_cm) > 0.01f)) return 0;

    fake_sync_pending = 1;
    fake_sync_action = 'R';
    art_replan_tick(&context, 1u, &update);
    publish_stable_map(&context, &update);
    if((3u != center_request_count) || (0 != strcmp(update.run_state, "RCtr"))) return 0;

    solve_before = solve_count;
    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS - 1u;
    art_replan_tick(&context, 1u, &update);
    if((solve_before != solve_count) || (0u != executor_error_count)) return 0;

    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    art_replan_tick(&context, 1u, &update);
    if((2u != solve_count) || (2u != executor_start_count)) return 0;

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

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS - 1u;
    art_replan_tick(&context, 1u, &update);
    if(0u != executor_error_count) return 0;

    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    art_replan_tick(&context, 1u, &update);
    if((6u != center_request_count) || (0 != strcmp(update.run_state, "RetChk"))) return 0;

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS - 1u;
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

        art_replan_begin_subject2(&update);
        fake_time_ms += 5000u;
        art_replan_tick(&context, 1u, &update);
        if(0 != strcmp(update.run_state, "WCTR")) return 0u;

        feed_center(250u, 550u, 251u, 550u, 249u, 550u);
        art_replan_tick(&context, 1u, &update);
        publish_stable_map(&context, &update);
        if(0 != strcmp(update.run_state, "ICtr")) return 0u;

        feed_center(260u, 550u, 259u, 550u, 261u, 550u);
        art_replan_tick(&context, 1u, &update);
        if((0u == update.subject2_map_ready) ||
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
    art_replan_tick(&context, 1u, &update);
    if((0u != fake_pre_push_pending) ||
       (1u != start_pre_push_alignment_count) ||
       (0u != continue_pre_push_count)) return 0u;

    art_replan_cancel();
    fake_pre_push_pending = 1u;
    fake_executor_state = EXEC_STATE_RUNNING;
    fake_time_ms += 100u;
    art_replan_tick(&context, 1u, &update);
    if((0 != strcmp(update.run_state, "PCtr")) ||
       (0u != continue_pre_push_count)) return 0u;

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    if((0u != fake_pre_push_pending) ||
       (1u != continue_pre_push_count) ||
       (1u != start_pre_push_alignment_count) ||
       (0u != executor_error_count) ||
       (0 != strcmp(update.run_state, "Running"))) return 0u;
#else
    if((1u != executor_error_count) ||
       (0 != strcmp(update.run_state, "E:Ctr"))) return 0u;
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
    uint8 tick;
#endif

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
    if((request_before + 1u != center_request_count) ||
       (0 != strcmp(update.run_state, "WCTR"))) return 0u;

    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
#if ART_CENTER_TIMEOUT_FALLBACK_ENABLE
    if(0 != strcmp(update.run_state, "LCH")) return 0u;
    fake_pose.x_cm = ART_LAUNCH_FALLBACK_MOVE_CM;
    for(tick = 0u; tick < EXEC_ARRIVAL_STABLE_TICKS; tick++)
    {
        art_replan_tick(&context, 1u, &update);
    }
    if(0 != strcmp(update.run_state, "WMAP")) return 0u;

    publish_stable_map(&context, &update);
    if((request_before + 2u != center_request_count) ||
       (0 != strcmp(update.run_state, "ICtr"))) return 0u;
    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    if((solve_before + 1u != solve_count) ||
       (start_before + 1u != executor_start_count) ||
       (fabsf(executor_initial_x_cm) > 0.01f) ||
       (fabsf(executor_initial_y_cm) > 0.01f)) return 0u;

    fake_sync_pending = 1u;
    fake_sync_action = 'R';
    art_replan_tick(&context, 1u, &update);
    publish_stable_map(&context, &update);
    if((request_before + 3u != center_request_count) ||
       (0 != strcmp(update.run_state, "RCtr"))) return 0u;
    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    if((solve_before + 2u != solve_count) ||
       (start_before + 2u != executor_start_count)) return 0u;

    live_rows[5][5] = '.';
    live_rows[5][6] = '.';
    fake_sync_pending = 1u;
    fake_sync_action = 'R';
    art_replan_tick(&context, 1u, &update);
    publish_stable_map(&context, &update);
    if((request_before + 4u != center_request_count) ||
       (0 != strcmp(update.run_state, "RCtr"))) return 0u;
    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    fake_executor_state = EXEC_STATE_DONE;
    art_replan_tick(&context, 1u, &update);
    if((request_before + 5u != center_request_count) ||
       (0 != strcmp(update.run_state, "RetCtr"))) return 0u;

    live_rows[5][2] = '.';
    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    fake_pose.x_cm = -ART_LAUNCH_FALLBACK_MOVE_CM;
    for(tick = 0u; tick < EXEC_ARRIVAL_STABLE_TICKS; tick++)
    {
        art_replan_tick(&context, 1u, &update);
    }
    if((request_before + 6u != center_request_count) ||
       (0 != strcmp(update.run_state, "RetChk"))) return 0u;
    fake_time_ms += EXEC_ART_SYNC_TIMEOUT_MS;
    art_replan_tick(&context, 1u, &update);
    return ((EXEC_STATE_DONE == fake_executor_state) &&
            (0u != update.return_complete) &&
            (error_before == executor_error_count)) ? 1u : 0u;
#else
    return ((error_before + 1u == executor_error_count) &&
            (0 == strcmp(update.run_state, "E:LCtr"))) ? 1u : 0u;
#endif
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
    if((EXEC_STATE_IDLE != fake_executor_state) ||
       (0 != strcmp(update.run_state, "Host Sync"))) return 0u;

    publish_stable_map(&context, &update);
    if(0 != strcmp(update.run_state, "RCtr")) return 0u;
    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    art_replan_tick(&context, 1u, &update);
    art_replan_get_debug_status(&status);

    return ((2u == executor_start_count) &&
            (EXEC_STATE_RUNNING == fake_executor_state) &&
            (1u == status.confirmed_box_count) &&
            (1u == status.confirmed_target_count)) ? 1u : 0u;
}

int main(void)
{
    uint8 passed = run_test();
    printf("request-driven-art-state      %s\n", (0 != passed) ? "PASS" : "FAIL");
    passed &= run_timeout_fallback_test();
    printf("request-center-timeout-policy %s\n", (0 != passed) ? "PASS" : "FAIL");
    passed &= host_completion_before_task_end_replans();
    printf("host-completion-replan        %s\n", (0 != passed) ? "PASS" : "FAIL");
    passed &= pre_push_box_observation_flow();
    printf("pre-push-box-observation      %s\n", (0 != passed) ? "PASS" : "FAIL");
    passed &= pre_push_box_prefetch_freshness();
    printf("pre-push-box-prefetch         %s\n", (0 != passed) ? "PASS" : "FAIL");
    passed &= pre_push_box_failures_follow_policy();
    printf("pre-push-box-failures         %s\n", (0 != passed) ? "PASS" : "FAIL");
    return (0 != passed) ? 0 : 1;
}
