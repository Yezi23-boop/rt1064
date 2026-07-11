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
static uint16 solve_count;
static uint16 executor_start_count;
static uint16 executor_error_count;
static float executor_initial_x_cm;
static float executor_initial_y_cm;
static uint8 fake_sync_pending;
static char fake_sync_action;
static executor_state_enum fake_executor_state;
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
    (void)error;
    executor_error_count++;
    fake_executor_state = EXEC_STATE_ERROR;
}
executor_state_enum executor_get_state(void) { return fake_executor_state; }
uint8 executor_art_pre_push_pending(void) { return 0; }
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
uint8 executor_continue_after_pre_push_center(void) { return 1; }

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
    center_col_q[0] = col0; center_row_q[0] = row0;
    center_col_q[1] = col1; center_row_q[1] = row1;
    center_col_q[2] = col2; center_row_q[2] = row2;
    center_count = 3;
    center_read = 0;
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

    fake_time_ms += 60000u;
    art_replan_tick(&context, 1u, &update);
    if((1u != center_request_count) || (0u != executor_error_count)) return 0;

    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    art_replan_tick(&context, 1u, &update);
    publish_stable_map(&context, &update);
    if((2u != center_request_count) || (0 != strcmp(update.run_state, "ICtr"))) return 0;

    solve_before = solve_count;
    fake_time_ms += 60000u;
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
    fake_time_ms += 60000u;
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

    fake_time_ms += 60000u;
    art_replan_tick(&context, 1u, &update);
    if(0u != executor_error_count) return 0;

    feed_center(250u, 550u, 251u, 550u, 249u, 550u);
    art_replan_tick(&context, 1u, &update);
    if((6u != center_request_count) || (0 != strcmp(update.run_state, "RetChk"))) return 0;

    fake_time_ms += 60000u;
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
    return 1u;
}

int main(void)
{
    uint8 passed = run_test();
    printf("request-driven-art-state      %s\n", (0 != passed) ? "PASS" : "FAIL");
    return (0 != passed) ? 0 : 1;
}
