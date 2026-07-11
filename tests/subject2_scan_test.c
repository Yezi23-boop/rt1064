#include <stdio.h>
#include <string.h>
#include "drive_pose.h"
#include "executor.h"
#include "map_utils.h"
#include "openart_uart.h"
#include "subject2.h"
#include "vision_uart.h"

static uint32 fake_time_ms;
static executor_state_enum fake_executor_state;
static drive_pose_struct fake_pose;
static uint8 fake_ready_box;
static uint8 fake_ready_target;
static vision_mode_enum last_mode;
static uint16 vision_request_id;
static vision_sample_struct vision_samples[8];
static uint8 vision_sample_count;
static uint8 vision_sample_read;
static uint16 vision_ack_count;
static uint16 center_col_q[ART_CENTER_SAMPLE_COUNT];
static uint16 center_row_q[ART_CENTER_SAMPLE_COUNT];
static uint8 center_count;
static uint8 center_read;
static uint16 center_request_count;
static uint16 pose_reset_count;
static uint16 executor_start_count;
static uint8 executor_start_art_sync;
static uint8 executor_target_row;
static uint8 executor_target_col;
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
    pose_reset_count++;
    fake_pose.x_cm = x;
    fake_pose.y_cm = y;
    fake_pose.yaw_deg = yaw;
}
void stop_motion(void) { }

void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 row, uint8 col, float x, float y,
                    uint8 single_step, uint8 art_sync)
{
    (void)row; (void)col; (void)x; (void)y; (void)single_step;
    executor_start_count++;
    executor_start_art_sync = art_sync;
    fake_executor_state = EXEC_STATE_RUNNING;
    if(0u != count)
    {
        executor_target_row = waypoints[count - 1u].row;
        executor_target_col = waypoints[count - 1u].col;
    }
}
void executor_stop(void) { fake_executor_state = EXEC_STATE_IDLE; }
executor_state_enum executor_get_state(void) { return fake_executor_state; }
void executor_set_error(executor_error_enum error)
{
    (void)error;
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
    return EXEC_ART_CENTER_APPLIED;
}
uint8 executor_art_pre_push_pending(void) { return 0u; }
uint8 executor_art_sync_pending(void) { return 0u; }
uint8 executor_continue_after_pre_push_center(void) { return 0u; }

void openart_request_player_center(void)
{
    center_request_count++;
    center_count = 0u;
    center_read = 0u;
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
const map_source_struct *openart_map_get(void) { return &live_source; }
uint32 openart_uart_get_frame_count(void) { return 0u; }

void vision_uart_set_mode(vision_mode_enum mode) { last_mode = mode; }
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
    memset(&fake_pose, 0, sizeof(fake_pose));
    fake_ready_box = 0u;
    fake_ready_target = 0u;
    last_mode = VISION_MODE_NONE;
    vision_request_id = 0u;
    vision_sample_count = 0u;
    vision_sample_read = 0u;
    vision_ack_count = 0u;
    center_count = 0u;
    center_read = 0u;
    center_request_count = 0u;
    pose_reset_count = 0u;
    executor_start_count = 0u;
    executor_start_art_sync = 0u;
    for(row = 0u; row < MAP_ROWS; row++)
    {
        for(col = 0u; col < MAP_COLS; col++)
        {
            live_rows[row][col] = ((0u == row) || ((MAP_ROWS - 1u) == row) ||
                                   (0u == col) || ((MAP_COLS - 1u) == col)) ? '#' : '.';
        }
        live_rows[row][MAP_COLS] = '\0';
        live_source.rows[row] = live_rows[row];
        snapshot.rows[row] = snapshot_rows[row];
    }
    live_source.name = "subject2-live";
    snapshot.name = "subject2-snapshot";
    live_rows[5][5] = 'C';
    live_rows[5][6] = 'B';
    live_rows[7][9] = 'T';
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

static uint8 run_scan(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    init_context(&context);

    subject2_begin(&context, 0.0f, 0.0f, &update);
    if((SUBJECT2_SCAN_BOX_MODE != subject2_get_state()) ||
       (VISION_MODE_BOX != last_mode)) return 0u;

    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_CENTER != subject2_get_state()) ||
       (1u != center_request_count)) return 0u;

    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_CLASSIFY != subject2_get_state()) ||
       (1u != pose_reset_count) || (1u != vision_request_id)) return 0u;

    feed_class(4u);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_TARGET_MODE != subject2_get_state()) ||
       (VISION_MODE_TARGET != last_mode) || (1u != vision_ack_count)) return 0u;

    fake_ready_target = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_TARGET_MOVE != subject2_get_state()) ||
       (1u != executor_start_count) || (0u != executor_start_art_sync)) return 0u;

    move_live_car(executor_target_row, executor_target_col);
    fake_executor_state = EXEC_STATE_DONE;
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_TARGET_CENTER != subject2_get_state()) ||
       (2u != center_request_count)) return 0u;

    feed_center((uint16)(executor_target_col * 100u + 50u),
                (uint16)(executor_target_row * 100u + 50u));
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_TARGET_CLASSIFY != subject2_get_state()) ||
       (2u != pose_reset_count) || (2u != vision_request_id)) return 0u;

    feed_class(4u);
    subject2_tick(&context, &update);
    if(SUBJECT2_VALIDATE_BINDINGS != subject2_get_state()) return 0u;
    subject2_tick(&context, &update);

    return ((SUBJECT2_SELECT_PUSH == subject2_get_state()) &&
            (2u == vision_ack_count)) ? 1u : 0u;
}

static uint8 classification_timeout_retries(void)
{
    subject2_context_struct context;
    subject2_update_struct update;

    build_map();
    init_context(&context);
    subject2_begin(&context, 0.0f, 0.0f, &update);
    fake_ready_box = 1u;
    subject2_tick(&context, &update);
    subject2_tick(&context, &update);
    feed_center(550u, 550u);
    subject2_tick(&context, &update);
    if(SUBJECT2_SCAN_BOX_CLASSIFY != subject2_get_state()) return 0u;

    fake_time_ms = SUBJECT2_VIEW_TIMEOUT_MS;
    subject2_tick(&context, &update);
    if((SUBJECT2_SCAN_BOX_PLAN != subject2_get_state()) ||
       (0 != strcmp(update.run_state, "VRetry"))) return 0u;
    subject2_tick(&context, &update);
    return (SUBJECT2_ERROR != subject2_get_state()) ? 1u : 0u;
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

int main(void)
{
    uint8 passed = 1u;

    build_map();
    passed &= run_scan();
    printf("subject2-scan-state         %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= classification_timeout_retries();
    printf("subject2-scan-timeout       %s\n", (0u != passed) ? "PASS" : "FAIL");
    passed &= map_change_stops_scan();
    printf("subject2-scan-map-change    %s\n", (0u != passed) ? "PASS" : "FAIL");
    return (0u != passed) ? 0 : 1;
}
