#include <math.h>
#include <stdio.h>
#include <string.h>
#include "executor.h"
#include "drive_control.h"
#include "drive_pose.h"

static drive_pose_struct test_pose;
static uint16 motion_call_count;
static float last_motion_vx;
static float last_motion_vy;
static uint16 reset_call_count;
static uint16 stop_call_count;
static uint16 critical_disable_count;
static uint16 critical_enable_count;
static uint8 critical_depth;
static uint8 observe_pose_reset;
static uint8 pose_reset_inside_critical;

uint32 interrupt_global_disable(void)
{
    critical_disable_count++;
    critical_depth++;
    return 0u;
}

void interrupt_global_enable(uint32 primask)
{
    (void)primask;
    critical_enable_count++;
    if(0u < critical_depth)
    {
        critical_depth--;
    }
}

const drive_pose_struct *drive_pose_get(void)
{
    return &test_pose;
}

void drive_pose_reset(float x_cm, float y_cm, float yaw_deg)
{
    if(0u != observe_pose_reset)
    {
        pose_reset_inside_critical = (0u != critical_depth) ? 1u : 0u;
    }
    test_pose.x_cm = x_cm;
    test_pose.y_cm = y_cm;
    test_pose.yaw_deg = yaw_deg;
    test_pose.body_vx_cm = 0.0f;
    test_pose.body_vy_cm = 0.0f;
}

void set_motion(float vx, float vy)
{
    last_motion_vx = vx;
    last_motion_vy = vy;
    motion_call_count++;
}

void reset_motion_segment(void)
{
    reset_call_count++;
}

void stop_motion(void)
{
    stop_call_count++;
}

static void reset_fixture(void)
{
    executor_stop();
    memset(&test_pose, 0, sizeof(test_pose));
    motion_call_count = 0;
    last_motion_vx = 0.0f;
    last_motion_vy = 0.0f;
    reset_call_count = 0;
    stop_call_count = 0;
    critical_disable_count = 0;
    critical_enable_count = 0;
    critical_depth = 0;
    observe_pose_reset = 0;
    pose_reset_inside_critical = 0;
    executor_init();
}

static uint8 run_case(const char *name, uint8 passed)
{
    printf("%-28s %s\n", name, (0 != passed) ? "PASS" : "FAIL");
    return passed;
}

static uint8 first_push_waits_before_motion(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 0u, 1u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();

    return ((0 != executor_art_pre_push_pending()) &&
            (0u == motion_call_count) &&
            (0u == executor_get_current_step())) ? 1u : 0u;
}

static uint8 start_switch_is_atomic(void)
{
    waypoint_struct waypoint = {5u, 6u, 'r', 0u, 1u, 0u, 0u};

    reset_fixture();
    observe_pose_reset = 1u;
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 0u);

    return ((1u == critical_disable_count) &&
            (1u == critical_enable_count) &&
            (0u == critical_depth) &&
            (0u != pose_reset_inside_critical)) ? 1u : 0u;
}

static uint8 error_switch_is_atomic(void)
{
    waypoint_struct waypoint = {5u, 6u, 'r', 0u, 1u, 0u, 0u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 0u);
    critical_disable_count = 0;
    critical_enable_count = 0;
    critical_depth = 0;

    executor_set_error(EXEC_ERROR_ART_CENTER);

    return ((1u == critical_disable_count) &&
            (1u == critical_enable_count) &&
            (0u == critical_depth) &&
            (EXEC_STATE_ERROR == executor_get_state())) ? 1u : 0u;
}

static uint8 successful_correction_releases_same_push(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 0u, 1u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    if(0 == executor_continue_after_pre_push_center())
    {
        return 0;
    }
    executor_update_20ms();

    return ((0 == executor_art_pre_push_pending()) &&
            (1u == motion_call_count) &&
            (0u == executor_get_current_step())) ? 1u : 0u;
}

static uint8 lowercase_turn_waits_without_push_alignment(void)
{
    waypoint_struct waypoint = {6u, 5u, 'd', 0u, 2u, 0u, 1u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    if((0 == executor_art_pre_push_pending()) ||
       (0 != executor_center_requires_push_alignment()))
    {
        return 0u;
    }
    if(0 == executor_continue_after_pre_push_center())
    {
        return 0u;
    }
    executor_update_20ms();

    return ((0 == executor_art_pre_push_pending()) &&
            (1u == motion_call_count) &&
            (last_motion_vy < 0.0f)) ? 1u : 0u;
}

static uint8 uppercase_center_requires_push_alignment(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 0u, 1u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();

    return executor_center_requires_push_alignment();
}

static uint8 horizontal_push_aligns_only_y(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 0u, 1u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 3.0f, 0u, 1u);
    executor_update_20ms();
    if(0 == executor_start_pre_push_alignment(5u, 5u))
    {
        return 0u;
    }
    executor_update_20ms();

    return ((1u == motion_call_count) &&
            (fabsf(last_motion_vx) < 0.0001f) &&
            (last_motion_vy < 0.0f) &&
            (0u == executor_get_current_step())) ? 1u : 0u;
}

static uint8 vertical_push_aligns_only_x(void)
{
    waypoint_struct waypoint = {4u, 5u, 'U', 0u, 1u, 0u, 1u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 3.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    if(0 == executor_start_pre_push_alignment(5u, 5u))
    {
        return 0u;
    }
    executor_update_20ms();

    return ((1u == motion_call_count) &&
            (last_motion_vx < 0.0f) &&
            (fabsf(last_motion_vy) < 0.0001f) &&
            (0u == executor_get_current_step())) ? 1u : 0u;
}

static uint8 aligned_push_resumes_same_waypoint(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 0u, 1u};
    uint8 tick;

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.2f, 0u, 1u);
    executor_update_20ms();
    if(0 == executor_start_pre_push_alignment(5u, 5u))
    {
        return 0u;
    }
    for(tick = 0u; tick < EXEC_ARRIVAL_STABLE_TICKS; tick++)
    {
        executor_update_20ms();
    }
    if(0u != motion_call_count)
    {
        return 0u;
    }
    executor_update_20ms();

    return ((1u == motion_call_count) &&
            (last_motion_vx > 0.0f) &&
            (fabsf(last_motion_vy) < 0.0001f) &&
            (0u == executor_get_current_step())) ? 1u : 0u;
}

static uint8 alignment_timeout_stops_with_center_error(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 0u, 1u};
    uint16 tick;
    uint16 timeout_ticks = (uint16)(EXEC_ART_SYNC_TIMEOUT_MS / CONTROL_PERIOD_MS + 1u);

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 3.0f, 0u, 1u);
    executor_update_20ms();
    if(0 == executor_start_pre_push_alignment(5u, 5u))
    {
        return 0u;
    }
    for(tick = 0u; tick < timeout_ticks; tick++)
    {
        executor_update_20ms();
    }

    return ((EXEC_STATE_ERROR == executor_get_state()) &&
            (EXEC_ERROR_ART_CENTER == executor_get_error()) &&
            (0u == executor_get_current_step())) ? 1u : 0u;
}

static uint8 alignment_jitter_still_times_out(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 0u, 1u};
    uint16 tick;
    uint16 timeout_ticks = (uint16)(EXEC_ART_SYNC_TIMEOUT_MS / CONTROL_PERIOD_MS + 1u);

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 3.0f, 0u, 1u);
    executor_update_20ms();
    if(0 == executor_start_pre_push_alignment(5u, 5u))
    {
        return 0u;
    }
    for(tick = 0u; tick < timeout_ticks; tick++)
    {
        test_pose.y_cm = (0u == (tick & 1u)) ? 0.2f : 3.0f;
        executor_update_20ms();
    }

    return ((EXEC_STATE_ERROR == executor_get_state()) &&
            (EXEC_ERROR_ART_CENTER == executor_get_error())) ? 1u : 0u;
}

static uint8 push_waypoints_do_not_wait(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 1u, 0u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();

    return ((0u == executor_get_current_step()) &&
            (0 == executor_art_pre_push_pending()) &&
            (1u == motion_call_count)) ? 1u : 0u;
}

static uint8 lowercase_and_offline_push_do_not_wait(void)
{
    waypoint_struct lowercase = {5u, 6u, 'r', 0u, 1u, 0u, 0u};
    waypoint_struct offline_marked = {5u, 6u, 'R', 0u, 1u, 0u, 1u};
    uint8 lowercase_ok;

    reset_fixture();
    executor_start(&lowercase, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    lowercase_ok = ((0 == executor_art_pre_push_pending()) &&
                    (1u == motion_call_count)) ? 1u : 0u;

    reset_fixture();
    executor_start(&offline_marked, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 0u);
    executor_update_20ms();

    return ((0 != lowercase_ok) &&
            (0 == executor_art_pre_push_pending()) &&
            (1u == motion_call_count)) ? 1u : 0u;
}

static uint8 center_error_stops_on_same_waypoint(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 0u, 1u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    executor_set_error(EXEC_ERROR_ART_CENTER);

    return ((EXEC_STATE_ERROR == executor_get_state()) &&
            (EXEC_ERROR_ART_CENTER == executor_get_error()) &&
            (0u == executor_get_current_step()) &&
            (0u != stop_call_count)) ? 1u : 0u;
}

static uint8 art_center_uses_configured_fusion(void)
{
    waypoint_struct waypoint = {5u, 6u, 'r', 0u, 1u, 0u, 0u};
    executor_art_center_result_enum result;

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    (void)executor_apply_art_player_center(568u, 550u, 1u);
    (void)executor_apply_art_player_center(570u, 550u, 2u);
    (void)executor_apply_art_player_center(572u, 550u, 3u);
    (void)executor_apply_art_player_center(570u, 550u, 4u);
    (void)executor_apply_art_player_center(570u, 550u, 5u);
    result = executor_commit_art_player_center(5u, 5u);

    return ((EXEC_ART_CENTER_APPLIED == result) &&
            (fabsf(test_pose.x_cm - (4.0f * EXEC_ART_CENTER_FUSE_ALPHA)) < 0.01f) &&
            (fabsf(test_pose.y_cm) < 0.01f)) ? 1u : 0u;
}

static uint8 art_center_samples_can_be_reset(void)
{
    waypoint_struct waypoint = {5u, 6u, 'r', 0u, 1u, 0u, 0u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    (void)executor_apply_art_player_center(568u, 550u, 1u);
    (void)executor_apply_art_player_center(570u, 550u, 2u);
    executor_reset_art_player_center_samples();

    return (0u == executor_apply_art_player_center(572u, 550u, 3u)) ? 1u : 0u;
}

static uint8 collect_box_observation(uint16 car_col_q, uint16 car_row_q,
                                     uint16 box_col_q, uint16 box_row_q)
{
    executor_reset_art_box_observation_samples();
    if(0u != executor_apply_art_box_observation(
            car_col_q, car_row_q, box_col_q, box_row_q))
    {
        return 0u;
    }
    if(0u != executor_apply_art_box_observation(
            car_col_q, car_row_q, box_col_q, box_row_q))
    {
        return 0u;
    }
    return executor_apply_art_box_observation(
        car_col_q, car_row_q, box_col_q, box_row_q);
}

static uint8 final_approach_requests_next_box(void)
{
    waypoint_struct waypoints[2] = {
        {5u, 5u, 'd', 0u, 1u, 0u, 1u},
        {5u, 6u, 'R', 1u, 2u, 1u, 0u}
    };
    uint8 box_row = 0u;
    uint8 box_col = 0u;

    reset_fixture();
    executor_start(waypoints, 2u, 4u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();

    return ((0u != executor_art_pre_push_pending()) &&
            (0u != executor_get_pre_push_box_request(&box_row, &box_col)) &&
            (5u == box_row) && (6u == box_col)) ? 1u : 0u;
}

static uint8 previous_waypoint_prefetches_next_box(void)
{
    waypoint_struct waypoints[3] = {
        {6u, 5u, 'u', 0u, 1u, 0u, 0u},
        {5u, 5u, 'u', 1u, 2u, 0u, 1u},
        {5u, 6u, 'R', 2u, 3u, 1u, 0u}
    };
    uint8 box_row = 0u;
    uint8 box_col = 0u;

    reset_fixture();
    executor_start(waypoints, 3u, 7u, 5u, 0.0f, 0.0f, 0u, 1u);

    return ((0u != executor_get_pre_push_box_prefetch_request(&box_row, &box_col)) &&
            (5u == box_row) && (6u == box_col)) ? 1u : 0u;
}

static uint8 retry_nudge_direction(char action, uint8 box_row, uint8 box_col,
                                   float expected_sign, uint8 x_axis)
{
    waypoint_struct waypoint = {box_row, box_col, action, 0u, 1u, 1u, 1u};
    uint8 tick;

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    if((0u == executor_start_pre_push_box_retry_nudge(5.0f)) ||
       (0 != strcmp(executor_pre_push_box_state_name(), "BRetry")))
    {
        return 0u;
    }
    executor_update_20ms();
    if(0u != x_axis)
    {
        if(((last_motion_vx * expected_sign) <= 0.0f) ||
           (fabsf(last_motion_vy) > 0.0001f)) return 0u;
        test_pose.x_cm = expected_sign * 5.0f;
    }
    else
    {
        if(((last_motion_vy * expected_sign) <= 0.0f) ||
           (fabsf(last_motion_vx) > 0.0001f)) return 0u;
        test_pose.y_cm = expected_sign * 5.0f;
    }
    for(tick = 0u; tick < EXEC_ARRIVAL_STABLE_TICKS; tick++)
    {
        executor_update_20ms();
    }
    return ((0u == executor_pre_push_box_preparation_active()) &&
            (0u != executor_art_pre_push_pending()) &&
            (0u == executor_get_current_step())) ? 1u : 0u;
}

static uint8 retry_nudge_moves_away_in_all_directions(void)
{
    return ((0u != retry_nudge_direction('R', 5u, 6u, -1.0f, 1u)) &&
            (0u != retry_nudge_direction('L', 5u, 4u, 1.0f, 1u)) &&
            (0u != retry_nudge_direction('U', 4u, 5u, -1.0f, 0u)) &&
            (0u != retry_nudge_direction('D', 6u, 5u, 1.0f, 0u))) ? 1u : 0u;
}

static uint8 box_preparation_aligns_and_consumes_approach(void)
{
    waypoint_struct waypoints[2] = {
        {5u, 5u, 'd', 0u, 1u, 0u, 1u},
        {5u, 6u, 'R', 1u, 2u, 1u, 0u}
    };
    uint8 tick;

    reset_fixture();
    executor_start(waypoints, 2u, 4u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    if((0u == collect_box_observation(550u, 450u, 650u, 550u)) ||
       (EXEC_ART_BOX_PREP_STARTED != executor_start_pre_push_box_preparation()))
    {
        return 0u;
    }

    executor_update_20ms();
    if((fabsf(last_motion_vx) > 0.0001f) || (last_motion_vy >= 0.0f) ||
       (0 != strcmp(executor_pre_push_box_state_name(), "BAlign")))
    {
        return 0u;
    }

    test_pose.y_cm = -20.0f;
    for(tick = 0u; tick < (EXEC_ARRIVAL_STABLE_TICKS * 2u); tick++)
    {
        executor_update_20ms();
    }
    if((1u != executor_get_current_step()) ||
       (0u != executor_pre_push_box_preparation_active()))
    {
        return 0u;
    }

    executor_update_20ms();
    return ((last_motion_vx > 0.0f) &&
            (fabsf(last_motion_vy) < 0.0001f)) ? 1u : 0u;
}

static uint8 short_gap_retreats_before_alignment(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 1u, 1u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 5.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    if((0u == collect_box_observation(575u, 550u, 650u, 550u)) ||
       (EXEC_ART_BOX_PREP_STARTED != executor_start_pre_push_box_preparation()))
    {
        return 0u;
    }
    executor_update_20ms();

    return ((last_motion_vx < 0.0f) &&
            (fabsf(last_motion_vy) < 0.0001f) &&
            (0 == strcmp(executor_pre_push_box_state_name(), "BGap"))) ? 1u : 0u;
}

static uint8 wrong_side_box_is_rejected(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 1u, 1u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 22.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    if(0u == collect_box_observation(660u, 550u, 650u, 550u))
    {
        return 0u;
    }

    return ((EXEC_ART_BOX_PREP_GEOMETRY_ERROR ==
             executor_start_pre_push_box_preparation()) &&
            (0u == executor_get_current_step()) &&
            (0u == executor_pre_push_box_preparation_active())) ? 1u : 0u;
}

static uint8 adjacent_box_cell_is_rejected(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 1u, 1u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    if(0u == collect_box_observation(550u, 550u, 750u, 550u))
    {
        return 0u;
    }

    return (EXEC_ART_BOX_PREP_GEOMETRY_ERROR ==
            executor_start_pre_push_box_preparation()) ? 1u : 0u;
}

static uint8 box_center_crossing_cell_boundary_is_accepted(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 1u, 1u};

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    if(0u == collect_box_observation(550u, 550u, 705u, 550u))
    {
        return 0u;
    }

    return (EXEC_ART_BOX_PREP_STARTED ==
            executor_start_pre_push_box_preparation()) ? 1u : 0u;
}

static uint8 box_preparation_approach_direction(char action,
                                                uint8 box_row, uint8 box_col,
                                                uint16 car_col_q, uint16 car_row_q,
                                                uint16 box_col_q, uint16 box_row_q,
                                                float expected_sign,
                                                uint8 x_axis)
{
    waypoint_struct waypoint = {box_row, box_col, action, 0u, 1u, 1u, 1u};
    uint8 tick;

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    if((0u == collect_box_observation(car_col_q, car_row_q,
                                      box_col_q, box_row_q)) ||
       (EXEC_ART_BOX_PREP_STARTED != executor_start_pre_push_box_preparation()))
    {
        return 0u;
    }

    if(0u != x_axis)
    {
        test_pose.y_cm = 0.0f;
    }
    else
    {
        test_pose.x_cm = 0.0f;
    }
    for(tick = 0u; tick < EXEC_ARRIVAL_STABLE_TICKS; tick++)
    {
        executor_update_20ms();
    }
    if(0 != strcmp(executor_pre_push_box_state_name(), "BNear"))
    {
        return 0u;
    }
    executor_update_20ms();

    if(0u != x_axis)
    {
        return (((last_motion_vx * expected_sign) > 0.0f) &&
                (fabsf(last_motion_vy) < 0.0001f)) ? 1u : 0u;
    }
    return (((last_motion_vy * expected_sign) > 0.0f) &&
            (fabsf(last_motion_vx) < 0.0001f)) ? 1u : 0u;
}

static uint8 box_preparation_targets_all_directions(void)
{
    return ((0u != box_preparation_approach_direction(
                        'R', 5u, 6u, 550u, 540u, 670u, 550u, 1.0f, 1u)) &&
            (0u != box_preparation_approach_direction(
                        'L', 5u, 4u, 550u, 540u, 430u, 550u, -1.0f, 1u)) &&
            (0u != box_preparation_approach_direction(
                        'U', 4u, 5u, 540u, 550u, 550u, 430u, 1.0f, 0u)) &&
            (0u != box_preparation_approach_direction(
                        'D', 6u, 5u, 540u, 550u, 550u, 670u, -1.0f, 0u))) ? 1u : 0u;
}

static uint8 step_box_preparation_pauses_before_push(void)
{
    waypoint_struct waypoints[2] = {
        {5u, 5u, 'd', 0u, 1u, 0u, 1u},
        {5u, 6u, 'R', 1u, 2u, 1u, 0u}
    };
    uint8 tick;

    reset_fixture();
    executor_start(waypoints, 2u, 4u, 5u, 0.0f, 0.0f, 1u, 1u);
    executor_resume();
    executor_update_20ms();
    if((0u == collect_box_observation(550u, 450u, 650u, 550u)) ||
       (EXEC_ART_BOX_PREP_STARTED != executor_start_pre_push_box_preparation()))
    {
        return 0u;
    }
    test_pose.y_cm = -20.0f;
    for(tick = 0u; tick < (EXEC_ARRIVAL_STABLE_TICKS * 2u); tick++)
    {
        executor_update_20ms();
    }

    return ((EXEC_STATE_PAUSED == executor_get_state()) &&
            (1u == executor_get_current_step())) ? 1u : 0u;
}

static uint8 run_push_chain_switches_without_stop(void)
{
    waypoint_struct waypoints[2] = {
        {5u, 6u, 'R', 0u, 1u, 0u, 1u},
        {5u, 7u, 'R', 1u, 2u, 1u, 0u}
    };
    uint16 reset_before;
    uint16 motion_before;

    reset_fixture();
    executor_start(waypoints, 2u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    if((0 == executor_art_pre_push_pending()) ||
       (0 == executor_continue_after_pre_push_center()))
    {
        return 0;
    }
    executor_update_20ms();

    drive_pose_reset(20.0f, 0.0f, 0.0f);
    reset_before = reset_call_count;
    motion_before = motion_call_count;
    executor_update_20ms();
    if((0u != executor_get_current_step()) ||
       (reset_before != reset_call_count) ||
       (motion_call_count <= motion_before))
    {
        return 0;
    }

    return ((0u == executor_get_current_step()) &&
            (reset_before == reset_call_count) &&
            (motion_call_count > motion_before) &&
            (0 == executor_art_pre_push_pending())) ? 1u : 0u;
}

static uint8 run_move_into_push_switches_without_stop(void)
{
    waypoint_struct waypoints[2] = {
        {5u, 6u, 'r', 0u, 1u, 0u, 0u},
        {5u, 7u, 'R', 1u, 2u, 1u, 0u}
    };
    uint16 reset_before;
    uint16 motion_before;

    reset_fixture();
    executor_start(waypoints, 2u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    drive_pose_reset(20.0f, 0.0f, 0.0f);
    reset_before = reset_call_count;
    motion_before = motion_call_count;
    executor_update_20ms();

    return ((0u == executor_get_current_step()) &&
            (reset_before == reset_call_count) &&
            (motion_call_count > motion_before) &&
            (0 == executor_art_pre_push_pending())) ? 1u : 0u;
}

static uint8 run_push_chain_targets_straight_end(void)
{
    waypoint_struct waypoints[3] = {
        {5u, 6u, 'r', 0u, 1u, 0u, 0u},
        {5u, 7u, 'R', 1u, 2u, 0u, 0u},
        {5u, 8u, 'R', 2u, 3u, 1u, 0u}
    };

    reset_fixture();
    executor_start(waypoints, 3u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    drive_pose_reset(30.0f, 0.0f, 0.0f);
    executor_update_20ms();

    return ((0u == executor_get_current_step()) &&
            (0u != motion_call_count) &&
            (last_motion_vx > 0.0f) &&
            (fabsf(last_motion_vy) < 0.0001f)) ? 1u : 0u;
}

static uint8 lookahead_stops_at_task_end(void)
{
    waypoint_struct waypoints[2] = {
        {5u, 6u, 'R', 0u, 1u, 1u, 0u},
        {5u, 7u, 'R', 1u, 2u, 1u, 0u}
    };

    reset_fixture();
    executor_start(waypoints, 2u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    drive_pose_reset(30.0f, 0.0f, 0.0f);
    executor_update_20ms();

    return ((0u == executor_get_current_step()) &&
            (0u != motion_call_count) &&
            (last_motion_vx < 0.0f)) ? 1u : 0u;
}

static uint8 push_turn_stops_before_next_waypoint(void)
{
    waypoint_struct waypoints[2] = {
        {5u, 6u, 'R', 0u, 1u, 0u, 0u},
        {4u, 6u, 'U', 1u, 2u, 0u, 1u}
    };
    uint16 tick;
    uint16 settle_ticks = (uint16)(EXEC_SEGMENT_SETTLE_MS / CONTROL_PERIOD_MS + 2u);

    reset_fixture();
    executor_start(waypoints, 2u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    drive_pose_reset(20.0f, 0.0f, 0.0f);
    executor_update_20ms();
    if(0u != executor_get_current_step())
    {
        return 0u;
    }

    for(tick = 1u; tick < EXEC_ARRIVAL_STABLE_TICKS; tick++)
    {
        executor_update_20ms();
    }
    for(tick = 0u; tick < settle_ticks; tick++)
    {
        executor_update_20ms();
    }
    executor_update_20ms();

    return ((1u == executor_get_current_step()) &&
            (0 != executor_art_pre_push_pending())) ? 1u : 0u;
}

static uint8 step_mode_still_pauses_after_first_push(void)
{
    waypoint_struct waypoints[2] = {
        {5u, 6u, 'R', 0u, 1u, 0u, 1u},
        {5u, 7u, 'R', 1u, 2u, 1u, 0u}
    };
    uint16 tick;
    uint16 max_ticks = (uint16)(EXEC_ARRIVAL_STABLE_TICKS +
                                (EXEC_SEGMENT_SETTLE_MS / CONTROL_PERIOD_MS) + 3u);

    reset_fixture();
    executor_start(waypoints, 2u, 5u, 5u, 0.0f, 0.0f, 1u, 1u);
    executor_resume();
    executor_update_20ms();
    if((0 == executor_art_pre_push_pending()) ||
       (0 == executor_continue_after_pre_push_center()))
    {
        return 0;
    }

    drive_pose_reset(20.0f, 0.0f, 0.0f);
    for(tick = 0; tick < max_ticks; tick++)
    {
        executor_update_20ms();
        if(EXEC_STATE_PAUSED == executor_get_state())
        {
            break;
        }
    }
    return ((EXEC_STATE_PAUSED == executor_get_state()) &&
            (1u == executor_get_current_step())) ? 1u : 0u;
}

static uint8 final_push_still_waits_for_art(void)
{
    waypoint_struct waypoint = {5u, 6u, 'R', 0u, 1u, 1u, 1u};
    uint16 tick;
    uint16 max_ticks = (uint16)(EXEC_ARRIVAL_STABLE_TICKS +
                                (EXEC_SEGMENT_SETTLE_MS / CONTROL_PERIOD_MS) + 3u);

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    executor_update_20ms();
    if((0 == executor_art_pre_push_pending()) ||
       (0 == executor_continue_after_pre_push_center()))
    {
        return 0;
    }
    drive_pose_reset(20.0f, 0.0f, 0.0f);
    for(tick = 0; tick < max_ticks; tick++)
    {
        executor_update_20ms();
        if(0 != executor_art_sync_pending())
        {
            break;
        }
    }

    return ((0 != executor_art_sync_pending()) &&
            (EXEC_STATE_RUNNING == executor_get_state()) &&
            (0u == executor_get_current_step())) ? 1u : 0u;
}

int main(void)
{
    uint8 passed = 1u;

    passed &= run_case("first-push-waits", first_push_waits_before_motion());
    passed &= run_case("start-switch-atomic", start_switch_is_atomic());
    passed &= run_case("error-switch-atomic", error_switch_is_atomic());
    passed &= run_case("release-same-push", successful_correction_releases_same_push());
    passed &= run_case("turn-center-no-push-align", lowercase_turn_waits_without_push_alignment());
    passed &= run_case("push-center-needs-align", uppercase_center_requires_push_alignment());
    passed &= run_case("horizontal-align-y-only", horizontal_push_aligns_only_y());
    passed &= run_case("vertical-align-x-only", vertical_push_aligns_only_x());
    passed &= run_case("aligned-push-resumes", aligned_push_resumes_same_waypoint());
    passed &= run_case("align-timeout-center-error", alignment_timeout_stops_with_center_error());
    passed &= run_case("align-jitter-timeout", alignment_jitter_still_times_out());
    passed &= run_case("push-does-not-wait", push_waypoints_do_not_wait());
    passed &= run_case("lowercase-offline-skip", lowercase_and_offline_push_do_not_wait());
    passed &= run_case("center-error-stops", center_error_stops_on_same_waypoint());
    passed &= run_case("art-fusion-configured", art_center_uses_configured_fusion());
    passed &= run_case("art-samples-reset", art_center_samples_can_be_reset());
    passed &= run_case("box-request-next-waypoint", final_approach_requests_next_box());
    passed &= run_case("box-prefetch-previous", previous_waypoint_prefetches_next_box());
    passed &= run_case("box-retry-nudge", retry_nudge_moves_away_in_all_directions());
    passed &= run_case("box-align-consumes-approach", box_preparation_aligns_and_consumes_approach());
    passed &= run_case("box-short-gap-retreat", short_gap_retreats_before_alignment());
    passed &= run_case("box-wrong-side-rejected", wrong_side_box_is_rejected());
    passed &= run_case("box-adjacent-cell-rejected", adjacent_box_cell_is_rejected());
    passed &= run_case("box-boundary-offset", box_center_crossing_cell_boundary_is_accepted());
    passed &= run_case("box-all-directions", box_preparation_targets_all_directions());
    passed &= run_case("box-step-pauses", step_box_preparation_pauses_before_push());
    passed &= run_case("run-push-chain-continuous", run_push_chain_switches_without_stop());
    passed &= run_case("run-move-push-continuous", run_move_into_push_switches_without_stop());
    passed &= run_case("run-push-chain-lookahead", run_push_chain_targets_straight_end());
    passed &= run_case("lookahead-stops-task-end", lookahead_stops_at_task_end());
    passed &= run_case("push-turn-stops-for-center", push_turn_stops_before_next_waypoint());
    passed &= run_case("step-still-pauses", step_mode_still_pauses_after_first_push());
    passed &= run_case("final-push-art-sync", final_push_still_waits_for_art());

    return (0 != passed) ? 0 : 1;
}
