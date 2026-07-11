#include <math.h>
#include <stdio.h>
#include <string.h>
#include "executor.h"
#include "drive_control.h"
#include "drive_pose.h"

static drive_pose_struct test_pose;
static uint16 motion_call_count;
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
    (void)vx;
    (void)vy;
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

static uint8 art_center_uses_ninety_percent_fusion(void)
{
    waypoint_struct waypoint = {5u, 6u, 'r', 0u, 1u, 0u, 0u};
    executor_art_center_result_enum result;

    reset_fixture();
    executor_start(&waypoint, 1u, 5u, 5u, 0.0f, 0.0f, 0u, 1u);
    (void)executor_apply_art_player_center(568u, 550u, 1u);
    (void)executor_apply_art_player_center(570u, 550u, 2u);
    (void)executor_apply_art_player_center(572u, 550u, 3u);
    result = executor_commit_art_player_center(5u, 5u);

    return ((EXEC_ART_CENTER_APPLIED == result) &&
            (fabsf(test_pose.x_cm - 3.6f) < 0.01f) &&
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
    if((1u != executor_get_current_step()) ||
       (reset_before != reset_call_count) ||
       (motion_call_count <= motion_before))
    {
        return 0;
    }

    return ((1u == executor_get_current_step()) &&
            (reset_before == reset_call_count) &&
            (motion_call_count > motion_before) &&
            (0 == executor_art_pre_push_pending())) ? 1u : 0u;
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
    passed &= run_case("push-does-not-wait", push_waypoints_do_not_wait());
    passed &= run_case("lowercase-offline-skip", lowercase_and_offline_push_do_not_wait());
    passed &= run_case("center-error-stops", center_error_stops_on_same_waypoint());
    passed &= run_case("art-fusion-90-percent", art_center_uses_ninety_percent_fusion());
    passed &= run_case("art-samples-reset", art_center_samples_can_be_reset());
    passed &= run_case("run-push-chain-continuous", run_push_chain_switches_without_stop());
    passed &= run_case("step-still-pauses", step_mode_still_pauses_after_first_push());
    passed &= run_case("final-push-art-sync", final_push_still_waits_for_art());

    return (0 != passed) ? 0 : 1;
}
