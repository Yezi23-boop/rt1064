#include <stdio.h>
#include <string.h>
#include "executor.h"
#include "drive_control.h"
#include "drive_pose.h"

static drive_pose_struct test_pose;
static float last_vx;
static float last_vy;
static uint16 motion_count;
static uint16 stop_count;
static uint8 critical_depth;
static uint8 reset_critical_depth;
static executor_state_enum state_before_enable;

uint32 interrupt_global_disable(void)
{
    critical_depth++;
    return 0u;
}
void interrupt_global_enable(uint32 primask)
{
    (void)primask;
    state_before_enable = executor_get_state();
    critical_depth--;
}
const drive_pose_struct *drive_pose_get(void) { return &test_pose; }
void drive_pose_reset(float x_cm, float y_cm, float yaw_deg)
{
    reset_critical_depth = critical_depth;
    test_pose.x_cm = x_cm;
    test_pose.y_cm = y_cm;
    test_pose.yaw_deg = yaw_deg;
}
void set_motion(float vx, float vy)
{
    last_vx = vx;
    last_vy = vy;
    motion_count++;
}
void reset_motion_segment(void) { }
void stop_motion(void) { stop_count++; }

static void reset_fixture(float x_cm, float y_cm)
{
    executor_stop();
    memset(&test_pose, 0, sizeof(test_pose));
    test_pose.x_cm = x_cm;
    test_pose.y_cm = y_cm;
    last_vx = 0.0f;
    last_vy = 0.0f;
    motion_count = 0u;
    stop_count = 0u;
    executor_init();
    reset_critical_depth = 0u;
    state_before_enable = EXEC_STATE_IDLE;
}

static uint8 run_case(const char *name, uint8 passed)
{
    printf("%-30s %s\n", name, (0u != passed) ? "PASS" : "FAIL");
    return passed;
}

int main(void)
{
    uint8 passed = 1u;
    uint8 tick;

    reset_fixture(2.0f, -1.0f);
    passed &= run_case("correction-start",
        0u != executor_start_position_correction(4.0f, 3.0f));
    executor_update_20ms();
    passed &= run_case("correction-motion-direction",
        (0u != motion_count) && (last_vx > 0.0f) && (last_vy > 0.0f));

    test_pose.x_cm = 4.0f;
    test_pose.y_cm = 3.0f;
    for(tick = 0u; tick < EXEC_ARRIVAL_STABLE_TICKS; tick++)
    {
        executor_update_20ms();
    }
    passed &= run_case("correction-arrival-done",
        (EXEC_STATE_DONE == executor_get_state()) && (0u != stop_count));

    reset_fixture(0.4f, 0.4f);
    (void)executor_start_position_correction(0.0f, 0.0f);
    for(tick = 0u; tick < EXEC_ARRIVAL_STABLE_TICKS; tick++)
    {
        executor_update_20ms();
    }
    passed &= run_case("correction-axis-deadband",
        EXEC_STATE_DONE == executor_get_state());

    reset_fixture(0.0f, 0.0f);
    (void)executor_start_position_correction(3.0f, 0.0f);
    executor_stop();
    motion_count = 0u;
    executor_update_20ms();
    passed &= run_case("correction-stop-cancels",
        (EXEC_STATE_IDLE == executor_get_state()) && (0u == motion_count));

    reset_fixture(7.0f, -3.0f);
    test_pose.yaw_deg = 12.0f;
    passed &= run_case("correction-reset-atomic",
        (0u != executor_start_position_correction_with_pose_reset(
                   1.5f, -2.5f, 0.0f, 0.0f)) &&
        (1u == reset_critical_depth) &&
        (EXEC_STATE_RUNNING == state_before_enable) &&
        (EXEC_STATE_RUNNING == executor_get_state()) &&
        (test_pose.x_cm == 1.5f) && (test_pose.y_cm == -2.5f) &&
        (test_pose.yaw_deg == 12.0f));

    return (0u != passed) ? 0 : 1;
}
