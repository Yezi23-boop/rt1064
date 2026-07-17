#include <math.h>
#include <stdio.h>
#include "drive_control.h"
#include "drive_imu.h"
#include "drive_output.h"
#include "drive_pose.h"
#include "drive_test.h"
#include "base_io.h"
#include "motion_math.h"

#define PIT_CH1 (1)
void pit_ms_init(int channel, uint32 period_ms);
static uint8 fake_io_state;

#include "../project/user/src/drive_control.c"

static float fake_attitude_output;

float limit_float(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        return max_value;
    }
    if(value < min_value)
    {
        return min_value;
    }
    return value;
}
void command_to_velocity(motion_command_enum command, float move_speed,
                         float turn_speed, float *vx, float *vy, float *vz)
{
    (void)command;
    (void)move_speed;
    (void)turn_speed;
    *vx = 0.0f;
    *vy = 0.0f;
    *vz = 0.0f;
}
void drive_imu_init(void) { }
void drive_output_init(void) { }
void drive_pose_init(void) { }
uint8 io_init(void) { return fake_io_state; }
void pit_ms_init(int channel, uint32 period_ms)
{
    (void)channel;
    (void)period_ms;
}
void read_encoder_counts(float wheel_feedback_count[WHEEL_COUNT])
{
    (void)wheel_feedback_count;
}
void drive_imu_sync_status(control_status_struct *status) { (void)status; }
uint8 drive_test_manual_pwm_active(void) { return 0u; }
void drive_output_clear_motion_outputs(control_status_struct *status)
{
    status->vx = 0.0f;
    status->vy = 0.0f;
    status->vz = 0.0f;
    status->vzt = 0.0f;
}
void drive_output_stop(control_status_struct *status) { (void)status; }
void drive_imu_lock_current_yaw(control_status_struct *status)
{
    status->target_yaw = status->current_yaw;
    status->yaw_error = 0.0f;
    status->vzt = 0.0f;
}
void drive_pose_reset_origin(float current_yaw_deg) { (void)current_yaw_deg; }
void drive_test_apply_attitude_target(float current_yaw) { (void)current_yaw; }
void set_motor_output_enabled(uint8 enabled) { (void)enabled; }
void drive_pose_update_20ms(const float encoder_count[WHEEL_COUNT], float yaw_deg)
{
    (void)encoder_count;
    (void)yaw_deg;
}
uint8 drive_test_try_update_speed_loop_20ms(control_status_struct *status)
{
    (void)status;
    return 0u;
}
void drive_imu_update_attitude_20ms(control_status_struct *status)
{
    status->vzt = fake_attitude_output;
}
void mecanum_mix(float vx, float vy, float vz, float vzt,
                 float wheel_norm[WHEEL_COUNT])
{
    (void)vx;
    (void)vy;
    (void)vz;
    (void)vzt;
    (void)wheel_norm;
}
void wheel_targets_from_norm(const float wheel_norm[WHEEL_COUNT],
                             float wheel_target_count[WHEEL_COUNT])
{
    (void)wheel_norm;
    (void)wheel_target_count;
}
void drive_output_update_and_output(control_status_struct *status) { (void)status; }
void drive_test_clear_manual_pwm(void) { }
void drive_imu_set_target_yaw(control_status_struct *status, float yaw)
{
    status->target_yaw = yaw;
}
uint32 interrupt_global_disable(void) { return 0u; }
void interrupt_global_enable(uint32 primask) { (void)primask; }
void drive_output_reset_and_stop(control_status_struct *status) { (void)status; }
void drive_imu_stop_lock_current_yaw(control_status_struct *status)
{
    drive_imu_lock_current_yaw(status);
}

static uint8 output_matches(float vx, float vy, float attitude_output,
                            float expected)
{
    const control_status_struct *status;

    set_motion(vx, vy);
    fake_attitude_output = attitude_output;
    control_output_update_20ms();
    status = get_control_status();
    return (fabsf(status->vzt - expected) < 0.0001f) ? 1u : 0u;
}

static uint8 run_case(const char *name, uint8 passed)
{
    printf("%-32s %s\n", name, (0u != passed) ? "PASS" : "FAIL");
    return passed;
}

static uint8 init_failure_is_latched(void)
{
    fake_io_state = 1u;
    (void)control_init();
    return ((0u == drive_control_is_healthy()) &&
            (DRIVE_HEALTH_IMU_INIT == drive_control_get_health_fault())) ? 1u : 0u;
}

static uint8 healthy_init_is_reported(void)
{
    fake_io_state = 0u;
    (void)control_init();
    return ((0u != drive_control_is_healthy()) &&
            (DRIVE_HEALTH_NONE == drive_control_get_health_fault())) ? 1u : 0u;
}

int main(void)
{
    uint8 passed = 1u;

    passed &= run_case("healthy-init", healthy_init_is_reported());
    passed &= run_case("imu-init-fault-latched", init_failure_is_latched());
    fake_io_state = 0u;
    (void)control_init();
    passed &= run_case("turn-positive-limited",
                       output_matches(0.0f, 0.0f, 1.0f, 0.4f));
    passed &= run_case("turn-negative-limited",
                       output_matches(0.0f, 0.0f, -1.0f, -0.4f));
    passed &= run_case("translation-positive-unchanged",
                       output_matches(0.2f, 0.0f, 1.0f,
                                      YAW_TRANSLATION_MAX_VZ));
    passed &= run_case("translation-negative-unchanged",
                       output_matches(0.0f, -0.2f, -1.0f,
                                      -YAW_TRANSLATION_MAX_VZ));

    return (0u != passed) ? 0 : 1;
}
