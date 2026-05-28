#include "drive_test.h"
#include "drive_config.h"
#include "drive_control.h"
#include "drive_output.h"
#include "base_io.h"
#include "motion_math.h"
#include "timebase.h"

static uint8 translate_test_started;
static uint8 translate_test_stopped;
static uint8 manual_pwm_active;

void drive_test_init(void)
{
    translate_test_started = 0;
    translate_test_stopped = 0;
    manual_pwm_active = 0;
}

void drive_test_poll(void)
{
#if DRIVE_TRANSLATE_TEST_ENABLE
    uint32 now_ms = time_ms();

    /* 平移测试只负责发一次上层命令；实际姿态环和速度环仍在 PIT 里按 20ms 执行。 */
    if((0 == translate_test_started) && (now_ms >= DRIVE_TRANSLATE_TEST_START_MS))
    {
        set_motion_command(DRIVE_TRANSLATE_TEST_COMMAND, DRIVE_TRANSLATE_TEST_SPEED, 0.0f);
        translate_test_started = 1;
    }

    if((0 != translate_test_started) &&
       (0 == translate_test_stopped) &&
       (now_ms >= (DRIVE_TRANSLATE_TEST_START_MS + DRIVE_TRANSLATE_TEST_DURATION_MS)))
    {
        stop_motion();
        translate_test_stopped = 1;
    }
#endif
}

uint8 drive_test_manual_pwm_active(void)
{
    return manual_pwm_active;
}

void drive_test_clear_manual_pwm(void)
{
    manual_pwm_active = 0;
}

uint8 drive_test_try_update_speed_loop_20ms(control_status_struct *status)
{
#if DRIVE_SPEED_LOOP_TEST_ENABLE
    drive_output_run_speed_loop_test(status, DRIVE_SPEED_LOOP_TEST_TARGET_COUNT);
    return 1;
#else
    (void)status;
    return 0;
#endif
}

void drive_test_apply_attitude_target(float current_yaw)
{
#if DRIVE_ATTITUDE_LOOP_TEST_ENABLE
    /* 姿态测试目标必须等 yaw 稳定锁定后再给，避免把上电漂移当作真实偏差。 */
    set_motion_target(0.0f,
                      0.0f,
                      current_yaw + DRIVE_ATTITUDE_LOOP_TEST_TARGET_OFFSET_DEG);
#else
    (void)current_yaw;
#endif
}

void test_wheel(wheel_enum wheel, float signed_pwm)
{
    if(wheel >= WHEEL_COUNT)
    {
        return;
    }

    stop_motion();
    manual_pwm_active = 1;
    set_wheel_pwm(wheel, limit_float(signed_pwm, -(float)MAX_PWM_DUTY, (float)MAX_PWM_DUTY));
}
