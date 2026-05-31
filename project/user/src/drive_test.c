#include "drive_test.h"
#include "drive_config.h"
#include "drive_control.h"
#include "drive_output.h"
#include "drive_pose.h"
#include "base_io.h"
#include "motion_math.h"
#include "timebase.h"

static uint8 translate_test_started;
static uint8 translate_test_stopped;
static uint8 square_test_started;
static uint8 square_test_stopped;
static uint8 square_test_step;
static uint32 square_test_step_start_ms;
static float square_test_origin_x_cm;
static float square_test_origin_y_cm;
static uint8 wheel_jog_started;
static uint8 wheel_jog_stopped;
static uint8 manual_pwm_active;

void drive_test_init(void)
{
    translate_test_started = 0;
    translate_test_stopped = 0;
    square_test_started = 0;
    square_test_stopped = 0;
    square_test_step = 0;
    square_test_step_start_ms = 0;
    square_test_origin_x_cm = 0.0f;
    square_test_origin_y_cm = 0.0f;
    wheel_jog_started = 0;
    wheel_jog_stopped = 0;
    manual_pwm_active = 0;
}

static void drive_translate_test_poll(void)
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

static void drive_square_test_apply_step(uint8 step)
{
    switch(step)
    {
        case 0:
            set_motion_command(MOTION_RIGHT, DRIVE_SQUARE_TEST_SPEED, 0.0f);
            break;

        case 1:
            set_motion_command(MOTION_BACKWARD, DRIVE_SQUARE_TEST_SPEED, 0.0f);
            break;

        case 2:
            set_motion_command(MOTION_LEFT, DRIVE_SQUARE_TEST_SPEED, 0.0f);
            break;

        case 3:
            set_motion_command(MOTION_FORWARD, DRIVE_SQUARE_TEST_SPEED, 0.0f);
            break;

        default:
            stop_motion();
            break;
    }
}

static void drive_square_test_capture_origin(uint32 now_ms)
{
    const drive_pose_struct *pose = drive_pose_get();

    square_test_origin_x_cm = pose->x_cm;
    square_test_origin_y_cm = pose->y_cm;
    square_test_step_start_ms = now_ms;
}

static void drive_square_test_target(uint8 step, float *target_x_cm, float *target_y_cm)
{
    switch(step)
    {
        case 0:
            *target_x_cm = square_test_origin_x_cm + DRIVE_SQUARE_TEST_SIDE_CM;
            *target_y_cm = square_test_origin_y_cm;
            break;

        case 1:
            *target_x_cm = square_test_origin_x_cm + DRIVE_SQUARE_TEST_SIDE_CM;
            *target_y_cm = square_test_origin_y_cm - DRIVE_SQUARE_TEST_SIDE_CM;
            break;

        case 2:
            *target_x_cm = square_test_origin_x_cm;
            *target_y_cm = square_test_origin_y_cm - DRIVE_SQUARE_TEST_SIDE_CM;
            break;

        default:
            *target_x_cm = square_test_origin_x_cm;
            *target_y_cm = square_test_origin_y_cm;
            break;
    }
}

static uint8 drive_square_test_pose_arrived(uint8 step)
{
    const drive_pose_struct *pose = drive_pose_get();
    float target_x_cm;
    float target_y_cm;

    drive_square_test_target(step, &target_x_cm, &target_y_cm);

    switch(step)
    {
        case 0:
            return (pose->x_cm >= (target_x_cm - DRIVE_SQUARE_TEST_ARRIVAL_THRESHOLD_CM)) ? 1u : 0u;

        case 1:
            return (pose->y_cm <= (target_y_cm + DRIVE_SQUARE_TEST_ARRIVAL_THRESHOLD_CM)) ? 1u : 0u;

        case 2:
            return (pose->x_cm <= (target_x_cm + DRIVE_SQUARE_TEST_ARRIVAL_THRESHOLD_CM)) ? 1u : 0u;

        case 3:
            return (pose->y_cm >= (target_y_cm - DRIVE_SQUARE_TEST_ARRIVAL_THRESHOLD_CM)) ? 1u : 0u;

        default:
            return 1u;
    }
}

static uint8 drive_square_test_step_timeout(uint32 now_ms)
{
    return ((now_ms - square_test_step_start_ms) >= DRIVE_SQUARE_TEST_STEP_TIMEOUT_MS) ? 1u : 0u;
}

static void drive_square_test_next_step(uint32 now_ms)
{
    square_test_step++;
    if(4u <= square_test_step)
    {
        stop_motion();
        square_test_stopped = 1;
        return;
    }

    square_test_step_start_ms = now_ms;
    drive_square_test_apply_step(square_test_step);
}

static void drive_square_test_poll(void)
{
#if DRIVE_SQUARE_TEST_ENABLE
    uint32 now_ms = time_ms();

    if(0 != square_test_stopped)
    {
        return;
    }

    if(0 == square_test_started)
    {
        if(now_ms < DRIVE_SQUARE_TEST_START_MS)
        {
            return;
        }
        square_test_started = 1;
        square_test_step = 0;
        drive_square_test_capture_origin(now_ms);
        drive_square_test_apply_step(square_test_step);
        return;
    }

#if DRIVE_SQUARE_TEST_POSE_MODE_ENABLE
    if(0 != drive_square_test_pose_arrived(square_test_step))
    {
        drive_square_test_next_step(now_ms);
        return;
    }

    if(0 != drive_square_test_step_timeout(now_ms))
    {
        stop_motion();
        square_test_stopped = 1;
        return;
    }
#else
    uint32 elapsed_ms;
    uint8 next_step;

    elapsed_ms = now_ms - DRIVE_SQUARE_TEST_START_MS;
    next_step = (uint8)(elapsed_ms / DRIVE_SQUARE_TEST_SIDE_DURATION_MS);
    if(4u <= next_step)
    {
        stop_motion();
        square_test_stopped = 1;
        return;
    }

    if(next_step != square_test_step)
    {
        square_test_step = next_step;
        drive_square_test_apply_step(square_test_step);
    }
#endif
#endif
}

static void drive_wheel_jog_poll(void)
{
#if DRIVE_WHEEL_JOG_ENABLE
    uint32 now_ms = time_ms();

    if((0 == wheel_jog_started) && (now_ms >= DRIVE_WHEEL_JOG_START_MS))
    {
        test_wheel(DRIVE_WHEEL_JOG_WHEEL, DRIVE_WHEEL_JOG_PWM);
        wheel_jog_started = 1;
    }

    if((0 != wheel_jog_started) &&
       (0 == wheel_jog_stopped) &&
       (now_ms >= (DRIVE_WHEEL_JOG_START_MS + DRIVE_WHEEL_JOG_DURATION_MS)))
    {
        stop_motion();
        wheel_jog_stopped = 1;
    }
#endif
}

void drive_test_poll(void)
{
    drive_translate_test_poll();
    drive_square_test_poll();
    drive_wheel_jog_poll();
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
