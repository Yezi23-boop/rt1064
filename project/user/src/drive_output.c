#include "drive_output.h"
#include "base_io.h"
#include "wheel_pid.h"

static wheel_pid_struct wheel_pid[WHEEL_COUNT];
static float start_pwm_ramp_limit = (float)DRIVE_START_PWM_RAMP_INITIAL_LIMIT;

static void reset_start_pwm_ramp(void)
{
    start_pwm_ramp_limit = (float)DRIVE_START_PWM_RAMP_INITIAL_LIMIT;
}

static void reset_wheel_pid_all(void)
{
    uint8 i;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        wheel_pid_reset(&wheel_pid[i]);
    }
}

static void limit_start_pwm_ramp(float pwm[WHEEL_COUNT])
{
#if DRIVE_START_PWM_RAMP_ENABLE
    uint8 i;
    float max_abs = 0.0f;
    float value_abs;

    if (start_pwm_ramp_limit <= 0.0f)
    {
        return;
    }

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        value_abs = pwm[i];
        if (value_abs < 0.0f)
        {
            value_abs = -value_abs;
        }
        if (value_abs > max_abs)
        {
            max_abs = value_abs;
        }
    }

    if ((0.0f == max_abs) || (max_abs <= start_pwm_ramp_limit))
    {
        return;
    }

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        pwm[i] = pwm[i] * start_pwm_ramp_limit / max_abs;
    }
#else
    (void)pwm;
#endif
}

static void update_start_pwm_ramp(const float pwm[WHEEL_COUNT])
{
#if DRIVE_START_PWM_RAMP_ENABLE
    uint8 i;
    uint8 has_output = 0;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        if ((pwm[i] > 0.0f) || (pwm[i] < 0.0f))
        {
            has_output = 1;
            break;
        }
    }

    if (0 == has_output)
    {
        reset_start_pwm_ramp();
        return;
    }

    if (start_pwm_ramp_limit < (float)MAX_PWM_DUTY)
    {
        start_pwm_ramp_limit += (float)DRIVE_START_PWM_RAMP_STEP;
        if (start_pwm_ramp_limit > (float)MAX_PWM_DUTY)
        {
            start_pwm_ramp_limit = (float)MAX_PWM_DUTY;
        }
    }
#else
    (void)pwm;
#endif
}

static void output_wheel_pwm_with_start_ramp(control_status_struct *status)
{
    uint8 i;

    limit_start_pwm_ramp(status->signed_pwm);
    update_start_pwm_ramp(status->signed_pwm);

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        set_wheel_pwm((wheel_enum)i, status->signed_pwm[i]);
    }
}

static void clear_signed_pwm(control_status_struct *status)
{
    uint8 i;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        status->signed_pwm[i] = 0.0f;
    }
}

void drive_output_init(void)
{
    uint8 i;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        wheel_pid_init(&wheel_pid[i],
                       WHEEL_PID_KP,
                       WHEEL_PID_KI,
                       WHEEL_PID_KD,
                       -(float)MAX_PWM_DUTY,
                       (float)MAX_PWM_DUTY);
    }

    reset_start_pwm_ramp();
}

void drive_output_clear_motion_outputs(control_status_struct *status)
{
    uint8 i;

    status->vx = 0.0f;
    status->vy = 0.0f;
    status->vz = 0.0f;
    status->vzt = 0.0f;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        status->wheel_norm[i] = 0.0f;
        status->wheel_target_count[i] = 0.0f;
        status->signed_pwm[i] = 0.0f;
    }
}

void drive_output_stop(control_status_struct *status)
{
    clear_signed_pwm(status);
    reset_start_pwm_ramp();
    stop_wheels();
}

void drive_output_reset_and_stop(control_status_struct *status)
{
    clear_signed_pwm(status);
    reset_wheel_pid_all();
    reset_start_pwm_ramp();
    stop_wheels();
}

void drive_output_update_and_output(control_status_struct *status)
{
    uint8 i;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        status->signed_pwm[i] = wheel_pid_update(&wheel_pid[i],
                                                 status->wheel_target_count[i],
                                                 status->wheel_feedback_count[i]);
    }

    output_wheel_pwm_with_start_ramp(status);
}

void drive_output_run_speed_loop_test(control_status_struct *status, float target_count)
{
    uint8 i;

    drive_output_clear_motion_outputs(status);
    for (i = 0; i < WHEEL_COUNT; i++)
    {
        status->wheel_target_count[i] = target_count;
    }

    drive_output_update_and_output(status);
}
