#include "wheel_pid.h"
#include "motion_math.h"

void wheel_pid_init(wheel_pid_struct *pid, float kp, float ki, float kd, float output_min, float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    wheel_pid_reset(pid);
}

void wheel_pid_reset(wheel_pid_struct *pid)
{
    pid->error_1 = 0.0f;
    pid->error_2 = 0.0f;
    pid->output = 0.0f;
}

float wheel_pid_update(wheel_pid_struct *pid, float target, float feedback)
{
    float error;
    float delta;

    error = target - feedback;
    // target/feedback 单位为 count/20ms；增量式 PID 每周期只计算 signed PWM 增量。
    delta = pid->kp * (error - pid->error_1) +
            pid->ki * error +
            pid->kd * (error - 2.0f * pid->error_1 + pid->error_2);

    pid->output += delta;
    pid->output = limit_float(pid->output, pid->output_min, pid->output_max);

    pid->error_2 = pid->error_1;
    pid->error_1 = error;

    return pid->output;
}
