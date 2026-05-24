#include "motion_math.h"

void attitude_pd_init(attitude_pd_struct *pid, float kp, float kd, float dt_s)
{
    pid->kp = kp;
    pid->kd = kd;
    pid->dt_s = dt_s;
    pid->last_error = 0.0f;
}

void attitude_pd_reset(attitude_pd_struct *pid)
{
    pid->last_error = 0.0f;
}

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

float shortest_angle_error(float target_yaw, float current_yaw)
{
    float error = target_yaw - current_yaw;

    /* 航向角跨越 0/360 degree 时必须取最短旋转方向，
     * 否则目标 1 degree 与当前 359 degree 会被误判为大角度回转。 */
    while(error > 180.0f)
    {
        error -= 360.0f;
    }
    while(error < -180.0f)
    {
        error += 360.0f;
    }
    return error;
}

float attitude_pd_update(attitude_pd_struct *pid, float target_yaw, float current_yaw)
{
    float error;
    float derivative;
    float output;

    error = shortest_angle_error(target_yaw, current_yaw);
    derivative = (error - pid->last_error) / pid->dt_s;
    output = pid->kp * error + pid->kd * derivative;
    pid->last_error = error;

    return limit_float(output, -MAX_VZ, MAX_VZ);
}

void command_to_velocity(motion_command_enum command, float move_speed, float turn_speed, float *vx, float *vy, float *vz)
{
    *vx = 0.0f;
    *vy = 0.0f;
    *vz = 0.0f;

    switch(command)
    {
        case MOTION_FORWARD:
        {
            *vy = move_speed;
        }break;

        case MOTION_BACKWARD:
        {
            *vy = -move_speed;
        }break;

        case MOTION_LEFT:
        {
            *vx = -move_speed;
        }break;

        case MOTION_RIGHT:
        {
            *vx = move_speed;
        }break;

        case MOTION_LEFT_FRONT:
        {
            *vx = -move_speed;
            *vy = move_speed;
        }break;

        case MOTION_LEFT_BACK:
        {
            *vx = -move_speed;
            *vy = -move_speed;
        }break;

        case MOTION_RIGHT_FRONT:
        {
            *vx = move_speed;
            *vy = move_speed;
        }break;

        case MOTION_RIGHT_BACK:
        {
            *vx = move_speed;
            *vy = -move_speed;
        }break;

        case MOTION_TURN_LEFT:
        {
            *vz = turn_speed;
        }break;

        case MOTION_TURN_RIGHT:
        {
            *vz = -turn_speed;
        }break;

        case MOTION_STOP:
        default:
        {
        }break;
    }
}

void mecanum_mix(float vx, float vy, float vz, float wheel_norm[WHEEL_COUNT])
{
    wheel_norm[WHEEL_LF] = vy - vx + vz;
    wheel_norm[WHEEL_LB] = vy + vx + vz;
    wheel_norm[WHEEL_RF] = vy + vx - vz;
    wheel_norm[WHEEL_RB] = vy - vx - vz;

    normalize_wheels(wheel_norm);
}

void normalize_wheels(float wheel_norm[WHEEL_COUNT])
{
    uint8 i;
    float max_abs = 0.0f;
    float value;

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        value = wheel_norm[i];
        if(value < 0.0f)
        {
            value = -value;
        }
        if(value > max_abs)
        {
            max_abs = value;
        }
    }

    if(max_abs > 1.0f)
    {
        /* 四轮共同缩放而非逐轮截断，保留麦轮合成运动的方向比例。 */
        for(i = 0; i < WHEEL_COUNT; i++)
        {
            wheel_norm[i] = wheel_norm[i] / max_abs;
        }
    }
}

void wheel_targets_from_norm(const float wheel_norm[WHEEL_COUNT], float wheel_target_count[WHEEL_COUNT])
{
    uint8 i;

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        wheel_target_count[i] = limit_float(wheel_norm[i], -1.0f, 1.0f) * MAX_WHEEL_TARGET_COUNT;
    }
}
