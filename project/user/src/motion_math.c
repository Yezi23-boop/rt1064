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
    /* 硬死区内认为车头已经到位，直接清掉历史误差。
     * 这样不会因为 0.x degree 的 IMU 抖动或旧 D 项让姿态环反复给轮子小输出。 */
    if((error >= -YAW_DEADBAND_DEG) && (error <= YAW_DEADBAND_DEG))
    {
        pid->last_error = 0.0f;
        return 0.0f;
    }

    derivative = (error - pid->last_error) / pid->dt_s;
    output = pid->kp * error + pid->kd * derivative;
    pid->last_error = error;

    return limit_float(output, -MAX_VZ, MAX_VZ);
}

void path_pid_init(path_pid_struct *pid, float kp, float ki, float kd,
                   float max_output, float max_integral)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->max_integral = max_integral;
    path_pid_reset(pid);
}

void path_pid_reset(path_pid_struct *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_output = 0.0f;
}

float path_pid_update(path_pid_struct *pid, float error, float dt_s)
{
    float p_term, i_term, d_term;
    float output;

    /* 位置式 PID 的输入是 cm 级位姿误差，输出直接作为归一化速度分量。 */
    p_term = pid->kp * error;

    /* 积分项用 cm*s 累积；限幅防止停车或定位漂移时把下一段速度顶满。 */
    pid->integral += error * dt_s;
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }
    i_term = pid->ki * pid->integral;

    /* 微分项对 20ms 位姿噪声敏感，当前参数默认关闭但保留公式便于后续调参。 */
    d_term = pid->kd * (error - pid->last_error) / dt_s;
    pid->last_error = error;

    output = p_term + i_term + d_term;

    /* 输出是给 set_motion() 的归一化速度，不能超过上层约定的 [-max_output, max_output]。 */
    output = limit_float(output, -pid->max_output, pid->max_output);

    pid->last_output = output;
    return output;
}

void command_to_velocity(motion_command_enum command, float move_speed, float turn_speed, float *vx, float *vy, float *vz)
{
    // 车体坐标：vx 右正、vy 前正、vz 逆时针正；转向命令后续会转成目标 yaw 步进。
    *vx = 0.0f;
    *vy = 0.0f;
    *vz = 0.0f;

    switch(command)
    {
        case MOTION_FORWARD:
            *vy = move_speed;
            break;

        case MOTION_BACKWARD:
            *vy = -move_speed;
            break;

        case MOTION_LEFT:
            *vx = -move_speed;
            break;

        case MOTION_RIGHT:
            *vx = move_speed;
            break;

        case MOTION_LEFT_FRONT:
            *vx = -move_speed;
            *vy = move_speed;
            break;

        case MOTION_LEFT_BACK:
            *vx = -move_speed;
            *vy = -move_speed;
            break;

        case MOTION_RIGHT_FRONT:
            *vx = move_speed;
            *vy = move_speed;
            break;

        case MOTION_RIGHT_BACK:
            *vx = move_speed;
            *vy = -move_speed;
            break;

        case MOTION_TURN_LEFT:
            *vz = turn_speed;
            break;

        case MOTION_TURN_RIGHT:
            *vz = -turn_speed;
            break;

        case MOTION_STOP:
        default:
            break;
    }
}

void mecanum_mix(float vx, float vy, float vz, float vzt, float wheel_norm[WHEEL_COUNT])
{
    // 车体坐标约定：vx 向右、vy 向前、vz/vzt 逆时针；四轮顺序固定为 LF/LB/RF/RB。
    wheel_norm[WHEEL_LF] = vy + vx - vz - vzt;
    wheel_norm[WHEEL_LB] = vy - vx - vz - vzt;
    wheel_norm[WHEEL_RF] = vy - vx + vz + vzt;
    wheel_norm[WHEEL_RB] = vy + vx + vz + vzt;

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
        // wheel_norm 是混控比例，不是编码器目标；满幅映射到 20ms 的目标 count。
        wheel_target_count[i] = limit_float(wheel_norm[i], -1.0f, 1.0f) * MAX_WHEEL_TARGET_COUNT;
    }
}
