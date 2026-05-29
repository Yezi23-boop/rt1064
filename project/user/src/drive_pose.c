#include "drive_pose.h"
#include <math.h>

#ifndef M_PI
#define M_PI (3.1415926f)
#endif

static drive_pose_struct drive_pose;
static float pose_yaw_zero_deg;

static float normalize_yaw_deg(float yaw_deg)
{
    while(yaw_deg > 180.0f)
    {
        yaw_deg -= 360.0f;
    }
    while(yaw_deg < -180.0f)
    {
        yaw_deg += 360.0f;
    }
    return yaw_deg;
}

void drive_pose_init(void)
{
    drive_pose_reset(0.0f, 0.0f, 0.0f);
}

void drive_pose_reset(float x_cm, float y_cm, float yaw_deg)
{
    drive_pose.x_cm = x_cm;
    drive_pose.y_cm = y_cm;
    drive_pose.yaw_deg = normalize_yaw_deg(yaw_deg);
    drive_pose.body_vx_cm = 0.0f;
    drive_pose.body_vy_cm = 0.0f;
}

void drive_pose_reset_origin(float current_yaw_deg)
{
    pose_yaw_zero_deg = current_yaw_deg;
    drive_pose_reset(0.0f, 0.0f, 0.0f);
}

void drive_pose_update_20ms(const float encoder_count[WHEEL_COUNT], float yaw_deg)
{
    float lf_count = encoder_count[WHEEL_LF];
    float lb_count = encoder_count[WHEEL_LB];
    float rf_count = encoder_count[WHEEL_RF];
    float rb_count = encoder_count[WHEEL_RB];
    float body_vx_count;
    float body_vy_count;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float dx_body_cm;
    float dy_body_cm;
    float pose_yaw_deg;

    /* 这里反解的是当前工程已经调通的混控：
     * LF=vy+vx, LB=vy-vx, RF=vy-vx, RB=vy+vx。
     * encoder_count 是 20ms 内的位移增量，因此积分时不再乘 CONTROL_DT_S。 */
    body_vy_count = (lf_count + lb_count + rf_count + rb_count) * 0.25f;
    body_vx_count = (lf_count + rb_count - lb_count - rf_count) * 0.25f;

    dx_body_cm = body_vx_count * POSE_CM_PER_COUNT * POSE_X_DIR_SIGN;
    dy_body_cm = body_vy_count * POSE_CM_PER_COUNT * POSE_Y_DIR_SIGN;

    pose_yaw_deg = normalize_yaw_deg(yaw_deg - pose_yaw_zero_deg);
    yaw_rad = pose_yaw_deg * (float)M_PI / 180.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    drive_pose.x_cm += dx_body_cm * cos_yaw - dy_body_cm * sin_yaw;
    drive_pose.y_cm += dx_body_cm * sin_yaw + dy_body_cm * cos_yaw;
    drive_pose.yaw_deg = pose_yaw_deg;
    drive_pose.body_vx_cm = dx_body_cm;
    drive_pose.body_vy_cm = dy_body_cm;
}

const drive_pose_struct *drive_pose_get(void)
{
    return &drive_pose;
}
