#include "drive_imu.h"
#include "motion_math.h"
#include "zf_device_imu660rc.h"

static attitude_pd_struct yaw_pid; // 20ms 姿态环私有状态；目标 yaw 切换或停车时必须 reset，避免旧 D 项冲击。

static float normalize_yaw(float yaw)
{
    // 控制层统一使用 [0, 360) degree，便于和最短角误差函数配合。
    while(yaw >= 360.0f)
    {
        yaw -= 360.0f;
    }
    while(yaw < 0.0f)
    {
        yaw += 360.0f;
    }
    return yaw;
}

static float imu_yaw_to_control_yaw(float imu_yaw)
{
    // IMU 安装方向和控制正方向在配置层收敛，外部模块只看到逆时针为正的 yaw。
    return normalize_yaw(imu_yaw * IMU_YAW_DIR_SIGN + IMU_YAW_ZERO_OFFSET_DEG);
}

void drive_imu_init(void)
{
    attitude_pd_init(&yaw_pid, YAW_KP, YAW_KD, CONTROL_DT_S);
}

void drive_imu_sync_status(control_status_struct *status)
{
    // imu660rc_* 由驱动解算更新；这里仅读取快照，不在 20ms 控制链路里做阻塞采样。
    status->current_roll = imu660rc_roll;
    status->current_pitch = imu660rc_pitch;
    status->current_yaw = imu_yaw_to_control_yaw(imu660rc_yaw);
    status->yaw_error = shortest_angle_error(status->target_yaw, status->current_yaw);
}

void drive_imu_lock_current_yaw(control_status_struct *status)
{
    status->target_yaw = status->current_yaw;
    status->yaw_error = 0.0f;
    status->vzt = 0.0f;
    attitude_pd_reset(&yaw_pid);
}

void drive_imu_update_attitude_20ms(control_status_struct *status)
{
    status->yaw_error = shortest_angle_error(status->target_yaw, status->current_yaw);
    status->vzt = attitude_pd_update(&yaw_pid, status->target_yaw, status->current_yaw);
}

void drive_imu_set_target_yaw(control_status_struct *status, float yaw)
{
    status->target_yaw = normalize_yaw(yaw);
    attitude_pd_reset(&yaw_pid);
}

void drive_imu_stop_lock_current_yaw(control_status_struct *status)
{
    drive_imu_sync_status(status);
    drive_imu_lock_current_yaw(status);
}
