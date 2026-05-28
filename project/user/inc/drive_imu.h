#ifndef _drive_imu_h_
#define _drive_imu_h_

#include "drive_control.h"

/** 初始化 yaw 姿态 PD。 */
void drive_imu_init(void);

/** 同步 IMU 驱动最新 roll/pitch/yaw 到控制状态。 */
void drive_imu_sync_status(control_status_struct *status);

/** 将当前 yaw 锁定为姿态目标。 */
void drive_imu_lock_current_yaw(control_status_struct *status);

/** 20ms 姿态环更新。 */
void drive_imu_update_attitude_20ms(control_status_struct *status);

/** 设置绝对目标 yaw，并清除姿态 PD 历史误差。 */
void drive_imu_set_target_yaw(control_status_struct *status, float yaw);

/** STOP 时把目标 yaw 更新为当前 yaw，避免旧目标继续回正。 */
void drive_imu_stop_lock_current_yaw(control_status_struct *status);

#endif
