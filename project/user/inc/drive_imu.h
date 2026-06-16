#ifndef _drive_imu_h_
#define _drive_imu_h_

#include "drive_control.h"

/**
 * @brief 初始化 yaw 姿态 PD 状态。
 * @note 在底盘初始化阶段调用一次；不直接访问 IMU 硬件寄存器。
 */
void drive_imu_init(void);

/**
 * @brief 同步 IMU 驱动最新 roll/pitch/yaw 到控制状态。
 * @param[in,out] status 底盘控制状态，写入 roll/pitch/yaw 与最短 yaw 误差。
 * @note 由 20ms 控制链路调用；yaw 会按 `IMU_YAW_DIR_SIGN` 和零点偏置转换到控制坐标系。
 */
void drive_imu_sync_status(control_status_struct *status);

/**
 * @brief 将当前 yaw 锁定为姿态目标。
 * @param[in,out] status 底盘控制状态。
 * @note 会清除姿态 PD 历史误差，避免锁定瞬间产生微分冲击。
 */
void drive_imu_lock_current_yaw(control_status_struct *status);

/**
 * @brief 按一次 20ms 控制周期更新姿态环输出。
 * @param[in,out] status 底盘控制状态，写入 `yaw_error` 和 `vzt`。
 * @note `vzt` 是归一化旋转修正分量，不是 PWM，也不是 encoder count。
 */
void drive_imu_update_attitude_20ms(control_status_struct *status);

/**
 * @brief 设置绝对目标 yaw，并清除姿态 PD 历史误差。
 * @param[in,out] status 底盘控制状态。
 * @param[in] yaw 目标航向角，单位 degree，函数内部环绕到 [0, 360)。
 */
void drive_imu_set_target_yaw(control_status_struct *status, float yaw);

/**
 * @brief STOP 时把目标 yaw 更新为当前 yaw。
 * @param[in,out] status 底盘控制状态。
 * @note 用于安全停机，防止下一次 20ms 周期沿旧目标 yaw 重新给轮子输出。
 */
void drive_imu_stop_lock_current_yaw(control_status_struct *status);

#endif
