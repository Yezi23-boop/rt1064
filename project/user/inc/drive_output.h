#ifndef _drive_output_h_
#define _drive_output_h_

#include "drive_control.h"

/**
 * @brief 初始化四轮速度 PID 和起步 PWM 窗口。
 * @note 在控制初始化阶段调用一次；PID 输入单位固定为 count/20ms，输出为 signed PWM。
 */
void drive_output_init(void);

/**
 * @brief 清除运动分量、四轮目标和当前 signed PWM。
 * @param[in,out] status 底盘控制状态。
 * @note 不改变目标 yaw；用于启动保护、段间停稳和测试链路接管前的输出清空。
 */
void drive_output_clear_motion_outputs(control_status_struct *status);

/**
 * @brief 清 signed PWM、重置起步窗口并关闭四轮。
 * @param[in,out] status 底盘控制状态。
 * @note 不清轮速 PID 历史，适合短暂停车但仍保留闭环状态的场景。
 */
void drive_output_stop(control_status_struct *status);

/**
 * @brief 清 signed PWM、清速度 PID、重置起步窗口并关闭四轮。
 * @param[in,out] status 底盘控制状态。
 * @note 停车、路径小段结束或点动模式退出时调用，避免旧 PWM 累计量带入下一段。
 */
void drive_output_reset_and_stop(control_status_struct *status);

/**
 * @brief 正常链路：四轮速度 PID 更新后经过起步 PWM 窗口输出。
 * @param[in,out] status 底盘控制状态，读取 `wheel_target_count`/`wheel_feedback_count`，写入 `signed_pwm`。
 * @note 仅在 20ms 控制周期调用；内部会对接近 0 的目标清 PID，避免静止时残留输出。
 */
void drive_output_update_and_output(control_status_struct *status);

/**
 * @brief 速度环测试：四轮直接使用同一个目标 count 并输出。
 * @param[in,out] status 底盘控制状态。
 * @param[in] target_count 四轮统一目标，单位 count/20ms。
 * @note 绕过 IMU 和麦轮混控，仅用于确认编码器方向、PID 参数和 PWM 输出链路。
 */
void drive_output_run_speed_loop_test(control_status_struct *status, float target_count);

#endif
