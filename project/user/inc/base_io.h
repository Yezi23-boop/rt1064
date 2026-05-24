#ifndef _base_io_h_
#define _base_io_h_

#include "zf_common_typedef.h"
#include "drive_config.h"

/**
 * @brief 初始化底盘执行器、编码器和 IMU660RC 四元数输出。
 * @return 0 表示 IMU 初始化成功；非 0 表示 IMU 初始化失败。
 * @note 初始化阶段会把四个电机 PWM 置零。
 */
uint8 io_init(void);

/**
 * @brief 读取一次 IMU660RC 内置四元数并刷新欧拉角全局量。
 * @note 仅由 PIT_CH0 的 5ms 中断路径调用；GPIO IMU 中断不得重复读取。
 */
void read_quaternion(void);

/**
 * @brief 获取驱动已经解算出的当前航向角。
 * @return `imu660rc_yaw`，单位为 degree。
 */
float read_yaw(void);

/**
 * @brief 读取并清零四轮编码器增量。
 * @param[out] wheel_feedback_count 四轮反馈，单位为 count/20ms，轮序为 LF/LB/RF/RB。
 * @note 函数会应用 `encoder_dir_sign`，使正轮速与控制坐标系一致。
 */
void read_encoder_counts(float wheel_feedback_count[WHEEL_COUNT]);

/**
 * @brief 向指定车轮输出带方向的 PWM。
 * @param[in] wheel 目标车轮。
 * @param[in] signed_pwm signed PWM 指令，正负号表示控制坐标系下的轮速方向。
 * @note 函数会应用 `motor_dir_sign` 并限制输出不超过板测安全初值。
 */
void set_wheel_pwm(wheel_enum wheel, float signed_pwm);

/**
 * @brief 将四个车轮 PWM 输出立即置零。
 * @note 本函数不决定姿态环后续是否重新接管，稳定停止由 `stop_motion()` 负责。
 */
void stop_wheels(void);

#endif
