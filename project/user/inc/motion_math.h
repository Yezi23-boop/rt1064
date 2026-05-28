#ifndef _motion_math_h_
#define _motion_math_h_

#include "zf_common_typedef.h"
#include "drive_config.h"

/** 姿态 PD 状态；仅处理 yaw 误差到归一化旋转修正分量 vzt 的转换。 */
typedef struct
{
    float kp;                            /**< yaw 比例系数。 */
    float kd;                            /**< yaw 微分系数。 */
    float dt_s;                          /**< 更新周期，单位为 s。 */
    float last_error;                    /**< 上一次最短角度误差，单位为 degree。 */
} attitude_pd_struct;

/**
 * @brief 初始化 yaw 姿态 PD。
 * @param[out] pid 待初始化的姿态 PD 状态。
 * @param[in] kp 比例系数。
 * @param[in] kd 微分系数。
 * @param[in] dt_s 控制周期，单位为 s，必须大于 0。
 */
void attitude_pd_init(attitude_pd_struct *pid, float kp, float kd, float dt_s);

/**
 * @brief 清除姿态 PD 的历史误差。
 * @param[in,out] pid 姿态 PD 状态。
 * @note 修改目标 yaw 或停止底盘时调用，避免旧误差产生微分冲击。
 */
void attitude_pd_reset(attitude_pd_struct *pid);

/**
 * @brief 计算经过环绕处理后的最短 yaw 误差。
 * @param[in] target_yaw 目标航向角，单位为 degree。
 * @param[in] current_yaw 当前航向角，单位为 degree。
 * @return 处于 [-180, 180] 区间内的目标减当前角度误差。
 */
float shortest_angle_error(float target_yaw, float current_yaw);

/**
 * @brief 由 yaw 误差生成姿态环旋转分量。
 * @param[in,out] pid 姿态 PD 状态。
 * @param[in] target_yaw 目标航向角，单位为 degree。
 * @param[in] current_yaw 当前航向角，单位为 degree。
 * @return 已限幅到 [-MAX_VZ, MAX_VZ] 的归一化姿态修正分量 vzt。
 */
float attitude_pd_update(attitude_pd_struct *pid, float target_yaw, float current_yaw);

/**
 * @brief 将浮点值约束在给定闭区间内。
 * @param[in] value 待限幅数值。
 * @param[in] min_value 下限。
 * @param[in] max_value 上限。
 * @return 限幅后的数值。
 */
float limit_float(float value, float min_value, float max_value);

/**
 * @brief 把上层离散运动命令转换为车体归一化运动分量。
 * @param[in] command 离散运动命令。
 * @param[in] move_speed 平移幅值，约定范围为 [0, 1]。
 * @param[in] turn_speed 转向幅值，约定范围为 [0, 1]。
 * @param[out] vx 车体右向分量，正值表示向右。
 * @param[out] vy 车体前向分量，正值表示向前。
 * @param[out] vz 逆时针分量，仅表示离散转向方向和幅值。
 */
void command_to_velocity(motion_command_enum command, float move_speed, float turn_speed, float *vx, float *vy, float *vz);

/**
 * @brief 按 X 型麦轮公式混控并保持四轮输出比例。
 * @param[in] vx 车体右向归一化平移分量。
 * @param[in] vy 车体前向归一化平移分量。
 * @param[in] vz 上层命令给出的逆时针归一化旋转分量。
 * @param[in] vzt 姿态环给出的逆时针归一化旋转修正分量。
 * @param[out] wheel_norm 四轮归一化输出，轮序为 LF/LB/RF/RB。
 */
void mecanum_mix(float vx, float vy, float vz, float vzt, float wheel_norm[WHEEL_COUNT]);

/**
 * @brief 将四轮混控结果等比缩放到 [-1, 1]。
 * @param[in,out] wheel_norm 四轮归一化输出，轮序为 LF/LB/RF/RB。
 * @note 等比缩放用于避免饱和改变车体运动方向。
 */
void normalize_wheels(float wheel_norm[WHEEL_COUNT]);

/**
 * @brief 将归一化轮速映射为编码器目标增量。
 * @param[in] wheel_norm 四轮归一化输出，轮序为 LF/LB/RF/RB。
 * @param[out] wheel_target_count 四轮目标反馈量，单位为 count/20ms。
 */
void wheel_targets_from_norm(const float wheel_norm[WHEEL_COUNT], float wheel_target_count[WHEEL_COUNT]);

#endif
