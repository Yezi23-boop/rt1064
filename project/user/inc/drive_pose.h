#ifndef _drive_pose_h_
#define _drive_pose_h_

#include "drive_config.h"

/** 由四轮 20ms 编码器增量积分得到的轻量位姿估计。 */
typedef struct
{
    float x_cm;                          /**< 全局 X 坐标，单位 cm，右移为正。 */
    float y_cm;                          /**< 全局 Y 坐标，单位 cm，前进为正。 */
    float yaw_deg;                       /**< 相对复位零点的航向角，单位 degree。 */
    float body_vx_cm;                    /**< 最近一个 20ms 周期的车体 X 位移增量，单位 cm，不是 cm/s。 */
    float body_vy_cm;                    /**< 最近一个 20ms 周期的车体 Y 位移增量，单位 cm，不是 cm/s。 */
} drive_pose_struct;

/**
 * @brief 初始化位姿估计状态。
 * @note 不访问硬件；只把内部位姿清到原点。
 */
void drive_pose_init(void);

/**
 * @brief 重置位姿估计值。
 * @param[in] x_cm 全局 X 坐标，单位 cm，右移为正。
 * @param[in] y_cm 全局 Y 坐标，单位 cm，前进为正。
 * @param[in] yaw_deg 相对零点 yaw，单位 degree。
 */
void drive_pose_reset(float x_cm, float y_cm, float yaw_deg);

/**
 * @brief 把当前 IMU yaw 设为位姿积分零点。
 * @param[in] current_yaw_deg 当前控制坐标系 yaw，单位 degree。
 * @note 上电稳定窗口结束后调用，避免启动阶段 yaw 漂移污染全局坐标。
 */
void drive_pose_reset_origin(float current_yaw_deg);

/**
 * @brief 按一次 20ms 编码器增量更新位姿。
 * @param[in] encoder_count 四轮编码器增量，单位 count/20ms，轮序 LF/LB/RF/RB。
 * @param[in] yaw_deg 当前控制坐标系 yaw，单位 degree。
 * @note `encoder_count` 已经过方向校正；函数内部积分的是位移增量，不再乘控制周期。
 */
void drive_pose_update_20ms(const float encoder_count[WHEEL_COUNT], float yaw_deg);

/**
 * @brief 获取位姿估计状态的只读地址。
 * @return 内部位姿状态地址，调用方不得写入。
 * @note 20ms 控制链路写入，主循环/执行器读取时不保证跨字段原子快照。
 */
const drive_pose_struct *drive_pose_get(void);

#endif
