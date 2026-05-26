#ifndef _drive_control_h_
#define _drive_control_h_

#include "zf_common_typedef.h"
#include "drive_config.h"

/** 底盘闭环状态快照，轮数组顺序固定为 LF/LB/RF/RB。 */
typedef struct
{
    float target_yaw;                   /**< 姿态环目标航向角，单位为 degree。 */
    float current_yaw;                  /**< IMU660RC 当前航向角，单位为 degree。 */
    float vx;                           /**< 车体右向平移分量，范围 [-1, 1]。 */
    float vy;                           /**< 车体前向平移分量，范围 [-1, 1]。 */
    float vz;                           /**< 姿态 PD 生成的逆时针旋转分量。 */
    float wheel_norm[WHEEL_COUNT];          /**< 混控后的归一化轮速。 */
    float wheel_target_count[WHEEL_COUNT];  /**< 轮速目标，单位 count/20ms。 */
    float wheel_feedback_count[WHEEL_COUNT];/**< 编码器反馈，单位 count/20ms。 */
    float signed_pwm[WHEEL_COUNT];          /**< 四轮 signed PWM 输出。 */
} control_status_struct;

/**
 * @brief 初始化底盘硬件、姿态环、四轮 PID 和两级 PIT 周期任务。
 * @return 0 表示 IMU 可用或当前未启用 IMU；非 0 仅表示 IMU 初始化失败。
 * @note 应在主程序初始化阶段调用一次。
 *       底盘编码器、电机和非姿态控制不依赖 IMU 初始化结果。
 */
uint8 control_init(void);

/**
 * @brief 执行 5ms IMU 四元数更新任务。
 * @note 仅由 PIT_CH0 ISR 调用；该入口是第一版 yaw 更新的唯一数据路径。
 */
void update_imu_5ms(void);

/**
 * @brief 执行 20ms 姿态环、麦轮混控和四轮速度闭环。
 * @note 仅由 PIT_CH1 ISR 调用；处于单轮点动模式时不接管 PWM。
 */
void update_control_20ms(void);

/**
 * @brief 设置平移分量，同时保持当前目标航向角。
 * @param[in] vx 车体右向分量，输入会限幅到 [-1, 1]。
 * @param[in] vy 车体前向分量，输入会限幅到 [-1, 1]。
 * @note 调用后退出单轮点动模式。
 */
void set_motion(float vx, float vy);

/**
 * @brief 接收一条离散运动命令。
 * @param[in] command 前后左右、斜向、离散转向或停止命令。
 * @param[in] move_speed 平移幅值，约定范围为 [0, 1]。
 * @param[in] turn_speed 转向步进比例，限幅到 [0, 1] 后乘以 `TURN_STEP_DEG`。
 * @note 左/右转命令每调用一次只追加一次目标 yaw 步进，实际旋转仍由姿态 PD 完成。
 */
void set_motion_command(motion_command_enum command, float move_speed, float turn_speed);

/**
 * @brief 设置绝对目标航向角。
 * @param[in] yaw 目标航向角，单位为 degree，函数会环绕到 [0, 360)。
 * @note 调用后退出单轮点动模式并清除姿态 PD 历史误差。
 */
void set_target_yaw(float yaw);

/**
 * @brief 同时设置平移分量和绝对目标航向角。
 * @param[in] vx 车体右向分量，范围 [-1, 1]。
 * @param[in] vy 车体前向分量，范围 [-1, 1]。
 * @param[in] yaw_target 目标航向角，单位为 degree。
 */
void set_motion_target(float vx, float vy, float yaw_target);

/**
 * @brief 稳定停止底盘输出。
 * @note 函数将目标 yaw 更新为当前 yaw，并重置姿态/轮速状态，防止下一周期因旧目标重新转动。
 */
void stop_motion(void);

/**
 * @brief 对单个车轮施加低 PWM 点动输出以校验接线方向。
 * @param[in] wheel 待点动的车轮。
 * @param[in] signed_pwm 点动 PWM，正负号表示期望轮速方向。
 * @note 进入点动模式后 20ms 闭环暂停接管 PWM；调用正常控制接口或 `stop_motion()` 退出。
 */
void test_wheel(wheel_enum wheel, float signed_pwm);

/**
 * @brief 获取底盘控制状态快照的只读地址。
 * @return 内部状态地址，调用方不得修改其内容。
 * @note 状态在中断中更新，主循环读取时用于显示和调试，不保证跨字段原子快照。
 */
const control_status_struct *get_control_status(void);

#endif
