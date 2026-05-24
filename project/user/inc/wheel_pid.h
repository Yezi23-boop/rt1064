#ifndef _wheel_pid_h_
#define _wheel_pid_h_

/** 单轮增量式 PID 状态，输入输出节拍固定为 20ms。 */
typedef struct
{
    float kp;                            /**< 比例系数。 */
    float ki;                            /**< 积分系数。 */
    float kd;                            /**< 微分系数。 */
    float error_1;                       /**< 上一次目标与反馈误差。 */
    float error_2;                       /**< 上上次目标与反馈误差。 */
    float output;                        /**< 当前 signed PWM 累计输出。 */
    float output_min;                    /**< signed PWM 下限。 */
    float output_max;                    /**< signed PWM 上限。 */
} wheel_pid_struct;

/**
 * @brief 初始化一个轮速增量式 PID 实例。
 *
 * @param[out] pid 待初始化的 PID 状态。
 * @param[in] kp 比例系数。
 * @param[in] ki 积分系数。
 * @param[in] kd 微分系数。
 * @param[in] output_min signed PWM 输出下限。
 * @param[in] output_max signed PWM 输出上限。
 * @note 初始化会清除历史误差和累计输出。
 */
void wheel_pid_init(wheel_pid_struct *pid, float kp, float ki, float kd, float output_min, float output_max);

/**
 * @brief 清除 PID 历史状态和累计输出。
 *
 * @param[in,out] pid 待重置的 PID 状态。
 * @note 停车、切换点动模式或重新开始闭环前调用，避免旧输出残留。
 */
void wheel_pid_reset(wheel_pid_struct *pid);

/**
 * @brief 按一次 20ms 采样更新单轮增量式 PID。
 *
 * @param[in,out] pid 轮速 PID 状态。
 * @param[in] target 目标编码器增量，单位为 count/20ms。
 * @param[in] feedback 实际编码器增量，单位为 count/20ms。
 * @return 限幅后的 signed PWM 输出。
 */
float wheel_pid_update(wheel_pid_struct *pid, float target, float feedback);

#endif
