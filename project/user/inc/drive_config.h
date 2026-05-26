#ifndef _drive_config_h_
#define _drive_config_h_

#include "zf_common_typedef.h"

/** 底盘固定为四轮麦轮结构，数组索引必须与 LF/LB/RF/RB 轮序一致。 */
#define WHEEL_COUNT              (4)
/** 轮速 PID 与姿态环的执行周期，单位为 ms。 */
#define CONTROL_PERIOD_MS        (20)
/** 轮速 PID 与姿态环的执行周期，单位为 s。 */
#define CONTROL_DT_S             (0.02f)

/** 第一版上板使用的保守目标轮速上限，单位为 encoder count/20ms。 */
#define MAX_WHEEL_TARGET_COUNT           (80.0f)
/** 第一版上板使用的保守 PWM 限幅，PWM_DUTY_MAX 的量程为 10000。 */
#define MAX_PWM_DUTY                     (3000)
/** 电机 PWM 频率，单位为 Hz。 */
#define PWM_FREQ_HZ              (17000)

/** yaw 姿态 PD 比例系数；误差单位为 degree，输出为归一化旋转分量 vz。 */
#define YAW_KP                   (0.020f)
/** yaw 姿态 PD 微分系数；不加入积分项，避免静态角度误差累积导致过冲。 */
#define YAW_KD                   (0.001f)
/** 姿态环允许输出的最大归一化旋转分量。 */
#define MAX_VZ                   (1.0f)
/** 离散原地转向每次命令对应的最大目标角步进，单位为 degree。 */
#define TURN_STEP_DEG            (10.0f)

/** 四轮增量式速度 PID 比例初值；上板后根据 encoder count/20ms 反馈调整。 */
#define WHEEL_PID_KP             (10.0f)
/** 四轮增量式速度 PID 积分初值；单位随 count/20ms 误差和 PWM 输出共同确定。 */
#define WHEEL_PID_KI             (0.5f)
/** 四轮增量式速度 PID 微分初值；第一版关闭微分。 */
#define WHEEL_PID_KD             (0.0f)

/** 底盘统一轮序，混控、硬件映射和调试输出均不得更换该顺序。 */
typedef enum
{
    WHEEL_LF = 0,               /**< 左前轮。 */
    WHEEL_LB,                   /**< 左后轮。 */
    WHEEL_RF,                   /**< 右前轮。 */
    WHEEL_RB,                   /**< 右后轮。 */
} wheel_enum;

/** 上层离散运动命令；转向命令通过目标 yaw 步进交给姿态环执行。 */
typedef enum
{
    MOTION_STOP = 0,            /**< 停止电机输出，不保持旧目标角。 */
    MOTION_FORWARD,             /**< 向车头方向平移。 */
    MOTION_BACKWARD,            /**< 向车尾方向平移。 */
    MOTION_LEFT,                /**< 向车体左侧平移。 */
    MOTION_RIGHT,               /**< 向车体右侧平移。 */
    MOTION_LEFT_FRONT,          /**< 左前斜向平移。 */
    MOTION_LEFT_BACK,           /**< 左后斜向平移。 */
    MOTION_RIGHT_FRONT,         /**< 右前斜向平移。 */
    MOTION_RIGHT_BACK,          /**< 右后斜向平移。 */
    MOTION_TURN_LEFT,           /**< 目标 yaw 增加，姿态环执行左转。 */
    MOTION_TURN_RIGHT,          /**< 目标 yaw 减少，姿态环执行右转。 */
} motion_command_enum;

/** 四轮电机正方向校正符号；板测点动确认后统一修改该数组中的 +/-1。 */
extern int8 motor_dir_sign[WHEEL_COUNT];
/** 四路编码器正方向校正符号；正转时反馈应与目标 count 同号。 */
extern int8 encoder_dir_sign[WHEEL_COUNT];

#endif
