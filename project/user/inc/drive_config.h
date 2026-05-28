#ifndef _drive_config_h_
#define _drive_config_h_

#include "zf_common_typedef.h"

/** 底盘固定为四轮麦轮结构，数组索引必须与 LF/LB/RF/RB 轮序一致。 */
#define WHEEL_COUNT (4)
/** 轮速 PID 与姿态环的执行周期，单位为 ms。 */
#define CONTROL_PERIOD_MS (20)
/** 轮速 PID 与姿态环的执行周期，单位为 s。 */
#define CONTROL_DT_S (0.02f)

/** 第一版上板使用的保守目标轮速上限，单位为 encoder count/20ms。 */
#define MAX_WHEEL_TARGET_COUNT (100.0f)
/** 第一版上板使用的保守 PWM 限幅，PWM_DUTY_MAX 的量程为 10000。 */
#define MAX_PWM_DUTY (5000)
/** 电机 PWM 频率，单位为 Hz。 */
#define PWM_FREQ_HZ (17000)

/** yaw 姿态 PD 比例系数；误差单位为 degree，输出为归一化姿态修正分量 vzt。 */
#define YAW_KP (0.20f)
/** yaw 姿态 PD 微分系数；不加入积分项，避免静态角度误差累积导致过冲。 */
#define YAW_KD (0.1f)
/** yaw 姿态环硬死区，单位 degree；死区内不输出姿态修正，避免零点附近 IMU 小抖动带动车轮。 */
#define YAW_DEADBAND_DEG (0.5f)
/** 姿态环允许输出的最大归一化旋转分量。 */
#define MAX_VZ (1.0f)
/** 离散原地转向每次命令对应的最大目标角步进，单位为 degree。 */
#define TURN_STEP_DEG (10.0f)

/** IMU 原始 yaw 到底盘控制 yaw 的方向符号。
 * 当前板测为顺时针 raw yaw 增大，因此取 -1，使控制层保持逆时针为正。
 * 如果换 IMU 方向后变成逆时针 raw yaw 增大，只改成 +1.0f。 */
#define IMU_YAW_DIR_SIGN (-1.0f)
/** IMU yaw 零点偏置，单位 degree；只在需要让绝对朝向对齐车头时调整。 */
#define IMU_YAW_ZERO_OFFSET_DEG (0.0f)
/** 上电后等待 IMU yaw 稳定的时间，单位 ms；该时间同时作为电机非零 PWM 输出的安全门槛。 */
#define IMU_YAW_STARTUP_STABLE_DELAY_MS (6000)

/** 四轮增量式速度 PID 比例初值；上板后根据 encoder count/20ms 反馈调整。 */
#define WHEEL_PID_KP (2.0f)
/** 四轮增量式速度 PID 积分初值；单位随 count/20ms 误差和 PWM 输出共同确定。 */
#define WHEEL_PID_KI (0.3f)
/** 四轮增量式速度 PID 微分初值；第一版关闭微分。 */
#define WHEEL_PID_KD (0.0f)

/** 起步 PWM 阶梯限幅开关；置 1 后速度环输出会先从较低 PWM 窗口逐步放开。 */
#define DRIVE_START_PWM_RAMP_ENABLE (1)
/** 起步首个非零输出周期允许的最大 signed PWM 绝对值。 */
#define DRIVE_START_PWM_RAMP_INITIAL_LIMIT (600)
/** 每个 20ms 控制周期放开的 signed PWM 窗口增量。 */
#define DRIVE_START_PWM_RAMP_STEP (10)

/** 底盘统一轮序，混控、硬件映射和调试输出均不得更换该顺序。 */
typedef enum
{
    WHEEL_LF = 0, /**< 左前轮。 */
    WHEEL_LB,     /**< 左后轮。 */
    WHEEL_RF,     /**< 右前轮。 */
    WHEEL_RB,     /**< 右后轮。 */
} wheel_enum;

/** 上层离散运动命令；转向命令通过目标 yaw 步进交给姿态环执行。 */
typedef enum
{
    MOTION_STOP = 0,    /**< 停止电机输出，不保持旧目标角。 */
    MOTION_FORWARD,     /**< 向车头方向平移。 */
    MOTION_BACKWARD,    /**< 向车尾方向平移。 */
    MOTION_LEFT,        /**< 向车体左侧平移。 */
    MOTION_RIGHT,       /**< 向车体右侧平移。 */
    MOTION_LEFT_FRONT,  /**< 左前斜向平移。 */
    MOTION_LEFT_BACK,   /**< 左后斜向平移。 */
    MOTION_RIGHT_FRONT, /**< 右前斜向平移。 */
    MOTION_RIGHT_BACK,  /**< 右后斜向平移。 */
    MOTION_TURN_LEFT,   /**< 目标 yaw 增加，姿态环执行左转。 */
    MOTION_TURN_RIGHT,  /**< 目标 yaw 减少，姿态环执行右转。 */
} motion_command_enum;

/** 四轮电机正方向校正符号；唯一定义在 `base_io.c`，板测点动确认后在那里统一修改 +/-1。 */
extern int8 motor_dir_sign[WHEEL_COUNT];
/** 四路编码器正方向校正符号；唯一定义在 `base_io.c`，正转时反馈应与目标 count 同号。 */
extern int8 encoder_dir_sign[WHEEL_COUNT];

#endif
