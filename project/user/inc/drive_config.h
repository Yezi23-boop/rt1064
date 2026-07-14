#ifndef _drive_config_h_
#define _drive_config_h_

#include "zf_common_typedef.h"

/** 底盘固定为四轮麦轮结构，数组索引必须与 LF/LB/RF/RB 轮序一致。 */
#define WHEEL_COUNT (4)
/** 轮速 PID 与姿态环的执行周期，单位为 ms。 */
#define CONTROL_PERIOD_MS (20)
/** 轮速 PID 与姿态环的执行周期，单位为 s。 */
#define CONTROL_DT_S (0.02f)
/** 推箱子地图每格物理尺寸，单位 cm。 */
#define GRID_SIZE_CM (20.0f)

/** 第一版上板使用的保守目标轮速上限，单位为 encoder count/20ms。 */
#define MAX_WHEEL_TARGET_COUNT (100.0f)
/** 前后方向编码器增量到地面位移的标定系数，单位 cm/count；上下准时保持该值不动。 */
#define POSE_Y_CM_PER_COUNT (0.0079f) // 86
/** 左右横移编码器增量到地面位移的标定系数，单位 cm/count；麦轮横移滑移通常需要单独标定。 */
#define POSE_X_CM_PER_COUNT (0.0079f)
/** 位姿 X 轴方向校正；当前取 +1，表示麦轮反解的正 X 直接对应右移为正。 */
#define POSE_X_DIR_SIGN (1.0f)
/** 位姿 Y 轴方向校正；当前取 +1，表示麦轮反解的正 Y 直接对应前进为正。 */
#define POSE_Y_DIR_SIGN (1.0f)
/** 第一版上板使用的保守 PWM 限幅，PWM_DUTY_MAX 的量程为 10000。 */
#define MAX_PWM_DUTY (8000)
/** 电机 PWM 频率，单位为 Hz。 */
#define PWM_FREQ_HZ (17000)
/** 电机最小有效 PWM 补偿开关；置 0 时交给轮速 PID 自行克服低速死区。 */
#define MOTOR_PWM_DEADBAND_ENABLE (0)
/** 左前轮最小有效 PWM，占空比量程同 MAX_PWM_DUTY/PWM_DUTY_MAX。 */
#define MOTOR_PWM_DEADBAND_LF (500)
/** 左后轮最小有效 PWM，占空比量程同 MAX_PWM_DUTY/PWM_DUTY_MAX。 */
#define MOTOR_PWM_DEADBAND_LB (500)
/** 右前轮最小有效 PWM，占空比量程同 MAX_PWM_DUTY/PWM_DUTY_MAX。 */
#define MOTOR_PWM_DEADBAND_RF (500)
/** 右后轮最小有效 PWM，占空比量程同 MAX_PWM_DUTY/PWM_DUTY_MAX。 */
#define MOTOR_PWM_DEADBAND_RB (500)

/** yaw 姿态 PD 比例系数；误差单位为 degree，输出为归一化姿态修正分量 vzt。 */
#define YAW_KP (0.08f)
/** yaw 姿态 PD 微分系数；不加入积分项，避免静态角度误差累积导致过冲。 */
#define YAW_KD (0.0001f)
/** yaw 姿态环硬死区，单位 degree；死区内不输出姿态修正，避免零点附近 IMU 小抖动带动车轮。 */
#define YAW_DEADBAND_DEG (0.05f)
/** 姿态环允许输出的最大归一化旋转分量。 */
#define MAX_VZ (1.0f)
/** 平移/路径执行时姿态保持允许叠加的最大旋转修正，避免横移被 yaw 环抢占。 */
#define YAW_TRANSLATION_MAX_VZ (0.3f)
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
#define WHEEL_PID_KP (5.0f)
/** 四轮增量式速度 PID 积分初值；单位随 count/20ms 误差和 PWM 输出共同确定。 */
#define WHEEL_PID_KI (1.0f)
/** 四轮增量式速度 PID 微分初值；第一版关闭微分。 */
#define WHEEL_PID_KD (0.0f)
/** 目标轮速小于该阈值时认为该轮应停转，不让编码器微小抖动触发速度环补偿。 */
#define WHEEL_TARGET_STOP_EPS_COUNT (0.3f)
/** 目标轮速达到该阈值才允许电机死区补偿，避免段末 yaw 小修正被放大成抖动。 */
#define MOTOR_PWM_DEADBAND_TARGET_THRESHOLD_COUNT (25.0f)

/** 起步 PWM 阶梯限幅开关；置 1 后速度环输出会先从较低 PWM 窗口逐步放开。 */
#define DRIVE_START_PWM_RAMP_ENABLE (0)
/** 起步首个非零输出周期允许的最大 signed PWM 绝对值。 */
#define DRIVE_START_PWM_RAMP_INITIAL_LIMIT (1000)
/** 每个 20ms 控制周期放开的 signed PWM 窗口增量。 */
#define DRIVE_START_PWM_RAMP_STEP (100)

/** 路径跟踪 PID 比例系数；误差单位为 cm，输出为归一化速度。 */
#define PATH_KP (0.07f)
/** 路径跟踪 PID 积分系数；当前使用很小的 0.001，补偿稳定的小范围残余误差。 */
#define PATH_KI (0.001f)
/** 路径跟踪 PID 微分系数；当前默认 0，避免 20ms 位姿增量噪声放大。 */
#define PATH_KD (0.0f)
/** 路径跟踪 PID 最大输出速度，归一化到 MAX_WHEEL_TARGET_COUNT。 */
#define PATH_MAX_SPEED (1.0f)
/** 路径跟踪 PID 积分限幅，单位 cm*s；只有 PATH_KI 非 0 时才影响输出。 */
#define PATH_MAX_INTEGRAL (5.0f)
/** 路径/发车移动到点阈值，单位 cm。 */
#define PATH_ARRIVAL_THRESHOLD_CM (0.5f)
/** waypoint/发车移动到点需要连续满足阈值的 20ms 周期数，用于过滤瞬时越界和惯性抖动。 */
#define EXEC_ARRIVAL_STABLE_TICKS (3u)
/** waypoint 切换前的停稳时间，单位 ms；只在拐点/路径点边界停，不拆连续直线段。 */
#define EXEC_SEGMENT_SETTLE_MS (500u)
/** 推箱动作目标点越界补偿开关；只作用于 U/D/L/R，不改变普通移动。 */
#define EXEC_PUSH_OVERSHOOT_ENABLE (0)
/** 推箱动作额外前压比例，单位为格；0.20 表示每格 20cm 时多走 4cm。 */
#define EXEC_PUSH_OVERSHOOT_RATIO (0.20f)
/** ART 等图、中心请求以及发车/返航/观察回中心动作的最长时间。 */
#define EXEC_ART_SYNC_TIMEOUT_MS (10000u)
/** CENTER_REQ 超时降级开关；1=按阶段使用安全兜底继续，0=严格停车报错。 */
#ifndef ART_CENTER_TIMEOUT_FALLBACK_ENABLE
#define ART_CENTER_TIMEOUT_FALLBACK_ENABLE (1)
#endif
/** 发车中心超时时固定向右移动的距离，单位 cm。 */
#define ART_LAUNCH_FALLBACK_MOVE_CM (30.0f)
/** ART 来源执行时，完整地图需要连续一致的新帧数量。 */
#define EXEC_ART_STABLE_FRAMES (1u)
/** 比赛运行范围：只跑科目一、只跑科目二或一次 K3 完整连续运行。 */
#define COMPETITION_MODE_SUBJECT1_DEBUG (1u)
#define COMPETITION_MODE_SUBJECT2_DEBUG (2u)
#define COMPETITION_MODE_FULL (3u)
#define COMPETITION_MODE (COMPETITION_MODE_SUBJECT2_DEBUG)
/** UART4 与 OpenART #2 板级自检；1=上电自动测试并禁止启动比赛，0=正常比赛。 */
#define VISION_UART_BOARD_TEST_ENABLE (0)
/** 科目二分类结果最低置信度，单位千分值。 */
#define SUBJECT2_CLASS_CONFIDENCE_Q (750u)
/** 科目二分类需要连续一致的有效样本数。 */
#define SUBJECT2_CLASS_STABLE_SAMPLES (3u)
/** 科目二单个视距等待分类结果的最长时间；超时后先后退扩大视野。 */
#define SUBJECT2_VIEW_TIMEOUT_MS (5000u)
/** 科目二首次识别失败时沿远离对象方向后退的距离，单位 cm。 */
#define SUBJECT2_VIEW_BACKOFF_CM (8.0f)
/** 科目二观察转向允许误差，单位 degree。 */
#define SUBJECT2_TURN_TOLERANCE_DEG (0.5f)
/** yaw 连续处于允许误差内的时间，单位 ms。 */
#define SUBJECT2_TURN_STABLE_MS (100u)
/** 科目二观察转向最长时间，单位 ms。 */
#define SUBJECT2_TURN_TIMEOUT_MS (10000u)
/** 等待 OpenART #2 READY 时的模式命令重发周期。 */
#define SUBJECT2_VISION_READY_RETRY_MS (1000u)
/** 等待 OpenART #2 READY 的总超时时间。 */
#define SUBJECT2_VISION_READY_TIMEOUT_MS (10000u)
/** ART 左发车区目标：第二个可走格中心 X，单位 cm；col=2.5, grid=20cm -> 50cm。 */
#define ART_LAUNCH_TARGET_X_CM (50.0f)
/** 每次 CENTER_REQ 需要的有效精确中心样本数；所有关键节点共用5帧中值滤波。 */
#define ART_CENTER_SAMPLE_COUNT (5u)
/** ART 发车移动最大归一化速度。 */
#define ART_LAUNCH_MOVE_MAX_SPEED (1.0f)
/** 推箱完成后自动返回左侧发车中心。 */
#define ART_RETURN_HOME_ENABLE (1)
/** 左侧发车通道在推箱地图内的入口列。 */
#define ART_RETURN_GATE_COL (2u)
/** 左侧发车通道允许的第一行。 */
#define ART_RETURN_GATE_ROW_MIN (5u)
/** 左侧发车通道允许的最后一行。 */
#define ART_RETURN_GATE_ROW_MAX (6u)
/** 最终中心与发车中心允许的行列误差，单位 1/100 格；5 对应约 1cm。 */
#define ART_RETURN_HOME_TOLERANCE_Q (5u)
/** 最终视觉复核未通过时允许的再次校正次数。 */
#define ART_RETURN_MAX_CORRECTIONS (2u)
/** ART waypoint 前中心矫正开关；只在实际运动方向发生变化的转折点采样。 */
#define EXEC_ART_WAYPOINT_CENTER_CORRECT_ENABLE (1)
/** 旧普通 waypoint 段末中心校正开关；请求式中心模式下保持关闭。 */
#define EXEC_ART_CENTER_CORRECT_ENABLE (0)
/** ART 中心点小于该偏差不修正，单位 cm，避免原地小抖动反复写 pose。 */
#define EXEC_ART_CENTER_IGNORE_CM (0.5f)
/** ART 中心点允许直接融合的最大偏差，单位 cm；当前允许最多一个 20cm 格子。 */
#define EXEC_ART_CENTER_FUSE_MAX_CM (20.0f)
/** ART 中心点超过一个 20cm 格子认为异常。 */
#define EXEC_ART_CENTER_ABNORMAL_CM (20.0f)
/** ART 中心点融合比例；0.90 表示本地 pose 保留 10%，ART 观测占 90%。 */
#define EXEC_ART_CENTER_FUSE_ALPHA (1.00f)
/** ART 中心格匹配范围；1=接受当前 C 格及八邻域，0=只接受当前 C 格。 */
#define EXEC_ART_CENTER_ALLOW_NEIGHBOR_CELL (1)

/** 底盘统一轮序，混控、硬件映射和调试输出均不得更换该顺序。 */
typedef enum
{
    WHEEL_LF = 0, /**< 左前轮。 */
    WHEEL_RF,     /**< 右前轮。 */
    WHEEL_LB,     /**< 左后轮。 */
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
