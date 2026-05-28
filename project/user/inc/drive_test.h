#ifndef _drive_test_h_
#define _drive_test_h_

#include "zf_common_typedef.h"
#include "drive_config.h"
#include "drive_control.h"

/** 速度环闭环测试开关；置 1 后 20ms 控制只给四轮同一个目标 count，不走 IMU/麦轮混控。 */
#define DRIVE_SPEED_LOOP_TEST_ENABLE (0)
/** 速度环闭环测试目标，单位为 encoder count/20ms；可改成负值检查反馈方向。 */
#define DRIVE_SPEED_LOOP_TEST_TARGET_COUNT (100.0f)

/** 原地姿态角闭环测试开关；置 1 后上电只给目标 yaw 偏移，不给 vx/vy 平移。 */
#define DRIVE_ATTITUDE_LOOP_TEST_ENABLE (0)
/** 原地姿态角闭环测试目标偏移，单位 degree；0 表示锁住上电当前 yaw，手拨后自动回正。 */
#define DRIVE_ATTITUDE_LOOP_TEST_TARGET_OFFSET_DEG (0.0f)

/** 平移保持姿态测试开关；置 1 后主循环会在启动安全窗口后自动发一次平移命令。 */
#define DRIVE_TRANSLATE_TEST_ENABLE (0)
/** 平移测试启动时间，单位 ms；默认等同上电 yaw 稳定/电机输出安全窗口。 */
#define DRIVE_TRANSLATE_TEST_START_MS (IMU_YAW_STARTUP_STABLE_DELAY_MS)
/** 平移测试持续时间，单位 ms；到时自动 stop_motion()。 */
#define DRIVE_TRANSLATE_TEST_DURATION_MS (3000u)
/** 平移测试方向；可改为 MOTION_BACKWARD/MOTION_LEFT/MOTION_RIGHT 分别测试四个基本方向。 */
#define DRIVE_TRANSLATE_TEST_COMMAND (MOTION_LEFT_BACK)
/** 平移测试速度幅值，范围 [0, 1]，先用低速观察姿态保持。 */
#define DRIVE_TRANSLATE_TEST_SPEED (1.0f)

/**
 * @brief 初始化代码开关式底盘调车测试状态。
 *
 * @note 仅在主循环启动前调用一次；函数不访问电机输出。
 */
void drive_test_init(void);

/**
 * @brief 主循环级调车测试轮询入口。
 *
 * 当前只承载平移保持姿态测试；速度环和姿态环测试仍由 20ms 控制链路实时执行。
 */
void drive_test_poll(void);

/**
 * @brief 查询点动测试是否正在保持人工 PWM。
 * @return 1 表示 20ms 闭环应暂停接管电机，0 表示正常控制。
 */
uint8 drive_test_manual_pwm_active(void);

/**
 * @brief 退出点动测试保持状态。
 *
 * 正常控制命令进入时必须清掉该状态，否则上一次点动 PWM 会继续保留。
 */
void drive_test_clear_manual_pwm(void);

/**
 * @brief 如果启用了速度环测试，则执行 20ms 测试链路。
 * @return 1 表示本周期已由测试链路处理，0 表示未启用测试。
 *
 * @note 仅由 `update_control_20ms()` 调用，保持速度环采样周期固定。
 */
uint8 drive_test_try_update_speed_loop_20ms(control_status_struct *status);

/**
 * @brief yaw 锁定后按宏开关应用姿态环测试目标。
 * @param[in] current_yaw 当前已稳定的 yaw，单位为 degree。
 */
void drive_test_apply_attitude_target(float current_yaw);

/**
 * @brief 对单个车轮施加低 PWM 点动输出以校验接线方向。
 * @param[in] wheel 待点动的车轮。
 * @param[in] signed_pwm 点动 PWM，正负号表示期望轮速方向。
 */
void test_wheel(wheel_enum wheel, float signed_pwm);

#endif
