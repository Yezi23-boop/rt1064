#include "zf_common_headfile.h"
#include "drive_control.h"
#include "motion_math.h"
#include "wheel_pid.h"
#include "base_io.h"
#include "zf_device_imu660rc.h"
#include "zf_driver_delay.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415926f
#endif

static control_status_struct control_status;
static attitude_pd_struct yaw_pid;
static wheel_pid_struct wheel_pid[WHEEL_COUNT];
static uint8 control_ready = 0;      // 底盘调度初始化完成后，PIT 回调即可处理编码器和电机。
static uint8 imu_ready = 0;          // IMU 可用时才启用 yaw 姿态保持。
static uint8 point_test_enabled = 0; // 点动时暂停闭环，保留人工核对轮向的 PWM。
static uint8 startup_yaw_locked = 1; // IMU 上电稳定等待完成后，才允许姿态环接管 yaw。
static uint32 startup_yaw_wait_ms = 0;
static float start_pwm_ramp_limit = (float)DRIVE_START_PWM_RAMP_INITIAL_LIMIT;

/**
 * @brief 重置起步 PWM 爬坡窗口。
 *
 * 停车后下一次再启动应重新从较低 PWM 放开，避免 PID 上一次留下的输出窗口
 * 让车辆下地瞬间直接获得大 PWM。
 */
static void reset_start_pwm_ramp(void)
{
    start_pwm_ramp_limit = (float)DRIVE_START_PWM_RAMP_INITIAL_LIMIT;
}

/**
 * @brief 将四轮 signed PWM 按当前起步窗口等比例限幅。
 *
 * 四轮必须共同按最大绝对值缩放，而不是逐轮截断；这样可以保留麦轮混控输出比例，
 * 同时避免启动时任意一个轮子的 PID 输出直接打到上限。
 *
 * @param[in,out] pwm 四轮 signed PWM 数组，函数会原地写入限幅后的实际输出。
 */
static void limit_start_pwm_ramp(float pwm[WHEEL_COUNT])
{
#if DRIVE_START_PWM_RAMP_ENABLE
    uint8 i;
    float max_abs = 0.0f;
    float value_abs;

    if(start_pwm_ramp_limit <= 0.0f)
    {
        return;
    }

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        value_abs = pwm[i];
        if(value_abs < 0.0f)
        {
            value_abs = -value_abs;
        }
        if(value_abs > max_abs)
        {
            max_abs = value_abs;
        }
    }

    if((0.0f == max_abs) || (max_abs <= start_pwm_ramp_limit))
    {
        return;
    }

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        pwm[i] = pwm[i] * start_pwm_ramp_limit / max_abs;
    }
#else
    (void)pwm;
#endif
}

/**
 * @brief 根据本周期实际输出推进或重置起步 PWM 窗口。
 *
 * 只有存在非零输出时才逐步放开窗口；全 0 输出说明车辆处于停止状态，
 * 下一次启动需要重新从 `DRIVE_START_PWM_RAMP_INITIAL_LIMIT` 开始。
 *
 * @param[in] pwm 四轮已经完成起步限幅后的 signed PWM。
 */
static void update_start_pwm_ramp(const float pwm[WHEEL_COUNT])
{
#if DRIVE_START_PWM_RAMP_ENABLE
    uint8 i;
    uint8 has_output = 0;

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        if((pwm[i] > 0.0f) || (pwm[i] < 0.0f))
        {
            has_output = 1;
            break;
        }
    }

    if(0 == has_output)
    {
        reset_start_pwm_ramp();
        return;
    }

    if(start_pwm_ramp_limit < (float)MAX_PWM_DUTY)
    {
        start_pwm_ramp_limit += (float)DRIVE_START_PWM_RAMP_STEP;
        if(start_pwm_ramp_limit > (float)MAX_PWM_DUTY)
        {
            start_pwm_ramp_limit = (float)MAX_PWM_DUTY;
        }
    }
#else
    (void)pwm;
#endif
}

/**
 * @brief 输出四轮 PWM，并在进入电机前统一应用起步阶梯限幅。
 *
 * `signed_pwm[]` 保存的是限幅后的实际输出值，便于 VOFA 直接观察启动爬坡窗口。
 * 点动测试不走该函数，仍保持人工指定 PWM 直接输出。
 */
static void output_wheel_pwm_with_start_ramp(void)
{
    uint8 i;

    limit_start_pwm_ramp(control_status.signed_pwm);
    update_start_pwm_ramp(control_status.signed_pwm);

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        set_wheel_pwm((wheel_enum)i, control_status.signed_pwm[i]);
    }
}

/**
 * @brief 将 IMU660RC 原始 yaw 转成底盘控制坐标系 yaw。
 *
 * 当前板测现象：车体顺时针转动时 IMU660RC yaw 增大；而底盘控制约定
 * `+yaw/+vz` 为逆时针左转。因此在 IMU 边界统一反号，后面的姿态 PD、
 * 转向命令和麦轮混控都继续使用“逆时针为正”的坐标定义。
 */
static float imu_yaw_to_control_yaw(float imu_yaw)
{
    float yaw = imu_yaw * IMU_YAW_DIR_SIGN + IMU_YAW_ZERO_OFFSET_DEG;

    while (yaw >= 360.0f)
    {
        yaw -= 360.0f;
    }
    while (yaw < 0.0f)
    {
        yaw += 360.0f;
    }
    return yaw;
}

/**
 * @brief 上电静止时采样 yaw 并做圆周平均。
 *
 * yaw 是 0~360 degree 环绕量，不能直接算普通平均；例如 359 和 1 的平均
 * 应接近 0，而不是 180。这里用 sin/cos 求平均方向，作为姿态保持的零点。
 */
static float calibrate_startup_yaw(void)
{
    uint8 i;
    float yaw_deg;
    float yaw_rad;
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    float yaw_avg;

    for(i = 0; i < IMU_YAW_ZERO_SAMPLE_COUNT; i++)
    {
        yaw_deg = imu_yaw_to_control_yaw(imu660rc_yaw);
        yaw_rad = yaw_deg * (float)M_PI / 180.0f;
        sin_sum += sinf(yaw_rad);
        cos_sum += cosf(yaw_rad);
        system_delay_ms(IMU_YAW_ZERO_SAMPLE_INTERVAL_MS);
    }

    yaw_avg = atan2f(sin_sum, cos_sum) * 180.0f / (float)M_PI;
    while (yaw_avg >= 360.0f)
    {
        yaw_avg -= 360.0f;
    }
    while (yaw_avg < 0.0f)
    {
        yaw_avg += 360.0f;
    }
    return yaw_avg;
}

/**
 * @brief 将 IMU660RC 中断回调已经解算好的欧拉角同步到控制状态。
 *
 * IMU 四元数读取由 D1/INT2 中断中的 `imu660rc_callback()` 完成；控制层只使用
 * 驱动保存的最新 roll/pitch/yaw，避免再包一层 `read_*()` 转发函数。
 */
static void sync_imu_attitude(void)
{
    if (0 == imu_ready)
    {
        return;
    }

    control_status.current_roll = imu660rc_roll;
    control_status.current_pitch = imu660rc_pitch;
    control_status.current_yaw = imu_yaw_to_control_yaw(imu660rc_yaw);
    control_status.yaw_error = shortest_angle_error(control_status.target_yaw, control_status.current_yaw);
}

/**
 * @brief 速度环闭环测试链路。
 *
 * 该函数只在 `DRIVE_SPEED_LOOP_TEST_ENABLE` 打开时由 20ms 控制中断调用。
 * 测试模式绕过 yaw 姿态环和麦轮混控，直接给四轮同一个 count/20ms 目标，
 * 方便在 VOFA+ 中只观察速度环的 target/feedback/pwm 响应。
 *
 * @note 车轮应悬空测试；目标值通过 `DRIVE_SPEED_LOOP_TEST_TARGET_COUNT` 修改。
 */
static void update_speed_loop_test_20ms(void)
{
    uint8 i;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        control_status.wheel_norm[i] = 0.0f;
        control_status.wheel_target_count[i] = DRIVE_SPEED_LOOP_TEST_TARGET_COUNT;
        control_status.signed_pwm[i] = wheel_pid_update(&wheel_pid[i],
                                                        control_status.wheel_target_count[i],
                                                        control_status.wheel_feedback_count[i]);
    }
    output_wheel_pwm_with_start_ramp();
}

uint8 control_init(void)
{
    uint8 i;
    uint8 hw_state;

    attitude_pd_init(&yaw_pid, YAW_KP, YAW_KD, CONTROL_DT_S);

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        wheel_pid_init(&wheel_pid[i],
                       WHEEL_PID_KP,
                       WHEEL_PID_KI,
                       WHEEL_PID_KD,
                       -(float)MAX_PWM_DUTY,
                       (float)MAX_PWM_DUTY);
    }

    hw_state = io_init();
    imu_ready = (0 == hw_state) ? 1 : 0;
    stop_motion();

    sync_imu_attitude();
    control_status.target_yaw = control_status.current_yaw;
    control_status.yaw_error = 0.0f;
    control_status.vx = 0.0f;
    control_status.vy = 0.0f;
    control_status.vz = 0.0f;
    control_status.vzt = 0.0f;
    attitude_pd_reset(&yaw_pid);
    startup_yaw_wait_ms = 0;
    startup_yaw_locked = (0 != imu_ready) ? 0 : 1;
    set_motor_output_enabled(0);

    pit_ms_init(PIT_CH1, CONTROL_PERIOD_MS); // 20ms 执行姿态环和四轮速度 PID，与 encoder count 单位一致。

    control_ready = 1;
    return hw_state;
}

/**
 * @brief 20ms 底盘实时控制链路。
 *
 * 该函数由 PIT_CH1 中断固定周期调用，按“编码器采样 -> 状态保护 ->
 * 姿态环 -> 麦轮混控 -> 速度环 -> 电机输出”的顺序串行执行。
 * 函数内不能加入 printf、延时或其它可能阻塞的逻辑，否则会拉长中断占用时间，
 * 影响四轮速度闭环的 20ms 采样节拍。
 *
 * @note 编码器反馈单位固定为 count/20ms；点动模式下只刷新反馈，不接管人工 PWM。
 */
void update_control_20ms(void)
{
    uint8 i;
    uint8 has_motion_target = 0;

    /* 1) 编码器采样：先于所有状态判断执行。
     * 原因：Home/VOFA 调试页依赖 wheel_feedback_count，即使闭环尚未 ready、
     * 或正在点动测试，也应能看到最新 count/20ms 反馈。 */
    read_encoder_counts(control_status.wheel_feedback_count);
    sync_imu_attitude();

    /* 2) 初始化保护：PIT 可能已经启动，但底盘调度尚未允许闭环接管。 */
    if (0 == control_ready)
    {
        return;
    }

    /* 3) 上电安全窗口：IMU 刚初始化后的 yaw 会缓慢漂到稳定值。
     * 等待期间继续用 PIT 刷新 Home/VOFA 所需的 IMU 和编码器反馈，但不让姿态环、
     * 速度环、点动测试或起步爬坡产生任何非零电机输出。 */
    if (startup_yaw_wait_ms < IMU_YAW_STARTUP_STABLE_DELAY_MS)
    {
        for (i = 0; i < WHEEL_COUNT; i++)
        {
            control_status.wheel_norm[i] = 0.0f;
            control_status.wheel_target_count[i] = 0.0f;
            control_status.signed_pwm[i] = 0.0f;
            wheel_pid_reset(&wheel_pid[i]);
        }

        if (0 != imu_ready)
        {
            control_status.target_yaw = control_status.current_yaw;
        }
        else
        {
            control_status.current_yaw = control_status.target_yaw;
        }
        control_status.yaw_error = 0.0f;
        control_status.vzt = 0.0f;
        attitude_pd_reset(&yaw_pid);
        reset_start_pwm_ramp();
        stop_wheels();

        startup_yaw_wait_ms += CONTROL_PERIOD_MS;
        if (startup_yaw_wait_ms > IMU_YAW_STARTUP_STABLE_DELAY_MS)
        {
            startup_yaw_wait_ms = IMU_YAW_STARTUP_STABLE_DELAY_MS;
        }

        return;
    }
    set_motor_output_enabled(1);

    /* 4) 点动保护：点动模式用于核对单轮方向，周期闭环不能覆盖人工 PWM。 */
    if (0 != point_test_enabled)
    {
        return;
    }

#if DRIVE_SPEED_LOOP_TEST_ENABLE
    /* 5) 速度环测试：编译期开关打开后，直接给四轮同目标。
     * 这样 VOFA 曲线只反映速度环本身，不受 IMU 和麦轮混控影响。 */
    update_speed_loop_test_20ms();
    return;
#endif

    if ((0 != imu_ready) && (0 == startup_yaw_locked))
    {
        startup_yaw_locked = 1;
        startup_yaw_wait_ms = IMU_YAW_STARTUP_STABLE_DELAY_MS;
        control_status.target_yaw = calibrate_startup_yaw();
        control_status.current_yaw = control_status.target_yaw;
        control_status.yaw_error = 0.0f;
        attitude_pd_reset(&yaw_pid);

#if DRIVE_ATTITUDE_LOOP_TEST_ENABLE
        /* 原地姿态闭环测试延迟到 yaw 稳定后再设置目标，避免把启动漂移当成真实姿态误差。 */
        set_motion_target(0.0f, 0.0f, control_status.current_yaw + DRIVE_ATTITUDE_LOOP_TEST_TARGET_OFFSET_DEG);
#endif
    }

    /* 6) 姿态环：IMU 初始化成功时由 yaw PD 生成 Vzt 修正分量；
     * 如果 IMU 初始化失败，平移控制继续工作，Vzt 固定为 0。 */
    if (0 != imu_ready)
    {
        control_status.yaw_error = shortest_angle_error(control_status.target_yaw, control_status.current_yaw);
        control_status.vzt = attitude_pd_update(&yaw_pid, control_status.target_yaw, control_status.current_yaw);
    }
    else
    {
        control_status.current_yaw = control_status.target_yaw;
        control_status.yaw_error = 0.0f;
        control_status.vzt = 0.0f;
    }

    /* 7) 麦轮混控：按逐飞公式把 vx/vy/vz/vzt 转成四轮归一化速度。
     * vz 是主动旋转命令，vzt 是姿态闭环修正量，二者在混控层共同作用。 */
    mecanum_mix(control_status.vx,
                control_status.vy,
                control_status.vz,
                control_status.vzt,
                control_status.wheel_norm);

    /* 8) 目标换算：归一化轮速转成速度环使用的 encoder count/20ms 目标。 */
    wheel_targets_from_norm(control_status.wheel_norm, control_status.wheel_target_count);

    /* 9) 空闲目标保护：增量式速度 PID 会保留上一周期累计 PWM。
     * 当四轮目标都为 0 时继续运行 PID，可能把编码器瞬时抖动留下的输出长期保持，
     * 表现为上电或停车后仍有 PWM。静止目标下直接清 PID 和电机输出。 */
    for (i = 0; i < WHEEL_COUNT; i++)
    {
        if ((control_status.wheel_target_count[i] > 0.01f) ||
            (control_status.wheel_target_count[i] < -0.01f))
        {
            has_motion_target = 1;
            break;
        }
    }
    if (0 == has_motion_target)
    {
        for (i = 0; i < WHEEL_COUNT; i++)
        {
            control_status.signed_pwm[i] = 0.0f;
            wheel_pid_reset(&wheel_pid[i]);
        }
        reset_start_pwm_ramp();
        stop_wheels();
        return;
    }

    /* 10) 四轮速度环：每个轮子独立用目标 count 和反馈 count 更新 signed PWM。 */
    for (i = 0; i < WHEEL_COUNT; i++)
    {
        control_status.signed_pwm[i] = wheel_pid_update(&wheel_pid[i],
                                                        control_status.wheel_target_count[i],
                                                        control_status.wheel_feedback_count[i]);
    }
    output_wheel_pwm_with_start_ramp();
}

void set_motion(float vx, float vy)
{
    point_test_enabled = 0;
    control_status.vx = limit_float(vx, -1.0f, 1.0f);
    control_status.vy = limit_float(vy, -1.0f, 1.0f);
    /* 平移接口不携带主动旋转命令，必须清掉上一次 MOTION_TURN_* 留下的 vz，
     * 否则左/右转后再前进会继续边走边转。 */
    control_status.vz = 0.0f;
}

void set_motion_command(motion_command_enum command, float move_speed, float turn_speed)
{
    float vx;
    float vy;
    float manual_vz;

    if (MOTION_STOP == command)
    {
        stop_motion();
        return;
    }

    point_test_enabled = 0;
    command_to_velocity(command, move_speed, turn_speed, &vx, &vy, &manual_vz);
    control_status.vx = limit_float(vx, -1.0f, 1.0f);
    control_status.vy = limit_float(vy, -1.0f, 1.0f);
    manual_vz = limit_float(manual_vz, -1.0f, 1.0f);
    control_status.vz = manual_vz;

    if (0.0f != manual_vz)
    {
        /* 主动转向写入 vz，同时追加目标 yaw 步进给 Vzt 做姿态修正。
         * 这样混控层能形成逐飞图里的 vz + vzt 两个旋转来源。 */
        set_target_yaw(control_status.target_yaw + manual_vz * TURN_STEP_DEG);
    }
}

void set_target_yaw(float yaw)
{
    point_test_enabled = 0;

    while (yaw >= 360.0f)
    {
        yaw -= 360.0f;
    }
    while (yaw < 0.0f)
    {
        yaw += 360.0f;
    }

    control_status.target_yaw = yaw;
    attitude_pd_reset(&yaw_pid);
}

void set_motion_target(float vx, float vy, float yaw_target)
{
    set_motion(vx, vy);
    set_target_yaw(yaw_target);
}

void stop_motion(void)
{
    uint8 i;

    point_test_enabled = 0;
    /* STOP 必须同步撤销旧姿态目标，否则下一次 20ms 更新会再次因
     * yaw 误差生成旋转输出，表现为刚停下又自动回正。 */
    sync_imu_attitude();
    control_status.current_yaw = (0 != imu_ready) ? control_status.current_yaw : control_status.target_yaw;
    control_status.target_yaw = control_status.current_yaw;
    control_status.yaw_error = 0.0f;
    control_status.vx = 0.0f;
    control_status.vy = 0.0f;
    control_status.vz = 0.0f;
    control_status.vzt = 0.0f;
    attitude_pd_reset(&yaw_pid);
    for(i = 0; i < WHEEL_COUNT; i++)
    {
        control_status.signed_pwm[i] = 0.0f;
    }
    reset_start_pwm_ramp();
    stop_wheels();
}

void test_wheel(wheel_enum wheel, float signed_pwm)
{
    if (wheel >= WHEEL_COUNT)
    {
        return;
    }

    control_status.vx = 0.0f;
    control_status.vy = 0.0f;
    control_status.vz = 0.0f;
    control_status.vzt = 0.0f;
    /* 先挂起周期闭环，再输出点动 PWM，避免 ISR 在两步之间把测试输出清零。 */
    point_test_enabled = 1;
    stop_wheels();
    set_wheel_pwm(wheel, limit_float(signed_pwm, -(float)MAX_PWM_DUTY, (float)MAX_PWM_DUTY));
}

const control_status_struct *get_control_status(void)
{
    return &control_status;
}
