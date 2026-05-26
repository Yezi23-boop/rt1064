#include "zf_common_headfile.h"
#include "drive_control.h"
#include "motion_math.h"
#include "wheel_pid.h"
#include "base_io.h"

static control_status_struct control_status;
static attitude_pd_struct yaw_pid;
static wheel_pid_struct wheel_pid[WHEEL_COUNT];
static uint8 control_ready = 0;          // 底盘调度初始化完成后，PIT 回调即可处理编码器和电机。
static uint8 imu_ready = 0;              // IMU 可用时才启用 yaw 姿态保持。
static uint8 point_test_enabled = 0;     // 点动时暂停闭环，保留人工核对轮向的 PWM。

/**
 * @brief 清除四轮速度环累积输出。
 *
 * 停车或切换到点动模式前清除历史状态，避免再次启用闭环时沿用旧 PWM。
 */
static void reset_wheel_pid(void)
{
    uint8 i;

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        wheel_pid_reset(&wheel_pid[i]);
        control_status.signed_pwm[i] = 0.0f;
    }
}

uint8 control_init(void)
{
    uint8 i;
    uint8 hw_state;

    attitude_pd_init(&yaw_pid, YAW_KP, YAW_KD, CONTROL_DT_S);

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        wheel_pid_init(&wheel_pid[i],
            WHEEL_PID_KP,
            WHEEL_PID_KI,
            WHEEL_PID_KD,
            -(float)MAX_PWM_DUTY,
            (float)MAX_PWM_DUTY);
    }

    hw_state = io_init();
    imu_ready = (0 != DRIVE_USE_IMU && 0 == hw_state) ? 1 : 0;
    control_status.current_yaw = (0 != imu_ready) ? read_yaw() : 0.0f;
    control_status.target_yaw = control_status.current_yaw;
    stop_motion();

    pit_ms_init(PIT_CH0, 5);                    // 5ms 只更新 IMU/yaw，避免控制周期内重复读取姿态。
    pit_ms_init(PIT_CH1, CONTROL_PERIOD_MS);    // 20ms 执行姿态环和四轮速度 PID，与 encoder count 单位一致。

    control_ready = 1;
    return hw_state;
}

void update_imu_5ms(void)
{
    if(0 == imu_ready)
    {
        return;
    }

    read_quaternion();
    control_status.current_yaw = read_yaw();
}

void update_control_20ms(void)
{
    uint8 i;
    uint8 all_zero_target = 1;

    read_encoder_counts(control_status.wheel_feedback_count);

    if(0 == control_ready)
    {
        return;
    }
    if(0 != point_test_enabled)
    {
        // 点动输出必须保持人工指定值，不能被周期闭环覆盖。
        return;
    }

    if(0 != imu_ready)
    {
        control_status.current_yaw = read_yaw();
        control_status.vz = attitude_pd_update(&yaw_pid, control_status.target_yaw, control_status.current_yaw);
    }
    else
    {
        control_status.current_yaw = control_status.target_yaw;
        control_status.vz = 0.0f;
    }

    mecanum_mix(control_status.vx, control_status.vy, control_status.vz, control_status.wheel_norm);
    wheel_targets_from_norm(control_status.wheel_norm, control_status.wheel_target_count);

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        if((control_status.wheel_target_count[i] > 0.01f) || (control_status.wheel_target_count[i] < -0.01f))
        {
            all_zero_target = 0;
            break;
        }
    }

    if(0 != all_zero_target)
    {
        reset_wheel_pid();
        stop_wheels();
        return;
    }

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        control_status.signed_pwm[i] = wheel_pid_update(&wheel_pid[i],
            control_status.wheel_target_count[i],
            control_status.wheel_feedback_count[i]);
        set_wheel_pwm((wheel_enum)i, control_status.signed_pwm[i]);
    }
}

void set_motion(float vx, float vy)
{
    point_test_enabled = 0;
    control_status.vx = limit_float(vx, -1.0f, 1.0f);
    control_status.vy = limit_float(vy, -1.0f, 1.0f);
}

void set_motion_command(motion_command_enum command, float move_speed, float turn_speed)
{
    float vx;
    float vy;
    float manual_vz;

    if(MOTION_STOP == command)
    {
        stop_motion();
        return;
    }

    point_test_enabled = 0;
    command_to_velocity(command, move_speed, turn_speed, &vx, &vy, &manual_vz);
    control_status.vx = limit_float(vx, -1.0f, 1.0f);
    control_status.vy = limit_float(vy, -1.0f, 1.0f);
    manual_vz = limit_float(manual_vz, -1.0f, 1.0f);

    if(0.0f != manual_vz)
    {
        /* 离散转向修改目标 yaw，旋转输出仍由姿态 PD 产生；
         * 不直接把上层转向命令写入电机混控。 */
        set_target_yaw(control_status.target_yaw + manual_vz * TURN_STEP_DEG);
    }
}

void set_target_yaw(float yaw)
{
    point_test_enabled = 0;

    while(yaw >= 360.0f)
    {
        yaw -= 360.0f;
    }
    while(yaw < 0.0f)
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
    point_test_enabled = 0;
    /* STOP 必须同步撤销旧姿态目标，否则下一次 20ms 更新会再次因
     * yaw 误差生成旋转输出，表现为刚停下又自动回正。 */
    control_status.current_yaw = (0 != imu_ready) ? read_yaw() : control_status.target_yaw;
    control_status.target_yaw = control_status.current_yaw;
    control_status.vx = 0.0f;
    control_status.vy = 0.0f;
    control_status.vz = 0.0f;
    attitude_pd_reset(&yaw_pid);
    reset_wheel_pid();
    stop_wheels();
}

void test_wheel(wheel_enum wheel, float signed_pwm)
{
    if(wheel >= WHEEL_COUNT)
    {
        return;
    }

    reset_wheel_pid();
    control_status.vx = 0.0f;
    control_status.vy = 0.0f;
    control_status.vz = 0.0f;
    /* 先挂起周期闭环，再输出点动 PWM，避免 ISR 在两步之间把测试输出清零。 */
    point_test_enabled = 1;
    stop_wheels();
    set_wheel_pwm(wheel, limit_float(signed_pwm, -(float)MAX_PWM_DUTY, (float)MAX_PWM_DUTY));
}

const control_status_struct *get_control_status(void)
{
    return &control_status;
}
