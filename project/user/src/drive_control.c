#include "zf_common_headfile.h"
#include "drive_control.h"
#include "base_io.h"
#include "drive_imu.h"
#include "drive_output.h"
#include "drive_pose.h"
#include "drive_test.h"
#include "motion_math.h"

static control_status_struct control_status; // 20ms ISR 写入，主循环只读显示；跨字段不保证原子快照。
static uint32 startup_yaw_wait_ms = 0;        // 上电 yaw 稳定等待时间，同时作为电机非零输出安全门。
static uint8 startup_yaw_locked = 0;          // 延时结束后只锁定一次 yaw 零点，避免后续重置位姿。
static volatile drive_health_fault_enum drive_health_fault = DRIVE_HEALTH_NONE;

static uint8 drive_feedback_value_valid(float value)
{
    return ((value == value) && (value > -1000000.0f) &&
            (value < 1000000.0f)) ? 1u : 0u;
}

static uint8 drive_feedback_is_valid(void)
{
    uint8 wheel;

    if((0u == drive_feedback_value_valid(control_status.current_yaw)) ||
       (0u == drive_feedback_value_valid(control_status.current_roll)) ||
       (0u == drive_feedback_value_valid(control_status.current_pitch)))
    {
        return 0u;
    }
    for(wheel = 0u; wheel < WHEEL_COUNT; wheel++)
    {
        if(0u == drive_feedback_value_valid(
                       control_status.wheel_feedback_count[wheel]))
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8 motion_is_translating(void)
{
    return ((control_status.vx != 0.0f) || (control_status.vy != 0.0f)) ? 1u : 0u;
}

static void limit_attitude_output(void)
{
    float max_vz = YAW_TURN_MAX_VZ;

    if(0 != motion_is_translating())
    {
        // 平移时限制姿态修正占比，避免 yaw 环为抢角度把横移/前进目标完全压扁。
        max_vz = YAW_TRANSLATION_MAX_VZ;
    }
    control_status.vzt = limit_float(control_status.vzt, -max_vz, max_vz);
}

static uint8 update_startup_guard_20ms(void)
{
    if (startup_yaw_wait_ms < IMU_YAW_STARTUP_STABLE_DELAY_MS)
    {
        // IMU 上电前几秒 yaw 会漂移；此阶段持续锁当前 yaw，并确保底层 PWM 为 0。
        drive_output_clear_motion_outputs(&control_status);
        drive_output_stop(&control_status);
        drive_imu_lock_current_yaw(&control_status);

        startup_yaw_wait_ms += CONTROL_PERIOD_MS;
        if (startup_yaw_wait_ms > IMU_YAW_STARTUP_STABLE_DELAY_MS)
        {
            startup_yaw_wait_ms = IMU_YAW_STARTUP_STABLE_DELAY_MS;
        }
        return 1;
    }

    if (0 == startup_yaw_locked)
    {
        // 稳定窗口结束后，以此刻 yaw 作为位姿零点，后续执行器的局部坐标才有统一参考。
        startup_yaw_locked = 1;
        drive_imu_lock_current_yaw(&control_status);
        drive_pose_reset_origin(control_status.current_yaw);
        drive_test_apply_attitude_target(control_status.current_yaw);
        return 1;
    }

    return 0;
}

uint8 control_init(void)
{
    uint8 hw_state;

    drive_imu_init();
    drive_output_init();
    drive_pose_init();

    hw_state = io_init();
    drive_health_fault = (0u == hw_state) ?
                         DRIVE_HEALTH_NONE : DRIVE_HEALTH_IMU_INIT;

    stop_motion();
    startup_yaw_wait_ms = 0;
    startup_yaw_locked = 0;

    pit_ms_init(PIT_CH1, CONTROL_PERIOD_MS); // 20ms 执行姿态环和四轮速度 PID，与 encoder count 单位一致。

    return hw_state;
}

uint8 control_feedback_update_20ms(void)
{
    // 反馈相位先刷新编码器、姿态和 pose；返回 0 时本周期不推进 executor，也不输出闭环。
    read_encoder_counts(control_status.wheel_feedback_count);
    drive_imu_sync_status(&control_status);
    if((DRIVE_HEALTH_NONE == drive_health_fault) &&
       (0u == drive_feedback_is_valid()))
    {
        drive_health_fault = DRIVE_HEALTH_FEEDBACK_INVALID;
    }
    if(DRIVE_HEALTH_NONE != drive_health_fault)
    {
        drive_output_clear_motion_outputs(&control_status);
        drive_output_stop(&control_status);
        return 0u;
    }

    if (0 != drive_test_manual_pwm_active())
    {
        // 单轮点动直接写 PWM；闭环和 executor 都让出，但保留反馈刷新给屏幕观察。
        return 0;
    }

    if (0 != update_startup_guard_20ms())
    {
        return 0;
    }

    // 启动保护结束后的第一个控制周期：用最新 IMU 读数重新锁一次 yaw。
    // 原因：保护窗口最后一个 lock 发生在 20ms 前，IMU 在这 20ms 内可能继续漂移，
    // 导致 target_yaw 和当前 yaw 在电机使能的同一瞬间存在误差，表现为启动时车体突然摆动。
    if (1 == startup_yaw_locked)
    {
        drive_imu_lock_current_yaw(&control_status);
        startup_yaw_locked = 2;
    }

    drive_pose_update_20ms(control_status.wheel_feedback_count, control_status.current_yaw);
    set_motor_output_enabled(1);
    return 1;
}

void control_output_update_20ms(void)
{
    if (0 != drive_test_try_update_speed_loop_20ms(&control_status))
    {
        return;
    }

    drive_imu_update_attitude_20ms(&control_status);
    limit_attitude_output();

    mecanum_mix(control_status.vx,
                control_status.vy,
                control_status.vz,
                control_status.vzt,
                control_status.wheel_norm);
    wheel_targets_from_norm(control_status.wheel_norm, control_status.wheel_target_count);

    drive_output_update_and_output(&control_status);
}

void set_motion(float vx, float vy)
{
    drive_test_clear_manual_pwm();
    control_status.vx = limit_float(vx, -1.0f, 1.0f);
    control_status.vy = limit_float(vy, -1.0f, 1.0f);
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

    drive_test_clear_manual_pwm();
    command_to_velocity(command, move_speed, turn_speed, &vx, &vy, &manual_vz);
    control_status.vx = limit_float(vx, -1.0f, 1.0f);
    control_status.vy = limit_float(vy, -1.0f, 1.0f);
    control_status.vz = limit_float(manual_vz, -1.0f, 1.0f);

    if (0.0f != control_status.vz)
    {
        drive_imu_set_target_yaw(&control_status, control_status.target_yaw + control_status.vz * TURN_STEP_DEG);
    }
}

void set_target_yaw(float yaw)
{
    drive_test_clear_manual_pwm();
    drive_imu_set_target_yaw(&control_status, yaw);
}

void drive_control_start_relative_yaw_correction(float delta_deg)
{
    uint32 primask = interrupt_global_disable();

    drive_test_clear_manual_pwm();
    drive_imu_sync_status(&control_status);
    drive_output_clear_motion_outputs(&control_status);
    drive_output_reset_and_stop(&control_status);
    drive_imu_set_target_yaw(&control_status,
                             control_status.current_yaw + delta_deg);
    interrupt_global_enable(primask);
}

void drive_control_lock_yaw_and_reset_pose(void)
{
    uint32 primask = interrupt_global_disable();

    drive_test_clear_manual_pwm();
    drive_imu_sync_status(&control_status);
    drive_imu_lock_current_yaw(&control_status);
    drive_output_clear_motion_outputs(&control_status);
    drive_output_reset_and_stop(&control_status);
    drive_pose_reset_origin(control_status.current_yaw);
    interrupt_global_enable(primask);
}

void set_motion_target(float vx, float vy, float yaw_target)
{
    set_motion(vx, vy);
    set_target_yaw(yaw_target);
}

void reset_motion_segment(void)
{
    drive_test_clear_manual_pwm();
    drive_output_clear_motion_outputs(&control_status);
    drive_output_reset_and_stop(&control_status);
}

void stop_motion(void)
{
    drive_test_clear_manual_pwm();
    drive_imu_stop_lock_current_yaw(&control_status);
    drive_output_clear_motion_outputs(&control_status);
    drive_output_reset_and_stop(&control_status);
}

const control_status_struct *get_control_status(void)
{
    return &control_status;
}

uint8 drive_control_is_healthy(void)
{
    return (DRIVE_HEALTH_NONE == drive_health_fault) ? 1u : 0u;
}

drive_health_fault_enum drive_control_get_health_fault(void)
{
    return drive_health_fault;
}
