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

static uint8 motion_is_translating(void)
{
    return ((control_status.vx != 0.0f) || (control_status.vy != 0.0f)) ? 1u : 0u;
}

static void limit_translation_attitude_output(void)
{
    if(0 != motion_is_translating())
    {
        // 平移时限制姿态修正占比，避免 yaw 环为抢角度把横移/前进目标完全压扁。
        control_status.vzt = limit_float(control_status.vzt,
                                         -YAW_TRANSLATION_MAX_VZ,
                                         YAW_TRANSLATION_MAX_VZ);
    }
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

    stop_motion();
    startup_yaw_wait_ms = 0;
    startup_yaw_locked = 0;

    pit_ms_init(PIT_CH1, CONTROL_PERIOD_MS); // 20ms 执行姿态环和四轮速度 PID，与 encoder count 单位一致。

    return hw_state;
}

void update_control_20ms(void)
{
    // 本函数与 executor_update_20ms() 共用 PIT_CH1 节拍：先读反馈/姿态，再决定本周期输出。
    read_encoder_counts(control_status.wheel_feedback_count);
    drive_imu_sync_status(&control_status);

    if (0 != update_startup_guard_20ms())
    {
        return;
    }
    drive_pose_update_20ms(control_status.wheel_feedback_count, control_status.current_yaw);
    set_motor_output_enabled(1);

    if (0 != drive_test_manual_pwm_active())
    {
        // 单轮点动直接写 PWM；闭环继续接管会掩盖接线/死区测试结果。
        return;
    }

    if (0 != drive_test_try_update_speed_loop_20ms(&control_status))
    {
        return;
    }

    drive_imu_update_attitude_20ms(&control_status);
    limit_translation_attitude_output();

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
