#include "drive_test.h"
#include "drive_config.h"
#include "drive_control.h"
#include "drive_output.h"
#include "drive_pose.h"
#include "base_io.h"
#include "motion_math.h"
#include "timebase.h"
#include <math.h>

// 这些开关式测试状态只在主循环轮询中写；manual_pwm_active 会被 20ms 控制链路读取。
static uint8 translate_test_started;      // 平移测试是否已发出启动命令，避免主循环重复下发 set_motion_command。
static uint8 translate_test_stopped;      // 平移测试是否已到距离停车，避免反复调用 stop_motion。
static float translate_test_origin_x_cm;  // 平移测试起点 X，单位 cm。
static float translate_test_origin_y_cm;  // 平移测试起点 Y，单位 cm。
static uint8 translate_test_arrival_ticks; // 平移测试连续到点计数，避免瞬时越界就停车。
static uint8 square_test_started;         // 小方形测试是否已经锁定起点并进入第一段。
static uint8 square_test_stopped;         // 小方形测试是否已完成四段或超时停车。
static uint8 square_test_step;            // 当前小方形边序号，0..3 分别对应右、后、左、前。
static uint32 square_test_step_start_ms;  // 当前边开始时间，单位 ms；固定时长和超时保护共用。
static float square_test_origin_x_cm;     // 当前边起点 X，单位 cm；位姿切段模式用来判断边长。
static float square_test_origin_y_cm;     // 当前边起点 Y，单位 cm；位姿切段模式用来判断边长。
static uint8 square_test_arrival_ticks;   // 小方形当前边连续到点计数。
static uint8 wheel_jog_started;           // 单轮点动是否已输出一次人工 PWM。
static uint8 manual_pwm_active;           // 1 表示点动测试占用电机输出，20ms 闭环本周期应让出。
static path_pid_struct test_x_pid;         // 测试用世界 X 位置环；切段/停车时清零。
static path_pid_struct test_y_pid;         // 测试用世界 Y 位置环；切段/停车时清零。

static float drive_test_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void drive_test_reset_position_pid(void)
{
    path_pid_reset(&test_x_pid);
    path_pid_reset(&test_y_pid);
}

static void drive_test_world_velocity_to_body(float vx_world, float vy_world,
                                              float yaw_deg, float *vx_body,
                                              float *vy_body)
{
    float yaw_rad = yaw_deg * 3.1415926f / 180.0f;
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    *vx_body = vx_world * cos_yaw + vy_world * sin_yaw;
    *vy_body = -vx_world * sin_yaw + vy_world * cos_yaw;
}

static uint8 drive_test_axis_arrived(float dx, float dy, float threshold_cm)
{
    return ((drive_test_abs_float(dx) < threshold_cm) &&
            (drive_test_abs_float(dy) < threshold_cm)) ? 1u : 0u;
}

static float drive_test_position_output(path_pid_struct *pid, float error,
                                        float threshold_cm, float max_speed)
{
    float output;

    if(drive_test_abs_float(error) < threshold_cm)
    {
        path_pid_reset(pid);
        return 0.0f;
    }

    output = path_pid_update(pid, error, CONTROL_DT_S);
    return limit_float(output, -max_speed, max_speed);
}

static uint8 drive_test_move_to_target(float target_x_cm, float target_y_cm,
                                       float threshold_cm, uint8 stable_ticks,
                                       float max_speed, uint8 *arrival_ticks)
{
    const drive_pose_struct *pose = drive_pose_get();
    float dx = target_x_cm - pose->x_cm;
    float dy = target_y_cm - pose->y_cm;
    float vx_world;
    float vy_world;
    float vx_body;
    float vy_body;

    if(0 != drive_test_axis_arrived(dx, dy, threshold_cm))
    {
        reset_motion_segment();
        if(*arrival_ticks < stable_ticks)
        {
            (*arrival_ticks)++;
        }
        return (*arrival_ticks >= stable_ticks) ? 1u : 0u;
    }

    *arrival_ticks = 0;
    vx_world = drive_test_position_output(&test_x_pid, dx, threshold_cm, max_speed);
    vy_world = drive_test_position_output(&test_y_pid, dy, threshold_cm, max_speed);
    drive_test_world_velocity_to_body(vx_world, vy_world, pose->yaw_deg,
                                      &vx_body, &vy_body);
    set_motion(vx_body, vy_body);
    return 0;
}

static uint8 drive_translate_test_target(float *target_x_cm, float *target_y_cm)
{
    *target_x_cm = translate_test_origin_x_cm;
    *target_y_cm = translate_test_origin_y_cm;

    switch(DRIVE_TRANSLATE_TEST_COMMAND)
    {
        case MOTION_FORWARD:
            *target_y_cm += DRIVE_TRANSLATE_TEST_DISTANCE_CM;
            return 1u;

        case MOTION_BACKWARD:
            *target_y_cm -= DRIVE_TRANSLATE_TEST_DISTANCE_CM;
            return 1u;

        case MOTION_RIGHT:
            *target_x_cm += DRIVE_TRANSLATE_TEST_DISTANCE_CM;
            return 1u;

        case MOTION_LEFT:
            *target_x_cm -= DRIVE_TRANSLATE_TEST_DISTANCE_CM;
            return 1u;

        default:
            return 0;
    }
}

void drive_test_init(void)
{
    translate_test_started = 0;
    translate_test_stopped = 0;
    translate_test_origin_x_cm = 0.0f;
    translate_test_origin_y_cm = 0.0f;
    translate_test_arrival_ticks = 0;
    square_test_started = 0;
    square_test_stopped = 0;
    square_test_step = 0;
    square_test_step_start_ms = 0;
    square_test_origin_x_cm = 0.0f;
    square_test_origin_y_cm = 0.0f;
    square_test_arrival_ticks = 0;
    wheel_jog_started = 0;
    manual_pwm_active = 0;
    path_pid_init(&test_x_pid, PATH_KP, PATH_KI, PATH_KD, 1.0f, PATH_MAX_INTEGRAL);
    path_pid_init(&test_y_pid, PATH_KP, PATH_KI, PATH_KD, 1.0f, PATH_MAX_INTEGRAL);
}

static void drive_translate_test_poll(void)
{
#if DRIVE_TRANSLATE_TEST_ENABLE
    uint32 now_ms = time_ms();
    const drive_pose_struct *pose;
    float target_x_cm;
    float target_y_cm;

    /* 平移测试按目标点位置环输出，接近目标时自动降速，停车方式贴近 executor。 */
    if((0 == translate_test_started) && (now_ms >= DRIVE_TRANSLATE_TEST_START_MS))
    {
        pose = drive_pose_get();
        translate_test_origin_x_cm = pose->x_cm;
        translate_test_origin_y_cm = pose->y_cm;
        translate_test_arrival_ticks = 0;
        drive_test_reset_position_pid();
        translate_test_started = 1;
    }

    if((0 != translate_test_started) && (0 == translate_test_stopped))
    {
        if(0 == drive_translate_test_target(&target_x_cm, &target_y_cm))
        {
            stop_motion();
            translate_test_stopped = 1;
            return;
        }
        if(0 != drive_test_move_to_target(target_x_cm,
                                          target_y_cm,
                                          DRIVE_TRANSLATE_TEST_ARRIVAL_THRESHOLD_CM,
                                          DRIVE_TRANSLATE_TEST_ARRIVAL_STABLE_TICKS,
                                          DRIVE_TRANSLATE_TEST_SPEED,
                                          &translate_test_arrival_ticks))
        {
            stop_motion();
            translate_test_stopped = 1;
        }
    }
#endif
}

static void drive_square_test_capture_origin(uint32 now_ms)
{
    const drive_pose_struct *pose = drive_pose_get();

    square_test_origin_x_cm = pose->x_cm;
    square_test_origin_y_cm = pose->y_cm;
    square_test_step_start_ms = now_ms;
    square_test_arrival_ticks = 0;
    drive_test_reset_position_pid();
}

static void drive_square_test_target(uint8 step, float *target_x_cm, float *target_y_cm)
{
    switch(step)
    {
        case 0:
            *target_x_cm = square_test_origin_x_cm + DRIVE_SQUARE_TEST_SIDE_CM;
            *target_y_cm = square_test_origin_y_cm;
            break;

        case 1:
            *target_x_cm = square_test_origin_x_cm + DRIVE_SQUARE_TEST_SIDE_CM;
            *target_y_cm = square_test_origin_y_cm - DRIVE_SQUARE_TEST_SIDE_CM;
            break;

        case 2:
            *target_x_cm = square_test_origin_x_cm;
            *target_y_cm = square_test_origin_y_cm - DRIVE_SQUARE_TEST_SIDE_CM;
            break;

        default:
            *target_x_cm = square_test_origin_x_cm;
            *target_y_cm = square_test_origin_y_cm;
            break;
    }
}

static void drive_square_test_next_step(uint32 now_ms)
{
    square_test_step++;
    if(4u <= square_test_step)
    {
        stop_motion();
        square_test_stopped = 1;
        return;
    }

    square_test_step_start_ms = now_ms;
    square_test_arrival_ticks = 0;
    drive_test_reset_position_pid();
}

static void drive_square_test_poll(void)
{
#if DRIVE_SQUARE_TEST_ENABLE
    uint32 now_ms = time_ms();
    float target_x_cm;
    float target_y_cm;

    if(0 != square_test_stopped)
    {
        return;
    }

    if(0 == square_test_started)
    {
        if(now_ms < DRIVE_SQUARE_TEST_START_MS)
        {
            return;
        }
        square_test_started = 1;
        square_test_step = 0;
        drive_square_test_capture_origin(now_ms);
        return;
    }

    drive_square_test_target(square_test_step, &target_x_cm, &target_y_cm);
    if(0 != drive_test_move_to_target(target_x_cm,
                                      target_y_cm,
                                      DRIVE_SQUARE_TEST_ARRIVAL_THRESHOLD_CM,
                                      DRIVE_SQUARE_TEST_ARRIVAL_STABLE_TICKS,
                                      DRIVE_SQUARE_TEST_SPEED,
                                      &square_test_arrival_ticks))
    {
        drive_square_test_next_step(now_ms);
        return;
    }

#endif
}

static void drive_wheel_jog_poll(void)
{
#if DRIVE_WHEEL_JOG_ENABLE
    uint32 now_ms = time_ms();

    if((0 == wheel_jog_started) && (now_ms >= DRIVE_WHEEL_JOG_START_MS))
    {
        // 点动模式绕过速度环，便于确认电机线序、方向和真实死区。
        test_wheel(DRIVE_WHEEL_JOG_WHEEL, DRIVE_WHEEL_JOG_PWM);
        wheel_jog_started = 1;
    }
#endif
}

void drive_test_poll(void)
{
    drive_translate_test_poll();
    drive_square_test_poll();
    drive_wheel_jog_poll();
}

uint8 drive_test_manual_pwm_active(void)
{
    return manual_pwm_active;
}

void drive_test_clear_manual_pwm(void)
{
    manual_pwm_active = 0;
}

uint8 drive_test_try_update_speed_loop_20ms(control_status_struct *status)
{
#if DRIVE_SPEED_LOOP_TEST_ENABLE
    drive_output_run_speed_loop_test(status, DRIVE_SPEED_LOOP_TEST_TARGET_COUNT);
    return 1;
#else
    (void)status;
    return 0;
#endif
}

void drive_test_apply_attitude_target(float current_yaw)
{
#if DRIVE_ATTITUDE_LOOP_TEST_ENABLE
    /* 姿态测试目标必须等 yaw 稳定锁定后再给，避免把上电漂移当作真实偏差。 */
    set_motion_target(0.0f,
                      0.0f,
                      current_yaw + DRIVE_ATTITUDE_LOOP_TEST_TARGET_OFFSET_DEG);
#else
    (void)current_yaw;
#endif
}

void test_wheel(wheel_enum wheel, float signed_pwm)
{
    if(wheel >= WHEEL_COUNT)
    {
        return;
    }

    stop_motion();
    manual_pwm_active = 1;
    set_motor_output_enabled(1);
    set_wheel_pwm(wheel, limit_float(signed_pwm, -(float)MAX_PWM_DUTY, (float)MAX_PWM_DUTY));
}
