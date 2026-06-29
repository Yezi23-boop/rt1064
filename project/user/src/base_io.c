// =============================================================================
// 底盘硬件边界 — 所有电机、编码器、IMU 的初始化与 I/O。
// 上层（控制/混控/PID）不直接访问硬件，只通过 base_io.h 声明的函数操作。
// =============================================================================

#include "zf_common_headfile.h"
#include "base_io.h"
#include "motion_math.h"

// -----------------------------------------------------------------------------
// 硬件映射 — 逻辑轮位 (0=LF, 1=RF, 2=LB, 3=RB) → 物理引脚
// -----------------------------------------------------------------------------
// 排故：
//   "手转 LF 但 Home 页 RF 的 Enc 在跳" → 把下面 3 组 encoder 数组
//     对应位置的元素一起交换，不要通过 encoder_dir_sign 修正轮位错配。
//   "轮位对了但 Enc 正负号反了" → 只改下方 encoder_dir_sign[] 的 ±1。
//   "轮位对了但电机转反了"       → 只改下方 motor_dir_sign[]   的 ±1。

// —— 编码器 —— (QTIMER 正交解码)
static const encoder_index_enum encoder_index[WHEEL_COUNT] =
    {
        QTIMER2_ENCODER2, // LF
        QTIMER2_ENCODER1, // RF
        QTIMER1_ENCODER1, // LB
        QTIMER1_ENCODER2, // RB
};

static const encoder_channel1_enum encoder_ch1[WHEEL_COUNT] =
    {
        QTIMER2_ENCODER2_CH1_C5, // LF
        QTIMER2_ENCODER1_CH1_C3, // RF
        QTIMER1_ENCODER1_CH1_C0, // LB
        QTIMER1_ENCODER2_CH1_C2, // RB
};

static const encoder_channel2_enum encoder_ch2[WHEEL_COUNT] =
    {
        QTIMER2_ENCODER2_CH2_C25, // LF
        QTIMER2_ENCODER1_CH2_C4,  // RF
        QTIMER1_ENCODER1_CH2_C1,  // LB
        QTIMER1_ENCODER2_CH2_C24, // RB
};

// —— 电机 —— (DRV8701E: 单 PWM + GPIO DIR)
static const pwm_channel_enum motor_pwm_pin[WHEEL_COUNT] =
    {
        PWM2_MODULE3_CHB_D3,  // LF 左前
        PWM2_MODULE2_CHB_C11, // RF 右前
        PWM2_MODULE0_CHA_C6,  // LB 左后
        PWM2_MODULE1_CHA_C8,  // RB 右后
};

static const gpio_pin_enum motor_dir_pin[WHEEL_COUNT] =
    {
        D2,  // LF 左前 — HIGH = 正转
        C10, // RF 右前
        C7,  // LB 左后
        C9,  // RB 右后
};

// -----------------------------------------------------------------------------
// 方向校正 — 接线完成后唯二的调向入口，不要交换上方映射数组来修正方向
// -----------------------------------------------------------------------------

int8 motor_dir_sign[WHEEL_COUNT] =
    {
        1,  // LF
        -1, // RF
        1,  // LB
        -1, // RB
};

int8 encoder_dir_sign[WHEEL_COUNT] =
    {
        -1, // LF
        1,  // RF
        -1, // LB
        1,  // RB
};

// -----------------------------------------------------------------------------
// 安全门 — 上电窗口内强锁零输出，20ms 控制环窗口结束才解禁
// -----------------------------------------------------------------------------

static uint8 motor_output_enabled = 0;

// -----------------------------------------------------------------------------
// 死区补偿 — 小 PWM 抬升到电机最小启动阈值
// -----------------------------------------------------------------------------

static uint16 motor_pwm_deadband_for_wheel(wheel_enum wheel)
{
    static const uint16 deadband[WHEEL_COUNT] =
        {
            MOTOR_PWM_DEADBAND_LF,
            MOTOR_PWM_DEADBAND_RF,
            MOTOR_PWM_DEADBAND_LB,
            MOTOR_PWM_DEADBAND_RB,
        };

    return deadband[wheel];
}

static float apply_motor_pwm_deadband(wheel_enum wheel, float signed_pwm)
{
#if MOTOR_PWM_DEADBAND_ENABLE
    float abs_pwm;
    float deadband_pwm;

    if (0.0f == signed_pwm)
    {
        return 0.0f;
    }

    abs_pwm = signed_pwm;
    if (abs_pwm < 0.0f)
    {
        abs_pwm = -abs_pwm;
    }

    deadband_pwm = (float)motor_pwm_deadband_for_wheel(wheel);
    if (abs_pwm >= deadband_pwm)
    {
        return signed_pwm;
    }

    return (signed_pwm > 0.0f) ? deadband_pwm : -deadband_pwm;
#else
    (void)wheel;
    return signed_pwm;
#endif
}

// =============================================================================
// 公开函数
// =============================================================================

uint8 io_init(void)
{
    uint8 imu_state;

    // 四轮显式展开，方便按 LF/LB/RF/RB 对照接线。
    // DRV8701E：1 路 PWM (17kHz) + 1 路 GPIO DIR (初始 HIGH = 正转)。
    pwm_init(motor_pwm_pin[WHEEL_LF], PWM_FREQ_HZ, 0);
    gpio_init(motor_dir_pin[WHEEL_LF], GPO, GPIO_HIGH, GPO_PUSH_PULL);
    encoder_quad_init(encoder_index[WHEEL_LF], encoder_ch1[WHEEL_LF], encoder_ch2[WHEEL_LF]);
    encoder_clear_count(encoder_index[WHEEL_LF]);

    pwm_init(motor_pwm_pin[WHEEL_LB], PWM_FREQ_HZ, 0);
    gpio_init(motor_dir_pin[WHEEL_LB], GPO, GPIO_HIGH, GPO_PUSH_PULL);
    encoder_quad_init(encoder_index[WHEEL_LB], encoder_ch1[WHEEL_LB], encoder_ch2[WHEEL_LB]);
    encoder_clear_count(encoder_index[WHEEL_LB]);

    pwm_init(motor_pwm_pin[WHEEL_RF], PWM_FREQ_HZ, 0);
    gpio_init(motor_dir_pin[WHEEL_RF], GPO, GPIO_HIGH, GPO_PUSH_PULL);
    encoder_quad_init(encoder_index[WHEEL_RF], encoder_ch1[WHEEL_RF], encoder_ch2[WHEEL_RF]);
    encoder_clear_count(encoder_index[WHEEL_RF]);

    pwm_init(motor_pwm_pin[WHEEL_RB], PWM_FREQ_HZ, 0);
    gpio_init(motor_dir_pin[WHEEL_RB], GPO, GPIO_HIGH, GPO_PUSH_PULL);
    encoder_quad_init(encoder_index[WHEEL_RB], encoder_ch1[WHEEL_RB], encoder_ch2[WHEEL_RB]);
    encoder_clear_count(encoder_index[WHEEL_RB]);

    // IMU660RC 与 660RA 同接口同协议，启用 240Hz 四元数解算。
    imu_state = imu660rc_init(IMU660RC_QUARTERNION_240HZ);
    stop_wheels();

    return imu_state;
}

void set_motor_output_enabled(uint8 enabled)
{
    motor_output_enabled = (0 == enabled) ? 0 : 1;
}

void read_encoder_counts(float wheel_feedback_count[WHEEL_COUNT])
{
    uint8 i;
    int16 count;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        count = encoder_get_count(encoder_index[i]);
        encoder_clear_count(encoder_index[i]);
        wheel_feedback_count[i] = (float)(count * encoder_dir_sign[i]);
    }
}

void set_wheel_pwm_with_deadband(wheel_enum wheel, float signed_pwm, uint8 deadband_enabled)
{
    float corrected_pwm;
    uint32 duty;

    if (wheel >= WHEEL_COUNT)
    {
        return;
    }

    if (0 == motor_output_enabled)
    {
        signed_pwm = 0.0f;
    }

    corrected_pwm = signed_pwm * (float)motor_dir_sign[wheel];
    if (0 != deadband_enabled)
    {
        corrected_pwm = apply_motor_pwm_deadband(wheel, corrected_pwm);
    }
    corrected_pwm = limit_float(corrected_pwm, -(float)MAX_PWM_DUTY, (float)MAX_PWM_DUTY);

    if (corrected_pwm >= 0.0f)
    {
        // 正转
        duty = (uint32)corrected_pwm;
        if (duty > PWM_DUTY_MAX)
        {
            duty = PWM_DUTY_MAX;
        }
        gpio_set_level(motor_dir_pin[wheel], GPIO_HIGH);
        pwm_set_duty(motor_pwm_pin[wheel], duty);
    }
    else
    {
        // 反转
        duty = (uint32)(-corrected_pwm);
        if (duty > PWM_DUTY_MAX)
        {
            duty = PWM_DUTY_MAX;
        }
        gpio_set_level(motor_dir_pin[wheel], GPIO_LOW);
        pwm_set_duty(motor_pwm_pin[wheel], duty);
    }
}

void set_wheel_pwm(wheel_enum wheel, float signed_pwm)
{
    set_wheel_pwm_with_deadband(wheel, signed_pwm, 1u);
}

void stop_wheels(void)
{
    uint8 i;

    for (i = 0; i < WHEEL_COUNT; i++)
    {
        set_wheel_pwm((wheel_enum)i, 0.0f);
    }
}
