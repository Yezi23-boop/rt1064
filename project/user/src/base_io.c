#include "zf_common_headfile.h"
#include "base_io.h"
#include "motion_math.h"

/* 硬件数组与 wheel_enum 使用同一轮序。
 * 板测只调整方向符号，不能通过交换数组元素修正接线。 */
static const encoder_index_enum encoder_index[WHEEL_COUNT] =
{
    QTIMER1_ENCODER1,
    QTIMER1_ENCODER2,
    QTIMER2_ENCODER1,
    QTIMER2_ENCODER2,
};

static const encoder_channel1_enum encoder_ch1[WHEEL_COUNT] =
{
    QTIMER1_ENCODER1_CH1_C0,
    QTIMER1_ENCODER2_CH1_C2,
    QTIMER2_ENCODER1_CH1_C3,
    QTIMER2_ENCODER2_CH1_C5,
};

static const encoder_channel2_enum encoder_ch2[WHEEL_COUNT] =
{
    QTIMER1_ENCODER1_CH2_C1,
    QTIMER1_ENCODER2_CH2_C24,
    QTIMER2_ENCODER1_CH2_C4,
    QTIMER2_ENCODER2_CH2_C25,
};

static const pwm_channel_enum motor_pwm_pin[WHEEL_COUNT] =
{
    PWM2_MODULE2_CHB_C11,
    PWM2_MODULE3_CHB_D3,
    PWM2_MODULE1_CHA_C8,
    PWM2_MODULE0_CHA_C6,
};

static const gpio_pin_enum motor_dir_pin[WHEEL_COUNT] =
{
    C10,
    D2,
    C9,
    C7,
};

uint8 io_init(void)
{
    uint8 imu_state;

    // 四个轮子的初始化显式展开，方便按 LF/LB/RF/RB 对照接线和板测现象。
    gpio_init(motor_dir_pin[WHEEL_LF], GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(motor_pwm_pin[WHEEL_LF], PWM_FREQ_HZ, 0);
    encoder_quad_init(encoder_index[WHEEL_LF], encoder_ch1[WHEEL_LF], encoder_ch2[WHEEL_LF]);
    encoder_clear_count(encoder_index[WHEEL_LF]);

    gpio_init(motor_dir_pin[WHEEL_LB], GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(motor_pwm_pin[WHEEL_LB], PWM_FREQ_HZ, 0);
    encoder_quad_init(encoder_index[WHEEL_LB], encoder_ch1[WHEEL_LB], encoder_ch2[WHEEL_LB]);
    encoder_clear_count(encoder_index[WHEEL_LB]);

    gpio_init(motor_dir_pin[WHEEL_RF], GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(motor_pwm_pin[WHEEL_RF], PWM_FREQ_HZ, 0);
    encoder_quad_init(encoder_index[WHEEL_RF], encoder_ch1[WHEEL_RF], encoder_ch2[WHEEL_RF]);
    encoder_clear_count(encoder_index[WHEEL_RF]);

    gpio_init(motor_dir_pin[WHEEL_RB], GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(motor_pwm_pin[WHEEL_RB], PWM_FREQ_HZ, 0);
    encoder_quad_init(encoder_index[WHEEL_RB], encoder_ch1[WHEEL_RB], encoder_ch2[WHEEL_RB]);
    encoder_clear_count(encoder_index[WHEEL_RB]);

    // IMU660RC 与 660RA 在本项目使用同接口同协议；这里直接启用驱动内置 240Hz 四元数解算。
    imu_state = imu660rc_init(IMU660RC_QUARTERNION_240HZ);
    stop_wheels();

    return imu_state;
}

void read_quaternion(void)
{
    /* 第一版统一由 PIT 周期读取 yaw，GPIO INT2 只清中断标志。
     * 保持单一路径可以避免同一周期内重复刷新四元数。 */
    imu660rc_get_quarternion();
}

float read_yaw(void)
{
    return imu660rc_yaw;
}

void read_encoder_counts(float wheel_feedback_count[WHEEL_COUNT])
{
    uint8 i;
    int16 count;

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        count = encoder_get_count(encoder_index[i]);
        encoder_clear_count(encoder_index[i]);
        // 将物理接线方向统一到控制坐标系的正轮速定义。
        wheel_feedback_count[i] = (float)(count * encoder_dir_sign[i]);
    }
}

void set_wheel_pwm(wheel_enum wheel, float signed_pwm)
{
    float corrected_pwm;
    uint32 duty;

    if(wheel >= WHEEL_COUNT)
    {
        return;
    }

    // 电机线序差异在硬件边界收敛，混控层始终只处理统一坐标系。
    corrected_pwm = signed_pwm * (float)motor_dir_sign[wheel];
    corrected_pwm = limit_float(corrected_pwm, -(float)MAX_PWM_DUTY, (float)MAX_PWM_DUTY);

    if(corrected_pwm >= 0.0f)
    {
        gpio_set_level(motor_dir_pin[wheel], GPIO_HIGH);
        duty = (uint32)corrected_pwm;
    }
    else
    {
        gpio_set_level(motor_dir_pin[wheel], GPIO_LOW);
        duty = (uint32)(-corrected_pwm);
    }

    if(duty > PWM_DUTY_MAX)
    {
        duty = PWM_DUTY_MAX;
    }

    pwm_set_duty(motor_pwm_pin[wheel], duty);
}

void stop_wheels(void)
{
    uint8 i;

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        set_wheel_pwm((wheel_enum)i, 0.0f);
    }
}
