#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "vofa.h"
#include "drive_control.h"
#include "motion_math.h"
#include "timebase.h"

/** VOFA+ 曲线刷新周期，单位 ms；与底盘 20ms 控制周期一致，避免重复发送同一帧反馈。 */
#define VOFA_SEND_PERIOD_MS     (20u)
/** 摇杆接管底盘开关；先保持关闭，确认上位机发送格式后再置 1 接入主程序。 */
#define VOFA_JOYSTICK_CONTROL_ENABLE (0)
/** 摇杆命令超时时间，超过该时间未收到新坐标即停车，避免无线链路中断后保持旧速度。 */
#define VOFA_JOYSTICK_TIMEOUT_MS (300u)
/** MaterialJoystick 默认范围为 [-1000, 1000]，中点为 0。 */
#define VOFA_JOYSTICK_MAX_ABS    (1000)
/** 小死区用于过滤摇杆中心回弹和手指轻微抖动。 */
#define VOFA_JOYSTICK_DEADBAND   (30)
/** 单次从无线串口 FIFO 取出的字节数。 */
#define VOFA_RX_CHUNK_SIZE       (32u)
/** 一行摇杆命令的最大缓存长度。 */
#define VOFA_RX_LINE_SIZE        (64u)

/** 主循环写入并读取的发送节拍记录，不在 ISR 中访问。 */
static uint32 vofa_last_send_ms;
/** 最近一次有效摇杆命令的时间，用于无线控制超时停车。 */
static uint32 vofa_last_joystick_ms;
/** 已收到过有效摇杆命令时置位，避免上电未连接 VOFA+ 时反复 stop。 */
static uint8 vofa_joystick_active;
/** 无线串口文本行缓存。 */
static char vofa_rx_line[VOFA_RX_LINE_SIZE];
static uint8 vofa_rx_line_len;

static uint8 vofa_is_digit(char ch)
{
    return (('0' <= ch) && ('9' >= ch)) ? 1 : 0;
}

static uint8 vofa_parse_int_token(const char **cursor, int16 *value)
{
    const char *p = *cursor;
    int32 sign = 1;
    int32 result = 0;
    uint8 has_digit = 0;

    while(('\0' != *p) && ('-' != *p) && ('+' != *p) && (0 == vofa_is_digit(*p)))
    {
        p++;
    }

    if('\0' == *p)
    {
        *cursor = p;
        return 0;
    }

    if('-' == *p)
    {
        sign = -1;
        p++;
    }
    else if('+' == *p)
    {
        p++;
    }

    while(0 != vofa_is_digit(*p))
    {
        result = result * 10 + (*p - '0');
        has_digit = 1;
        p++;
    }

    if('.' == *p)
    {
        p++;
        while(0 != vofa_is_digit(*p))
        {
            p++;
        }
    }

    *cursor = p;
    if(0 == has_digit)
    {
        return 0;
    }

    result *= sign;
    if(result > 32767)
    {
        result = 32767;
    }
    else if(result < -32768)
    {
        result = -32768;
    }

    *value = (int16)result;
    return 1;
}

static int16 vofa_limit_joystick_value(int16 value)
{
    if(value > VOFA_JOYSTICK_MAX_ABS)
    {
        return VOFA_JOYSTICK_MAX_ABS;
    }
    if(value < -VOFA_JOYSTICK_MAX_ABS)
    {
        return -VOFA_JOYSTICK_MAX_ABS;
    }
    if((value >= -VOFA_JOYSTICK_DEADBAND) && (value <= VOFA_JOYSTICK_DEADBAND))
    {
        return 0;
    }
    return value;
}

static void vofa_apply_joystick(int16 x, int16 y)
{
    float vx;
    float vy;

    x = vofa_limit_joystick_value(x);
    y = vofa_limit_joystick_value(y);

    vx = (float)x / (float)VOFA_JOYSTICK_MAX_ABS;
    vy = (float)y / (float)VOFA_JOYSTICK_MAX_ABS;
    set_motion(limit_float(vx, -1.0f, 1.0f), limit_float(vy, -1.0f, 1.0f));

    vofa_last_joystick_ms = time_ms();
    vofa_joystick_active = 1;
}

static void vofa_parse_line(const char *line)
{
    const char *cursor = line;
    const char *colon_cursor = line;
    int16 x;
    int16 y;

    while('\0' != *colon_cursor)
    {
        if(':' == *colon_cursor)
        {
            cursor = colon_cursor + 1;
        }
        colon_cursor++;
    }

    if((0 != vofa_parse_int_token(&cursor, &x)) &&
       (0 != vofa_parse_int_token(&cursor, &y)))
    {
        vofa_apply_joystick(x, y);
    }
}

static void vofa_rx_push_byte(uint8 data)
{
    if(('\r' == data) || ('\n' == data))
    {
        if(0 != vofa_rx_line_len)
        {
            vofa_rx_line[vofa_rx_line_len] = '\0';
            vofa_parse_line(vofa_rx_line);
            vofa_rx_line_len = 0;
        }
        return;
    }

    if(vofa_rx_line_len < (VOFA_RX_LINE_SIZE - 1u))
    {
        vofa_rx_line[vofa_rx_line_len] = (char)data;
        vofa_rx_line_len++;
    }
    else
    {
        vofa_rx_line_len = 0;
    }
}

static void vofa_receive_joystick(void)
{
    uint8 buffer[VOFA_RX_CHUNK_SIZE];
    uint32 length;
    uint32 i;

    do
    {
        length = wireless_uart_read_buffer(buffer, VOFA_RX_CHUNK_SIZE);
        for(i = 0; i < length; i++)
        {
            vofa_rx_push_byte(buffer[i]);
        }
    }while(VOFA_RX_CHUNK_SIZE == length);
}

static void vofa_check_joystick_timeout(uint32 now_ms)
{
    if((0 != vofa_joystick_active) &&
       ((now_ms - vofa_last_joystick_ms) >= VOFA_JOYSTICK_TIMEOUT_MS))
    {
        stop_motion();
        vofa_joystick_active = 0;
        vofa_rx_line_len = 0;
    }
}

/**
 * @brief 初始化 VOFA+ FireWater 曲线发送节拍和摇杆接收状态。
 *
 * 当前 printf 已全局重定向到无线串口，因此本模块只维护发送周期，
 * 不再单独初始化串口，避免和系统启动阶段的无线串口初始化重复。
 *
 * @note 仅在 `app_init()` 中调用一次。
 */
void vofa_init(void)
{
    vofa_last_send_ms = time_ms();
    vofa_last_joystick_ms = vofa_last_send_ms;
    vofa_joystick_active = 0;
    vofa_rx_line_len = 0;
}

/**
 * @brief 按配置处理 VOFA+ MaterialJoystick 控制，并周期输出底盘调试曲线。
 *
 * 摇杆控件未绑定命令时，VOFA+ 会发送类似 `MaterialJoystick:x,y` 的字符串；
 * 绑定命令时建议使用 `JOY:%d,%d\n` 或 `JOY:%f,%f\n`。这里按行提取前两个数字，
 * 映射关系为 X 轴右正、Y 轴上正，对应底盘 `vx` 右正、`vy` 前正。
 * 当前 `VOFA_JOYSTICK_CONTROL_ENABLE` 默认为 0，只输出曲线，不接管底盘。
 *
 * 每 20ms 输出一行文本 CSV，一行就是 VOFA+ FireWater 的一帧数据。
 * 当前字段共 12 个，全部来自 `get_control_status()`：
 *
 * 1. target_lf, target_rf, target_lb, target_rb：四轮目标编码器增量，单位 count/20ms。
 * 2. feedback_lf, feedback_rf, feedback_lb, feedback_rb：四轮实际编码器增量，单位 count/20ms。
 * 3. pwm_lf, pwm_rf, pwm_lb, pwm_rb：四轮 signed PWM 输出，正负号表示电机方向。
 *
 * @note 在主循环调用；不要放到 ISR 中，否则 `printf`/无线串口阻塞发送会影响控制周期。
 */
void vofa_service(void)
{
    const control_status_struct *status;
    uint32 now_ms = time_ms();

#if VOFA_JOYSTICK_CONTROL_ENABLE
    vofa_receive_joystick();
    vofa_check_joystick_timeout(now_ms);
#endif

    if((now_ms - vofa_last_send_ms) < VOFA_SEND_PERIOD_MS)
    {
        return;
    }
    vofa_last_send_ms = now_ms;

    status = get_control_status();

    /* FireWater 没有字段名，VOFA+ 端曲线名需要按下面顺序手动配置：
     * target_lf,target_rf,target_lb,target_rb,
     * feedback_lf,feedback_rf,feedback_lb,feedback_rb,
     * pwm_lf,pwm_rf,pwm_lb,pwm_rb */
    printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        (int16)status->wheel_target_count[WHEEL_LF],
        (int16)status->wheel_target_count[WHEEL_RF],
        (int16)status->wheel_target_count[WHEEL_LB],
        (int16)status->wheel_target_count[WHEEL_RB],
        (int16)status->wheel_feedback_count[WHEEL_LF],
        (int16)status->wheel_feedback_count[WHEEL_RF],
        (int16)status->wheel_feedback_count[WHEEL_LB],
        (int16)status->wheel_feedback_count[WHEEL_RB],
        (int16)status->signed_pwm[WHEEL_LF],
        (int16)status->signed_pwm[WHEEL_RF],
        (int16)status->signed_pwm[WHEEL_LB],
        (int16)status->signed_pwm[WHEEL_RB]);
}
