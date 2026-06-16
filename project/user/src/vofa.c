#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "vofa.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "motion_math.h"
#include "timebase.h"

/** VOFA+ 曲线输出开关；遥控时保持低频输出，避免同一无线串口边发曲线边收摇杆导致延迟。 */
#define VOFA_CURVE_OUTPUT_ENABLE (0)
/** VOFA+ 曲线刷新周期，单位 ms；需要边看曲线边遥控时先用低频，避免挤占摇杆 RX。 */
#define VOFA_SEND_PERIOD_MS (100)
/** 位姿调试输出开关；置 1 后 FireWater 只发送 pose_x/y/yaw 和车体本周期 X/Y 位移增量。 */
#define VOFA_POSE_ONLY_ENABLE (0)
/** 摇杆接管底盘开关；先保持关闭，确认上位机发送格式后再置 1 接入主程序。 */
#define VOFA_JOYSTICK_CONTROL_ENABLE (0)
/** 摇杆命令超时时间，超过该时间未收到新坐标即停车，避免无线链路中断后保持旧速度。 */
#define VOFA_JOYSTICK_TIMEOUT_MS (300u)
/** MaterialJoystick 默认范围为 [-1000, 1000]，中点为 0。 */
#define VOFA_JOYSTICK_MAX_ABS (1000)
/** 小死区用于过滤摇杆中心回弹和手指轻微抖动。 */
#define VOFA_JOYSTICK_DEADBAND (30)
/** 单次从无线串口 FIFO 取出的字节数。 */
#define VOFA_RX_CHUNK_SIZE (32u)
/** 一行摇杆命令的最大缓存长度。 */
#define VOFA_RX_LINE_SIZE (64u)

static uint32 vofa_last_send_ms;       // 主循环写入并读取的曲线发送节拍，单位 ms；不在 ISR 中访问。
static uint32 vofa_last_joystick_ms;   // 最近一次有效摇杆命令时间，单位 ms；超过超时窗口后强制停车。
static uint8 vofa_joystick_active;     // 已收到过有效摇杆命令时置 1，避免上电未连接 VOFA+ 时反复 stop。
static char vofa_rx_line[VOFA_RX_LINE_SIZE]; // 无线串口行缓存，只保存一行摇杆文本，不缓存历史命令。
static uint8 vofa_rx_line_len;         // 当前行长度，不含结尾 NUL；溢出时整行丢弃。
static int16 vofa_pending_x;           // 本轮主循环内最新摇杆 X，范围最终钳制到 [-1000, 1000]。
static int16 vofa_pending_y;           // 本轮主循环内最新摇杆 Y，范围最终钳制到 [-1000, 1000]。
static uint8 vofa_pending_valid;       // 1 表示本轮已解析到新坐标，只提交最后一帧以降低遥控滞后。

// 只解析 VOFA 文本协议中的 ASCII 数字；不依赖 ctype，避免不同 C 库 locale/宏实现带来的体积和可移植性问题。
static uint8 vofa_is_digit(char ch)
{
    return (('0' <= ch) && ('9' >= ch)) ? 1 : 0;
}

// MaterialJoystick 可能带控件名前缀或浮点文本，这里提取整数部分即可满足 [-1000, 1000] 摇杆量程。
static uint8 vofa_parse_int_token(const char **cursor, int16 *value)
{
    const char *p = *cursor;
    int32 sign = 1;
    int32 result = 0;
    uint8 has_digit = 0;

    while (('\0' != *p) && ('-' != *p) && ('+' != *p) && (0 == vofa_is_digit(*p)))
    {
        p++;
    }

    if ('\0' == *p)
    {
        *cursor = p;
        return 0;
    }

    if ('-' == *p)
    {
        sign = -1;
        p++;
    }
    else if ('+' == *p)
    {
        p++;
    }

    while (0 != vofa_is_digit(*p))
    {
        result = result * 10 + (*p - '0');
        has_digit = 1;
        p++;
    }

    if ('.' == *p)
    {
        p++;
        while (0 != vofa_is_digit(*p))
        {
            p++;
        }
    }

    *cursor = p;
    if (0 == has_digit)
    {
        return 0;
    }

    result *= sign;
    if (result > 32767)
    {
        result = 32767;
    }
    else if (result < -32768)
    {
        result = -32768;
    }

    *value = (int16)result;
    return 1;
}

static int16 vofa_limit_joystick_value(int16 value)
{
    if (value > VOFA_JOYSTICK_MAX_ABS)
    {
        return VOFA_JOYSTICK_MAX_ABS;
    }
    if (value < -VOFA_JOYSTICK_MAX_ABS)
    {
        return -VOFA_JOYSTICK_MAX_ABS;
    }
    if ((value >= -VOFA_JOYSTICK_DEADBAND) && (value <= VOFA_JOYSTICK_DEADBAND))
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

    // VOFA 坐标归一化后直接映射到底盘速度接口；限幅放在接入点，避免异常上位机数据穿透到底层控制。
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

    // 支持 `MaterialJoystick:x,y` 和 `JOY:x,y` 两种形式；冒号前的控件名不参与数值解析。
    while ('\0' != *colon_cursor)
    {
        if (':' == *colon_cursor)
        {
            cursor = colon_cursor + 1;
        }
        colon_cursor++;
    }

    if ((0 != vofa_parse_int_token(&cursor, &x)) &&
        (0 != vofa_parse_int_token(&cursor, &y)))
    {
        vofa_pending_x = x;
        vofa_pending_y = y;
        vofa_pending_valid = 1;
    }
}

static void vofa_rx_push_byte(uint8 data)
{
    if (('\r' == data) || ('\n' == data))
    {
        if (0 != vofa_rx_line_len)
        {
            vofa_rx_line[vofa_rx_line_len] = '\0';
            vofa_parse_line(vofa_rx_line);
            vofa_rx_line_len = 0;
        }
        return;
    }

    if (vofa_rx_line_len < (VOFA_RX_LINE_SIZE - 1u))
    {
        vofa_rx_line[vofa_rx_line_len] = (char)data;
        vofa_rx_line_len++;
    }
    else
    {
        // 行缓存溢出通常意味着上位机格式错误或丢换行，整行丢弃比截断解析更安全。
        vofa_rx_line_len = 0;
    }
}

static void vofa_receive_joystick(void)
{
    uint8 buffer[VOFA_RX_CHUNK_SIZE];
    uint32 length;
    uint32 i;

    // 一次服务尽量清空当前 FIFO，但每次按固定块读取，避免主循环在无线串口高流量下长期停留。
    do
    {
        length = wireless_uart_read_buffer(buffer, VOFA_RX_CHUNK_SIZE);
        for (i = 0; i < length; i++)
        {
            vofa_rx_push_byte(buffer[i]);
        }
    } while (VOFA_RX_CHUNK_SIZE == length);

    if (0 != vofa_pending_valid)
    {
        // 同一轮主循环只提交最新坐标，减少无线缓存积压造成的遥控滞后。
        vofa_apply_joystick(vofa_pending_x, vofa_pending_y);
        vofa_pending_valid = 0;
    }
}

static void vofa_check_joystick_timeout(uint32 now_ms)
{
    if ((0 != vofa_joystick_active) &&
        ((now_ms - vofa_last_joystick_ms) >= VOFA_JOYSTICK_TIMEOUT_MS))
    {
        // 无线链路断开时旧摇杆值不能继续驱动车；超时后停车并丢弃半行输入。
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
    vofa_pending_valid = 0;
}

/**
 * @brief 按配置处理 VOFA+ MaterialJoystick 控制，并周期输出底盘调试曲线。
 *
 * 摇杆控件未绑定命令时，VOFA+ 会发送类似 `MaterialJoystick:x,y` 的字符串；
 * 绑定命令时建议使用 `JOY:%d,%d\n`，量程为 [-1000, 1000]。这里按行提取前两个整数，
 * 映射关系为 X 轴右正、Y 轴上正，对应底盘 `vx` 右正、`vy` 前正。
 * 当前 `VOFA_JOYSTICK_CONTROL_ENABLE` 为 1 时接管底盘；遥控模式下默认关闭曲线输出，
 * 避免同一无线串口上周期 printf 和摇杆命令互相挤占。
 *
 * 每 `VOFA_SEND_PERIOD_MS` 输出一行文本 CSV，一行就是 VOFA+ FireWater 的一帧数据。
 * `VOFA_POSE_ONLY_ENABLE=0` 时，字段前 12 个来自 `get_control_status()`，后 5 个来自 `drive_pose_get()`：
 *
 * 1. target_lf, target_rf, target_lb, target_rb：VOFA 曲线顺序下的四轮目标编码器增量，单位 count/20ms。
 * 2. feedback_lf, feedback_rf, feedback_lb, feedback_rb：VOFA 曲线顺序下的四轮实际编码器增量，单位 count/20ms。
 * 3. pwm_lf, pwm_rf, pwm_lb, pwm_rb：VOFA 曲线顺序下的四轮 signed PWM 输出，正负号表示电机方向。
 * 4. pose_x_cm, pose_y_cm, pose_yaw_deg：全局位姿。
 * 5. body_vx_cm, body_vy_cm：本周期车体坐标位移增量，单位 cm/20ms。
 *
 * `VOFA_POSE_ONLY_ENABLE=1` 时只输出第 4/5 组，便于 VOFA+ 单独观察定位曲线。
 *
 * @note 在主循环调用；不要放到 ISR 中，否则 `printf`/无线串口阻塞发送会影响控制周期。
 */
void vofa_service(void)
{
#if VOFA_CURVE_OUTPUT_ENABLE
    const control_status_struct *status;
    const drive_pose_struct *pose;
#endif
    uint32 now_ms = time_ms();

#if VOFA_JOYSTICK_CONTROL_ENABLE
    vofa_receive_joystick();
    vofa_check_joystick_timeout(now_ms);
#endif

#if VOFA_CURVE_OUTPUT_ENABLE
    if ((now_ms - vofa_last_send_ms) < VOFA_SEND_PERIOD_MS)
    {
        return;
    }
    vofa_last_send_ms = now_ms;

    status = get_control_status();
    pose = drive_pose_get();

#if VOFA_POSE_ONLY_ENABLE
    printf("%.2f,%.2f,%.1f,%.2f,%.2f\n",
           pose->x_cm,
           pose->y_cm,
           pose->yaw_deg,
           pose->body_vx_cm,
           pose->body_vy_cm);
#else
    /* FireWater 没有字段名，VOFA+ 端曲线名需要按下面顺序手动配置；这里使用 LF/RF/LB/RB 的曲线显示顺序：
     * target_lf,target_rf,target_lb,target_rb,
     * feedback_lf,feedback_rf,feedback_lb,feedback_rb,
     * pwm_lf,pwm_rf,pwm_lb,pwm_rb,
     * pose_x_cm,pose_y_cm,pose_yaw_deg,body_vx_cm,body_vy_cm */
    printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.1f,%.2f,%.2f\n",
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
           (int16)status->signed_pwm[WHEEL_RB],
           pose->x_cm,
           pose->y_cm,
           pose->yaw_deg,
           pose->body_vx_cm,
           pose->body_vy_cm);
#endif
#endif
}
