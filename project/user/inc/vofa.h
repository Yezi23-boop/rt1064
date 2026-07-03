#ifndef _vofa_h_
#define _vofa_h_

/**
 * @file vofa.h
 * @brief VOFA+ 单向调试曲线输出接口。
 *
 * 本模块复用无线串口通过 `printf` 输出 FireWater 文本 CSV。
 * UART1 保留给 OpenART 地图协议，不在这里初始化或占用。
 */

typedef enum
{
    VOFA_OUTPUT_OFF = 0,      /**< 不输出 VOFA 曲线。 */
    VOFA_OUTPUT_CONTROL,      /**< 输出四轮目标/反馈/PWM 和 pose。 */
    VOFA_OUTPUT_POSE,         /**< 只输出 pose 和车体位移。 */
    VOFA_OUTPUT_ART           /**< 输出 ART 跑图定位和同步状态。 */
} vofa_output_mode_enum;

/** VOFA+ 输出模式；现场只改这个宏选择调试曲线组。 */
#define VOFA_OUTPUT_MODE VOFA_OUTPUT_OFF
/** VOFA+ 曲线刷新周期，单位 ms。 */
#define VOFA_SEND_PERIOD_MS (300)

/**
 * @brief 初始化 VOFA+ FireWater 曲线输出节拍。
 *
 * 重置发送节拍。函数不初始化无线串口硬件，
 * 调用前应已执行 `wireless_uart_init()`，并完成 `timebase_init()` 以提供 `time_ms()`。
 *
 * @note 仅在 `app_init()` 中调用一次；不得在 ISR 中调用。
 */
void vofa_init(void);

/**
 * @brief VOFA+ 周期服务入口。
 *
 * 在主循环中调用，函数内部按固定周期限速输出 FireWater 曲线。
 *
 * @note 该函数必须保持非阻塞，曲线发送频率需要低于无线串口实际吞吐能力。
 */
void vofa_service(void);

#endif
