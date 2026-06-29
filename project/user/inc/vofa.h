#ifndef _vofa_h_
#define _vofa_h_

/**
 * @file vofa.h
 * @brief VOFA+ 调试曲线和无线摇杆应用层接口。
 *
 * 本模块复用无线串口：`printf` 输出 FireWater 文本 CSV，MaterialJoystick 文本命令从同一通道接收。
 * UART1 保留给 OpenART 地图协议，不在这里初始化或占用。
 */

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

/**
 * @brief 初始化 VOFA+ FireWater 曲线输出和摇杆控制状态。
 *
 * 重置发送节拍、摇杆超时计时和无线串口行缓存。函数不初始化无线串口硬件，
 * 调用前应已执行 `wireless_uart_init()`，并完成 `timebase_init()` 以提供 `time_ms()`。
 *
 * @note 仅在 `app_init()` 中调用一次；不得在 ISR 中调用。
 */
void vofa_init(void);

/**
 * @brief VOFA+ 周期服务入口。
 *
 * 在主循环中调用，函数内部按配置读取无线摇杆命令、执行摇杆超时停车，
 * 并按固定周期限速输出 FireWater 曲线。
 *
 * @note 该函数必须保持非阻塞，曲线发送频率需要低于无线串口实际吞吐能力。
 * @warning VOFA 曲线和摇杆共用无线串口；提高输出频率可能增加摇杆延迟，影响手动接管安全性。
 */
void vofa_service(void);

#endif
