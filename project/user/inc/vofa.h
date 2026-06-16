#ifndef _vofa_h_
#define _vofa_h_

/**
 * @file vofa.h
 * @brief VOFA+ 调试曲线和无线摇杆应用层接口。
 *
 * 本模块复用无线串口：`printf` 输出 FireWater 文本 CSV，MaterialJoystick 文本命令从同一通道接收。
 * UART1 保留给 OpenART 地图协议，不在这里初始化或占用。
 */

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
