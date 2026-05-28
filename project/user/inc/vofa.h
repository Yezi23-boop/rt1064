#ifndef _vofa_h_
#define _vofa_h_

/**
 * @brief 初始化 VOFA+ FireWater 曲线输出和摇杆控制状态。
 *
 * FireWater 使用文本 CSV，一行代表一帧数据；MaterialJoystick 命令通过无线串口接收。
 */
void vofa_init(void);

/**
 * @brief VOFA+ 周期服务入口。
 *
 * 在主循环中调用，函数内部处理摇杆 RX，并按固定周期限速输出曲线。
 */
void vofa_service(void);

#endif
