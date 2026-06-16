#ifndef _isr_h_
#define _isr_h_

/**
 * @file isr.h
 * @brief 用户中断入口声明文件。
 *
 * 实际 ISR 符号由启动文件和 `src/isr.c` 绑定，本头文件只让工程内用户代码显式包含中断模块。
 * 用户业务代码不应直接调用中断函数，也不应在 ISR 内添加阻塞式串口打印、屏幕刷新或复杂求解。
 *
 * 中断职责：
 * - PIT_CH1：20ms 执行器和底盘闭环更新。
 * - PIT_CH2：5ms 菜单按键扫描。
 * - LPUART1：OpenART 地图帧接收，不能与 `printf` 调试输出混用。
 * - LPUART8：无线串口和 VOFA/调试输出通道。
 * - GPIO IMU：触发 IMU 驱动读取姿态数据。
 */

#endif
