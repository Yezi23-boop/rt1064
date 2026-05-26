#ifndef _isr_h_
#define _isr_h_

/**
 * @file isr.h
 * @brief 用户中断入口声明文件。
 *
 * 实际 ISR 符号由启动文件和 `src/isr.c` 绑定；用户业务代码不应直接调用中断函数。
 * 5ms IMU 更新和 20ms 控制更新的接入关系在 `src/isr.c` 中维护。
 */

#endif
