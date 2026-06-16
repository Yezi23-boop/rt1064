#ifndef _timebase_h_
#define _timebase_h_

#include "zf_common_typedef.h"

/**
 * @brief 初始化毫秒级软件时间基准。
 *
 * 时间基准来自 GPT_TIM_1 的自由运行计数，用于菜单按键扫描、回放步进和求解耗时统计。
 *
 * @pre 芯片时钟和基础板级初始化已完成。
 *
 * @note 应在菜单、OpenART 和执行器等模块读取 `time_ms()` 前调用一次。
 * @note 本模块独占 GPT_TIM_1；若其它模块复用该定时器会破坏全局超时判断。
 */
void timebase_init(void);

/**
 * @brief 获取系统启动后的毫秒计数。
 *
 * @return 单调递增的毫秒计数，单位为 ms，计数溢出时按无符号差值方式使用。
 *
 * @note 可在主循环和中断上下文读取；调用方比较时间差时应使用无符号减法。
 * @note 不提供实时时钟日期语义，只适合相对超时和周期调度。
 */
uint32 time_ms(void);

#endif
