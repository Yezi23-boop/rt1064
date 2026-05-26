#ifndef _timebase_h_
#define _timebase_h_

#include "zf_common_typedef.h"

/**
 * @brief 初始化毫秒级软件时间基准。
 *
 * 时间基准由 PIT 周期中断推进，用于菜单按键扫描、回放步进和求解耗时统计。
 *
 * @note 应在 PIT 中断开始工作前调用一次。
 */
void timebase_init(void);

/**
 * @brief 获取系统启动后的毫秒计数。
 *
 * @return 单调递增的毫秒计数，单位为 ms，计数溢出时按无符号差值方式使用。
 *
 * @note 可在主循环和中断上下文读取；调用方比较时间差时应使用无符号减法。
 */
uint32 time_ms(void);

#endif
