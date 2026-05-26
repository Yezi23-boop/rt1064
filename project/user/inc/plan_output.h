#ifndef _plan_output_h_
#define _plan_output_h_

#include "map_types.h"

/**
 * @brief 通过串口打印求解得到的动作计划。
 *
 * 该接口只负责调试输出，不改变求解结果、菜单状态或底盘控制目标。
 *
 * @param[in] result 求解结果地址，不能为空；动作数组以 `'\0'` 结尾。
 * @note 当前用于算法验证阶段，输出耗时取决于串口波特率，不应放入 ISR。
 */
void print_plan(const solve_result_struct *result);

#endif
