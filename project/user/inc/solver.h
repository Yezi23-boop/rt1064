#ifndef _solver_h_
#define _solver_h_

#include "map_types.h"

/**
 * @brief 清空 Push Box 求解结果结构。
 *
 * @param[out] result 待清空的结果结构，不能为空。
 * @note 清空后 `solved` 为 0，动作数、路径点数和提示消息都会回到空状态。
 */
void clear_result(solve_result_struct *result);

/**
 * @brief 对一张离线地图执行 Push Box 求解。
 *
 * 求解器按当前实现把多箱任务拆成多次单箱 BFS，不在本接口中驱动真实底盘。
 *
 * @param[in] source 地图源数据，不能为空。
 * @param[out] result 求解结果，包含动作序列、路径点和失败原因。
 * @return 1 表示成功找到完整计划；0 表示地图无法求解或结果容量不足。
 *
 * @note BFS 大数组放在片上 RAM 以保证求解速度；调用方不应在 ISR 中执行该函数。
 */
uint8 solve_map(const map_source_struct *source, solve_result_struct *result);

#endif
