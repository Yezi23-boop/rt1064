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
 * 动作字符约定为小写 `u/d/l/r` 表示小车移动，大写 `U/D/L/R` 表示推箱动作，
 * 多个单箱任务之间用 `|` 分隔。
 *
 * @param[in] source 地图源数据，不能为空。
 * @param[out] result 求解结果，不能为空；包含动作序列、路径点和失败原因。
 * @return 1 表示成功找到完整计划；0 表示地图无法求解或结果容量不足。
 *
 * @note BFS 大数组放在片上 RAM 以保证求解速度；调用方不应在 ISR 中执行该函数。
 * 该函数会清空并重写 `result`，不会修改 `source` 指向的地图行文本。
 */
uint8 solve_map(const map_source_struct *source, solve_result_struct *result);

/**
 * @brief 求解指定箱子到指定目标的单箱任务。
 * @param[in] source 当前完整字符地图；MCU 内部允许 `*` 表示箱子站在目标上。
 * @param[in] box_cell 必须与地图中某个 `B` 坐标一致。
 * @param[in] target_cell 必须与地图中某个 `T` 坐标一致。
 * @param[out] result 单任务动作和 waypoint；最后一个 waypoint 标记 task_end。
 * @return 1 表示指定配对可解，0 表示地图无效、坐标不存在或无路径。
 */
uint8 solve_bound_box_path(const map_source_struct *source,
                           uint16 box_cell,
                           uint16 target_cell,
                           solve_result_struct *result);

/**
 * @brief 在当前字符地图上规划小车到指定格的最短移动路径。
 * @param[in] source 地图源；`#`、`X`、`B` 不可通行，其余合法格可通行。
 * @param[in] target_row 目标行，范围 0..MAP_ROWS-1。
 * @param[in] target_col 目标列，范围 0..MAP_COLS-1。
 * @param[out] result 输出小写移动动作和合并 waypoint；起终点相同时 waypoint_count 为 0。
 * @return 1 表示目标可达，0 表示地图无效、目标被占用或无路径。
 */
uint8 solve_navigation_path(const map_source_struct *source,
                            uint8 target_row,
                            uint8 target_col,
                            solve_result_struct *result);

#endif
