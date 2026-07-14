#ifndef _map_utils_h_
#define _map_utils_h_

#include "map_types.h"

/**
 * @brief 地图字符统计结果。
 *
 * 该结构只描述 `map_source_struct` 中的原始字符数量，用于菜单提示、
 * OpenART 地图合法性提示和调试输出，不参与 BFS 状态搜索。
 */
typedef struct
{
    uint8 car_count;                   /**< `C/+` 的数量；正常可求解地图应为 1。 */
    uint8 box_count;                   /**< `B` 的数量；应与目标点数量一致。 */
    uint8 target_count;                /**< `T/+` 的数量；当前求解器按一箱一目标拆解。 */
    uint8 car_row;                     /**< 第一个 `C/+` 的行号；没有小车时保持 0。 */
    uint8 car_col;                     /**< 第一个 `C/+` 的列号；没有小车时保持 0。 */
} map_scan_stats_struct;

/**
 * @brief 将二维行列坐标压成一维 cell 编号。
 *
 * @param[in] row 行号，合法范围为 `[0, MAP_ROWS - 1]`。
 * @param[in] col 列号，合法范围为 `[0, MAP_COLS - 1]`。
 * @return 一维 cell 编号，范围为 `[0, MAP_CELLS - 1]`。
 *
 * @note 调用方负责保证坐标在地图内；本函数不做边界钳制，便于 BFS 内层保持轻量。
 */
uint16 map_cell_index(uint8 row, uint8 col);

/**
 * @brief 从一维 cell 编号取行号。
 *
 * @param[in] cell cell 编号，合法范围为 `[0, MAP_CELLS - 1]`。
 * @return 行号，范围为 `[0, MAP_ROWS - 1]`。
 */
uint8 map_cell_row(uint16 cell);

/**
 * @brief 从一维 cell 编号取列号。
 *
 * @param[in] cell cell 编号，合法范围为 `[0, MAP_CELLS - 1]`。
 * @return 列号，范围为 `[0, MAP_COLS - 1]`。
 */
uint8 map_cell_col(uint16 cell);

/**
 * @brief 从地图来源复制 12 行文本。
 *
 * @param[out] dst 目标行缓存，每行容量必须为 `MAP_COLS + 1`。
 * @param[in] source 地图来源，不能为空。
 *
 * @note 复制后每行都会补 `'\0'`，便于屏幕和调试代码按 C 字符串读取。
 */
void map_copy_rows(char dst[MAP_ROWS][MAP_COLS + 1], const map_source_struct *source);

/**
 * @brief 比较行缓存和地图来源是否完全一致。
 *
 * @param[in] rows 待比较的行缓存，必须包含 `MAP_ROWS` 行。
 * @param[in] source 地图来源，不能为空。
 * @return 1 表示所有地图字符一致；0 表示至少一个格子不同。
 */
uint8 map_rows_equal(const char rows[MAP_ROWS][MAP_COLS + 1], const map_source_struct *source);

/**
 * @brief 用外部行缓存构造可读地图快照。
 *
 * @param[out] dst 快照结构，不能为空。
 * @param[out] rows 快照持有的行缓存，生命周期必须覆盖 `dst` 的使用期。
 * @param[in] source 原始地图来源，不能为空。
 *
 * @note `map_source_struct` 只保存行指针，不能指向栈上临时字符串后继续跨函数使用。
 */
void map_source_snapshot(map_source_struct *dst,
                         char rows[MAP_ROWS][MAP_COLS + 1],
                         const map_source_struct *source);

/**
 * @brief 统计地图中的 `C`、`B`、`T` 数量。
 *
 * @param[in] source 地图来源；可为 NULL，NULL 时统计结果清零。
 * @param[out] stats 统计输出，不能为空。
 *
 * @note 该函数不判断墙、炸弹或非法字符，只用于轻量统计和提示。
 */
void map_scan_stats(const map_source_struct *source, map_scan_stats_struct *stats);

/**
 * @brief 查找地图中的唯一小车起点 `C`。
 *
 * @param[in] source 地图来源；可为 NULL，NULL 时返回 0。
 * @param[out] row 小车行号输出，可传 NULL。
 * @param[out] col 小车列号输出，可传 NULL。
 * @param[out] count 地图中 `C` 的数量输出，可传 NULL。
 * @return 1 表示恰好找到一个 `C`；0 表示没有或存在多个 `C`。
 */
uint8 map_find_car(const map_source_struct *source, uint8 *row, uint8 *col, uint8 *count);

#endif
