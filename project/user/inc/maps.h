#ifndef _maps_h_
#define _maps_h_

#include "map_types.h"

/**
 * @brief 获取当前固件内置离线地图数量。
 *
 * 地图由 `maps.c` 中的常量表提供，启动后不从 Flash 动态加载地图内容。
 *
 * @return 可通过 `map_get()` 访问的地图数量。
 */
uint8 map_count(void);

/**
 * @brief 按编号获取一张离线地图。
 *
 * @param[in] index 地图编号，合法范围为 `[0, map_count() - 1]`。
 * @return 地图常量表中的只读地址；越界时由实现层钳制到有效地图。
 *
 * @note 返回值指向静态常量数据，调用方不得修改。
 */
const map_source_struct *map_get(uint8 index);

#endif
