#ifndef _map_types_h_
#define _map_types_h_

#include "zf_common_typedef.h"

/** 地图行数，单位为格；当前比赛地图固定为 12 行。 */
#define MAP_ROWS                (12)
/** 地图列数，单位为格；当前比赛地图固定为 16 列。 */
#define MAP_COLS                (16)
/** 地图总格数，用于把 row/col 压成一维 cell 编号。 */
#define MAP_CELLS               (MAP_ROWS * MAP_COLS)
/** 单箱 BFS 状态数，状态由 player cell 和 box cell 组合而成。 */
#define SEARCH_STATE_COUNT             (MAP_CELLS * MAP_CELLS)
/** 无效状态哨兵值；`uint16` 最大值不会和 12x16 地图 cell 冲突。 */
#define INVALID_STATE              (0xFFFF)
/** 当前静态数组支持的最大箱子数量。 */
#define MAX_BOXES               (8)
/** 单次单箱 BFS 动作上限，单位为 action 字符。 */
#define MAX_SINGLE_PATH         (512)
/** 多箱任务合并后的动作上限，单位为 action 字符。 */
#define MAX_TOTAL_ACTIONS       (1024)
/** 屏幕/执行层路径点上限，单位为格点。 */
#define MAX_WAYPOINTS           (256)
/** RT 内部地图格式：使用 C/B/T 等字符描述元素。 */
#define MAP_FORMAT_RT           (0)
/** 逐飞/XSB 风格地图格式：`.` 表示目标点，`$` 表示箱子。 */
#define MAP_FORMAT_SEEKFREE     (1)

/**
 * @brief 一张离线地图的只读来源。
 *
 * 地图内容在固件中以常量表保存，Flash 菜单只保存地图编号和模式，
 * 不保存整张地图数据。
 */
typedef struct
{
    const char *name;                   /**< 地图显示名，用于串口或调试输出。 */
    uint8 format;                       /**< 地图字符格式，取值为 `MAP_FORMAT_RT` 或 `MAP_FORMAT_SEEKFREE`。 */
    const char *rows[MAP_ROWS];         /**< 12 行地图文本，每行至少包含 `MAP_COLS` 个字符。 */
} map_source_struct;

/**
 * @brief 求解器内部使用的地图状态。
 *
 * 该结构是从只读地图解析出的可变状态，BFS 和多箱拆解过程会更新其中的箱子位置。
 */
typedef struct
{
    char grid[MAP_ROWS][MAP_COLS];      /**< 静态墙/空地网格，不包含动态箱子位置。 */
    uint16 player;                      /**< 小车所在 cell 编号。 */
    uint16 boxes[MAX_BOXES];            /**< 箱子所在 cell 编号数组。 */
    uint16 targets[MAX_BOXES];          /**< 目标点所在 cell 编号数组。 */
    uint8 box_count;                    /**< 当前有效箱子数量。 */
    uint8 target_count;                 /**< 当前有效目标点数量。 */
} map_state_struct;

/**
 * @brief 屏幕回放或后续执行层使用的格点动作。
 */
typedef struct
{
    uint8 row;                          /**< 行号，范围 `[0, MAP_ROWS - 1]`。 */
    uint8 col;                          /**< 列号，范围 `[0, MAP_COLS - 1]`。 */
    char action;                        /**< 到达该点对应的动作字符，小写移动、大写推箱。 */
} waypoint_struct;

/**
 * @brief Push Box 求解输出。
 *
 * `actions` 中用 `|` 分隔多箱拆解任务；屏幕回放会跳过该分隔符，
 * 但它对理解多箱任务边界有用。
 */
typedef struct
{
    uint8 solved;                       /**< 1 表示完整地图已求解，0 表示失败或尚未求解。 */
    uint8 task_count;                   /**< 多箱拆解出的单箱任务数量。 */
    uint16 action_count;                /**< `actions` 中有效动作字符数量，不含结尾 `'\0'`。 */
    uint16 waypoint_count;              /**< `waypoints` 中有效路径点数量。 */
    char actions[MAX_TOTAL_ACTIONS + 1];/**< 动作序列，以 `'\0'` 结尾。 */
    waypoint_struct waypoints[MAX_WAYPOINTS]; /**< 动作序列转换得到的格点路径。 */
    char message[48];                   /**< 求解失败或状态提示文本。 */
} solve_result_struct;

#endif
