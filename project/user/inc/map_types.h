#ifndef _map_types_h_
#define _map_types_h_

#include "zf_common_typedef.h"

/** @brief 地图行数，单位为格；当前比赛地图固定为 12 行。 */
#define MAP_ROWS                (12)
/** @brief 地图列数，单位为格；当前比赛地图固定为 16 列。 */
#define MAP_COLS                (16)
/** @brief 地图总格数，用于把 row/col 压成一维 cell 编号。 */
#define MAP_CELLS               (MAP_ROWS * MAP_COLS)
/** @brief 单箱 BFS 状态数，状态由 player cell 和 box cell 组合而成。 */
#define SEARCH_STATE_COUNT             (MAP_CELLS * MAP_CELLS)
/** @brief 无效状态哨兵值；`uint16` 最大值不会和 12x16 地图 cell 冲突。 */
#define INVALID_STATE              (0xFFFF)
/** @brief 当前静态数组支持的最大箱子数量。 */
#define MAX_BOXES               (8)
/** @brief 单次单箱 BFS 动作上限，单位为 action 字符。 */
#define MAX_SINGLE_PATH         (512)
/** @brief 多箱任务合并后的动作上限，单位为 action 字符。 */
#define MAX_TOTAL_ACTIONS       (1024)
/** @brief 屏幕/执行层路径点上限，单位为格点。 */
#define MAX_WAYPOINTS           (256)
/** @brief MCU 内部复合格：箱子暂时站在目标上，OpenART 不直接发送该字符。 */
#define MAP_BOX_ON_TARGET       ('*')
/**
 * @brief 一张离线地图的只读来源。
 *
 * 地图统一使用 RT 字符：`#` 墙，`.` 空地，`B` 箱子，`T` 目标点，`C` 小车，
 * `+` 小车站在目标上，`*` 箱子站在目标上，`X` 炸弹/障碍。
 * 地图内容在固件中以常量表保存，Flash 菜单只保存地图编号和模式，不保存整张地图数据。
 *
 * @note `rows` 指向的每行必须在调用期间保持有效；离线地图使用静态常量，
 * OpenART 或缓存快照需要由调用方提供稳定的行缓存。
 */
typedef struct
{
    const char *name;                   /**< 地图显示名，用于串口或调试输出。 */
    const char *rows[MAP_ROWS];         /**< 12 行地图文本，每行必须恰好 `MAP_COLS` 个字符并以 NUL 结尾。 */
} map_source_struct;

/**
 * @brief 求解器内部使用的地图状态。
 *
 * 该结构是从只读地图解析出的可变状态，BFS 和多箱拆解过程会更新其中的箱子位置。
 * `grid` 只保存静态障碍，动态箱子位置单独放在 `boxes` 中，避免推箱后反复改写字符地图。
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
 *
 * 连续同方向、同类型的动作会被合并成一个 waypoint，以减少屏幕回放和后续底盘路径点数量。
 * `action_start`/`action_end` 保留原动作区间，便于调试时回查 `actions` 中的任务边界。
 */
typedef struct
{
    uint8 row;                          /**< 行号，范围 `[0, MAP_ROWS - 1]`。 */
    uint8 col;                          /**< 列号，范围 `[0, MAP_COLS - 1]`。 */
    char action;                        /**< 到达该点对应的动作字符，小写移动、大写推箱。 */
    uint16 action_start;                /**< 该 waypoint 覆盖的第一个 action 下标，包含前序任务分隔符之后的位置。 */
    uint16 action_end;                  /**< 该 waypoint 覆盖的尾后 action 下标，不包含 `|` 分隔符。 */
    uint8 task_end;                     /**< 1 表示该 waypoint 是当前单箱任务完成点，需要 ART 确认。 */
    uint8 center_correct_before;         /**< 1 表示在方向转折点停车后、执行该 waypoint 前做 ART 中心矫正。 */
    uint8 near_box_axis_lock;           /**< 1 表示普通移动邻近箱子，世界坐标速度必须单轴输出。 */
    uint8 pre_push_extra_gap;           /**< 1 表示推箱准备位后方有空间，可在1.5格距离横向对齐。 */
} waypoint_struct;

/**
 * @brief Push Box 求解输出。
 *
 * `actions` 中用 `|` 分隔多箱拆解任务；屏幕回放会跳过该分隔符，
 * 但它对理解多箱任务边界有用，也避免 waypoint 合并跨过两个单箱任务。
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
