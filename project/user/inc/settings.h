#ifndef _settings_h_
#define _settings_h_

#include "zf_common_typedef.h"

/**
 * @brief 菜单运行模式。
 *
 * 模式只决定 Run 页面按 K3 后执行哪类动作，不改变地图来源。
 */
typedef enum
{
    RUN_MODE_SOLVE = 0,        /**< 执行 BFS 并进入屏幕回放。 */
    RUN_MODE_RUN,              /**< 连续执行模式，求解后驱动底盘连续运行。 */
    RUN_MODE_STEP,             /**< 单步执行模式，每步暂停等待 K3 确认。 */
    RUN_MODE_COUNT,            /**< 模式数量，只用于边界检查。 */
} run_mode_enum;

/**
 * @brief 地图来源。
 *
 * 来源决定求解器使用离线地图还是 OpenART 实时地图。
 */
typedef enum
{
    MAP_SOURCE_OFFLINE = 0,    /**< 使用固件内置离线地图。 */
    MAP_SOURCE_ART,            /**< 使用 OpenART UART 接收的实时地图。 */
    MAP_SOURCE_COUNT,          /**< 来源数量，只用于边界检查。 */
} map_source_enum;

/**
 * @brief Flash 中保存项的当前状态。
 *
 * 状态用于屏幕提示，不作为底盘控制的安全条件。
 */
typedef enum
{
    SAVE_STATE_EMPTY = 0,      /**< Flash 中没有可用设置，使用默认地图和模式。 */
    SAVE_STATE_SAVED,          /**< 当前运行设置与 Flash 中保存值一致。 */
    SAVE_STATE_DIRTY,          /**< 当前运行设置已修改，但尚未写入 Flash。 */
    SAVE_STATE_ERROR,          /**< Flash 数据格式非法或读取失败。 */
    SAVE_STATE_CHECK_ERROR,    /**< Flash 数据保护字段不匹配。 */
    SAVE_STATE_WRITE_ERROR,    /**< 最近一次 Flash 写入失败。 */
} save_state_enum;

/**
 * @brief 初始化掉电设置模块。
 *
 * 函数会读取 Flash 中保存的地图编号和运行模式；无有效数据时使用默认值。
 *
 * @note 只在启动阶段调用一次；不会主动写 Flash。
 */
void settings_init(void);

/**
 * @brief 获取当前运行地图编号。
 * @return 当前地图编号，范围会被限制在内置地图数量内。
 */
uint8 settings_get_map(void);

/**
 * @brief 获取当前运行模式。
 * @return 当前菜单运行模式。
 */
run_mode_enum settings_get_mode(void);

/**
 * @brief 获取当前设置保存状态。
 * @return Flash 保存状态，用于屏幕提示。
 */
save_state_enum settings_get_save_state(void);

/**
 * @brief 更新运行时地图编号和模式。
 *
 * @param[in] current_map 当前地图编号。
 * @param[in] run_mode 当前运行模式。
 *
 * @note 该函数只修改 RAM 中的运行设置并标记 Dirty，不立即写 Flash。
 */
void settings_set_runtime(uint8 current_map, run_mode_enum run_mode);

/**
 * @brief 将当前地图编号和运行模式写入 Flash。
 *
 * @return 1 表示写入成功；0 表示写入失败。
 *
 * @note Flash 写入有擦写代价，因此只由 Home 页 K4 短按触发，不在每次切换地图时写入。
 */
uint8 settings_save(void);

/**
 * @brief 获取当前地图来源。
 * @return 当前地图来源。
 */
map_source_enum settings_get_source(void);

/**
 * @brief 更新地图来源。
 *
 * @param[in] source 新来源值。
 *
 * @note 该函数只修改 RAM 中的来源并标记 Dirty，不立即写 Flash。
 */
void settings_set_source(map_source_enum source);

/**
 * @brief 获取运行模式显示名。
 * @param[in] mode 运行模式。
 * @return 常量字符串地址。
 */
const char *mode_name(run_mode_enum mode);

/**
 * @brief 获取地图来源显示名。
 * @param[in] source 地图来源。
 * @return 常量字符串地址。
 */
const char *source_name(map_source_enum source);

/**
 * @brief 获取保存状态显示名。
 * @param[in] state 保存状态。
 * @return 常量字符串地址。
 */
const char *save_state_name(save_state_enum state);

/**
 * @brief 获取 Flash 状态显示名。
 * @param[in] state 保存状态。
 * @return 常量字符串地址。
 */
const char *flash_state_name(save_state_enum state);

#endif
