#ifndef _screen_h_
#define _screen_h_

#include "map_types.h"
#include "settings.h"

/**
 * @brief 求解结果回放状态。
 */
typedef enum
{
    PLAYBACK_STATE_PAUSED = 0,  /**< 暂停在当前动作步。 */
    PLAYBACK_STATE_PLAYING,     /**< 按固定周期自动推进动作步。 */
    PLAYBACK_STATE_DONE,        /**< 已播放到动作序列末尾。 */
    PLAYBACK_STATE_FAIL,        /**< 当前地图求解失败，只显示失败信息。 */
} playback_state_enum;

/**
 * @brief 初始化 IPS200 屏幕显示参数。
 *
 * 初始化白底黑字、竖屏方向和 8x16 字体，并清空当前屏幕。
 *
 * @note 只在启动阶段调用一次；屏幕刷新由各 `screen_draw_*` 接口完成。
 */
void screen_init(void);

/**
 * @brief 绘制 Home 页面。
 *
 * @param[in] items 一级菜单项文本数组。
 * @param[in] item_count 菜单项数量。
 * @param[in] cursor 当前光标位置。
 * @param[in] current_map 当前已确认地图编号。
 * @param[in] mode 当前运行模式。
 * @param[in] save_state Flash 保存状态。
 */
void screen_draw_home(const char *const *items, uint8 item_count, uint8 cursor, uint8 current_map, run_mode_enum mode, save_state_enum save_state);

/**
 * @brief 绘制 Run 页面。
 *
 * @param[in] current_map 当前已确认地图编号。
 * @param[in] candidate_map 当前候选地图编号，按 K3 后才确认。
 * @param[in] mode 当前运行模式。
 * @param[in] save_state Flash 保存状态。
 * @param[in] state 运行状态文本，指向常量字符串或长期有效内存。
 * @param[in] elapsed_ms 最近一次求解耗时，单位为 ms。
 *
 * @note 该接口只显示状态，不改变候选地图或运行模式。
 */
void screen_draw_run(uint8 current_map, uint8 candidate_map, run_mode_enum mode, save_state_enum save_state, const char *state, uint32 elapsed_ms);

/**
 * @brief 绘制模式选择页面。
 * @param[in] candidate_mode 当前候选模式，按 K3 后才写入运行设置。
 */
void screen_draw_mode_select(run_mode_enum candidate_mode);

/**
 * @brief 绘制 BFS 结果回放页面。
 *
 * @param[in] map_index 当前地图编号，用于标题显示。
 * @param[in] source 地图常量源。
 * @param[in] result 求解结果。
 * @param[in] step 当前回放动作步。
 * @param[in] elapsed_ms 求解耗时，单位为 ms。
 * @param[in] state 回放状态。
 */
void screen_draw_playback(uint8 map_index, const map_source_struct *source, const solve_result_struct *result, uint16 step, uint32 elapsed_ms, playback_state_enum state);

/**
 * @brief 绘制 Demo 批量验证页面。
 *
 * @param[in] index 当前显示的地图序号，面向用户从 1 开始。
 * @param[in] total 内置地图总数。
 * @param[in] ok_count 已求解成功数量。
 * @param[in] fail_count 已求解失败数量。
 * @param[in] elapsed_ms 最近一张地图求解耗时，单位为 ms。
 * @param[in] state Demo 状态文本。
 */
void screen_draw_demo(uint8 index, uint8 total, uint8 ok_count, uint8 fail_count, uint32 elapsed_ms, const char *state);

/**
 * @brief 绘制 Debug 占位页面。
 * @note 当前只保留入口，后续调车功能可在不改 Home 结构的前提下扩展。
 */
void screen_draw_debug(void);

/**
 * @brief 绘制 Info 页面。
 *
 * @param[in] map_count_value 内置地图数量。
 * @param[in] save_state Flash 保存状态。
 */
void screen_draw_info(uint8 map_count_value, save_state_enum save_state);

#endif
