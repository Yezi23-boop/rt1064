#ifndef _screen_h_
#define _screen_h_

#include "map_types.h"
#include "settings.h"
#include "drive_config.h"

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
 * @param[in] encoder_count 四轮编码器反馈增量，单位为 count/20ms，轮序为 LF/LB/RF/RB。
 * @param[in] imu_roll IMU 横滚角，单位为 degree。
 * @param[in] imu_pitch IMU 俯仰角，单位为 degree。
 * @param[in] imu_yaw IMU 航向角，单位为 degree。
 * @param[in] target_yaw 姿态环目标航向角，单位为 degree。
 * @param[in] yaw_error 姿态环最短角度误差，单位为 degree。
 * @param[in] vz 上层命令给出的归一化旋转分量。
 * @param[in] vzt 姿态环输出的归一化旋转修正分量。
 * @param[in] pose_x_cm 全局定位 X 坐标，单位为 cm，右移为正。
 * @param[in] pose_y_cm 全局定位 Y 坐标，单位为 cm，前进为正。
 * @param[in] openart_frame_count OpenART 地图接收成功帧数。
 */
void screen_draw_home(const char *const *items, uint8 item_count, uint8 cursor, uint8 current_map, run_mode_enum mode, save_state_enum save_state, const float encoder_count[WHEEL_COUNT], float imu_roll, float imu_pitch, float imu_yaw, float target_yaw, float yaw_error, float vz, float vzt, float pose_x_cm, float pose_y_cm, uint32 openart_frame_count);

/**
 * @brief 局部刷新 Home 页面动态状态行。
 *
 * 只更新编码器、IMU、Yaw 和定位坐标，不重绘菜单和光标。
 */
void screen_draw_home_status(const float encoder_count[WHEEL_COUNT], float imu_roll, float imu_pitch, float imu_yaw, float target_yaw, float yaw_error, float vz, float vzt, float pose_x_cm, float pose_y_cm, uint32 openart_frame_count);

/**
 * @brief 只刷新列表页光标，不重绘整页。
 *
 * @param[in] previous_cursor 移动前的光标行号，从 0 开始，对应第一条菜单项。
 * @param[in] cursor 移动后的光标行号，从 0 开始，对应第一条菜单项。
 */
void screen_draw_nav_cursor(uint8 previous_cursor, uint8 cursor);

/**
 * @brief 绘制 Run 根菜单页面。
 */
void screen_draw_run_root(const char *const *items, uint8 item_count, uint8 cursor, uint8 current_map, run_mode_enum mode, save_state_enum save_state, const char *state, uint32 elapsed_ms);

/**
 * @brief 绘制 Run/Map 地图选择与预览页面。
 */
void screen_draw_map_select(uint8 current_map, uint8 candidate_map, save_state_enum save_state, const char *state);

/**
 * @brief 绘制 Run/Mode 模式选择页面。
 */
void screen_draw_mode_page(run_mode_enum candidate_mode);

/**
 * @brief 绘制 Run 工作台页面，显示当前地图并允许直接执行。
 */
void screen_draw_run_workbench(uint8 current_map, run_mode_enum mode, save_state_enum save_state, const char *state, uint32 elapsed_ms);

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
 * @brief 绘制 Debug 实时定位地图页面。
 *
 * @param[in] map_index 当前地图编号，用于标题显示。
 * @param[in] source 当前地图常量源，地图中的 `C` 作为定位起点格。
 * @param[in] pose_x_cm 全局定位 X 坐标，单位 cm，右移为正。
 * @param[in] pose_y_cm 全局定位 Y 坐标，单位 cm，前进为正。
 */
void screen_draw_debug(uint8 map_index, const map_source_struct *source, float pose_x_cm, float pose_y_cm);

/**
 * @brief 绘制 Info 页面。
 *
 * @param[in] map_count_value 内置地图数量。
 * @param[in] save_state Flash 保存状态。
 */
void screen_draw_info(uint8 map_count_value, save_state_enum save_state);

#endif
