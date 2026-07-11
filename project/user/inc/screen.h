#ifndef _screen_h_
#define _screen_h_

#include "map_types.h"
#include "settings.h"
#include "executor.h"

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
 * @brief Home 页面显示所需的只读视图数据。
 *
 * 屏幕层只读取这些字段，不回写业务状态；菜单层负责从设置、控制器和定位模块组装。
 */
typedef struct
{
    const char *const *items;   /**< 菜单项文本数组，长度为 `item_count`。 */
    uint8 item_count;           /**< 菜单项数量，必须与 `items` 匹配。 */
    uint8 cursor;               /**< 当前光标行号，范围 0..item_count-1。 */
    uint8 current_map;          /**< 当前内置地图索引，从 0 开始。 */
    run_mode_enum mode;         /**< 当前运行模式。 */
    save_state_enum save_state; /**< Flash 保存状态，用于提示是否已落盘。 */
    const float *encoder_count; /**< 四轮编码器反馈计数数组，顺序为 `wheel_index_enum`。 */
    float imu_yaw;              /**< 当前 IMU yaw，单位 degree。 */
    float target_yaw;           /**< 姿态环目标 yaw，单位 degree。 */
    float yaw_error;            /**< yaw 误差，单位 degree。 */
    float vz;                   /**< 车体自转速度反馈/命令显示值，沿用控制模块单位。 */
    float vzt;                  /**< 车体自转目标显示值，沿用控制模块单位。 */
    float pose_x_cm;            /**< 全局定位 X，单位 cm，右移为正。 */
    float pose_y_cm;            /**< 全局定位 Y，单位 cm，前进为正。 */
    uint32 openart_frame_count; /**< OpenART 成功接收的完整地图帧数。 */
} screen_home_view_struct;

/**
 * @brief Run 工作台页面显示所需的只读视图数据。
 */
typedef struct
{
    uint8 current_map;                    /**< 当前内置地图索引，从 0 开始。 */
    run_mode_enum mode;                   /**< 当前运行模式。 */
    map_source_enum source_type;          /**< 当前地图来源：离线或 OpenART。 */
    save_state_enum save_state;           /**< Flash 保存状态。 */
    const char *state_text;               /**< 菜单层生成的运行状态短文本。 */
    uint32 elapsed_ms;                    /**< 最近一次求解耗时，单位 ms。 */
    const map_source_struct *source;      /**< 当前要显示的地图源，可为 NULL。 */
    const solve_result_struct *result;    /**< 最近一次求解结果，可未求解。 */
    uint8 executor_active;                /**< 非 0 表示执行器正在执行、暂停、完成或报错。 */
    executor_state_enum executor_state;   /**< 执行器状态。 */
    executor_error_enum executor_error;   /**< 执行器错误码，仅 ERROR 状态下重点显示。 */
    uint16 current_step;                  /**< 当前 waypoint 步号。 */
    uint16 total_steps;                   /**< waypoint 总数。 */
    float pose_x_cm;                      /**< 全局定位 X，单位 cm，右移为正。 */
    float pose_y_cm;                      /**< 全局定位 Y，单位 cm，前进为正。 */
} screen_run_view_struct;

/**
 * @brief Run/Step 执行页面显示所需的只读视图数据。
 *
 * `start_row`/`start_col` 是执行开始时的格点，屏幕用它把连续 pose 换算到当前格子；
 * OpenART 车格字段用于对照屏幕再识别结果和 MCU 本地定位。
 */
typedef struct
{
    uint8 current_map;                  /**< 当前内置地图索引，从 0 开始，仅用于标题显示。 */
    const map_source_struct *source;    /**< 执行时使用的地图快照，可为 NULL。 */
    const solve_result_struct *result;  /**< 执行路径对应的求解结果。 */
    const char *state_text;             /**< 菜单层运行状态，发车/等图阶段优先显示。 */
    uint16 current_step;                /**< 当前 waypoint 步号。 */
    executor_state_enum state;          /**< 执行器状态。 */
    executor_error_enum error;          /**< 执行器错误码。 */
    uint8 start_row;                    /**< 执行起点行号，范围 0..MAP_ROWS-1。 */
    uint8 start_col;                    /**< 执行起点列号，范围 0..MAP_COLS-1。 */
    float pose_x_cm;                    /**< 相对执行起点的 X 位移，单位 cm，右移为正。 */
    float pose_y_cm;                    /**< 相对执行起点的 Y 位移，单位 cm，前进为正。 */
    uint8 art_launch_pending;           /**< 非 0 表示等待 K3 人工确认后再发车。 */
    uint8 art_player_enabled;           /**< 非 0 表示当前显示 ART 再识别车格。 */
    uint8 art_player_valid;             /**< 非 0 表示 ART 地图中恰好有一个 `C`。 */
    uint8 art_player_count;             /**< ART 地图中 `C` 的数量，用于诊断识别歧义。 */
    uint8 art_row;                      /**< ART 识别到的车格行号，仅 `art_player_valid` 时可信。 */
    uint8 art_col;                      /**< ART 识别到的车格列号，仅 `art_player_valid` 时可信。 */
} screen_execute_view_struct;

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
 * @param[in] view Home 页面视图数据，不能为空。
 *
 * @note 本函数会绘制完整页面；动态数据可用 `screen_draw_home_status()` 局部刷新。
 */
void screen_draw_home(const screen_home_view_struct *view);

/**
 * @brief 局部刷新 Home 页面动态状态行。
 *
 * 只更新编码器、IMU、Yaw 和定位坐标，不重绘菜单和光标。
 *
 * @param[in] view Home 页面视图数据，不能为空。
 */
void screen_draw_home_status(const screen_home_view_struct *view);

/**
 * @brief 只刷新列表页光标，不重绘整页。
 *
 * @param[in] previous_cursor 移动前的光标行号，从 0 开始，对应第一条菜单项。
 * @param[in] cursor 移动后的光标行号，从 0 开始，对应第一条菜单项。
 */
void screen_draw_nav_cursor(uint8 previous_cursor, uint8 cursor);

/**
 * @brief 绘制 Run/Mode 模式选择页面。
 *
 * @param[in] candidate_mode 当前候选运行模式，范围 0..RUN_MODE_COUNT-1。
 */
void screen_draw_mode_page(run_mode_enum candidate_mode);

/**
 * @brief 绘制 Run 工作台页面，显示当前地图并允许直接执行。
 *
 * @param[in] view Run 页面视图数据，不能为空。
 */
void screen_draw_run_workbench(const screen_run_view_struct *view);

/**
 * @brief 绘制 BFS 结果回放页面。
 *
 * @param[in] map_index 当前地图编号，用于标题显示。
 * @param[in] source 地图常量源。
 * @param[in] result 求解结果。
 * @param[in] step 当前回放动作步，范围 0..result->action_count。
 * @param[in] elapsed_ms 求解耗时，单位为 ms。
 * @param[in] state 回放状态。
 *
 * @note `result->actions` 中的 `|` 是任务分隔符，屏幕会在步数显示中隐藏它。
 */
void screen_draw_playback(uint8 map_index, const map_source_struct *source, const solve_result_struct *result, uint16 step, uint32 elapsed_ms, playback_state_enum state);

/**
 * @brief 绘制 Debug 实时定位地图页面。
 *
 * @param[in] map_index 当前地图编号，用于标题显示。
 * @param[in] source 当前地图常量源，地图中的 `C` 作为定位起点格。
 * @param[in] pose_x_cm 全局定位 X 坐标，单位 cm，右移为正。
 * @param[in] pose_y_cm 全局定位 Y 坐标，单位 cm，前进为正。
 *
 * @note 该页面只做定位可视化，不改变执行器状态。
 */
void screen_draw_debug(uint8 map_index, const map_source_struct *source, float pose_x_cm, float pose_y_cm);

/**
 * @brief 绘制 ART Map 页面，显示 OpenART 接收的实时地图。
 *
 * @param[in] frame_count 成功接收的完整地图帧数。
 * @param[in] age_s 距最近一次解析到 OpenART 文本行的秒数；若从未解析到则显示 "-"。
 * @param[in] source OpenART 地图数据源；若为 NULL 则显示 "No OpenART map"。
 *
 * @note 显示的是最近完整帧快照，不代表 UART 环形缓冲中没有更新的半帧数据。
 */
void screen_draw_art_map(uint32 frame_count, uint32 age_s, const map_source_struct *source);

/**
 * @brief 绘制 Info 页面。
 *
 * @param[in] map_count_value 内置地图数量。
 * @param[in] save_state Flash 保存状态。
 */
void screen_draw_info(uint8 map_count_value, save_state_enum save_state);

/**
 * @brief 绘制 Run/Step 执行页面
 *
 * @param[in] view 执行页面视图数据，不能为空。
 *
 * @note 页面会同时显示 MCU 本地 pose 推算格子和 ART 再识别格子，便于发现定位漂移。
 */
void screen_draw_execute(const screen_execute_view_struct *view);

#endif
