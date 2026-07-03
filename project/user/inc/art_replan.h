#ifndef _art_replan_h_
#define _art_replan_h_

#include "map_types.h"
#include "settings.h"

/**
 * @brief ART 重解算对回放页面状态的建议。
 *
 * 重解算模块只产出状态建议，不直接操作屏幕；菜单层负责把这些值映射到
 * `playback_state_enum`，避免 ART 同步逻辑依赖具体页面实现。
 */
typedef enum
{
    ART_REPLAN_PLAYBACK_KEEP = 0,   /**< 保持菜单层当前回放状态不变。 */
    ART_REPLAN_PLAYBACK_PAUSED,     /**< 新路径已可查看，回放停在第 0 步等待人工确认。 */
    ART_REPLAN_PLAYBACK_DONE,       /**< ART 地图显示任务已完成，不再需要继续求解。 */
    ART_REPLAN_PLAYBACK_FAIL,       /**< 当前稳定帧无法求解，回放页应显示失败信息。 */
} art_replan_playback_enum;

/**
 * @brief ART 重解算所需的菜单/执行器上下文。
 *
 * 结构体内大多数字段是外部状态指针，调用方必须保证这些对象在
 * `art_replan_tick()` 或确认启动期间一直有效。模块会写入求解结果、快照、
 * 耗时和执行起点，但不拥有这些内存。
 */
typedef struct
{
    solve_result_struct *result;          /**< 输出：最近一次 ART 稳定帧的 BFS 求解结果。 */
    map_source_struct *snapshot;          /**< 输出：求解时使用的地图快照描述符。 */
    char (*snapshot_rows)[MAP_COLS + 1];  /**< 输出：12x16 地图文本快照存储区。 */
    uint8 *snapshot_valid;                /**< 输出：非 0 表示 `snapshot` 可用于屏幕和执行器。 */
    uint32 *elapsed_ms;                   /**< 输出：本次求解耗时，单位 ms。 */
    uint8 *start_row;                     /**< 输出：执行器起点行号，范围 0..MAP_ROWS-1。 */
    uint8 *start_col;                     /**< 输出：执行器起点列号，范围 0..MAP_COLS-1。 */
    run_mode_enum run_mode;               /**< 输入：当前运行模式，决定是否单步启动执行器。 */
} art_replan_context_struct;

/**
 * @brief ART 重解算对菜单层的一次性更新请求。
 *
 * 每次调用 `art_replan_tick()` 或 `art_replan_confirm_launch()` 前都会被模块清零；
 * 菜单层应在当轮轮询内消费这些标志，避免旧状态影响下一帧刷新。
 */
typedef struct
{
    uint8 redraw;                         /**< 非 0 表示当前页面需要重绘。 */
    uint8 enter_execute;                  /**< 非 0 表示初次 ART 求解成功后应进入执行页。 */
    uint8 reset_playback_step;            /**< 非 0 表示新路径产生，回放步号应归零。 */
    art_replan_playback_enum playback;    /**< 回放状态建议。 */
    const char *run_state;                /**< 可显示的运行状态文本；NULL 表示不更新。 */
} art_replan_update_struct;

typedef struct
{
    uint8 phase;                          /**< ART 重解算阶段，取内部 art_replan_phase_enum 数值。 */
    uint8 stable_count;                   /**< 当前连续稳定帧计数。 */
    uint8 candidate_valid;                /**< 1 表示已有候选稳定帧。 */
    uint8 confirmed_box_count;            /**< 上一次确认同步后的箱子数量。 */
    uint8 confirmed_target_count;         /**< 上一次确认同步后的目标数量。 */
    uint8 confirmed_counts_valid;         /**< 1 表示 B/T 基线有效。 */
    uint8 launch_pending;                 /**< 1 表示等待 K3 确认启动。 */
} art_replan_debug_status_struct;

/**
 * @brief 取消当前 ART 等待、稳定帧统计和 K3 待确认启动状态。
 *
 * @note 可由菜单安全退出、切换地图或执行器结束路径调用；不会清除最近收到的
 * OpenART 地图，也不会修改已有求解结果。
 */
void art_replan_cancel(void);

/**
 * @brief 开始一次 ART 初始求解等待。
 *
 * @param[out] update 菜单更新请求，可为 NULL；非 NULL 时会请求刷新并显示等待状态。
 *
 * 初始求解用于 ART 地图来源下的执行模式。模块会丢弃 UART 中尚未解析的旧字节，
 * 只接受调用之后到达的新帧，降低使用过期画面的风险。
 */
void art_replan_begin_initial(art_replan_update_struct *update);

/**
 * @brief 推进 ART 重解算状态机。
 *
 * @param[in,out] context 求解、快照、耗时和执行器起点上下文，不能为空。
 * @param[in] art_source_enabled 非 0 表示当前地图来源允许 ART 同步。
 * @param[out] update 本轮菜单更新请求，可为 NULL。
 *
 * @note 必须在主循环中高频调用；函数会读取 OpenART 完整帧计数、等待稳定帧并可能
 * 调用 BFS 求解或启动执行器。不得放入 ISR，因为求解可能耗时。
 */
void art_replan_tick(const art_replan_context_struct *context,
                     uint8 art_source_enabled,
                     art_replan_update_struct *update);

/**
 * @brief 确认初始 ART 求解后的 K3 启动请求。
 *
 * @param[in,out] context 执行器起点和求解结果上下文，不能为空。
 * @param[out] update 菜单更新请求，可为 NULL。
 *
 * 初次稳定帧求解成功后不会立刻发车，而是进入待确认状态；本函数只在用户按 K3
 * 时启动执行器，避免屏幕识别到意外地图后车辆自动运动。
 */
void art_replan_confirm_launch(const art_replan_context_struct *context,
                               art_replan_update_struct *update);

/**
 * @brief 查询是否存在等待 K3 确认的 ART 启动请求。
 *
 * @return 1 表示已有求解结果并等待人工确认启动；0 表示无需确认。
 */
uint8 art_replan_launch_pending(void);

/**
 * @brief 获取 VOFA/屏幕调试用 ART 重解算状态快照。
 * @param[out] status 调试状态输出，不能为空。
 */
void art_replan_get_debug_status(art_replan_debug_status_struct *status);

#endif
