#ifndef _executor_h_
#define _executor_h_

#include "map_types.h"

/** 执行器外层任务状态；由 20ms 执行器和主循环菜单共同读取。 */
typedef enum {
    EXEC_STATE_IDLE,      /**< 空闲或已急停，不再向底盘发送路径命令。 */
    EXEC_STATE_RUNNING,   /**< 20ms 周期正在推进当前 waypoint。 */
    EXEC_STATE_PAUSED,    /**< 单步模式下段间暂停，等待 K3 恢复。 */
    EXEC_STATE_DONE,      /**< 所有 waypoint 已执行完成。 */
    EXEC_STATE_ERROR      /**< 地图、ART 同步或重解算错误，底盘已停止。 */
} executor_state_enum;

/** 当前箱子执行阶段；保留给显示/调试，现有路径跟踪以 waypoint 为最小单位推进。 */
typedef enum {
    EXEC_STEP_MOVING,     /**< 移动到下一个格子。 */
    EXEC_STEP_PUSHING,    /**< 执行虚拟推箱动作对应的移动段。 */
    EXEC_STEP_RETURNING   /**< 推完后返回下一段起点。 */
} executor_step_enum;

/** 执行器错误类型；错误态下由主循环显示并决定是否重新规划。 */
typedef enum {
    EXEC_ERROR_NONE,      /**< 无错误。 */
    EXEC_ERROR_MAP,       /**< 输入 waypoint 为空或数量非法。 */
    EXEC_ERROR_ART_TIMEOUT, /**< ART 低频重定位等待超时 */
    EXEC_ERROR_ART_SYNC,  /**< ART 稳定地图无效。 */
    EXEC_ERROR_ART_PLAN   /**< ART 稳定地图重解算失败。 */
} executor_error_enum;

typedef enum {
    EXEC_ART_CENTER_NONE = 0,      /**< 没有足够中心点样本。 */
    EXEC_ART_CENTER_IGNORED,       /**< 中心偏差很小，不修正 pose。 */
    EXEC_ART_CENTER_APPLIED,       /**< 已按融合比例修正 pose。 */
    EXEC_ART_CENTER_REJECTED,      /**< 中心点不可信，没有修正 pose。 */
    EXEC_ART_CENTER_ABNORMAL       /**< 偏差过大，需要 ART 地图确认/重算。 */
} executor_art_center_result_enum;

typedef struct {
    float target_x_cm;             /**< 当前 waypoint 目标 X，单位 cm。 */
    float target_y_cm;             /**< 当前 waypoint 目标 Y，单位 cm。 */
    float error_x_cm;              /**< target_x - pose_x，单位 cm。 */
    float error_y_cm;              /**< target_y - pose_y，单位 cm。 */
    float art_center_dx_cm;        /**< 最近一次 ART 中心观测相对本地 pose 的 X 差值。 */
    float art_center_dy_cm;        /**< 最近一次 ART 中心观测相对本地 pose 的 Y 差值。 */
    float art_center_diff_cm;      /**< 最近一次 ART 中心观测与本地 pose 的距离。 */
    uint16 current_step;           /**< 当前 waypoint 下标。 */
    uint16 total_steps;            /**< waypoint 总数。 */
    char action;                   /**< 当前或刚完成 waypoint 动作。 */
    uint8 state;                   /**< executor_state_enum 数值。 */
    uint8 art_sync_pending;        /**< 1 表示正在等待 ART 同步。 */
    uint8 art_center_result;       /**< executor_art_center_result_enum 数值。 */
} executor_debug_status_struct;

/**
 * @brief 初始化执行器路径跟踪 PID 参数。
 * @note 应在主程序初始化阶段调用一次。
 */
void executor_init(void);

/**
 * @brief 启动 waypoint 执行器。
 * @param[in] waypoints 求解器输出的路径点数组；调用期间必须保持有效。
 * @param[in] count 路径点数量，必须大于 0。
 * @param[in] start_row 起始格子行号，用作局部坐标原点。
 * @param[in] start_col 起始格子列号，用作局部坐标原点。
 * @param[in] single_step 非 0 表示段间暂停等待人工继续。
 * @param[in] art_sync 非 0 表示单箱任务结束点停稳后等待 ART 稳定地图确认。
 *                    普通移动和任务中途推箱 waypoint 不等待 ART。
 * @note 会把当前位姿重置为以起始 C 格为原点；yaw 保留当前 IMU 相对航向。
 */
void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 start_row, uint8 start_col, uint8 single_step,
                    uint8 art_sync);

/**
 * @brief 停止执行器（急停）。
 * @note 会调用 `stop_motion()` 并清除当前 waypoint 引用。
 */
void executor_stop(void);

/**
 * @brief 20ms 中断调用，用于推进当前 waypoint 执行状态。
 *
 * @note 仅由 PIT_CH1 ISR 在反馈相位之后调用，因此读取的是本周期最新 pose。
 */
void executor_update_20ms(void);

/**
 * @brief 单步模式下恢复执行（K3 按键调用）。
 * @note 只在 `EXEC_STATE_PAUSED` 生效，恢复前会清除段内 PID 与到点计数。
 */
void executor_resume(void);

/**
 * @brief ART 低频重定位是否正在等待主循环处理。
 * @return 1 表示单箱任务结束点已本地到点停车，等待主循环读取稳定 ART 地图并重解算。
 */
uint8 executor_art_sync_pending(void);

/**
 * @brief ART 段末视觉中心采样窗口是否打开。
 * @return 1 表示当前 waypoint 已到点，正在停稳或等待 ART，同步层可以收集中心样本。
 */
uint8 executor_art_center_sampling_active(void);

/**
 * @brief 获取触发当前 ART 等待的已完成 waypoint 动作。
 * @return 动作字符；正常为大写推箱动作，0 表示当前没有 ART 等待。
 */
char executor_get_art_sync_action(void);

/**
 * @brief ART 段末同步完成后继续当前路径。
 * @return 1 表示已切到下一 waypoint 或任务完成；0 表示当前没有等待 ART。
 */
uint8 executor_continue_after_art_sync(void);

/**
 * @brief 主循环确认任务已完成后，把执行器置为 DONE。
 * @note 用于 ART 重解算后发现无剩余路径的情况；调用时会停止底盘。
 */
void executor_finish_done(void);

/**
 * @brief 主循环确认低频重定位失败后，把执行器置为 ERROR。
 * @param[in] error 错误类型；应为非 `EXEC_ERROR_NONE` 的具体错误。
 * @note 调用时会停止底盘，并清除段内 PID/等待状态。
 */
void executor_set_error(executor_error_enum error);

/**
 * @brief 在 ART 同步段末缓存一个视觉中心点样本。
 * @param[in] center_col_q OpenART 视觉中心列坐标，单位 1/100 格。
 * @param[in] center_row_q OpenART 视觉中心行坐标，单位 1/100 格。
 * @param[in] sample_count OpenART 中心点样本序号，用于过滤重复读取。
 * @return 1 表示已经收满 3 个有效样本并得到中值；0 表示样本不足或重复。
 * @note 这里只缓存中值，不立刻重置 pose；pose 会在 ART 稳定地图重启 executor 后应用。
 */
uint8 executor_apply_art_player_center(uint16 center_col_q, uint16 center_row_q, uint32 sample_count);

/**
 * @brief 把已缓存的 ART 视觉中心中值应用到当前 executor 局部位姿。
 * @return 视觉中心校正结果；只有 APPLIED 会修改 pose。
 */
executor_art_center_result_enum executor_commit_art_player_center(void);

/**
 * @brief 获取当前状态。
 * @return 当前执行器外层状态。
 */
executor_state_enum executor_get_state(void);

/**
 * @brief 获取错误类型。
 * @return 当前错误类型。
 */
executor_error_enum executor_get_error(void);

/**
 * @brief 获取状态名称字符串（用于显示）。
 * @return 静态字符串指针，不需要释放。
 */
const char *executor_state_name(void);

/**
 * @brief 获取当前 waypoint 序号。
 * @return 当前正在执行的 waypoint 下标，从 0 开始。
 */
uint16 executor_get_current_step(void);

/**
 * @brief 获取本次执行总 waypoint 数。
 * @return `executor_start()` 接收的路径点数量。
 */
uint16 executor_get_total_steps(void);

/**
 * @brief 获取 VOFA/屏幕调试用执行器状态快照。
 * @param[out] status 调试状态输出，不能为空。
 */
void executor_get_debug_status(executor_debug_status_struct *status);

#endif /* _executor_h_ */
