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
    EXEC_ERROR_ART_PLAN,  /**< ART 稳定地图重解算失败。 */
    EXEC_ERROR_ART_CENTER, /**< ART 小车中心采样、可信校验或位置修正失败。 */
    EXEC_ERROR_SUBJECT2_CLASS, /**< 科目二分类或绑定失败。 */
    EXEC_ERROR_SUBJECT2_TRACK, /**< 科目二箱子身份无法恢复。 */
    EXEC_ERROR_SUBJECT2_PLAN,  /**< 科目二剩余绑定均不可解。 */
    EXEC_ERROR_SUBJECT2_YAW    /**< 科目二观察转向无法在时限内稳定。 */
} executor_error_enum;

typedef enum {
    EXEC_ART_CENTER_NONE = 0,      /**< 没有足够中心点样本。 */
    EXEC_ART_CENTER_IGNORED,       /**< 中心偏差很小，不修正 pose。 */
    EXEC_ART_CENTER_APPLIED,       /**< 已按融合比例修正 pose。 */
    EXEC_ART_CENTER_REJECTED,      /**< 中心点不可信，没有修正 pose。 */
    EXEC_ART_CENTER_ABNORMAL       /**< 偏差过大，需要 ART 地图确认/重算。 */
} executor_art_center_result_enum;

/** 推箱前车箱二维准备位启动结果。 */
typedef enum {
    EXEC_ART_BOX_PREP_NONE = 0,        /**< 当前没有可启动的推箱准备请求或样本不足。 */
    EXEC_ART_BOX_PREP_STARTED,         /**< 车箱样本有效，二维安全准备位已接管 executor。 */
    EXEC_ART_BOX_PREP_GEOMETRY_ERROR   /**< 小车或箱子中心与当前推箱几何不一致。 */
} executor_art_box_prep_result_enum;

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
 * @param[in] initial_pose_x_cm 小车中心相对起始格中心的 X 偏移，离线地图传 0。
 * @param[in] initial_pose_y_cm 小车中心相对起始格中心的 Y 偏移，离线地图传 0。
 * @param[in] single_step 非 0 表示段间暂停等待人工继续。
 * @param[in] art_sync 非 0 表示单箱任务结束点停稳后等待 ART 稳定地图确认。
 *                    普通移动和任务中途推箱 waypoint 不等待 ART。
 * @note 会在进入 RUNNING/PAUSED 前写入初始偏移；yaw 保留当前 IMU 相对航向。
 */
void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 start_row, uint8 start_col,
                    float initial_pose_x_cm, float initial_pose_y_cm,
                    uint8 single_step,
                    uint8 art_sync);

/**
 * @brief 使用现有20ms位置环移动到当前局部坐标系中的任意 X/Y 目标。
 * @return 1 表示已启动；0 表示 executor 正在运行或暂停，未接管。
 * @note 不重置 pose 和 yaw；发车、返航和科目二观察格回正均复用该接口。
 */
uint8 executor_start_position_correction(float target_x_cm, float target_y_cm);

/**
 * @brief 原子重置局部 pose，并使用现有20ms位置环移动到指定目标。
 * @return 1 表示已启动；0 表示 executor 正在运行或暂停，未接管。
 * @note pose 重置、PID/到点状态清零、目标设置和 RUNNING 切换位于同一临界区。
 */
uint8 executor_start_position_correction_with_pose_reset(
    float initial_x_cm,
    float initial_y_cm,
    float target_x_cm,
    float target_y_cm);

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
 * @brief 是否正在等待 waypoint 执行前的中心矫正。
 * @return 1 表示底盘已在方向转折点停车，主循环必须完成中心矫正后才能放行。
 */
uint8 executor_art_pre_push_pending(void);

/**
 * @brief 当前中心矫正完成后是否还需要执行推箱垂直轴对齐。
 * @return 1 表示当前 waypoint 是大写推箱动作；0 表示普通转折，修正 pose 后直接放行。
 */
uint8 executor_center_requires_push_alignment(void);

/**
 * @brief 中心矫正成功后放行当前 waypoint。
 * @return 1 表示已放行同一个 waypoint；0 表示当前没有中心等待。
 * @note 本函数不推进 waypoint 下标，只允许 20ms executor 开始执行当前动作。
 */
uint8 executor_continue_after_pre_push_center(void);

/**
 * @brief ART 中心校正成功后，启动当前推箱 waypoint 的垂直轴单轴对齐。
 * @param[in] reference_row 当前 ART 地图中 `C` 所在行。
 * @param[in] reference_col 当前 ART 地图中 `C` 所在列。
 * @return 1 表示已接管当前中心等待并开始对齐；0 表示当前状态或动作不允许启动。
 * @note L/R 只对齐世界 Y，U/D 只对齐世界 X；完成后自动放行同一个 waypoint。
 */
uint8 executor_start_pre_push_alignment(uint8 reference_row, uint8 reference_col);

/**
 * @brief 查询当前中心等待是否对应一段推箱链，并取得应观察的箱子格。
 * @return 1 表示应发送 `OBSERVE_REQ row,col`；0 表示仍走普通小车中心校正。
 */
uint8 executor_get_pre_push_box_request(uint8 *box_row, uint8 *box_col);

/** 清空当前推箱准备的车箱配对观察样本。 */
void executor_reset_art_box_observation_samples(void);

/**
 * @brief 缓存一帧配套的小车中心和箱子中心。
 * @return 收满3帧并得到中值时返回1，否则返回0。
 */
uint8 executor_apply_art_box_observation(uint16 car_col_q, uint16 car_row_q,
                                         uint16 box_col_q, uint16 box_row_q);

/** 使用已缓存的3帧中值启动推箱前二维安全准备位。 */
executor_art_box_prep_result_enum executor_start_pre_push_box_preparation(void);

/**
 * @brief 观察超时后沿推箱反方向退开指定距离，完成后仍保持当前推箱等待。
 * @return 1 表示恢复动作已启动；0 表示当前状态不允许启动。
 */
uint8 executor_start_pre_push_box_retry_nudge(float distance_cm);

/** 返回1表示正在执行退开、垂直对齐或靠近阶段。 */
uint8 executor_pre_push_box_preparation_active(void);

/** 返回当前二维准备位短状态：BGap、BAlign、BNear或空字符串。 */
const char *executor_pre_push_box_state_name(void);

/**
 * @brief ART 视觉中心采样窗口是否打开。
 * @return 1 表示正在段末停稳/同步，或正在等待方向转折点中心矫正。
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
 * @brief 缓存一个 ART 视觉中心点样本。
 * @param[in] center_col_q OpenART 视觉中心列坐标，单位 1/100 格。
 * @param[in] center_row_q OpenART 视觉中心行坐标，单位 1/100 格。
 * @param[in] sample_count OpenART 中心点样本序号，用于过滤重复读取。
 * @return 1 表示已经收满配置数量的有效样本并得到中值；0 表示样本不足或重复。
 * @note 这里只缓存中值，不立刻重置 pose；段末或 waypoint 前流程随后决定是否提交。
 */
uint8 executor_apply_art_player_center(uint16 center_col_q, uint16 center_row_q, uint32 sample_count);

/** 清空尚未提交的 ART 中心样本，开始一次独立的多帧请求。 */
void executor_reset_art_player_center_samples(void);

/** @brief 读取已收满样本的 ART 中心中值，不消费待提交状态。 */
uint8 executor_get_art_player_center_median(uint16 *col_q, uint16 *row_q);

/**
 * @brief 把已缓存的 ART 视觉中心中值应用到当前 executor 局部位姿。
 * @param[in] current_car_row 最新 ART 地图中 `C` 所在行。
 * @param[in] current_car_col 最新 ART 地图中 `C` 所在列。
 * @return 视觉中心校正结果；只有 APPLIED 会修改 pose，是否启用由调用流程控制。
 */
executor_art_center_result_enum executor_commit_art_player_center(uint8 current_car_row,
                                                                  uint8 current_car_col);

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
