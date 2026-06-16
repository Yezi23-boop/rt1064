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
 * @param[in] art_sync 非 0 表示每段停稳后等待 ART 地图同步并重解算。
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
 * @note 仅由 PIT_CH1 ISR 调用，当前调度顺序是先推进执行器，再执行底盘控制更新。
 */
void executor_update_20ms(void);

/**
 * @brief 单步模式下恢复执行（K3 按键调用）。
 * @note 只在 `EXEC_STATE_PAUSED` 生效，恢复前会清除段内 PID 与到点计数。
 */
void executor_resume(void);

/**
 * @brief ART 低频重定位是否正在等待主循环处理。
 * @return 1 表示当前 waypoint 已本地到点停车，等待主循环读取稳定 ART 地图并重解算。
 */
uint8 executor_art_sync_pending(void);

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

#endif /* _executor_h_ */
