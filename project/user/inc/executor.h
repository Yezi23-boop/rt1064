#ifndef _executor_h_
#define _executor_h_

#include "map_types.h"

/** @brief 执行器外层状态（任务级） */
typedef enum {
    EXEC_STATE_IDLE,      /**< 等待启动 */
    EXEC_STATE_RUNNING,   /**< 正在执行 */
    EXEC_STATE_PAUSED,    /**< 单步暂停 */
    EXEC_STATE_DONE,      /**< 全部完成 */
    EXEC_STATE_ERROR      /**< 出错 */
} executor_state_enum;

/** @brief 执行器内层状态（当前箱子执行） */
typedef enum {
    EXEC_STEP_MOVING,     /**< 移动到下一个格子 */
    EXEC_STEP_PUSHING,    /**< 推箱子 */
    EXEC_STEP_RETURNING   /**< 推完后返回起点 */
} executor_step_enum;

/** @brief 执行器错误类型 */
typedef enum {
    EXEC_ERROR_NONE,      /**< 无错误 */
    EXEC_ERROR_MAP,       /**< 地图数据异常 */
    EXEC_ERROR_ART_TIMEOUT, /**< ART 低频重定位等待超时 */
    EXEC_ERROR_ART_SYNC,  /**< ART 稳定地图无效 */
    EXEC_ERROR_ART_PLAN   /**< ART 稳定地图重解算失败 */
} executor_error_enum;

/**
 * @brief 初始化执行器PID参数。
 * @note 应在主程序初始化阶段调用一次。
 */
void executor_init(void);

/**
 * @brief 启动执行器。
 * @param waypoints 求解器输出的路径点数组
 * @param count     路径点数量
 * @param start_row 起始格子行号
 * @param start_col 起始格子列号
 * @param single_step 是否单步模式
 */
void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 start_row, uint8 start_col, uint8 single_step,
                    uint8 art_sync);

/**
 * @brief 停止执行器（急停）。
 */
void executor_stop(void);

/**
 * @brief 20ms 中断调用（在 update_control_20ms() 内调用）。
 */
void executor_update_20ms(void);

/**
 * @brief 单步模式下恢复执行（K3 按键调用）。
 */
void executor_resume(void);

/**
 * @brief ART 低频重定位是否正在等待主循环处理。
 * @return 1 表示当前 waypoint 已本地到点停车，等待主循环读取稳定 ART 地图并重解算。
 */
uint8 executor_art_sync_pending(void);

/**
 * @brief 主循环确认任务已完成后，把执行器置为 DONE。
 */
void executor_finish_done(void);

/**
 * @brief 主循环确认低频重定位失败后，把执行器置为 ERROR。
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
 * @return 状态名称字符串指针。
 */
const char *executor_state_name(void);

/**
 * @brief 获取当前进度信息。
 */
uint16 executor_get_current_step(void);
uint16 executor_get_total_steps(void);

/**
 * @brief 输出执行器调试信息。
 * @note 用于调试路径跟踪PID。
 */
void executor_debug_output(void);

#endif /* _executor_h_ */
