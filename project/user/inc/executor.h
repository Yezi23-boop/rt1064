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
    EXEC_ERROR_TIMEOUT,   /**< 单步超时（3秒） */
    EXEC_ERROR_MAP        /**< 地图数据异常 */
} executor_error_enum;

/**
 * @brief 启动执行器。
 * @param waypoints 求解器输出的路径点数组
 * @param count     路径点数量
 * @param start_row 起始格子行号
 * @param start_col 起始格子列号
 * @param single_step 是否单步模式
 */
void executor_start(const waypoint_struct *waypoints, uint16 count,
                    uint8 start_row, uint8 start_col, uint8 single_step);

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
uint16 executor_get_current_box(void);
uint16 executor_get_total_boxes(void);

#endif /* _executor_h_ */
