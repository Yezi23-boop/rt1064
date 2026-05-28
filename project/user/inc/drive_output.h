#ifndef _drive_output_h_
#define _drive_output_h_

#include "drive_control.h"

/** 初始化四轮速度 PID 和起步 PWM 窗口。 */
void drive_output_init(void);

/** 清除运动分量、四轮目标和当前 signed PWM，不改变 yaw 状态。 */
void drive_output_clear_motion_outputs(control_status_struct *status);

/** 清 signed PWM、重置起步窗口并关闭四轮。 */
void drive_output_stop(control_status_struct *status);

/** 清 signed PWM、清速度 PID、重置起步窗口并关闭四轮。 */
void drive_output_reset_and_stop(control_status_struct *status);

/** 正常链路：四轮速度 PID 更新后经过起步 PWM 窗口输出。 */
void drive_output_update_and_output(control_status_struct *status);

/** 速度环测试：四轮直接使用同一个 count/20ms 目标并输出。 */
void drive_output_run_speed_loop_test(control_status_struct *status, float target_count);

#endif
