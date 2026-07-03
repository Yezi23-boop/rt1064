#include "zf_common_headfile.h"
#include "vofa.h"
#include "art_replan.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "executor.h"
#include "openart_uart.h"
#include "timebase.h"

static uint32 vofa_last_send_ms; // 主循环写入并读取的曲线发送节拍，单位 ms；不在 ISR 中访问。

static void vofa_send_control(void)
{
    const control_status_struct *status = get_control_status();
    const drive_pose_struct *pose = drive_pose_get();

    /* FireWater 字段顺序：
     * target_lf,target_rf,target_lb,target_rb,
     * feedback_lf,feedback_rf,feedback_lb,feedback_rb,
     * pwm_lf,pwm_rf,pwm_lb,pwm_rb,
     * pose_x_cm,pose_y_cm,pose_yaw_deg,body_vx_cm,body_vy_cm */
    printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.1f,%.2f,%.2f\n",
           (int16)status->wheel_target_count[WHEEL_LF],
           (int16)status->wheel_target_count[WHEEL_RF],
           (int16)status->wheel_target_count[WHEEL_LB],
           (int16)status->wheel_target_count[WHEEL_RB],
           (int16)status->wheel_feedback_count[WHEEL_LF],
           (int16)status->wheel_feedback_count[WHEEL_RF],
           (int16)status->wheel_feedback_count[WHEEL_LB],
           (int16)status->wheel_feedback_count[WHEEL_RB],
           (int16)status->signed_pwm[WHEEL_LF],
           (int16)status->signed_pwm[WHEEL_RF],
           (int16)status->signed_pwm[WHEEL_LB],
           (int16)status->signed_pwm[WHEEL_RB],
           pose->x_cm,
           pose->y_cm,
           pose->yaw_deg,
           pose->body_vx_cm,
           pose->body_vy_cm);
}

static void vofa_send_pose(void)
{
    const drive_pose_struct *pose = drive_pose_get();

    printf("%.2f,%.2f,%.1f,%.2f,%.2f\n",
           pose->x_cm,
           pose->y_cm,
           pose->yaw_deg,
           pose->body_vx_cm,
           pose->body_vy_cm);
}

static void vofa_send_art(void)
{
    const drive_pose_struct *pose = drive_pose_get();
    executor_debug_status_struct executor_debug;
    art_replan_debug_status_struct art_debug;
    uint32 last_rx_ms = openart_last_rx_ms();
    uint32 age_ms = (0u == last_rx_ms) ? 0u : (time_ms() - last_rx_ms);

    executor_get_debug_status(&executor_debug);
    art_replan_get_debug_status(&art_debug);
    (void)art_debug;

    /* FireWater 字段顺序：
     * pose_x,pose_y,yaw,target_x,target_y,err_x,err_y,
     * art_result,art_dx,art_dy,art_diff,step,total,action,state,
     * sync_pending,art_phase,art_stable,art_frame,art_age_ms */
    printf("%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%lu,%lu\n",
           pose->x_cm,
           pose->y_cm,
           pose->yaw_deg,
           executor_debug.target_x_cm,
           executor_debug.target_y_cm,
           executor_debug.error_x_cm,
           executor_debug.error_y_cm,
           executor_debug.art_center_result,
           executor_debug.art_center_dx_cm,
           executor_debug.art_center_dy_cm,
           executor_debug.art_center_diff_cm,
           executor_debug.current_step,
           executor_debug.total_steps,
           (int)executor_debug.action,
           executor_debug.state,
           executor_debug.art_sync_pending,
           art_debug.phase,
           art_debug.stable_count,
           (unsigned long)openart_uart_get_frame_count(),
           (unsigned long)age_ms);
}

/**
 * @brief 初始化 VOFA+ FireWater 曲线发送节拍。
 *
 * 当前 printf 已全局重定向到无线串口，因此本模块只维护发送周期，
 * 不再单独初始化串口，避免和系统启动阶段的无线串口初始化重复。
 *
 * @note 仅在 `app_init()` 中调用一次。
 */
void vofa_init(void)
{
    vofa_last_send_ms = time_ms();
}

/**
 * @brief 按 `VOFA_OUTPUT_MODE` 周期输出 VOFA+ FireWater CSV。
 *
 * @note 在主循环调用；不要放到 ISR 中，否则 `printf`/无线串口阻塞发送会影响控制周期。
 */
void vofa_service(void)
{
    uint32 now_ms = time_ms();

    if(VOFA_OUTPUT_OFF == VOFA_OUTPUT_MODE)
    {
        return;
    }

    if((now_ms - vofa_last_send_ms) < VOFA_SEND_PERIOD_MS)
    {
        return;
    }
    vofa_last_send_ms = now_ms;

    switch(VOFA_OUTPUT_MODE)
    {
        case VOFA_OUTPUT_CONTROL:
            vofa_send_control();
            break;

        case VOFA_OUTPUT_POSE:
            vofa_send_pose();
            break;

        case VOFA_OUTPUT_ART:
            vofa_send_art();
            break;

        default:
            break;
    }
}
