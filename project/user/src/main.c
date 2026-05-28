#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "drive_control.h"
#include "base_io.h"
#include "timebase.h"
#include "app.h"

int main(void)
{
    uint8 control_init_state;

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    wireless_uart_init();
    control_init_state = control_init();

//    test_wheel(WHEEL_LF, 500);
//	  set_wheel_pwm(WHEEL_RF, 500);
//	  set_wheel_pwm(WHEEL_LB, 500);
//    set_wheel_pwm(WHEEL_RB, 500);
//    set_motion_command(MOTION_FORWARD,  1, 0.0f); // 前进
//    set_motion_command(MOTION_BACKWARD, 1, 0.0f); // 后退
//    set_motion_command(MOTION_LEFT_BACK, 1, 0.0f); // 左后
//    set_motion_command(MOTION_RIGHT,    1, 0.0f); // 右移
    // 应用层在底盘初始化后启动；菜单和屏幕只做非阻塞轮询，不影响 PIT 闭环节拍。
    app_init();

    while(1)
    {
        app_poll();

#if DRIVE_TRANSLATE_TEST_ENABLE
        {
            static uint8 translate_test_started = 0;
            static uint8 translate_test_stopped = 0;
            uint32 now_ms = time_ms();

            /* 平移测试只在主循环发一次命令；实时闭环仍由 PIT 负责。
             * 启动时间默认等于上电安全窗口，避免 IMU 锁 yaw 前电机被测试命令带动。 */
            if((0 == translate_test_started) && (now_ms >= DRIVE_TRANSLATE_TEST_START_MS))
            {
                set_motion_command(DRIVE_TRANSLATE_TEST_COMMAND, DRIVE_TRANSLATE_TEST_SPEED, 0.0f);
                translate_test_started = 1;
            }

            if((0 != translate_test_started) &&
               (0 == translate_test_stopped) &&
               (now_ms >= (DRIVE_TRANSLATE_TEST_START_MS + DRIVE_TRANSLATE_TEST_DURATION_MS)))
            {
                stop_motion();
                translate_test_stopped = 1;
            }
        }
#endif
    }
}
