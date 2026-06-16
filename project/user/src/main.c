#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "drive_control.h"
#include "openart_uart.h"
#include "app.h"
#include "executor.h"

int main(void)
{
    uint8 control_init_state;

    clock_init(SYSTEM_CLOCK_600M);
    // 时钟先于所有外设；后续串口、PIT 和 PWM 分频都建立在 SYSTEM_CLOCK_600M 上。
    // debug_init 保留库调试状态并初始化 UART1 硬件，printf 实际由库重定向到无线串口。
    debug_init();
    wireless_uart_init();
    // UART1 运行期专用于 OpenART 地图帧；openart_uart_init 只清协议状态和接收缓冲，不改串口分工。
    openart_uart_init();
    control_init_state = control_init();
    executor_init();

    // 应用层在底盘和执行器之后启动，菜单首次显示才能读取稳定的控制状态。
    // 主循环不得承担控制闭环；20ms 运动更新由 PIT ISR 保证，主循环只做非阻塞服务。
    app_init();

    while (1)
    {
        app_poll();
    }
}
