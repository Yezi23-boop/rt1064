#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "drive_control.h"
#include "openart_uart.h"
#include "app.h"

int main(void)
{
    uint8 control_init_state;

    clock_init(SYSTEM_CLOCK_600M);
    // debug_init 保留库调试状态，并初始化 UART1；printf 实际重定向到无线串口。
    debug_init();
    wireless_uart_init();
    // UART1 运行期专用于 OpenART，openart_uart_init 只清协议状态和接收缓冲。
    openart_uart_init();
    control_init_state = control_init();

    // 应用层在底盘初始化后启动；菜单和屏幕只做非阻塞轮询，不影响 PIT 闭环节拍。
    app_init();

    while (1)
    {
        app_poll();
    }
}
