#include "app.h"
#include "settings.h"
#include "timebase.h"
#include "screen.h"
#include "menu.h"
#include "vofa.h"
#include "drive_test.h"
#include "openart_uart.h"
#include "vision_uart.h"

void app_init(void)
{
    // 初始化顺序是应用层契约的一部分：菜单按键消抖、VOFA 发送节拍和回放显示都依赖 time_ms()。
    // 屏幕先于菜单初始化，避免菜单首次绘制时访问未准备好的 IPS200 状态。
    timebase_init();
    settings_init();
    screen_init();
    vofa_init();
    drive_test_init();
    vision_uart_board_test_init();
    menu_init();
}

void app_poll(void)
{
    // 主循环只做短轮询：实时运动闭环在 PIT 中断里运行，这里阻塞会拖慢屏幕、OpenART ACK 和 VOFA 遥控超时。
    // 新增应用任务应拆成可重入的小步执行，避免一次循环内长时间占用无线串口或 CPU。
    menu_poll();
    vofa_service();
    openart_uart_poll();
    vision_uart_poll();
    vision_uart_board_test_poll();
    drive_test_poll();
}
