#include "app.h"
#include "settings.h"
#include "timebase.h"
#include "screen.h"
#include "menu.h"
#include "vofa.h"
#include "drive_test.h"

void app_init(void)
{
    // 初始化顺序保持固定：时间基准先于菜单，菜单按键消抖和回放都依赖 time_ms()。
    timebase_init();
    settings_init();
    screen_init();
    vofa_init();
    drive_test_init();
    menu_init();
}

void app_poll(void)
{
    menu_poll();
    vofa_service();
    drive_test_poll();
}
