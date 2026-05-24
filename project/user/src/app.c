#include "app.h"
#include "settings.h"
#include "timebase.h"
#include "screen.h"
#include "menu.h"

void app_init(void)
{
    timebase_init();
    settings_init();
    screen_init();
    menu_init();
}

void app_poll(void)
{
    menu_poll();
}
