#include "zf_common_headfile.h"
#include "drive_control.h"
#include "app.h"

int main(void)
{
    uint8 control_init_state;

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    printf("\r\nRT1064_PUSH_BOX_READY\r\n");
    printf("KEY_1=C15 KEY_2=C14 KEY_3=C13 KEY_4=C12\r\n");

    control_init_state = control_init();
    printf("CONTROL_INIT=%d\r\n", control_init_state);

    // 应用层在底盘初始化后启动；菜单和屏幕只做非阻塞轮询，不影响 PIT 闭环节拍。
    app_init();

    while(1)
    {
        app_poll();
    }
}
