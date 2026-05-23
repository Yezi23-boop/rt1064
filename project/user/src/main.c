#include "zf_common_headfile.h"
#include "sokoban_app.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    printf("\r\nRT1064_OFFLINE_SOKOBAN_READY\r\n");
    printf("KEY_1=C15 KEY_2=C14 KEY_3=C13 KEY_4=C12\r\n");

    sokoban_app_init();

    while(1)
    {
        sokoban_app_poll();
        system_delay_ms(5);
    }
}
