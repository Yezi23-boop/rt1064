#include "zf_common_headfile.h"
#include "timebase.h"

void timebase_init(void)
{
    // GPT_TIM_1 作为全局毫秒基准，主循环和菜单状态机都用无符号差值计算超时。
    timer_init(GPT_TIM_1, TIMER_MS);
    timer_start(GPT_TIM_1);
}

uint32 time_ms(void)
{
    return timer_get(GPT_TIM_1);
}
