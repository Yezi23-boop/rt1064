#include "zf_common_headfile.h"
#include "timebase.h"

void timebase_init(void)
{
    timer_init(GPT_TIM_1, TIMER_MS);
    timer_start(GPT_TIM_1);
}

uint32 time_ms(void)
{
    return timer_get(GPT_TIM_1);
}
