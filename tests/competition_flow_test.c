#include <stdio.h>
#include "competition_flow.h"
#include "drive_config.h"

static uint8 run_case(const char *name, uint8 passed)
{
    printf("%-28s %s\n", name, (0u != passed) ? "PASS" : "FAIL");
    return passed;
}

static uint8 subject1_debug_flow(void)
{
    competition_flow_start(COMPETITION_MODE_SUBJECT1_DEBUG);
    if(COMPETITION_ACTION_START_SUBJECT1 != competition_flow_take_action()) return 0u;
    competition_flow_on_return_complete();
    return (COMPETITION_ACTION_FINISH == competition_flow_take_action()) ? 1u : 0u;
}

static uint8 subject2_debug_flow(void)
{
    competition_flow_start(COMPETITION_MODE_SUBJECT2_DEBUG);
    if(COMPETITION_ACTION_START_SUBJECT2 != competition_flow_take_action()) return 0u;
    competition_flow_on_return_complete();
    return (COMPETITION_ACTION_FINISH == competition_flow_take_action()) ? 1u : 0u;
}

static uint8 full_flow(void)
{
    competition_flow_start(COMPETITION_MODE_FULL);
    if(COMPETITION_ACTION_START_SUBJECT1 != competition_flow_take_action()) return 0u;
    competition_flow_on_return_complete();
    if(COMPETITION_ACTION_START_SUBJECT2 != competition_flow_take_action()) return 0u;
    competition_flow_on_return_complete();
    return (COMPETITION_ACTION_FINISH == competition_flow_take_action()) ? 1u : 0u;
}

int main(void)
{
    uint8 passed = 1u;

    passed &= run_case("subject1-debug-flow", subject1_debug_flow());
    passed &= run_case("subject2-debug-flow", subject2_debug_flow());
    passed &= run_case("full-flow", full_flow());
    return (0u != passed) ? 0 : 1;
}
