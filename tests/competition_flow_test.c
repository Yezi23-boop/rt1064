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
    if((0u == competition_flow_is_active()) ||
       (2u != competition_flow_get_subject())) return 0u;
    competition_flow_on_return_complete();
    if(COMPETITION_ACTION_FINISH != competition_flow_take_action()) return 0u;
    competition_flow_cancel();
    return ((0u == competition_flow_is_active()) &&
            (0u == competition_flow_get_subject())) ? 1u : 0u;
}

static uint8 full_flow(void)
{
    competition_flow_start(COMPETITION_MODE_FULL);
    if(COMPETITION_ACTION_START_SUBJECT1 != competition_flow_take_action()) return 0u;
    competition_flow_on_return_complete();
    if(COMPETITION_ACTION_START_SUBJECT2 != competition_flow_take_action()) return 0u;
    competition_flow_on_return_complete();
    if(COMPETITION_ACTION_START_SUBJECT3 != competition_flow_take_action()) return 0u;
    competition_flow_on_return_complete();
    return (COMPETITION_ACTION_FINISH == competition_flow_take_action()) ? 1u : 0u;
}

static uint8 subject3_debug_flow(void)
{
    competition_flow_start(COMPETITION_MODE_SUBJECT3_DEBUG);
    if(COMPETITION_ACTION_START_SUBJECT3 != competition_flow_take_action()) return 0u;
    if((0u == competition_flow_is_active()) ||
       (3u != competition_flow_get_subject())) return 0u;
    competition_flow_on_return_complete();
    return (COMPETITION_ACTION_FINISH == competition_flow_take_action()) ? 1u : 0u;
}

static uint8 fatal_latch_preserves_stage(void)
{
    competition_flow_start(COMPETITION_MODE_SUBJECT2_DEBUG);
    if(COMPETITION_ACTION_START_SUBJECT2 != competition_flow_take_action()) return 0u;

    competition_flow_latch_fatal(COMPETITION_FATAL_MAP);
    if((0u == competition_flow_is_fatal()) ||
       (COMPETITION_FATAL_MAP != competition_flow_get_fatal_reason())) return 0u;

    competition_flow_clear_fatal();
    if(0u != competition_flow_is_fatal()) return 0u;
    competition_flow_on_return_complete();
    return (COMPETITION_ACTION_FINISH == competition_flow_take_action()) ? 1u : 0u;
}

int main(void)
{
    uint8 passed = 1u;

    passed &= run_case("subject1-debug-flow", subject1_debug_flow());
    passed &= run_case("subject2-debug-flow", subject2_debug_flow());
    passed &= run_case("subject3-debug-flow", subject3_debug_flow());
    passed &= run_case("full-flow", full_flow());
    passed &= run_case("fatal-preserves-stage", fatal_latch_preserves_stage());
    return (0u != passed) ? 0 : 1;
}
