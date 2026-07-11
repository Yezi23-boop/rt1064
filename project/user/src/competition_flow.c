#include "drive_config.h"
#include "competition_flow.h"

typedef enum
{
    COMPETITION_STAGE_IDLE = 0,
    COMPETITION_STAGE_SUBJECT1,
    COMPETITION_STAGE_SUBJECT2,
    COMPETITION_STAGE_DONE
} competition_stage_enum;

static uint8 competition_mode;
static competition_stage_enum competition_stage;
static competition_action_enum pending_action;

void competition_flow_start(uint8 mode)
{
    competition_mode = mode;
    pending_action = COMPETITION_ACTION_NONE;
    if(COMPETITION_MODE_SUBJECT1_DEBUG == mode)
    {
        competition_stage = COMPETITION_STAGE_SUBJECT1;
        pending_action = COMPETITION_ACTION_START_SUBJECT1;
    }
    else if(COMPETITION_MODE_SUBJECT2_DEBUG == mode)
    {
        competition_stage = COMPETITION_STAGE_SUBJECT2;
        pending_action = COMPETITION_ACTION_START_SUBJECT2;
    }
    else if(COMPETITION_MODE_FULL == mode)
    {
        competition_stage = COMPETITION_STAGE_SUBJECT1;
        pending_action = COMPETITION_ACTION_START_SUBJECT1;
    }
    else
    {
        competition_stage = COMPETITION_STAGE_IDLE;
    }
}

void competition_flow_on_return_complete(void)
{
    if(COMPETITION_STAGE_SUBJECT1 == competition_stage)
    {
        if(COMPETITION_MODE_FULL == competition_mode)
        {
            competition_stage = COMPETITION_STAGE_SUBJECT2;
            pending_action = COMPETITION_ACTION_START_SUBJECT2;
        }
        else
        {
            competition_stage = COMPETITION_STAGE_DONE;
            pending_action = COMPETITION_ACTION_FINISH;
        }
    }
    else if(COMPETITION_STAGE_SUBJECT2 == competition_stage)
    {
        competition_stage = COMPETITION_STAGE_DONE;
        pending_action = COMPETITION_ACTION_FINISH;
    }
}

competition_action_enum competition_flow_take_action(void)
{
    competition_action_enum action = pending_action;
    pending_action = COMPETITION_ACTION_NONE;
    return action;
}

void competition_flow_cancel(void)
{
    competition_mode = 0u;
    competition_stage = COMPETITION_STAGE_IDLE;
    pending_action = COMPETITION_ACTION_NONE;
}
