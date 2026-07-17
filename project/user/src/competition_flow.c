#include "drive_config.h"
#include "competition_flow.h"

typedef enum
{
    COMPETITION_STAGE_IDLE = 0,
    COMPETITION_STAGE_SUBJECT1,
    COMPETITION_STAGE_SUBJECT2,
    COMPETITION_STAGE_SUBJECT3,
    COMPETITION_STAGE_DONE
} competition_stage_enum;

static uint8 competition_mode;
static competition_stage_enum competition_stage;
static competition_action_enum pending_action;
static competition_fatal_reason_enum fatal_reason;

void competition_flow_start(uint8 mode)
{
    competition_mode = mode;
    fatal_reason = COMPETITION_FATAL_NONE;
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
    else if(COMPETITION_MODE_SUBJECT3_DEBUG == mode)
    {
        competition_stage = COMPETITION_STAGE_SUBJECT3;
        pending_action = COMPETITION_ACTION_START_SUBJECT3;
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
    if(COMPETITION_FATAL_NONE != fatal_reason)
    {
        return;
    }
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
        if(COMPETITION_MODE_FULL == competition_mode)
        {
            competition_stage = COMPETITION_STAGE_SUBJECT3;
            pending_action = COMPETITION_ACTION_START_SUBJECT3;
        }
        else
        {
            competition_stage = COMPETITION_STAGE_DONE;
            pending_action = COMPETITION_ACTION_FINISH;
        }
    }
    else if(COMPETITION_STAGE_SUBJECT3 == competition_stage)
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

uint8 competition_flow_get_subject(void)
{
    if(COMPETITION_STAGE_SUBJECT1 == competition_stage) return 1u;
    if(COMPETITION_STAGE_SUBJECT2 == competition_stage) return 2u;
    if(COMPETITION_STAGE_SUBJECT3 == competition_stage) return 3u;
    return 0u;
}

uint8 competition_flow_is_active(void)
{
    return ((COMPETITION_STAGE_SUBJECT1 == competition_stage) ||
            (COMPETITION_STAGE_SUBJECT2 == competition_stage) ||
            (COMPETITION_STAGE_SUBJECT3 == competition_stage)) ? 1u : 0u;
}

void competition_flow_latch_fatal(competition_fatal_reason_enum reason)
{
    if((COMPETITION_FATAL_NONE == fatal_reason) &&
       (COMPETITION_FATAL_NONE != reason))
    {
        fatal_reason = reason;
        pending_action = COMPETITION_ACTION_NONE;
    }
}

void competition_flow_clear_fatal(void)
{
    fatal_reason = COMPETITION_FATAL_NONE;
}

uint8 competition_flow_is_fatal(void)
{
    return (COMPETITION_FATAL_NONE != fatal_reason) ? 1u : 0u;
}

competition_fatal_reason_enum competition_flow_get_fatal_reason(void)
{
    return fatal_reason;
}

const char *competition_flow_fatal_text(void)
{
    switch(fatal_reason)
    {
        case COMPETITION_FATAL_DRIVE:  return "F:Drive";
        case COMPETITION_FATAL_ART1:   return "F:ART1";
        case COMPETITION_FATAL_ART2:   return "F:ART2";
        case COMPETITION_FATAL_MAP:    return "F:Map";
        case COMPETITION_FATAL_PLAN:   return "F:Plan";
        case COMPETITION_FATAL_CLASS:  return "F:Class";
        case COMPETITION_FATAL_TRACK:  return "F:Track";
        case COMPETITION_FATAL_RETURN: return "F:Return";
        default:                       return "F:?";
    }
}

void competition_flow_cancel(void)
{
    competition_mode = 0u;
    competition_stage = COMPETITION_STAGE_IDLE;
    pending_action = COMPETITION_ACTION_NONE;
    fatal_reason = COMPETITION_FATAL_NONE;
}
