#ifndef _competition_flow_h_
#define _competition_flow_h_

#include "zf_common_typedef.h"

typedef enum
{
    COMPETITION_ACTION_NONE = 0,
    COMPETITION_ACTION_START_SUBJECT1,
    COMPETITION_ACTION_START_SUBJECT2,
    COMPETITION_ACTION_START_SUBJECT3,
    COMPETITION_ACTION_FINISH
} competition_action_enum;

typedef enum
{
    COMPETITION_FATAL_NONE = 0,
    COMPETITION_FATAL_DRIVE,
    COMPETITION_FATAL_ART1,
    COMPETITION_FATAL_ART2,
    COMPETITION_FATAL_MAP,
    COMPETITION_FATAL_PLAN,
    COMPETITION_FATAL_CLASS,
    COMPETITION_FATAL_TRACK,
    COMPETITION_FATAL_RETURN
} competition_fatal_reason_enum;

void competition_flow_start(uint8 mode);
void competition_flow_on_return_complete(void);
competition_action_enum competition_flow_take_action(void);
uint8 competition_flow_get_subject(void);
uint8 competition_flow_is_active(void);
void competition_flow_latch_fatal(competition_fatal_reason_enum reason);
void competition_flow_clear_fatal(void);
uint8 competition_flow_is_fatal(void);
competition_fatal_reason_enum competition_flow_get_fatal_reason(void);
const char *competition_flow_fatal_text(void);
void competition_flow_cancel(void);

#endif
