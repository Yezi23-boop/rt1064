#ifndef _competition_flow_h_
#define _competition_flow_h_

#include "zf_common_typedef.h"

typedef enum
{
    COMPETITION_ACTION_NONE = 0,
    COMPETITION_ACTION_START_SUBJECT1,
    COMPETITION_ACTION_START_SUBJECT2,
    COMPETITION_ACTION_FINISH
} competition_action_enum;

void competition_flow_start(uint8 mode);
void competition_flow_on_return_complete(void);
competition_action_enum competition_flow_take_action(void);
void competition_flow_cancel(void);

#endif
