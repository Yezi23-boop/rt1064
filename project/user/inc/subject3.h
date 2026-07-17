#ifndef _subject3_h_
#define _subject3_h_

#include "subject2.h"

typedef enum
{
    SUBJECT3_IDLE = 0,
    SUBJECT3_RUN_SUBJECT2,
    SUBJECT3_SELECT_BLAST,
    SUBJECT3_EXECUTE_BOMB,
    SUBJECT3_CONFIRM_BLAST,
    SUBJECT3_CENTER,
    SUBJECT3_ERROR
} subject3_state_enum;

void subject3_begin(const subject2_context_struct *context,
                    float initial_pose_x_cm,
                    float initial_pose_y_cm,
                    subject2_update_struct *update);
void subject3_tick(const subject2_context_struct *context,
                   subject2_update_struct *update);
void subject3_cancel(void);
uint8 subject3_manual_recover(subject2_update_struct *update);
subject3_state_enum subject3_get_state(void);

#endif
