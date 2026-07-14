#ifndef _subject2_h_
#define _subject2_h_

#include "map_types.h"
#include "settings.h"
#include "subject2_logic.h"

typedef enum
{
    SUBJECT2_IDLE = 0,
    SUBJECT2_SCAN_BOX_MODE,
    SUBJECT2_SCAN_BOX_PLAN,
    SUBJECT2_SCAN_BOX_MOVE,
    SUBJECT2_SCAN_BOX_TURN,
    SUBJECT2_SCAN_BOX_CENTER,
    SUBJECT2_SCAN_BOX_ADJUST,
    SUBJECT2_SCAN_BOX_CLASSIFY,
    SUBJECT2_SCAN_BOX_BACKOFF,
    SUBJECT2_SCAN_BOX_BACKOFF_RETURN,
    SUBJECT2_SCAN_TARGET_MODE,
    SUBJECT2_SCAN_TARGET_PLAN,
    SUBJECT2_SCAN_TARGET_MOVE,
    SUBJECT2_SCAN_TARGET_TURN,
    SUBJECT2_SCAN_TARGET_CENTER,
    SUBJECT2_SCAN_TARGET_ADJUST,
    SUBJECT2_SCAN_TARGET_CLASSIFY,
    SUBJECT2_SCAN_TARGET_BACKOFF,
    SUBJECT2_SCAN_TARGET_BACKOFF_RETURN,
    SUBJECT2_VALIDATE_BINDINGS,
    SUBJECT2_RESTORE_HEADING,
    SUBJECT2_SELECT_PUSH,
    SUBJECT2_EXECUTE_PUSH,
    SUBJECT2_PRE_PUSH_CENTER,
    SUBJECT2_CONFIRM_MAP,
    SUBJECT2_REPLAN_CENTER,
    SUBJECT2_RETURN_REQUESTED,
    SUBJECT2_DONE,
    SUBJECT2_ERROR
} subject2_state_enum;

typedef struct
{
    solve_result_struct *result;
    map_source_struct *snapshot;
    char (*snapshot_rows)[MAP_COLS + 1];
    uint8 *snapshot_valid;
    uint32 *elapsed_ms;
    uint8 *start_row;
    uint8 *start_col;
    run_mode_enum run_mode;
    float launch_yaw_deg;
} subject2_context_struct;

typedef struct
{
    uint8 redraw;
    uint8 enter_execute;
    uint8 return_requested;
    float return_pose_x_cm;
    float return_pose_y_cm;
    const char *run_state;
} subject2_update_struct;

void subject2_begin(const subject2_context_struct *context,
                    float initial_pose_x_cm,
                    float initial_pose_y_cm,
                    subject2_update_struct *update);
void subject2_tick(const subject2_context_struct *context,
                   subject2_update_struct *update);
void subject2_cancel(void);
subject2_state_enum subject2_get_state(void);
uint8 subject2_get_active_class(void);
uint8 subject2_get_last_recognition(uint8 *is_target, uint8 *class_id);

#endif
