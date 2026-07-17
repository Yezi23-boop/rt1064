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
    SUBJECT2_SCAN_MAP_SYNC,
    SUBJECT2_VALIDATE_BINDINGS,
    SUBJECT2_RESTORE_HEADING,
    SUBJECT2_POST_OBSERVE_YAW_SAMPLE,
    SUBJECT2_POST_OBSERVE_YAW_FIX,
    SUBJECT2_SELECT_PUSH,
    SUBJECT2_EXECUTE_PUSH,
    SUBJECT2_PRE_PUSH_CENTER,
    SUBJECT2_CONFIRM_MAP,
    SUBJECT2_REPLAN_CENTER,
    SUBJECT2_WAIT_BLAST,
    SUBJECT2_RETURN_REQUESTED,
    SUBJECT2_DONE,
    SUBJECT2_ERROR
} subject2_state_enum;

typedef enum
{
    SUBJECT2_BLOCK_NONE = 0,
    SUBJECT2_BLOCK_OBSERVE_BOX,
    SUBJECT2_BLOCK_OBSERVE_TARGET,
    SUBJECT2_BLOCK_PUSH
} subject2_block_reason_enum;

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
    uint8 art_yaw_bias_valid;
    float art_yaw_bias_deg;
    uint8 allow_blast_fallback;
} subject2_context_struct;

typedef struct
{
    uint8 redraw;
    uint8 enter_execute;
    uint8 return_requested;
    float return_pose_x_cm;
    float return_pose_y_cm;
    uint8 blast_requested;
    subject2_block_reason_enum block_reason;
    const char *run_state;
} subject2_update_struct;

void subject2_begin(const subject2_context_struct *context,
                    float initial_pose_x_cm,
                    float initial_pose_y_cm,
                    subject2_update_struct *update);
void subject2_tick(const subject2_context_struct *context,
                   subject2_update_struct *update);
void subject2_cancel(void);
uint8 subject2_manual_recover(subject2_update_struct *update);
subject2_block_reason_enum subject2_get_block_reason(void);
uint8 subject2_retry_blocked_plan(const map_source_struct *source,
                                  solve_result_struct *result);
uint8 subject2_resume_after_blast(const subject2_context_struct *context,
                                  const map_source_struct *source,
                                  float pose_x_cm,
                                  float pose_y_cm,
                                  subject2_update_struct *update);
uint8 subject2_refresh_blocked_map(const subject2_context_struct *context,
                                   const map_source_struct *source,
                                   float pose_x_cm,
                                   float pose_y_cm);
void subject2_get_pose_offset(float *pose_x_cm, float *pose_y_cm);
void subject2_reject_blast_fallback(subject2_update_struct *update);
subject2_state_enum subject2_get_state(void);
uint8 subject2_get_active_class(void);
uint8 subject2_get_last_recognition(uint8 *is_target, uint8 *class_id);

#endif
