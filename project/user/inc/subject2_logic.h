#ifndef _subject2_logic_h_
#define _subject2_logic_h_

#include "map_types.h"

#define SUBJECT2_CLASS_COUNT        (10u)
#define SUBJECT2_INVALID_CLASS      (0xFFu)
#define SUBJECT2_OBSERVATION_COUNT  (4u)

#define SUBJECT2_OBSERVE_UP         (1u << 0)
#define SUBJECT2_OBSERVE_DOWN       (1u << 1)
#define SUBJECT2_OBSERVE_LEFT       (1u << 2)
#define SUBJECT2_OBSERVE_RIGHT      (1u << 3)

typedef struct
{
    uint16 cell;
    uint8 class_id;
    uint8 recognized;
    uint8 tried_observation_mask;
} subject2_object_struct;

typedef struct
{
    uint8 object_index;
    uint8 observation_bit;
    uint8 row;
    uint8 col;
    float target_yaw_deg;
} subject2_observation_plan_struct;

typedef struct
{
    uint8 candidate_class;
    uint8 consecutive_count;
} subject2_classifier_struct;

typedef enum
{
    SUBJECT2_SYNC_OK = 0,
    SUBJECT2_SYNC_RESCAN,
    SUBJECT2_SYNC_AMBIGUOUS
} subject2_sync_result_enum;

typedef struct
{
    uint8 need_box_scan;
    uint8 need_target_scan;
    uint8 completed_count;
    uint8 active_box_valid;
    uint8 active_target_removed;
    uint16 active_box_cell;
} subject2_sync_update_struct;

typedef struct
{
    uint8 class_id;
    uint8 box_index;
    uint8 target_index;
    uint16 box_cell;
    uint16 target_cell;
} subject2_push_plan_struct;

uint8 subject2_collect_objects(const map_source_struct *source,
                               char symbol,
                               subject2_object_struct *objects,
                               uint8 *count);
uint8 subject2_collect_cells(const map_source_struct *source,
                             char symbol,
                             uint16 *cells,
                             uint8 *count);
uint8 subject2_select_observation(const map_source_struct *source,
                                  const subject2_object_struct *objects,
                                  uint8 object_count,
                                  float current_yaw_deg,
                                  subject2_observation_plan_struct *plan,
                                  solve_result_struct *path);

void subject2_classifier_reset(subject2_classifier_struct *filter);
uint8 subject2_classifier_push(subject2_classifier_struct *filter,
                               uint8 class_id,
                               uint16 confidence_q,
                               uint16 threshold_q,
                               uint8 stable_samples,
                               uint8 *confirmed_class);

uint8 subject2_object_class_counts_match(
    const subject2_object_struct *box_objects,
    uint8 box_count,
    const subject2_object_struct *target_objects,
    uint8 target_count);
uint8 subject2_infer_last_target_class(
    const subject2_object_struct *box_objects,
    uint8 box_count,
    const subject2_object_struct *target_objects,
    uint8 target_count,
    uint8 *target_index,
    uint8 *class_id);
void subject2_invalidate_mismatched_classes(
    subject2_object_struct *box_objects,
    uint8 box_count,
    subject2_object_struct *target_objects,
    uint8 target_count,
    uint8 *need_box_scan,
    uint8 *need_target_scan);
uint8 subject2_select_push_plan(
    const map_source_struct *source,
    const subject2_object_struct *box_objects,
    uint8 box_count,
    const subject2_object_struct *target_objects,
    uint8 target_count,
    uint8 retry_active_only,
    uint8 active_box_valid,
    uint16 active_box_cell,
    uint16 active_target_cell,
    subject2_push_plan_struct *plan,
    solve_result_struct *result);
uint8 subject2_normalize_transit_box_overlap(
    const map_source_struct *source,
    const solve_result_struct *result,
    uint16 current_step,
    uint16 active_box_cell,
    uint16 final_target_cell,
    char completed_action,
    char normalized_rows[MAP_ROWS][MAP_COLS + 1],
    map_source_struct *normalized_source,
    uint16 *overlap_cell);
subject2_sync_result_enum subject2_reconcile_object_lists(
    const map_source_struct *source,
    subject2_object_struct box_objects[MAX_BOXES],
    uint8 *box_count,
    subject2_object_struct target_objects[MAX_BOXES],
    uint8 *target_count,
    uint16 active_box_cell,
    uint16 active_target_cell,
    subject2_sync_update_struct *update);

#endif
