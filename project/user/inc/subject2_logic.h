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
    uint16 box_cell;
    uint16 target_cell;
    uint8 box_valid;
    uint8 target_valid;
    uint8 completed;
} subject2_binding_struct;

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
    SUBJECT2_TRACK_UNCHANGED = 0,
    SUBJECT2_TRACK_MOVED,
    SUBJECT2_TRACK_AMBIGUOUS
} subject2_track_result_enum;

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
} subject2_sync_update_struct;

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
                                  subject2_observation_plan_struct *plan,
                                  solve_result_struct *path);

void subject2_classifier_reset(subject2_classifier_struct *filter);
uint8 subject2_classifier_push(subject2_classifier_struct *filter,
                               uint8 class_id,
                               uint16 confidence_q,
                               uint16 threshold_q,
                               uint8 stable_samples,
                               uint8 *confirmed_class);

void subject2_bindings_clear(subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT]);
uint8 subject2_bind_box(subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT],
                        uint8 class_id,
                        uint16 cell);
uint8 subject2_bind_target(subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT],
                           uint8 class_id,
                           uint16 cell);
uint8 subject2_binding_sets_match(
    const subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT]);

subject2_track_result_enum subject2_track_active_box(
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT],
    uint8 active_class,
    const uint16 *old_boxes,
    uint8 old_box_count,
    const uint16 *new_boxes,
    uint8 new_box_count);

/**
 * @brief 将冻结地图中的唯一小车格归一化到多帧中心所在格。
 * @note 中心格只允许位于原 C/+ 格或其八邻域；输出快照由调用方持有。
 */
uint8 subject2_normalize_center_map(
    const map_source_struct *source,
    uint16 center_col_q,
    uint16 center_row_q,
    map_source_struct *normalized,
    char normalized_rows[MAP_ROWS][MAP_COLS + 1],
    uint8 *car_row,
    uint8 *car_col);

subject2_sync_result_enum subject2_reconcile_objects(
    const map_source_struct *source,
    subject2_object_struct box_objects[MAX_BOXES],
    uint8 *box_count,
    subject2_object_struct target_objects[MAX_BOXES],
    uint8 *target_count,
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT],
    uint8 strict_push_tracking,
    uint8 active_class,
    subject2_sync_update_struct *update);

#endif
