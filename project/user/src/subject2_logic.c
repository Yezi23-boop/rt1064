#include "zf_common_headfile.h"
#include "map_utils.h"
#include "solver.h"
#include "subject2_logic.h"

static const int8 observation_dr[SUBJECT2_OBSERVATION_COUNT] = {-1, 1, 0, 0};
static const int8 observation_dc[SUBJECT2_OBSERVATION_COUNT] = {0, 0, -1, 1};
static const uint8 observation_bits[SUBJECT2_OBSERVATION_COUNT] = {
    SUBJECT2_OBSERVE_UP,
    SUBJECT2_OBSERVE_DOWN,
    SUBJECT2_OBSERVE_LEFT,
    SUBJECT2_OBSERVE_RIGHT
};
/* 候选格依次位于对象上、下、左、右；板测绝对 yaw 0 度朝地图下方。 */
static const float observation_target_yaw_deg[SUBJECT2_OBSERVATION_COUNT] = {
    0.0f, 180.0f, 90.0f, 270.0f
};
static solve_result_struct candidate_path;
static solve_result_struct push_candidate_path;

static uint16 absolute_difference(uint8 left, uint8 right)
{
    return (left >= right) ? (uint16)(left - right) : (uint16)(right - left);
}

static uint16 cell_distance(uint16 first, uint16 second)
{
    return (uint16)(absolute_difference(map_cell_row(first), map_cell_row(second)) +
                    absolute_difference(map_cell_col(first), map_cell_col(second)));
}

static uint8 cell_in_list(uint16 cell, const uint16 *cells, uint8 count)
{
    uint8 index;

    for(index = 0u; index < count; index++)
    {
        if(cell == cells[index])
        {
            return 1u;
        }
    }
    return 0u;
}

static uint8 map_value_matches_object(char value, char symbol)
{
    if('B' == symbol)
    {
        return (('B' == value) || (MAP_BOX_ON_TARGET == value)) ? 1u : 0u;
    }
    return (('T' == value) || ('+' == value) ||
            (MAP_BOX_ON_TARGET == value)) ? 1u : 0u;
}

uint8 subject2_collect_objects(const map_source_struct *source,
                               char symbol,
                               subject2_object_struct *objects,
                               uint8 *count)
{
    uint8 row;
    uint8 col;

    if((0 == source) || (0 == objects) || (0 == count) ||
       (('B' != symbol) && ('T' != symbol)))
    {
        return 0u;
    }
    *count = 0u;
    for(row = 0u; row < MAP_ROWS; row++)
    {
        for(col = 0u; col < MAP_COLS; col++)
        {
            if(0u != map_value_matches_object(source->rows[row][col], symbol))
            {
                if(MAX_BOXES <= *count)
                {
                    *count = 0u;
                    return 0u;
                }
                objects[*count].cell = map_cell_index(row, col);
                objects[*count].class_id = SUBJECT2_INVALID_CLASS;
                objects[*count].recognized = 0u;
                objects[*count].tried_observation_mask = 0u;
                (*count)++;
            }
        }
    }
    return (0u != *count) ? 1u : 0u;
}

uint8 subject2_collect_cells(const map_source_struct *source,
                             char symbol,
                             uint16 *cells,
                             uint8 *count)
{
    uint8 row;
    uint8 col;

    if((0 == source) || (0 == cells) || (0 == count))
    {
        return 0u;
    }
    *count = 0u;
    for(row = 0u; row < MAP_ROWS; row++)
    {
        for(col = 0u; col < MAP_COLS; col++)
        {
            if(0u != map_value_matches_object(source->rows[row][col], symbol))
            {
                if(MAX_BOXES <= *count)
                {
                    *count = 0u;
                    return 0u;
                }
                cells[(*count)++] = map_cell_index(row, col);
            }
        }
    }
    return 1u;
}

uint8 subject2_select_observation(const map_source_struct *source,
                                  const subject2_object_struct *objects,
                                  uint8 object_count,
                                  subject2_observation_plan_struct *plan,
                                  solve_result_struct *path)
{
    uint16 best_actions = 0xFFFFu;
    uint16 object_cell;
    uint16 candidate_cell;
    int16 candidate_row;
    int16 candidate_col;
    uint8 object_index;
    uint8 other_index;
    uint8 direction;
    uint8 ambiguous;
    uint8 found = 0u;
    char candidate_value;

    if((0 == source) || (0 == objects) || (0u == object_count) ||
       (0 == plan) || (0 == path))
    {
        return 0u;
    }

    for(object_index = 0u; object_index < object_count; object_index++)
    {
        if(0u != objects[object_index].recognized)
        {
            continue;
        }
        object_cell = objects[object_index].cell;
        for(direction = 0u; direction < SUBJECT2_OBSERVATION_COUNT; direction++)
        {
            if(0u != (objects[object_index].tried_observation_mask &
                      observation_bits[direction]))
            {
                continue;
            }
            candidate_row = (int16)map_cell_row(object_cell) + observation_dr[direction];
            candidate_col = (int16)map_cell_col(object_cell) + observation_dc[direction];
            if((0 > candidate_row) || (MAP_ROWS <= candidate_row) ||
               (0 > candidate_col) || (MAP_COLS <= candidate_col))
            {
                continue;
            }
            candidate_value = source->rows[candidate_row][candidate_col];
            if(('#' == candidate_value) || ('X' == candidate_value) ||
               ('B' == candidate_value))
            {
                continue;
            }

            candidate_cell = map_cell_index((uint8)candidate_row, (uint8)candidate_col);
            ambiguous = 0u;
            for(other_index = 0u; other_index < object_count; other_index++)
            {
                if((other_index != object_index) &&
                   (cell_distance(candidate_cell, objects[other_index].cell) <= 1u))
                {
                    ambiguous = 1u;
                    break;
                }
            }
            if(0u != ambiguous)
            {
                continue;
            }
            if(0 == solve_navigation_path(source,
                                           (uint8)candidate_row,
                                           (uint8)candidate_col,
                                           &candidate_path))
            {
                continue;
            }
            if((0u == found) || (candidate_path.action_count < best_actions))
            {
                found = 1u;
                best_actions = candidate_path.action_count;
                plan->object_index = object_index;
                plan->observation_bit = observation_bits[direction];
                plan->row = (uint8)candidate_row;
                plan->col = (uint8)candidate_col;
                plan->target_yaw_deg = observation_target_yaw_deg[direction];
                memcpy(path, &candidate_path, sizeof(*path));
            }
        }
    }
    return found;
}

void subject2_classifier_reset(subject2_classifier_struct *filter)
{
    if(0 != filter)
    {
        filter->candidate_class = SUBJECT2_INVALID_CLASS;
        filter->consecutive_count = 0u;
    }
}

uint8 subject2_classifier_push(subject2_classifier_struct *filter,
                               uint8 class_id,
                               uint16 confidence_q,
                               uint16 threshold_q,
                               uint8 stable_samples,
                               uint8 *confirmed_class)
{
    if((0 == filter) || (0 == confirmed_class) ||
       (0u == stable_samples) || (SUBJECT2_CLASS_COUNT <= class_id) ||
       (confidence_q < threshold_q))
    {
        if(0 != filter)
        {
            subject2_classifier_reset(filter);
        }
        return 0u;
    }
    if(filter->candidate_class != class_id)
    {
        filter->candidate_class = class_id;
        filter->consecutive_count = 1u;
    }
    else if(filter->consecutive_count < stable_samples)
    {
        filter->consecutive_count++;
    }
    if(filter->consecutive_count < stable_samples)
    {
        return 0u;
    }
    *confirmed_class = class_id;
    return 1u;
}

uint8 subject2_object_class_counts_match(
    const subject2_object_struct *box_objects,
    uint8 box_count,
    const subject2_object_struct *target_objects,
    uint8 target_count)
{
    uint8 box_counts[SUBJECT2_CLASS_COUNT] = {0};
    uint8 target_counts[SUBJECT2_CLASS_COUNT] = {0};
    uint8 index;

    if((0 == box_objects) || (0 == target_objects) ||
       (0u == box_count) || (box_count != target_count))
    {
        return 0u;
    }
    for(index = 0u; index < box_count; index++)
    {
        if((0u == box_objects[index].recognized) ||
           (SUBJECT2_CLASS_COUNT <= box_objects[index].class_id) ||
           (0u == target_objects[index].recognized) ||
           (SUBJECT2_CLASS_COUNT <= target_objects[index].class_id))
        {
            return 0u;
        }
        box_counts[box_objects[index].class_id]++;
        target_counts[target_objects[index].class_id]++;
    }
    for(index = 0u; index < SUBJECT2_CLASS_COUNT; index++)
    {
        if(box_counts[index] != target_counts[index])
        {
            return 0u;
        }
    }
    return 1u;
}

static void invalidate_object(subject2_object_struct *object)
{
    object->recognized = 0u;
    object->class_id = SUBJECT2_INVALID_CLASS;
    object->tried_observation_mask = 0u;
}

void subject2_invalidate_mismatched_classes(
    subject2_object_struct *box_objects,
    uint8 box_count,
    subject2_object_struct *target_objects,
    uint8 target_count,
    uint8 *need_box_scan,
    uint8 *need_target_scan)
{
    uint8 box_counts[SUBJECT2_CLASS_COUNT] = {0};
    uint8 target_counts[SUBJECT2_CLASS_COUNT] = {0};
    uint8 index;

    if((0 == box_objects) || (0 == target_objects) ||
       (0 == need_box_scan) || (0 == need_target_scan))
    {
        return;
    }
    *need_box_scan = 0u;
    *need_target_scan = 0u;
    for(index = 0u; index < box_count; index++)
    {
        if((0u != box_objects[index].recognized) &&
           (box_objects[index].class_id < SUBJECT2_CLASS_COUNT))
        {
            box_counts[box_objects[index].class_id]++;
        }
        else
        {
            invalidate_object(&box_objects[index]);
            *need_box_scan = 1u;
        }
    }
    for(index = 0u; index < target_count; index++)
    {
        if((0u != target_objects[index].recognized) &&
           (target_objects[index].class_id < SUBJECT2_CLASS_COUNT))
        {
            target_counts[target_objects[index].class_id]++;
        }
        else
        {
            invalidate_object(&target_objects[index]);
            *need_target_scan = 1u;
        }
    }
    for(index = 0u; index < box_count; index++)
    {
        if((0u != box_objects[index].recognized) &&
           (box_counts[box_objects[index].class_id] !=
            target_counts[box_objects[index].class_id]))
        {
            invalidate_object(&box_objects[index]);
            *need_box_scan = 1u;
        }
    }
    for(index = 0u; index < target_count; index++)
    {
        if((0u != target_objects[index].recognized) &&
           (box_counts[target_objects[index].class_id] !=
            target_counts[target_objects[index].class_id]))
        {
            invalidate_object(&target_objects[index]);
            *need_target_scan = 1u;
        }
    }
}

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
    solve_result_struct *result)
{
    uint16 best_actions = 0xFFFFu;
    uint8 box_index;
    uint8 target_index;
    uint8 found = 0u;

    if((0 == source) || (0 == box_objects) || (0 == target_objects) ||
       (0 == plan) || (0 == result))
    {
        return 0u;
    }
    for(box_index = 0u; box_index < box_count; box_index++)
    {
        if((0u == box_objects[box_index].recognized) ||
           (SUBJECT2_CLASS_COUNT <= box_objects[box_index].class_id) ||
           ((0u != retry_active_only) && (0u != active_box_valid) &&
            (box_objects[box_index].cell != active_box_cell)))
        {
            continue;
        }
        for(target_index = 0u; target_index < target_count; target_index++)
        {
            if((0u == target_objects[target_index].recognized) ||
               (box_objects[box_index].class_id != target_objects[target_index].class_id) ||
               ((0u != retry_active_only) &&
                (target_objects[target_index].cell != active_target_cell)))
            {
                continue;
            }
            if((0u != solve_bound_box_path(source,
                                            box_objects[box_index].cell,
                                            target_objects[target_index].cell,
                                            &push_candidate_path)) &&
               ((0u == found) ||
                (push_candidate_path.action_count < best_actions)))
            {
                found = 1u;
                best_actions = push_candidate_path.action_count;
                plan->class_id = box_objects[box_index].class_id;
                plan->box_index = box_index;
                plan->target_index = target_index;
                plan->box_cell = box_objects[box_index].cell;
                plan->target_cell = target_objects[target_index].cell;
                memcpy(result, &push_candidate_path, sizeof(*result));
            }
        }
    }
    return found;
}

static uint8 step_push_cell(uint16 cell, char action, uint16 *next_cell)
{
    int16 row = (int16)map_cell_row(cell);
    int16 col = (int16)map_cell_col(cell);

    if('U' == action)
    {
        row--;
    }
    else if('D' == action)
    {
        row++;
    }
    else if('L' == action)
    {
        col--;
    }
    else if('R' == action)
    {
        col++;
    }
    else
    {
        return 0u;
    }
    if((0 > row) || (MAP_ROWS <= row) ||
       (0 > col) || (MAP_COLS <= col))
    {
        return 0u;
    }
    *next_cell = map_cell_index((uint8)row, (uint8)col);
    return 1u;
}

uint8 subject2_normalize_transit_box_overlap(
    const map_source_struct *source,
    const solve_result_struct *result,
    uint16 current_step,
    uint16 active_box_cell,
    uint16 final_target_cell,
    char completed_action,
    char normalized_rows[MAP_ROWS][MAP_COLS + 1],
    map_source_struct *normalized_source,
    uint16 *overlap_cell)
{
    map_scan_stats_struct stats;
    uint16 predicted_box_cell = active_box_cell;
    uint16 candidate_cell = INVALID_STATE;
    uint16 action_end;
    uint16 index;
    char last_push_action = '\0';

    if((0 == source) || (0 == result) || (0 == normalized_rows) ||
       (0 == normalized_source) || (MAP_CELLS <= active_box_cell) ||
       (MAP_CELLS <= final_target_cell) ||
       (current_step >= result->waypoint_count) ||
       (('U' != completed_action) && ('D' != completed_action) &&
        ('L' != completed_action) && ('R' != completed_action)))
    {
        return 0u;
    }
    map_scan_stats(source, &stats);
    if((1u != stats.car_count) ||
       ((uint8)(stats.box_count + 1u) != stats.target_count))
    {
        return 0u;
    }
    action_end = result->waypoints[current_step].action_end;
    if((0u == action_end) || (result->action_count < action_end))
    {
        return 0u;
    }
    for(index = 0u; index < action_end; index++)
    {
        char action = result->actions[index];

        if((action >= 'A') && (action <= 'Z'))
        {
            if(0u == step_push_cell(predicted_box_cell, action,
                                    &predicted_box_cell))
            {
                return 0u;
            }
            last_push_action = action;
            /* 上位机可能只执行到规划推箱链的中间位置；只接受轨迹上唯一的非最终目标格。 */
            if((predicted_box_cell != final_target_cell) &&
               ('T' == source->rows[map_cell_row(predicted_box_cell)]
                                   [map_cell_col(predicted_box_cell)]))
            {
                if(INVALID_STATE == candidate_cell)
                {
                    candidate_cell = predicted_box_cell;
                }
                else if(candidate_cell != predicted_box_cell)
                {
                    return 0u;
                }
            }
        }
    }
    if((completed_action != last_push_action) ||
       (INVALID_STATE == candidate_cell))
    {
        return 0u;
    }

    map_source_snapshot(normalized_source, normalized_rows, source);
    normalized_rows[map_cell_row(candidate_cell)][map_cell_col(candidate_cell)] =
        MAP_BOX_ON_TARGET;
    if(0 != overlap_cell)
    {
        *overlap_cell = candidate_cell;
    }
    return 1u;
}

static uint8 collect_cells_allow_empty(const map_source_struct *source,
                                       char symbol,
                                       uint16 cells[MAX_BOXES],
                                       uint8 *count)
{
    uint8 row;
    uint8 col;

    if((0 == source) || (0 == cells) || (0 == count))
    {
        return 0u;
    }
    *count = 0u;
    for(row = 0u; row < MAP_ROWS; row++)
    {
        for(col = 0u; col < MAP_COLS; col++)
        {
            if(0u != map_value_matches_object(source->rows[row][col], symbol))
            {
                if(*count >= MAX_BOXES)
                {
                    return 0u;
                }
                cells[(*count)++] = map_cell_index(row, col);
            }
        }
    }
    return 1u;
}

static int8 object_index_for_cell(const subject2_object_struct *objects,
                                  uint8 count,
                                  uint16 cell)
{
    uint8 index;

    for(index = 0u; index < count; index++)
    {
        if(cell == objects[index].cell)
        {
            return (int8)index;
        }
    }
    return -1;
}

subject2_sync_result_enum subject2_reconcile_object_lists(
    const map_source_struct *source,
    subject2_object_struct box_objects[MAX_BOXES],
    uint8 *box_count,
    subject2_object_struct target_objects[MAX_BOXES],
    uint8 *target_count,
    uint16 active_box_cell,
    uint16 active_target_cell,
    subject2_sync_update_struct *update)
{
    subject2_object_struct next_boxes[MAX_BOXES];
    subject2_object_struct next_targets[MAX_BOXES];
    uint16 new_boxes[MAX_BOXES];
    uint16 new_targets[MAX_BOXES];
    uint8 new_box_unmatched[MAX_BOXES] = {0};
    uint8 required_count[SUBJECT2_CLASS_COUNT] = {0};
    uint8 assigned_count[SUBJECT2_CLASS_COUNT] = {0};
    uint8 new_box_count = 0u;
    uint8 new_target_count = 0u;
    uint8 deficit_class = SUBJECT2_INVALID_CLASS;
    uint8 deficit_class_count = 0u;
    uint8 deficit_total = 0u;
    uint8 unmatched_count = 0u;
    uint8 active_class = SUBJECT2_INVALID_CLASS;
    uint8 active_candidate_count = 0u;
    uint8 index;
    uint8 class_id;
    int8 old_index;
    subject2_sync_result_enum result = SUBJECT2_SYNC_OK;

    if(0 != update)
    {
        memset(update, 0, sizeof(*update));
        update->active_box_cell = INVALID_STATE;
    }
    if((0 == source) || (0 == box_objects) || (0 == box_count) ||
       (0 == target_objects) || (0 == target_count) || (0 == update) ||
       (*box_count > MAX_BOXES) || (*target_count > MAX_BOXES) ||
       (0u == collect_cells_allow_empty(source, 'B', new_boxes, &new_box_count)) ||
       (0u == collect_cells_allow_empty(source, 'T', new_targets, &new_target_count)) ||
       (new_box_count != new_target_count) ||
       (new_box_count > *box_count) || (new_target_count > *target_count))
    {
        return SUBJECT2_SYNC_AMBIGUOUS;
    }

    memset(next_boxes, 0, sizeof(next_boxes));
    memset(next_targets, 0, sizeof(next_targets));
    old_index = object_index_for_cell(target_objects, *target_count,
                                      active_target_cell);
    if((old_index >= 0) &&
       (0u != target_objects[(uint8)old_index].recognized) &&
       (target_objects[(uint8)old_index].class_id < SUBJECT2_CLASS_COUNT))
    {
        active_class = target_objects[(uint8)old_index].class_id;
    }
    else
    {
        old_index = object_index_for_cell(box_objects, *box_count,
                                          active_box_cell);
        if((old_index >= 0) &&
           (0u != box_objects[(uint8)old_index].recognized) &&
           (box_objects[(uint8)old_index].class_id < SUBJECT2_CLASS_COUNT))
        {
            active_class = box_objects[(uint8)old_index].class_id;
        }
    }

    for(index = 0u; index < new_target_count; index++)
    {
        old_index = object_index_for_cell(target_objects, *target_count,
                                          new_targets[index]);
        if(old_index < 0)
        {
            return SUBJECT2_SYNC_AMBIGUOUS;
        }
        next_targets[index] = target_objects[(uint8)old_index];
        next_targets[index].cell = new_targets[index];
        if((0u == next_targets[index].recognized) ||
           (SUBJECT2_CLASS_COUNT <= next_targets[index].class_id))
        {
            invalidate_object(&next_targets[index]);
            update->need_target_scan = 1u;
            result = SUBJECT2_SYNC_RESCAN;
        }
        else
        {
            required_count[next_targets[index].class_id]++;
        }
    }
    update->completed_count = (uint8)(*target_count - new_target_count);
    update->active_target_removed =
        ((active_target_cell < MAP_CELLS) &&
         (0u == cell_in_list(active_target_cell,
                             new_targets, new_target_count))) ? 1u : 0u;

    for(index = 0u; index < new_box_count; index++)
    {
        old_index = object_index_for_cell(box_objects, *box_count,
                                          new_boxes[index]);
        if((old_index >= 0) &&
           (0u != box_objects[(uint8)old_index].recognized) &&
           (box_objects[(uint8)old_index].class_id < SUBJECT2_CLASS_COUNT) &&
           (assigned_count[box_objects[(uint8)old_index].class_id] <
            required_count[box_objects[(uint8)old_index].class_id]))
        {
            next_boxes[index] = box_objects[(uint8)old_index];
            next_boxes[index].cell = new_boxes[index];
            assigned_count[next_boxes[index].class_id]++;
        }
        else
        {
            invalidate_object(&next_boxes[index]);
            next_boxes[index].cell = new_boxes[index];
            new_box_unmatched[index] = 1u;
            unmatched_count++;
        }
    }

    for(class_id = 0u; class_id < SUBJECT2_CLASS_COUNT; class_id++)
    {
        uint8 deficit = (uint8)(required_count[class_id] - assigned_count[class_id]);

        if(0u != deficit)
        {
            deficit_class = class_id;
            deficit_class_count++;
            deficit_total = (uint8)(deficit_total + deficit);
        }
    }
    if((0u != unmatched_count) &&
       (1u == deficit_class_count) &&
       (unmatched_count == deficit_total))
    {
        for(index = 0u; index < new_box_count; index++)
        {
            if(0u != new_box_unmatched[index])
            {
                next_boxes[index].recognized = 1u;
                next_boxes[index].class_id = deficit_class;
                next_boxes[index].tried_observation_mask = 0u;
            }
        }
    }
    else if(0u != unmatched_count)
    {
        update->need_box_scan = 1u;
        result = SUBJECT2_SYNC_RESCAN;
    }

    if((0u == update->active_target_removed) &&
       (active_class < SUBJECT2_CLASS_COUNT))
    {
        for(index = 0u; index < new_box_count; index++)
        {
            if((0u != next_boxes[index].recognized) &&
               (active_class == next_boxes[index].class_id))
            {
                if(next_boxes[index].cell == active_box_cell)
                {
                    update->active_box_valid = 1u;
                    update->active_box_cell = active_box_cell;
                    active_candidate_count = 0u;
                    break;
                }
                if(0u != new_box_unmatched[index])
                {
                    update->active_box_cell = next_boxes[index].cell;
                    active_candidate_count++;
                }
            }
        }
        if(1u == active_candidate_count)
        {
            update->active_box_valid = 1u;
        }
        else if(1u < active_candidate_count)
        {
            update->active_box_valid = 0u;
            update->active_box_cell = INVALID_STATE;
        }
    }

    if((SUBJECT2_SYNC_OK == result) && (0u != new_box_count) &&
       (0u == subject2_object_class_counts_match(next_boxes, new_box_count,
                                                  next_targets, new_target_count)))
    {
        return SUBJECT2_SYNC_AMBIGUOUS;
    }
    memcpy(box_objects, next_boxes, sizeof(next_boxes));
    memcpy(target_objects, next_targets, sizeof(next_targets));
    *box_count = new_box_count;
    *target_count = new_target_count;
    return result;
}
