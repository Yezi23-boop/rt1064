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
            if((symbol == source->rows[row][col]) ||
               (('T' == symbol) && ('+' == source->rows[row][col])))
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
            if((symbol == source->rows[row][col]) ||
               (('T' == symbol) && ('+' == source->rows[row][col])))
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

void subject2_bindings_clear(subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT])
{
    uint8 class_id;

    if(0 == bindings)
    {
        return;
    }
    for(class_id = 0u; class_id < SUBJECT2_CLASS_COUNT; class_id++)
    {
        bindings[class_id].box_cell = INVALID_STATE;
        bindings[class_id].target_cell = INVALID_STATE;
        bindings[class_id].box_valid = 0u;
        bindings[class_id].target_valid = 0u;
        bindings[class_id].completed = 0u;
    }
}

uint8 subject2_bind_box(subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT],
                        uint8 class_id,
                        uint16 cell)
{
    if((0 == bindings) || (SUBJECT2_CLASS_COUNT <= class_id) ||
       (MAP_CELLS <= cell) || (0u != bindings[class_id].box_valid))
    {
        return 0u;
    }
    bindings[class_id].box_cell = cell;
    bindings[class_id].box_valid = 1u;
    return 1u;
}

uint8 subject2_bind_target(subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT],
                           uint8 class_id,
                           uint16 cell)
{
    if((0 == bindings) || (SUBJECT2_CLASS_COUNT <= class_id) ||
       (MAP_CELLS <= cell) || (0u != bindings[class_id].target_valid))
    {
        return 0u;
    }
    bindings[class_id].target_cell = cell;
    bindings[class_id].target_valid = 1u;
    return 1u;
}

uint8 subject2_binding_sets_match(
    const subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT])
{
    uint8 class_id;
    uint8 any = 0u;

    if(0 == bindings)
    {
        return 0u;
    }
    for(class_id = 0u; class_id < SUBJECT2_CLASS_COUNT; class_id++)
    {
        if(bindings[class_id].box_valid != bindings[class_id].target_valid)
        {
            return 0u;
        }
        if(0u != bindings[class_id].box_valid)
        {
            any = 1u;
        }
    }
    return any;
}

subject2_track_result_enum subject2_track_active_box(
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT],
    uint8 active_class,
    const uint16 *old_boxes,
    uint8 old_box_count,
    const uint16 *new_boxes,
    uint8 new_box_count)
{
    uint16 unmatched_cell = INVALID_STATE;
    uint8 class_id;
    uint8 index;
    uint8 active_binding_count = 0u;
    uint8 unmatched_count = 0u;
    uint8 belongs_to_non_active;

    if((0 == bindings) || (0 == old_boxes) || (0 == new_boxes) ||
       (SUBJECT2_CLASS_COUNT <= active_class) ||
       (0u == bindings[active_class].box_valid) ||
       (0u != bindings[active_class].completed) ||
       (old_box_count != new_box_count))
    {
        return SUBJECT2_TRACK_AMBIGUOUS;
    }

    for(class_id = 0u; class_id < SUBJECT2_CLASS_COUNT; class_id++)
    {
        if((0u == bindings[class_id].box_valid) ||
           (0u != bindings[class_id].completed))
        {
            continue;
        }
        active_binding_count++;
        if(0u == cell_in_list(bindings[class_id].box_cell,
                              old_boxes, old_box_count))
        {
            return SUBJECT2_TRACK_AMBIGUOUS;
        }
        if((class_id != active_class) &&
           (0u == cell_in_list(bindings[class_id].box_cell,
                               new_boxes, new_box_count)))
        {
            return SUBJECT2_TRACK_AMBIGUOUS;
        }
    }
    if(active_binding_count != old_box_count)
    {
        return SUBJECT2_TRACK_AMBIGUOUS;
    }

    for(index = 0u; index < new_box_count; index++)
    {
        belongs_to_non_active = 0u;
        for(class_id = 0u; class_id < SUBJECT2_CLASS_COUNT; class_id++)
        {
            if((class_id != active_class) &&
               (0u != bindings[class_id].box_valid) &&
               (0u == bindings[class_id].completed) &&
               (new_boxes[index] == bindings[class_id].box_cell))
            {
                belongs_to_non_active = 1u;
                break;
            }
        }
        if(0u == belongs_to_non_active)
        {
            unmatched_cell = new_boxes[index];
            unmatched_count++;
        }
    }
    if(1u != unmatched_count)
    {
        return SUBJECT2_TRACK_AMBIGUOUS;
    }
    if(unmatched_cell == bindings[active_class].box_cell)
    {
        return SUBJECT2_TRACK_UNCHANGED;
    }
    bindings[active_class].box_cell = unmatched_cell;
    return SUBJECT2_TRACK_MOVED;
}

uint8 subject2_normalize_center_map(
    const map_source_struct *source,
    uint16 center_col_q,
    uint16 center_row_q,
    map_source_struct *normalized,
    char normalized_rows[MAP_ROWS][MAP_COLS + 1],
    uint8 *car_row,
    uint8 *car_col)
{
    uint8 source_row;
    uint8 source_col;
    uint8 median_row;
    uint8 median_col;
    char destination;

    if((0 == source) || (0 == normalized) || (0 == normalized_rows) ||
       (0 == car_row) || (0 == car_col) ||
       (center_col_q >= (MAP_COLS * 100u)) ||
       (center_row_q >= (MAP_ROWS * 100u)) ||
       (0 == map_find_car(source, &source_row, &source_col, 0)))
    {
        return 0u;
    }

    median_col = (uint8)(center_col_q / 100u);
    median_row = (uint8)(center_row_q / 100u);
    if((absolute_difference(source_row, median_row) > 1u) ||
       (absolute_difference(source_col, median_col) > 1u))
    {
        return 0u;
    }

    destination = source->rows[median_row][median_col];
    if(('.' != destination) && ('T' != destination) &&
       ('C' != destination) && ('+' != destination))
    {
        return 0u;
    }

    map_source_snapshot(normalized, normalized_rows, source);
    if((source_row != median_row) || (source_col != median_col))
    {
        normalized_rows[source_row][source_col] =
            ('+' == normalized_rows[source_row][source_col]) ? 'T' : '.';
        normalized_rows[median_row][median_col] =
            ('T' == destination) ? '+' : 'C';
    }
    *car_row = median_row;
    *car_col = median_col;
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
            if((symbol == source->rows[row][col]) ||
               (('T' == symbol) && ('+' == source->rows[row][col])))
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

static int8 object_index_for_class(const subject2_object_struct *objects,
                                   uint8 count,
                                   uint8 class_id)
{
    uint8 index;

    for(index = 0u; index < count; index++)
    {
        if((0u != objects[index].recognized) &&
           (class_id == objects[index].class_id))
        {
            return (int8)index;
        }
    }
    return -1;
}

subject2_sync_result_enum subject2_reconcile_objects(
    const map_source_struct *source,
    subject2_object_struct box_objects[MAX_BOXES],
    uint8 *box_count,
    subject2_object_struct target_objects[MAX_BOXES],
    uint8 *target_count,
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT],
    uint8 strict_push_tracking,
    uint8 active_class,
    subject2_sync_update_struct *update)
{
    subject2_object_struct next_boxes[MAX_BOXES];
    subject2_object_struct next_targets[MAX_BOXES];
    subject2_binding_struct next_bindings[SUBJECT2_CLASS_COUNT];
    uint16 new_boxes[MAX_BOXES];
    uint16 new_targets[MAX_BOXES];
    uint16 remaining_old_boxes[MAX_BOXES];
    uint8 remaining_old_classes[MAX_BOXES];
    uint8 old_box_matched[MAX_BOXES] = {0};
    uint8 new_box_matched[MAX_BOXES] = {0};
    uint8 new_box_count = 0u;
    uint8 new_target_count = 0u;
    uint8 remaining_old_count = 0u;
    uint8 next_box_count = 0u;
    uint8 next_target_count = 0u;
    uint8 unmatched_old_count = 0u;
    uint8 unmatched_new_count = 0u;
    uint8 unmatched_old_index = 0u;
    uint8 unmatched_new_index = 0u;
    uint8 known_completed_removed = 0u;
    uint8 reduction;
    uint8 unknown_removed;
    uint8 class_id;
    uint8 index;
    int8 old_index;
    subject2_sync_update_struct next_update;
    subject2_sync_result_enum result = SUBJECT2_SYNC_OK;

    (void)active_class;
    if(0 != update)
    {
        memset(update, 0, sizeof(*update));
    }
    if((0 == source) || (0 == box_objects) || (0 == box_count) ||
       (0 == target_objects) || (0 == target_count) ||
       (0 == bindings) || (0 == update) ||
       (*box_count > MAX_BOXES) || (*target_count > MAX_BOXES) ||
       (0u == collect_cells_allow_empty(source, 'B', new_boxes, &new_box_count)) ||
       (0u == collect_cells_allow_empty(source, 'T', new_targets, &new_target_count)) ||
       (new_box_count != new_target_count) ||
       (new_box_count > *box_count) || (new_target_count > *target_count))
    {
        return SUBJECT2_SYNC_AMBIGUOUS;
    }

    memset(&next_update, 0, sizeof(next_update));
    memset(next_boxes, 0, sizeof(next_boxes));
    memset(next_targets, 0, sizeof(next_targets));
    memcpy(next_bindings, bindings, sizeof(next_bindings));

    for(index = 0u; index < new_target_count; index++)
    {
        old_index = object_index_for_cell(target_objects, *target_count,
                                          new_targets[index]);
        if(old_index < 0)
        {
            return SUBJECT2_SYNC_AMBIGUOUS;
        }
        next_targets[next_target_count++] = target_objects[(uint8)old_index];
    }

    for(class_id = 0u; class_id < SUBJECT2_CLASS_COUNT; class_id++)
    {
        if((0u == next_bindings[class_id].box_valid) ||
           (0u == next_bindings[class_id].target_valid) ||
           (0u != next_bindings[class_id].completed) ||
           (0u != cell_in_list(next_bindings[class_id].target_cell,
                               new_targets, new_target_count)))
        {
            continue;
        }
        next_bindings[class_id].completed = 1u;
        next_update.completed_count++;
    }

    if(0u != strict_push_tracking)
    {
        for(class_id = 0u; class_id < SUBJECT2_CLASS_COUNT; class_id++)
        {
            if((0u != next_bindings[class_id].box_valid) &&
               (0u == next_bindings[class_id].completed))
            {
                if(remaining_old_count >= MAX_BOXES)
                {
                    return SUBJECT2_SYNC_AMBIGUOUS;
                }
                remaining_old_boxes[remaining_old_count] =
                    next_bindings[class_id].box_cell;
                remaining_old_classes[remaining_old_count] = class_id;
                remaining_old_count++;
            }
        }
        if(remaining_old_count != new_box_count)
        {
            return SUBJECT2_SYNC_AMBIGUOUS;
        }

        for(index = 0u; index < remaining_old_count; index++)
        {
            uint8 new_index;

            old_index = object_index_for_class(box_objects, *box_count,
                                               remaining_old_classes[index]);
            if(old_index < 0)
            {
                return SUBJECT2_SYNC_AMBIGUOUS;
            }
            for(new_index = 0u; new_index < new_box_count; new_index++)
            {
                if((0u == new_box_matched[new_index]) &&
                   (remaining_old_boxes[index] == new_boxes[new_index]))
                {
                    next_boxes[new_index] = box_objects[(uint8)old_index];
                    next_boxes[new_index].cell = new_boxes[new_index];
                    old_box_matched[index] = 1u;
                    new_box_matched[new_index] = 1u;
                    break;
                }
            }
        }

        for(index = 0u; index < remaining_old_count; index++)
        {
            if(0u == old_box_matched[index])
            {
                unmatched_old_count++;
                unmatched_old_index = index;
            }
        }
        for(index = 0u; index < new_box_count; index++)
        {
            if(0u == new_box_matched[index])
            {
                unmatched_new_count++;
                unmatched_new_index = index;
            }
        }
        if(unmatched_old_count != unmatched_new_count)
        {
            return SUBJECT2_SYNC_AMBIGUOUS;
        }
        if(1u == unmatched_old_count)
        {
            class_id = remaining_old_classes[unmatched_old_index];
            old_index = object_index_for_class(box_objects, *box_count, class_id);
            if(old_index < 0)
            {
                return SUBJECT2_SYNC_AMBIGUOUS;
            }
            next_boxes[unmatched_new_index] = box_objects[(uint8)old_index];
            next_boxes[unmatched_new_index].cell = new_boxes[unmatched_new_index];
            next_bindings[class_id].box_cell = new_boxes[unmatched_new_index];
        }
        else if(unmatched_old_count > 1u)
        {
            for(index = 0u; index < remaining_old_count; index++)
            {
                if(0u == old_box_matched[index])
                {
                    class_id = remaining_old_classes[index];
                    next_bindings[class_id].box_valid = 0u;
                    next_bindings[class_id].box_cell = INVALID_STATE;
                }
            }
            for(index = 0u; index < new_box_count; index++)
            {
                if(0u == new_box_matched[index])
                {
                    next_boxes[index].cell = new_boxes[index];
                    next_boxes[index].class_id = SUBJECT2_INVALID_CLASS;
                    next_boxes[index].recognized = 0u;
                    next_boxes[index].tried_observation_mask = 0u;
                }
            }
            next_update.need_box_scan = 1u;
            result = SUBJECT2_SYNC_RESCAN;
        }
        next_box_count = new_box_count;
    }
    else
    {
        for(index = 0u; index < new_box_count; index++)
        {
            old_index = object_index_for_cell(box_objects, *box_count,
                                              new_boxes[index]);
            if(old_index >= 0)
            {
                if((0u != box_objects[(uint8)old_index].recognized) &&
                   (box_objects[(uint8)old_index].class_id < SUBJECT2_CLASS_COUNT) &&
                   (0u != next_bindings[box_objects[(uint8)old_index].class_id].completed))
                {
                    continue;
                }
                next_boxes[index] = box_objects[(uint8)old_index];
                old_box_matched[(uint8)old_index] = 1u;
                new_box_matched[index] = 1u;
            }
        }

        for(index = 0u; index < *box_count; index++)
        {
            if(0u != old_box_matched[index])
            {
                continue;
            }
            if((0u != box_objects[index].recognized) &&
               (box_objects[index].class_id < SUBJECT2_CLASS_COUNT) &&
               (0u != next_bindings[box_objects[index].class_id].completed))
            {
                old_box_matched[index] = 1u;
                known_completed_removed++;
                continue;
            }
            unmatched_old_count++;
            unmatched_old_index = index;
        }
        for(index = 0u; index < new_box_count; index++)
        {
            if(0u == new_box_matched[index])
            {
                unmatched_new_count++;
                unmatched_new_index = index;
            }
        }

        reduction = (uint8)(*box_count - new_box_count);
        if(known_completed_removed > reduction)
        {
            return SUBJECT2_SYNC_AMBIGUOUS;
        }
        unknown_removed = (uint8)(reduction - known_completed_removed);
        if((1u == unmatched_old_count) &&
           (1u == unmatched_new_count) &&
           (0u == unknown_removed))
        {
            next_boxes[unmatched_new_index] = box_objects[unmatched_old_index];
            next_boxes[unmatched_new_index].cell = new_boxes[unmatched_new_index];
            if((0u != next_boxes[unmatched_new_index].recognized) &&
               (next_boxes[unmatched_new_index].class_id < SUBJECT2_CLASS_COUNT) &&
               (0u != next_bindings[next_boxes[unmatched_new_index].class_id].box_valid))
            {
                next_bindings[next_boxes[unmatched_new_index].class_id].box_cell =
                    new_boxes[unmatched_new_index];
            }
        }
        else if((0u != unmatched_old_count) || (0u != unmatched_new_count))
        {
            for(index = 0u; index < *box_count; index++)
            {
                if((0u == old_box_matched[index]) &&
                   (0u != box_objects[index].recognized) &&
                   (box_objects[index].class_id < SUBJECT2_CLASS_COUNT) &&
                   (0u == next_bindings[box_objects[index].class_id].completed))
                {
                    next_bindings[box_objects[index].class_id].box_valid = 0u;
                    next_bindings[box_objects[index].class_id].box_cell = INVALID_STATE;
                }
            }
            for(index = 0u; index < new_box_count; index++)
            {
                if(0u == new_box_matched[index])
                {
                    next_boxes[index].cell = new_boxes[index];
                    next_boxes[index].class_id = SUBJECT2_INVALID_CLASS;
                    next_boxes[index].recognized = 0u;
                    next_boxes[index].tried_observation_mask = 0u;
                }
            }
            next_update.need_box_scan = (0u != unmatched_new_count) ? 1u : 0u;
            result = SUBJECT2_SYNC_RESCAN;
        }
        next_box_count = new_box_count;
    }

    for(index = 0u; index < next_box_count; index++)
    {
        if(0u == next_boxes[index].recognized)
        {
            next_update.need_box_scan = 1u;
            result = SUBJECT2_SYNC_RESCAN;
        }
    }
    for(index = 0u; index < next_target_count; index++)
    {
        if(0u == next_targets[index].recognized)
        {
            next_update.need_target_scan = 1u;
            result = SUBJECT2_SYNC_RESCAN;
        }
    }

    memcpy(box_objects, next_boxes, sizeof(next_boxes));
    memcpy(target_objects, next_targets, sizeof(next_targets));
    memcpy(bindings, next_bindings, sizeof(next_bindings));
    *box_count = next_box_count;
    *target_count = next_target_count;
    *update = next_update;
    return result;
}
