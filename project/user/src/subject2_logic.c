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
            if(symbol == source->rows[row][col])
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
            if(symbol == source->rows[row][col])
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
