#include <stdio.h>
#include <string.h>
#include "map_utils.h"
#include "solver.h"
#include "subject2_logic.h"

typedef struct
{
    char rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct source;
} test_map_struct;

static void init_map(test_map_struct *map, uint8 car_row, uint8 car_col)
{
    uint8 row;
    uint8 col;

    memset(map, 0, sizeof(*map));
    map->source.name = "subject2-logic";
    for(row = 0u; row < MAP_ROWS; row++)
    {
        for(col = 0u; col < MAP_COLS; col++)
        {
            map->rows[row][col] = ((0u == row) || ((MAP_ROWS - 1u) == row) ||
                                   (0u == col) || ((MAP_COLS - 1u) == col)) ? '#' : '.';
        }
        map->rows[row][MAP_COLS] = '\0';
        map->source.rows[row] = map->rows[row];
    }
    map->rows[car_row][car_col] = 'C';
}

static uint8 run_case(const char *name, uint8 passed)
{
    printf("%-30s %s\n", name, (0u != passed) ? "PASS" : "FAIL");
    return passed;
}

static uint8 collect_and_select_nearest(void)
{
    test_map_struct map;
    subject2_object_struct objects[MAX_BOXES];
    subject2_observation_plan_struct plan;
    solve_result_struct path;
    uint8 object_count = 0u;

    init_map(&map, 5u, 5u);
    map.rows[5][8] = 'B';
    map.rows[8][8] = 'B';

    if(0 == subject2_collect_objects(&map.source, 'B', objects, &object_count))
    {
        return 0u;
    }
    if((2u != object_count) ||
       (map_cell_index(5u, 8u) != objects[0].cell) ||
       (SUBJECT2_INVALID_CLASS != objects[0].class_id))
    {
        return 0u;
    }
    if(0 == subject2_select_observation(&map.source, objects, object_count, 0.0f,
                                         &plan, &path))
    {
        return 0u;
    }
    return ((0u == plan.object_index) &&
            (5u == plan.row) && (7u == plan.col) &&
            (SUBJECT2_OBSERVE_LEFT == plan.observation_bit) &&
            (2u == path.action_count)) ? 1u : 0u;
}

static uint8 ambiguity_and_retry_mask_work(void)
{
    test_map_struct map;
    subject2_object_struct objects[MAX_BOXES];
    subject2_observation_plan_struct first;
    subject2_observation_plan_struct second;
    solve_result_struct path;
    uint8 object_count = 0u;

    init_map(&map, 5u, 5u);
    map.rows[5][6] = 'B';
    map.rows[5][8] = 'B';
    if(0 == subject2_collect_objects(&map.source, 'B', objects, &object_count))
    {
        return 0u;
    }
    if(0 == subject2_select_observation(&map.source, objects, object_count, 0.0f,
                                         &first, &path))
    {
        return 0u;
    }
    if((5u == first.row) && (7u == first.col))
    {
        return 0u;
    }

    objects[first.object_index].tried_observation_mask |= first.observation_bit;
    if(0 == subject2_select_observation(&map.source, objects, object_count, 0.0f,
                                         &second, &path))
    {
        return 0u;
    }
    return ((first.object_index != second.object_index) ||
            (first.observation_bit != second.observation_bit)) ? 1u : 0u;
}

static uint8 observation_yaw_matches_four_directions(void)
{
    static const uint8 car_rows[4] = {6u, 5u, 4u, 5u};
    static const uint8 car_cols[4] = {5u, 4u, 5u, 6u};
    static const float expected_yaw[4] = {180.0f, 90.0f, 0.0f, 270.0f};
    uint8 index;

    for(index = 0u; index < 4u; index++)
    {
        test_map_struct map;
        subject2_object_struct objects[MAX_BOXES];
        subject2_observation_plan_struct plan;
        solve_result_struct path;
        uint8 object_count = 0u;

        init_map(&map, car_rows[index], car_cols[index]);
        map.rows[5][5] = 'B';
        map.rows[4][5] = (car_rows[index] == 4u) ? 'C' : '#';
        map.rows[6][5] = (car_rows[index] == 6u) ? 'C' : '#';
        map.rows[5][4] = (car_cols[index] == 4u) ? 'C' : '#';
        map.rows[5][6] = (car_cols[index] == 6u) ? 'C' : '#';

        if((0u == subject2_collect_objects(&map.source, 'B', objects, &object_count)) ||
           (0u == subject2_select_observation(&map.source, objects, object_count,
                                               0.0f,
                                               &plan, &path)) ||
           (expected_yaw[index] != plan.target_yaw_deg))
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8 classifier_requires_consecutive_confident_samples(void)
{
    subject2_classifier_struct filter;
    uint8 confirmed = SUBJECT2_INVALID_CLASS;

    subject2_classifier_reset(&filter);
    if(0 != subject2_classifier_push(&filter, 8u, 740u, 750u, 3u, &confirmed))
    {
        return 0u;
    }
    if(0 != subject2_classifier_push(&filter, 8u, 900u, 750u, 3u, &confirmed))
    {
        return 0u;
    }
    if(0 != subject2_classifier_push(&filter, 4u, 900u, 750u, 3u, &confirmed))
    {
        return 0u;
    }
    if((4u != filter.candidate_class) || (1u != filter.consecutive_count))
    {
        return 0u;
    }
    if(0 != subject2_classifier_push(&filter, 4u, 910u, 750u, 3u, &confirmed))
    {
        return 0u;
    }
    return ((0 != subject2_classifier_push(&filter, 4u, 920u, 750u, 3u, &confirmed)) &&
            (4u == confirmed)) ? 1u : 0u;
}

static uint8 equal_distance_prefers_smaller_turn(void)
{
    test_map_struct map;
    subject2_object_struct objects[MAX_BOXES];
    subject2_observation_plan_struct plan;
    solve_result_struct path;
    uint8 object_count = 0u;

    init_map(&map, 4u, 4u);
    map.rows[5][5] = 'B';
    if((0u == subject2_collect_objects(&map.source, 'B', objects, &object_count)) ||
       (0u == subject2_select_observation(&map.source, objects, object_count,
                                           90.0f, &plan, &path)))
    {
        return 0u;
    }
    return ((1u == path.action_count) &&
            (5u == plan.row) && (4u == plan.col) &&
            (SUBJECT2_OBSERVE_LEFT == plan.observation_bit) &&
            (90.0f == plan.target_yaw_deg)) ? 1u : 0u;
}

static uint8 equal_distance_handles_yaw_wrap_and_stable_tie(void)
{
    test_map_struct map;
    subject2_object_struct objects[MAX_BOXES];
    subject2_observation_plan_struct plan;
    solve_result_struct path;
    uint8 object_count = 0u;

    init_map(&map, 4u, 4u);
    map.rows[5][5] = 'B';
    if(0u == subject2_collect_objects(&map.source, 'B', objects, &object_count))
    {
        return 0u;
    }
    if((0u == subject2_select_observation(&map.source, objects, object_count,
                                           359.0f, &plan, &path)) ||
       (SUBJECT2_OBSERVE_UP != plan.observation_bit) ||
       (0.0f != plan.target_yaw_deg))
    {
        return 0u;
    }
    if((0u == subject2_select_observation(&map.source, objects, object_count,
                                           45.0f, &plan, &path)) ||
       (SUBJECT2_OBSERVE_UP != plan.observation_bit))
    {
        return 0u;
    }
    return 1u;
}

static uint8 shorter_path_beats_zero_turn(void)
{
    test_map_struct map;
    subject2_object_struct objects[MAX_BOXES];
    subject2_observation_plan_struct plan;
    solve_result_struct path;
    uint8 object_count = 0u;

    init_map(&map, 4u, 4u);
    map.rows[5][5] = 'B';
    if((0u == subject2_collect_objects(&map.source, 'B', objects, &object_count)) ||
       (0u == subject2_select_observation(&map.source, objects, object_count,
                                           180.0f, &plan, &path)))
    {
        return 0u;
    }
    /* LEFT 只走1格但需转90度；DOWN 不需转向但要走3格。 */
    return ((SUBJECT2_OBSERVE_LEFT == plan.observation_bit) &&
            (1u == path.action_count) &&
            (90.0f == plan.target_yaw_deg)) ? 1u : 0u;
}

static uint8 car_on_target_is_collected(void)
{
    test_map_struct map;
    subject2_object_struct objects[MAX_BOXES];
    uint16 cells[MAX_BOXES];
    map_scan_stats_struct stats;
    uint8 object_count = 0u;
    uint8 cell_count = 0u;

    init_map(&map, 5u, 5u);
    map.rows[5][5] = '+';
    map_scan_stats(&map.source, &stats);
    if((1u != stats.car_count) || (1u != stats.target_count) ||
       (5u != stats.car_row) || (5u != stats.car_col))
    {
        return 0u;
    }
    if((0u == subject2_collect_objects(&map.source, 'T', objects, &object_count)) ||
       (1u != object_count) ||
       (map_cell_index(5u, 5u) != objects[0].cell))
    {
        return 0u;
    }
    return ((0u != subject2_collect_cells(&map.source, 'T', cells, &cell_count)) &&
            (1u == cell_count) &&
            (map_cell_index(5u, 5u) == cells[0])) ? 1u : 0u;
}

static uint8 center_map_same_cell_is_preserved(void)
{
    test_map_struct map;
    uint8 car_row = 0u;
    uint8 car_col = 0u;

    init_map(&map, 5u, 5u);
    map.rows[4][8] = 'B';
    map.rows[7][9] = 'T';

    if(0u == map_validate_player_center(&map.source, 550u, 550u,
                                        &car_row, &car_col))
    {
        return 0u;
    }
    return ((5u == car_row) && (5u == car_col) &&
            ('C' == map.rows[5][5]) &&
            ('B' == map.rows[4][8]) &&
            ('T' == map.rows[7][9])) ? 1u : 0u;
}

static uint8 center_map_target_cell_is_preserved(void)
{
    test_map_struct map;
    uint8 car_row = 0u;
    uint8 car_col = 0u;

    init_map(&map, 5u, 6u);
    map.rows[5][6] = '+';

    if(0u == map_validate_player_center(&map.source, 650u, 550u,
                                        &car_row, &car_col))
    {
        return 0u;
    }
    return ((5u == car_row) && (6u == car_col) &&
            ('+' == map.rows[5][6])) ? 1u : 0u;
}

static uint8 center_map_invalid_reference_is_rejected(void)
{
    test_map_struct map;
    uint8 car_row = 0u;
    uint8 car_col = 0u;

    init_map(&map, 5u, 5u);
    if(0u != map_validate_player_center(&map.source, 750u, 550u,
                                        &car_row, &car_col))
    {
        return 0u;
    }
    if(0u != map_validate_player_center(&map.source,
                                        (uint16)(MAP_COLS * 100u), 550u,
                                        &car_row, &car_col))
    {
        return 0u;
    }
    map.rows[5][6] = 'B';
    return (0u == map_validate_player_center(&map.source, 650u, 550u,
                                              &car_row, &car_col)) ? 1u : 0u;
}

static uint8 multiple_car_map_has_no_position(void)
{
    test_map_struct map;
    map_scan_stats_struct stats;
    uint8 car_row = 0u;
    uint8 car_col = 0u;
    uint8 car_count = 0u;

    init_map(&map, 5u, 5u);
    map.rows[5][6] = 'C';
    map_scan_stats(&map.source, &stats);
    return ((2u == stats.car_count) &&
            (0xFFu == stats.car_row) && (0xFFu == stats.car_col) &&
            (0u == map_find_car(&map.source, &car_row, &car_col, &car_count)) &&
            (2u == car_count) &&
            (0xFFu == car_row) && (0xFFu == car_col)) ? 1u : 0u;
}

static uint8 player_center_requires_same_cell(void)
{
    test_map_struct map;
    uint8 row = 0u;
    uint8 col = 0u;

    init_map(&map, 5u, 5u);
    return ((0u != map_validate_player_center(&map.source, 599u, 550u,
                                               &row, &col)) &&
            (5u == row) && (5u == col) &&
            (0u == map_validate_player_center(&map.source, 600u, 550u,
                                               &row, &col))) ? 1u : 0u;
}

static uint8 stability_tracker_counts_new_equal_frames(void)
{
    test_map_struct first;
    test_map_struct changed;
    map_stability_tracker_struct tracker;

    init_map(&first, 5u, 5u);
    init_map(&changed, 5u, 6u);
    map_stability_tracker_reset(&tracker, 10u);
    if(MAP_STABILITY_NO_NEW_FRAME !=
       map_stability_tracker_push(&tracker, 10u, &first.source, 2u)) return 0u;
    if(MAP_STABILITY_PENDING !=
       map_stability_tracker_push(&tracker, 11u, &first.source, 2u)) return 0u;
    if(MAP_STABILITY_READY !=
       map_stability_tracker_push(&tracker, 12u, &first.source, 2u)) return 0u;
    if(MAP_STABILITY_PENDING !=
       map_stability_tracker_push(&tracker, 13u, &changed.source, 2u)) return 0u;
    map_stability_tracker_reset_candidate(&tracker);
    return ((0u == tracker.candidate_valid) &&
            (0u == tracker.stable_count) &&
            (13u == tracker.last_frame)) ? 1u : 0u;
}

static void recognize_object(subject2_object_struct *object, uint8 class_id)
{
    object->recognized = 1u;
    object->class_id = class_id;
}

static uint8 last_target_elimination_handles_duplicates(void)
{
    subject2_object_struct boxes[3] = {0};
    subject2_object_struct targets[3] = {0};
    uint8 target_index = 0xFFu;
    uint8 class_id = SUBJECT2_INVALID_CLASS;

    recognize_object(&boxes[0], 2u);
    recognize_object(&boxes[1], 2u);
    recognize_object(&boxes[2], 5u);
    recognize_object(&targets[0], 2u);
    recognize_object(&targets[1], 5u);
    targets[2].class_id = SUBJECT2_INVALID_CLASS;
    return ((0u != subject2_infer_last_target_class(boxes, 3u, targets, 3u,
                                                     &target_index, &class_id)) &&
            (2u == target_index) && (2u == class_id) &&
            (0u == targets[2].recognized) &&
            (SUBJECT2_INVALID_CLASS == targets[2].class_id)) ? 1u : 0u;
}

static uint8 last_target_elimination_handles_single_pair(void)
{
    subject2_object_struct box = {0};
    subject2_object_struct target = {0};
    uint8 target_index = 0xFFu;
    uint8 class_id = SUBJECT2_INVALID_CLASS;

    recognize_object(&box, 7u);
    target.class_id = SUBJECT2_INVALID_CLASS;
    return ((0u != subject2_infer_last_target_class(&box, 1u, &target, 1u,
                                                     &target_index, &class_id)) &&
            (0u == target_index) && (7u == class_id)) ? 1u : 0u;
}

static uint8 last_target_elimination_rejects_unsafe_counts(void)
{
    subject2_object_struct boxes[3] = {0};
    subject2_object_struct targets[3] = {0};
    uint8 target_index;
    uint8 class_id;

    recognize_object(&boxes[0], 2u);
    recognize_object(&boxes[1], 5u);
    recognize_object(&boxes[2], 7u);
    recognize_object(&targets[0], 2u);
    targets[1].class_id = SUBJECT2_INVALID_CLASS;
    targets[2].class_id = SUBJECT2_INVALID_CLASS;
    target_index = 0xA5u;
    class_id = 0x5Au;
    if((0u != subject2_infer_last_target_class(boxes, 3u, targets, 3u,
                                               &target_index, &class_id)) ||
       (0xA5u != target_index) || (0x5Au != class_id))
    {
        return 0u;
    }

    recognize_object(&targets[1], 2u);
    if(0u != subject2_infer_last_target_class(boxes, 3u, targets, 3u,
                                              &target_index, &class_id))
    {
        return 0u;
    }

    targets[1].class_id = 5u;
    boxes[2].recognized = 0u;
    if(0u != subject2_infer_last_target_class(boxes, 3u, targets, 3u,
                                              &target_index, &class_id))
    {
        return 0u;
    }

    boxes[2].recognized = 1u;
    boxes[2].class_id = SUBJECT2_CLASS_COUNT;
    return (0u == subject2_infer_last_target_class(boxes, 3u, targets, 3u,
                                                    &target_index, &class_id)) ? 1u : 0u;
}

static uint8 duplicate_class_counts_match(void)
{
    subject2_object_struct boxes[2] = {0};
    subject2_object_struct targets[2] = {0};

    recognize_object(&boxes[0], 8u);
    recognize_object(&boxes[1], 8u);
    recognize_object(&targets[0], 8u);
    recognize_object(&targets[1], 8u);
    return subject2_object_class_counts_match(boxes, 2u, targets, 2u);
}

static uint8 class_count_validation_rejects_invalid_objects(void)
{
    subject2_object_struct boxes[2] = {0};
    subject2_object_struct targets[2] = {0};

    recognize_object(&boxes[0], 8u);
    recognize_object(&boxes[1], 8u);
    recognize_object(&targets[0], 8u);
    if(0u != subject2_object_class_counts_match(boxes, 2u, targets, 1u))
    {
        return 0u;
    }

    recognize_object(&targets[1], 8u);
    boxes[1].recognized = 0u;
    if(0u != subject2_object_class_counts_match(boxes, 2u, targets, 2u))
    {
        return 0u;
    }

    boxes[1].recognized = 1u;
    boxes[1].class_id = SUBJECT2_CLASS_COUNT;
    return (0u == subject2_object_class_counts_match(boxes, 2u,
                                                      targets, 2u)) ? 1u : 0u;
}

static uint8 mismatched_classes_are_invalidated_locally(void)
{
    subject2_object_struct boxes[4] = {0};
    subject2_object_struct targets[4] = {0};
    static const uint8 box_classes[4] = {8u, 8u, 3u, 5u};
    static const uint8 target_classes[4] = {8u, 3u, 3u, 5u};
    uint8 need_box_scan = 0u;
    uint8 need_target_scan = 0u;
    uint8 index;

    for(index = 0u; index < 4u; index++)
    {
        recognize_object(&boxes[index], box_classes[index]);
        recognize_object(&targets[index], target_classes[index]);
        boxes[index].tried_observation_mask = 0x0Fu;
        targets[index].tried_observation_mask = 0x0Fu;
    }

    subject2_invalidate_mismatched_classes(boxes, 4u, targets, 4u,
                                           &need_box_scan, &need_target_scan);
    if((0u == need_box_scan) || (0u == need_target_scan))
    {
        return 0u;
    }
    for(index = 0u; index < 3u; index++)
    {
        if((0u != boxes[index].recognized) ||
           (SUBJECT2_INVALID_CLASS != boxes[index].class_id) ||
           (0u != boxes[index].tried_observation_mask) ||
           (0u != targets[index].recognized) ||
           (SUBJECT2_INVALID_CLASS != targets[index].class_id) ||
           (0u != targets[index].tried_observation_mask))
        {
            return 0u;
        }
    }
    return ((0u != boxes[3].recognized) && (5u == boxes[3].class_id) &&
            (0u != targets[3].recognized) && (5u == targets[3].class_id)) ? 1u : 0u;
}

static uint8 duplicate_class_push_plan_selects_shortest(void)
{
    test_map_struct map;
    subject2_object_struct boxes[2] = {0};
    subject2_object_struct targets[2] = {0};
    subject2_push_plan_struct plan;
    solve_result_struct selected;
    solve_result_struct candidate;
    uint16 best_actions = 0xFFFFu;
    uint16 expected_box = INVALID_STATE;
    uint16 expected_target = INVALID_STATE;
    uint8 box_index;
    uint8 target_index;

    init_map(&map, 5u, 5u);
    map.rows[5][6] = 'B';
    map.rows[7][6] = 'B';
    map.rows[5][9] = 'T';
    map.rows[7][9] = 'T';
    boxes[0].cell = map_cell_index(5u, 6u);
    boxes[1].cell = map_cell_index(7u, 6u);
    targets[0].cell = map_cell_index(5u, 9u);
    targets[1].cell = map_cell_index(7u, 9u);
    recognize_object(&boxes[0], 4u);
    recognize_object(&boxes[1], 4u);
    recognize_object(&targets[0], 4u);
    recognize_object(&targets[1], 4u);

    for(box_index = 0u; box_index < 2u; box_index++)
    {
        for(target_index = 0u; target_index < 2u; target_index++)
        {
            if((0u != solve_bound_box_path(&map.source,
                                            boxes[box_index].cell,
                                            targets[target_index].cell,
                                            &candidate)) &&
               (candidate.action_count < best_actions))
            {
                best_actions = candidate.action_count;
                expected_box = boxes[box_index].cell;
                expected_target = targets[target_index].cell;
            }
        }
    }
    if(INVALID_STATE == expected_box)
    {
        return 0u;
    }
    if(0u == subject2_select_push_plan(&map.source,
                                       boxes, 2u, targets, 2u,
                                       0u, 0u, INVALID_STATE, INVALID_STATE,
                                       &plan, &selected))
    {
        return 0u;
    }
    return ((4u == plan.class_id) &&
            (expected_box == plan.box_cell) &&
            (expected_target == plan.target_cell) &&
            (best_actions == selected.action_count)) ? 1u : 0u;
}

static uint8 push_retry_keeps_target_and_optional_box(void)
{
    test_map_struct map;
    subject2_object_struct boxes[2] = {0};
    subject2_object_struct targets[2] = {0};
    subject2_push_plan_struct plan;
    solve_result_struct selected;

    init_map(&map, 5u, 5u);
    map.rows[5][6] = 'B';
    map.rows[7][6] = 'B';
    map.rows[5][9] = 'T';
    map.rows[7][9] = 'T';
    boxes[0].cell = map_cell_index(5u, 6u);
    boxes[1].cell = map_cell_index(7u, 6u);
    targets[0].cell = map_cell_index(5u, 9u);
    targets[1].cell = map_cell_index(7u, 9u);
    recognize_object(&boxes[0], 4u);
    recognize_object(&boxes[1], 4u);
    recognize_object(&targets[0], 4u);
    recognize_object(&targets[1], 4u);

    if((0u == subject2_select_push_plan(&map.source,
                                        boxes, 2u, targets, 2u,
                                        1u, 0u, INVALID_STATE, targets[1].cell,
                                        &plan, &selected)) ||
       (targets[1].cell != plan.target_cell))
    {
        return 0u;
    }
    if(0u == subject2_select_push_plan(&map.source,
                                       boxes, 2u, targets, 2u,
                                       1u, 1u, boxes[1].cell, targets[1].cell,
                                       &plan, &selected))
    {
        return 0u;
    }
    return ((boxes[1].cell == plan.box_cell) &&
            (targets[1].cell == plan.target_cell)) ? 1u : 0u;
}

static uint8 object_sync_completes_one_duplicate_class(void)
{
    test_map_struct old_map;
    test_map_struct new_map;
    subject2_object_struct boxes[MAX_BOXES];
    subject2_object_struct targets[MAX_BOXES];
    subject2_sync_update_struct update;
    uint8 box_count = 0u;
    uint8 target_count = 0u;

    init_map(&old_map, 5u, 5u);
    old_map.rows[4][4] = 'B';
    old_map.rows[6][6] = 'B';
    old_map.rows[8][4] = 'T';
    old_map.rows[8][6] = 'T';
    subject2_collect_objects(&old_map.source, 'B', boxes, &box_count);
    subject2_collect_objects(&old_map.source, 'T', targets, &target_count);
    recognize_object(&boxes[0], 8u);
    recognize_object(&boxes[1], 8u);
    recognize_object(&targets[0], 8u);
    recognize_object(&targets[1], 8u);

    init_map(&new_map, 5u, 5u);
    new_map.rows[6][6] = 'B';
    new_map.rows[8][6] = 'T';
    if(SUBJECT2_SYNC_OK != subject2_reconcile_object_lists(
            &new_map.source, boxes, &box_count, targets, &target_count,
            map_cell_index(4u, 4u), map_cell_index(8u, 4u), &update))
    {
        return 0u;
    }
    return ((1u == box_count) && (1u == target_count) &&
            (8u == boxes[0].class_id) && (0u != boxes[0].recognized) &&
            (1u == update.completed_count) &&
            (0u != update.active_target_removed) &&
            (0u == update.active_box_valid)) ? 1u : 0u;
}

static uint8 object_sync_restores_single_class_deficit(void)
{
    test_map_struct old_map;
    test_map_struct new_map;
    subject2_object_struct boxes[MAX_BOXES];
    subject2_object_struct targets[MAX_BOXES];
    subject2_sync_update_struct update;
    uint16 moved_cell = map_cell_index(4u, 5u);
    uint8 box_count = 0u;
    uint8 target_count = 0u;

    init_map(&old_map, 5u, 5u);
    old_map.rows[4][4] = 'B';
    old_map.rows[6][6] = 'B';
    old_map.rows[8][4] = 'T';
    old_map.rows[8][6] = 'T';
    subject2_collect_objects(&old_map.source, 'B', boxes, &box_count);
    subject2_collect_objects(&old_map.source, 'T', targets, &target_count);
    recognize_object(&boxes[0], 8u);
    recognize_object(&boxes[1], 3u);
    recognize_object(&targets[0], 8u);
    recognize_object(&targets[1], 3u);

    init_map(&new_map, 5u, 5u);
    new_map.rows[4][5] = 'B';
    new_map.rows[6][6] = 'B';
    new_map.rows[8][4] = 'T';
    new_map.rows[8][6] = 'T';
    if(SUBJECT2_SYNC_OK != subject2_reconcile_object_lists(
            &new_map.source, boxes, &box_count, targets, &target_count,
            map_cell_index(4u, 4u), map_cell_index(8u, 4u), &update))
    {
        return 0u;
    }
    return ((8u == boxes[0].class_id) && (moved_cell == boxes[0].cell) &&
            (0u != update.active_box_valid) &&
            (moved_cell == update.active_box_cell) &&
            (0u == update.active_target_removed) &&
            (0u == update.need_box_scan)) ? 1u : 0u;
}

static uint8 object_sync_rescans_multiple_class_deficits(void)
{
    test_map_struct old_map;
    test_map_struct new_map;
    subject2_object_struct boxes[MAX_BOXES];
    subject2_object_struct targets[MAX_BOXES];
    subject2_sync_update_struct update;
    uint8 box_count = 0u;
    uint8 target_count = 0u;

    init_map(&old_map, 5u, 5u);
    old_map.rows[4][4] = 'B';
    old_map.rows[6][6] = 'B';
    old_map.rows[8][4] = 'T';
    old_map.rows[8][6] = 'T';
    subject2_collect_objects(&old_map.source, 'B', boxes, &box_count);
    subject2_collect_objects(&old_map.source, 'T', targets, &target_count);
    recognize_object(&boxes[0], 8u);
    recognize_object(&boxes[1], 3u);
    recognize_object(&targets[0], 8u);
    recognize_object(&targets[1], 3u);

    init_map(&new_map, 5u, 5u);
    new_map.rows[4][5] = 'B';
    new_map.rows[6][5] = 'B';
    new_map.rows[8][4] = 'T';
    new_map.rows[8][6] = 'T';
    if(SUBJECT2_SYNC_RESCAN != subject2_reconcile_object_lists(
            &new_map.source, boxes, &box_count, targets, &target_count,
            map_cell_index(4u, 4u), map_cell_index(8u, 4u), &update))
    {
        return 0u;
    }
    return ((0u == boxes[0].recognized) && (0u == boxes[1].recognized) &&
            (0u != update.need_box_scan) &&
            (0u == update.need_target_scan)) ? 1u : 0u;
}

static uint8 object_sync_accepts_nonactive_completion(void)
{
    test_map_struct old_map;
    test_map_struct new_map;
    subject2_object_struct boxes[MAX_BOXES];
    subject2_object_struct targets[MAX_BOXES];
    subject2_sync_update_struct update;
    uint8 box_count = 0u;
    uint8 target_count = 0u;

    init_map(&old_map, 5u, 5u);
    old_map.rows[4][4] = 'B';
    old_map.rows[6][6] = 'B';
    old_map.rows[8][4] = 'T';
    old_map.rows[8][6] = 'T';
    subject2_collect_objects(&old_map.source, 'B', boxes, &box_count);
    subject2_collect_objects(&old_map.source, 'T', targets, &target_count);
    recognize_object(&boxes[0], 3u);
    recognize_object(&boxes[1], 8u);
    recognize_object(&targets[0], 3u);
    recognize_object(&targets[1], 8u);

    init_map(&new_map, 5u, 5u);
    new_map.rows[6][6] = 'B';
    new_map.rows[8][6] = 'T';
    if(SUBJECT2_SYNC_OK != subject2_reconcile_object_lists(
            &new_map.source, boxes, &box_count, targets, &target_count,
            map_cell_index(6u, 6u), map_cell_index(8u, 6u), &update))
    {
        return 0u;
    }
    return ((1u == update.completed_count) &&
            (0u == update.active_target_removed) &&
            (0u != update.active_box_valid) &&
            (map_cell_index(6u, 6u) == update.active_box_cell) &&
            (8u == boxes[0].class_id)) ? 1u : 0u;
}

static uint8 object_sync_ambiguous_update_is_atomic(void)
{
    test_map_struct old_map;
    test_map_struct new_map;
    subject2_object_struct boxes[MAX_BOXES];
    subject2_object_struct targets[MAX_BOXES];
    subject2_object_struct original_boxes[MAX_BOXES];
    subject2_object_struct original_targets[MAX_BOXES];
    subject2_sync_update_struct update;
    uint8 box_count = 0u;
    uint8 target_count = 0u;

    init_map(&old_map, 5u, 5u);
    old_map.rows[4][4] = 'B';
    old_map.rows[8][4] = 'T';
    subject2_collect_objects(&old_map.source, 'B', boxes, &box_count);
    subject2_collect_objects(&old_map.source, 'T', targets, &target_count);
    recognize_object(&boxes[0], 8u);
    recognize_object(&targets[0], 8u);
    memcpy(original_boxes, boxes, sizeof(original_boxes));
    memcpy(original_targets, targets, sizeof(original_targets));

    init_map(&new_map, 5u, 5u);
    new_map.rows[4][4] = 'B';
    new_map.rows[8][5] = 'T';
    memset(&update, 0xA5, sizeof(update));
    if(SUBJECT2_SYNC_AMBIGUOUS != subject2_reconcile_object_lists(
            &new_map.source, boxes, &box_count, targets, &target_count,
            map_cell_index(4u, 4u), map_cell_index(8u, 4u), &update))
    {
        return 0u;
    }
    return ((1u == box_count) && (1u == target_count) &&
            (0 == memcmp(boxes, original_boxes, sizeof(original_boxes))) &&
            (0 == memcmp(targets, original_targets, sizeof(original_targets))) &&
            (0u == update.need_box_scan) &&
            (0u == update.need_target_scan) &&
            (0u == update.completed_count)) ? 1u : 0u;
}

static uint8 box_on_target_char_has_dual_semantics(void)
{
    test_map_struct map;
    subject2_object_struct boxes[MAX_BOXES];
    subject2_object_struct targets[MAX_BOXES];
    solve_result_struct path;
    map_scan_stats_struct stats;
    uint16 overlap_cell = map_cell_index(5u, 5u);
    uint8 box_count = 0u;
    uint8 target_count = 0u;

    init_map(&map, 5u, 4u);
    map.rows[5][5] = '*';
    map.rows[5][8] = 'T';
    map.rows[7][7] = 'B';
    map.rows[7][9] = 'T';
    map_scan_stats(&map.source, &stats);
    if((2u != stats.box_count) || (3u != stats.target_count))
    {
        return 0u;
    }
    if((0u == subject2_collect_objects(&map.source, 'B', boxes, &box_count)) ||
       (0u == subject2_collect_objects(&map.source, 'T', targets, &target_count)) ||
       (2u != box_count) || (3u != target_count) ||
       (overlap_cell != boxes[0].cell) ||
       (overlap_cell != targets[0].cell))
    {
        return 0u;
    }
    map.rows[7][9] = '.';
    return solve_bound_box_path(&map.source, overlap_cell,
                                map_cell_index(5u, 8u), &path);
}

static void build_transit_result(solve_result_struct *result,
                                 uint16 action_end)
{
    memset(result, 0, sizeof(*result));
    result->actions[0] = 'R';
    result->actions[1] = 'R';
    result->actions[2] = 'R';
    result->actions[3] = '\0';
    result->action_count = 3u;
    result->waypoint_count = 1u;
    result->waypoints[0].action = 'R';
    result->waypoints[0].action_end = action_end;
}

static uint8 planned_transit_target_is_normalized(void)
{
    test_map_struct map;
    solve_result_struct result;
    char normalized_rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct normalized;
    uint16 overlap_cell = INVALID_STATE;

    init_map(&map, 5u, 5u);
    map.rows[5][6] = 'T';
    map.rows[5][8] = 'T';
    map.rows[7][7] = 'B';
    build_transit_result(&result, 1u);
    if(0u == subject2_normalize_transit_box_overlap(
                  &map.source, &result, 0u,
                  map_cell_index(5u, 5u), map_cell_index(5u, 8u),
                  'R', normalized_rows, &normalized, &overlap_cell))
    {
        return 0u;
    }
    return ((map_cell_index(5u, 6u) == overlap_cell) &&
            ('*' == normalized.rows[5][6]) &&
            ('T' == map.rows[5][6])) ? 1u : 0u;
}

static uint8 lagging_transit_target_is_normalized(void)
{
    test_map_struct map;
    solve_result_struct result;
    char normalized_rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct normalized;
    uint16 overlap_cell = INVALID_STATE;

    init_map(&map, 5u, 5u);
    map.rows[5][6] = 'T';
    map.rows[5][8] = 'T';
    map.rows[7][7] = 'B';
    build_transit_result(&result, 3u);
    if(0u == subject2_normalize_transit_box_overlap(
                  &map.source, &result, 0u,
                  map_cell_index(5u, 5u), map_cell_index(5u, 8u),
                  'R', normalized_rows, &normalized, &overlap_cell))
    {
        return 0u;
    }
    return ((map_cell_index(5u, 6u) == overlap_cell) &&
            ('*' == normalized.rows[5][6]) &&
            ('T' == map.rows[5][6])) ? 1u : 0u;
}

static uint8 final_target_is_not_normalized_as_transit(void)
{
    test_map_struct map;
    solve_result_struct result;
    char normalized_rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct normalized;
    uint16 overlap_cell = INVALID_STATE;

    init_map(&map, 5u, 7u);
    map.rows[5][8] = 'T';
    build_transit_result(&result, 3u);
    return (0u == subject2_normalize_transit_box_overlap(
                       &map.source, &result, 0u,
                       map_cell_index(5u, 5u), map_cell_index(5u, 8u),
                       'R', normalized_rows, &normalized,
                       &overlap_cell)) ? 1u : 0u;
}

static uint8 ambiguous_transit_targets_are_rejected(void)
{
    test_map_struct map;
    solve_result_struct result;
    char normalized_rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct normalized;
    uint16 overlap_cell = INVALID_STATE;

    init_map(&map, 5u, 5u);
    map.rows[5][6] = 'T';
    map.rows[5][7] = 'T';
    map.rows[5][8] = 'T';
    map.rows[7][7] = 'B';
    map.rows[7][9] = 'B';
    build_transit_result(&result, 3u);
    return (0u == subject2_normalize_transit_box_overlap(
                       &map.source, &result, 0u,
                       map_cell_index(5u, 5u), map_cell_index(5u, 8u),
                       'R', normalized_rows, &normalized,
                       &overlap_cell)) ? 1u : 0u;
}

int main(void)
{
    uint8 passed = 1u;

    passed &= run_case("collect-select-nearest", collect_and_select_nearest());
    passed &= run_case("ambiguity-retry-mask", ambiguity_and_retry_mask_work());
    passed &= run_case("observation-yaw", observation_yaw_matches_four_directions());
    passed &= run_case("observation-less-turn", equal_distance_prefers_smaller_turn());
    passed &= run_case("observation-yaw-wrap", equal_distance_handles_yaw_wrap_and_stable_tie());
    passed &= run_case("observation-path-first", shorter_path_beats_zero_turn());
    passed &= run_case("classifier-consecutive", classifier_requires_consecutive_confident_samples());
    passed &= run_case("car-on-target", car_on_target_is_collected());
    passed &= run_case("center-map-same-cell", center_map_same_cell_is_preserved());
    passed &= run_case("center-map-target", center_map_target_cell_is_preserved());
    passed &= run_case("center-map-invalid", center_map_invalid_reference_is_rejected());
    passed &= run_case("center-map-multiple", multiple_car_map_has_no_position());
    passed &= run_case("center-cell-strict", player_center_requires_same_cell());
    passed &= run_case("map-stability", stability_tracker_counts_new_equal_frames());
    passed &= run_case("class-count-duplicates", duplicate_class_counts_match());
    passed &= run_case("class-count-invalid", class_count_validation_rejects_invalid_objects());
    passed &= run_case("last-target-duplicates", last_target_elimination_handles_duplicates());
    passed &= run_case("last-target-single", last_target_elimination_handles_single_pair());
    passed &= run_case("last-target-reject", last_target_elimination_rejects_unsafe_counts());
    passed &= run_case("class-count-invalidate", mismatched_classes_are_invalidated_locally());
    passed &= run_case("push-plan-duplicates", duplicate_class_push_plan_selects_shortest());
    passed &= run_case("push-plan-retry", push_retry_keeps_target_and_optional_box());
    passed &= run_case("object-sync-duplicate-done", object_sync_completes_one_duplicate_class());
    passed &= run_case("object-sync-one-deficit", object_sync_restores_single_class_deficit());
    passed &= run_case("object-sync-multi-deficit", object_sync_rescans_multiple_class_deficits());
    passed &= run_case("object-sync-other-done", object_sync_accepts_nonactive_completion());
    passed &= run_case("object-sync-atomic", object_sync_ambiguous_update_is_atomic());
    passed &= run_case("box-on-target-dual", box_on_target_char_has_dual_semantics());
    passed &= run_case("transit-target-normalize", planned_transit_target_is_normalized());
    passed &= run_case("transit-target-lagging", lagging_transit_target_is_normalized());
    passed &= run_case("final-target-no-normalize", final_target_is_not_normalized_as_transit());
    passed &= run_case("transit-target-ambiguous", ambiguous_transit_targets_are_rejected());

    return (0u != passed) ? 0 : 1;
}
