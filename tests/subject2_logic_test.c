#include <stdio.h>
#include <string.h>
#include "map_utils.h"
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
    if(0 == subject2_select_observation(&map.source, objects, object_count,
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
    if(0 == subject2_select_observation(&map.source, objects, object_count,
                                        &first, &path))
    {
        return 0u;
    }
    if((5u == first.row) && (7u == first.col))
    {
        return 0u;
    }

    objects[first.object_index].tried_observation_mask |= first.observation_bit;
    if(0 == subject2_select_observation(&map.source, objects, object_count,
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

static uint8 bindings_are_unique_and_sets_match(void)
{
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT];

    subject2_bindings_clear(bindings);
    if(0 == subject2_bind_box(bindings, 8u, map_cell_index(2u, 3u)))
    {
        return 0u;
    }
    if(0 != subject2_bind_box(bindings, 8u, map_cell_index(4u, 5u)))
    {
        return 0u;
    }
    if(0 != subject2_binding_sets_match(bindings))
    {
        return 0u;
    }
    if(0 == subject2_bind_target(bindings, 8u, map_cell_index(7u, 9u)))
    {
        return 0u;
    }
    return subject2_binding_sets_match(bindings);
}

static uint8 active_box_identity_tracks_one_move(void)
{
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT];
    uint16 old_boxes[3] = {
        map_cell_index(2u, 2u), map_cell_index(3u, 3u), map_cell_index(4u, 4u)
    };
    uint16 new_boxes[3] = {
        map_cell_index(2u, 2u), map_cell_index(3u, 4u), map_cell_index(4u, 4u)
    };

    subject2_bindings_clear(bindings);
    subject2_bind_box(bindings, 1u, old_boxes[0]);
    subject2_bind_box(bindings, 3u, old_boxes[1]);
    subject2_bind_box(bindings, 7u, old_boxes[2]);

    if(SUBJECT2_TRACK_MOVED !=
       subject2_track_active_box(bindings, 3u, old_boxes, 3u, new_boxes, 3u))
    {
        return 0u;
    }
    return (new_boxes[1] == bindings[3].box_cell) ? 1u : 0u;
}

static uint8 multiple_changes_are_ambiguous(void)
{
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT];
    uint16 old_boxes[3] = {
        map_cell_index(2u, 2u), map_cell_index(3u, 3u), map_cell_index(4u, 4u)
    };
    uint16 new_boxes[3] = {
        map_cell_index(2u, 3u), map_cell_index(3u, 4u), map_cell_index(4u, 4u)
    };

    subject2_bindings_clear(bindings);
    subject2_bind_box(bindings, 1u, old_boxes[0]);
    subject2_bind_box(bindings, 3u, old_boxes[1]);
    subject2_bind_box(bindings, 7u, old_boxes[2]);

    return (SUBJECT2_TRACK_AMBIGUOUS ==
            subject2_track_active_box(bindings, 3u,
                                      old_boxes, 3u,
                                      new_boxes, 3u)) ? 1u : 0u;
}

static uint8 center_map_same_cell_is_preserved(void)
{
    test_map_struct map;
    test_map_struct normalized;
    uint8 car_row = 0u;
    uint8 car_col = 0u;

    init_map(&map, 5u, 5u);
    map.rows[4][8] = 'B';
    map.rows[7][9] = 'T';

    if(0u == subject2_normalize_center_map(&map.source, 550u, 550u,
                                            &normalized.source, normalized.rows,
                                            &car_row, &car_col))
    {
        return 0u;
    }
    return ((5u == car_row) && (5u == car_col) &&
            ('C' == normalized.rows[5][5]) &&
            ('B' == normalized.rows[4][8]) &&
            ('T' == normalized.rows[7][9])) ? 1u : 0u;
}

static uint8 center_map_neighbor_preserves_targets(void)
{
    test_map_struct map;
    test_map_struct normalized;
    uint8 car_row = 0u;
    uint8 car_col = 0u;

    init_map(&map, 5u, 6u);
    map.rows[5][6] = '+';
    map.rows[5][5] = 'T';

    if(0u == subject2_normalize_center_map(&map.source, 550u, 550u,
                                            &normalized.source, normalized.rows,
                                            &car_row, &car_col))
    {
        return 0u;
    }
    return ((5u == car_row) && (5u == car_col) &&
            ('T' == normalized.rows[5][6]) &&
            ('+' == normalized.rows[5][5])) ? 1u : 0u;
}

static uint8 center_map_invalid_reference_is_rejected(void)
{
    test_map_struct map;
    test_map_struct normalized;
    uint8 car_row = 0u;
    uint8 car_col = 0u;

    init_map(&map, 5u, 5u);
    if(0u != subject2_normalize_center_map(&map.source, 750u, 550u,
                                           &normalized.source, normalized.rows,
                                           &car_row, &car_col))
    {
        return 0u;
    }
    if(0u != subject2_normalize_center_map(&map.source,
                                           (uint16)(MAP_COLS * 100u), 550u,
                                           &normalized.source, normalized.rows,
                                           &car_row, &car_col))
    {
        return 0u;
    }
    map.rows[5][6] = 'B';
    return (0u == subject2_normalize_center_map(&map.source, 650u, 550u,
                                                &normalized.source, normalized.rows,
                                                &car_row, &car_col)) ? 1u : 0u;
}

static void recognize_object(subject2_object_struct *object, uint8 class_id)
{
    object->recognized = 1u;
    object->class_id = class_id;
}

static uint8 reconcile_one_box_move_preserves_identity(void)
{
    test_map_struct old_map;
    test_map_struct new_map;
    subject2_object_struct boxes[MAX_BOXES];
    subject2_object_struct targets[MAX_BOXES];
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT];
    subject2_sync_update_struct update;
    uint8 box_count = 0u;
    uint8 target_count = 0u;

    init_map(&old_map, 5u, 5u);
    old_map.rows[4][4] = 'B';
    old_map.rows[8][8] = 'T';
    subject2_collect_objects(&old_map.source, 'B', boxes, &box_count);
    subject2_collect_objects(&old_map.source, 'T', targets, &target_count);
    recognize_object(&boxes[0], 3u);
    recognize_object(&targets[0], 3u);
    subject2_bindings_clear(bindings);
    subject2_bind_box(bindings, 3u, boxes[0].cell);
    subject2_bind_target(bindings, 3u, targets[0].cell);

    init_map(&new_map, 5u, 5u);
    new_map.rows[4][5] = 'B';
    new_map.rows[8][8] = 'T';
    if(SUBJECT2_SYNC_OK != subject2_reconcile_objects(
            &new_map.source, boxes, &box_count, targets, &target_count,
            bindings, 0u, SUBJECT2_INVALID_CLASS, &update))
    {
        return 0u;
    }
    return ((1u == box_count) && (1u == boxes[0].recognized) &&
            (3u == boxes[0].class_id) &&
            (map_cell_index(4u, 5u) == boxes[0].cell) &&
            (map_cell_index(4u, 5u) == bindings[3].box_cell) &&
            (0u == update.need_box_scan)) ? 1u : 0u;
}

static uint8 reconcile_box_move_keeps_map_order(void)
{
    test_map_struct old_map;
    test_map_struct new_map;
    subject2_object_struct boxes[MAX_BOXES];
    subject2_object_struct targets[MAX_BOXES];
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT];
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
    recognize_object(&boxes[0], 1u);
    recognize_object(&boxes[1], 2u);
    recognize_object(&targets[0], 1u);
    recognize_object(&targets[1], 2u);
    subject2_bindings_clear(bindings);
    subject2_bind_box(bindings, 1u, boxes[0].cell);
    subject2_bind_box(bindings, 2u, boxes[1].cell);
    subject2_bind_target(bindings, 1u, targets[0].cell);
    subject2_bind_target(bindings, 2u, targets[1].cell);

    init_map(&new_map, 5u, 5u);
    new_map.rows[3][3] = 'B';
    new_map.rows[6][6] = 'B';
    new_map.rows[8][4] = 'T';
    new_map.rows[8][6] = 'T';
    if(SUBJECT2_SYNC_OK != subject2_reconcile_objects(
            &new_map.source, boxes, &box_count, targets, &target_count,
            bindings, 0u, SUBJECT2_INVALID_CLASS, &update))
    {
        return 0u;
    }
    return ((map_cell_index(3u, 3u) == boxes[0].cell) &&
            (map_cell_index(6u, 6u) == boxes[1].cell)) ? 1u : 0u;
}

static uint8 reconcile_ambiguous_moves_only_invalidates_boxes(void)
{
    test_map_struct old_map;
    test_map_struct new_map;
    subject2_object_struct boxes[MAX_BOXES];
    subject2_object_struct targets[MAX_BOXES];
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT];
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
    recognize_object(&boxes[0], 1u);
    recognize_object(&boxes[1], 2u);
    recognize_object(&targets[0], 1u);
    recognize_object(&targets[1], 2u);
    subject2_bindings_clear(bindings);
    subject2_bind_box(bindings, 1u, boxes[0].cell);
    subject2_bind_box(bindings, 2u, boxes[1].cell);
    subject2_bind_target(bindings, 1u, targets[0].cell);
    subject2_bind_target(bindings, 2u, targets[1].cell);

    init_map(&new_map, 5u, 5u);
    new_map.rows[4][5] = 'B';
    new_map.rows[6][5] = 'B';
    new_map.rows[8][4] = 'T';
    new_map.rows[8][6] = 'T';
    if(SUBJECT2_SYNC_RESCAN != subject2_reconcile_objects(
            &new_map.source, boxes, &box_count, targets, &target_count,
            bindings, 0u, SUBJECT2_INVALID_CLASS, &update))
    {
        return 0u;
    }
    return ((0u == bindings[1].box_valid) &&
            (0u == bindings[2].box_valid) &&
            (1u == bindings[1].target_valid) &&
            (1u == bindings[2].target_valid) &&
            (0u == boxes[0].recognized) &&
            (0u == boxes[1].recognized) &&
            (1u == update.need_box_scan) &&
            (0u == update.need_target_scan)) ? 1u : 0u;
}

static uint8 reconcile_multiple_completions_preserves_bindings(void)
{
    test_map_struct old_map;
    test_map_struct new_map;
    subject2_object_struct boxes[MAX_BOXES];
    subject2_object_struct targets[MAX_BOXES];
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT];
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
    recognize_object(&boxes[0], 1u);
    recognize_object(&boxes[1], 2u);
    recognize_object(&targets[0], 1u);
    recognize_object(&targets[1], 2u);
    subject2_bindings_clear(bindings);
    subject2_bind_box(bindings, 1u, boxes[0].cell);
    subject2_bind_box(bindings, 2u, boxes[1].cell);
    subject2_bind_target(bindings, 1u, targets[0].cell);
    subject2_bind_target(bindings, 2u, targets[1].cell);

    init_map(&new_map, 5u, 5u);
    if(SUBJECT2_SYNC_OK != subject2_reconcile_objects(
            &new_map.source, boxes, &box_count, targets, &target_count,
            bindings, 1u, 1u, &update))
    {
        return 0u;
    }
    return ((0u == box_count) && (0u == target_count) &&
            (1u == bindings[1].completed) &&
            (1u == bindings[2].completed) &&
            (2u == update.completed_count)) ? 1u : 0u;
}

static uint8 ambiguous_sync_does_not_publish_partial_update(void)
{
    test_map_struct old_map;
    test_map_struct new_map;
    subject2_object_struct boxes[MAX_BOXES];
    subject2_object_struct targets[MAX_BOXES];
    subject2_binding_struct bindings[SUBJECT2_CLASS_COUNT];
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
    recognize_object(&boxes[0], 1u);
    recognize_object(&boxes[1], 2u);
    recognize_object(&targets[0], 1u);
    recognize_object(&targets[1], 2u);
    subject2_bindings_clear(bindings);
    subject2_bind_box(bindings, 1u, boxes[0].cell);
    subject2_bind_box(bindings, 2u, boxes[1].cell);
    subject2_bind_target(bindings, 1u, targets[0].cell);
    subject2_bind_target(bindings, 2u, targets[1].cell);

    init_map(&new_map, 5u, 5u);
    new_map.rows[6][5] = 'B';
    new_map.rows[8][6] = 'T';
    memset(&update, 0xA5, sizeof(update));
    if(SUBJECT2_SYNC_AMBIGUOUS != subject2_reconcile_objects(
            &new_map.source, boxes, &box_count, targets, &target_count,
            bindings, 1u, 1u, &update))
    {
        return 0u;
    }
    return ((0u == update.need_box_scan) &&
            (0u == update.need_target_scan) &&
            (0u == update.completed_count)) ? 1u : 0u;
}

int main(void)
{
    uint8 passed = 1u;

    passed &= run_case("collect-select-nearest", collect_and_select_nearest());
    passed &= run_case("ambiguity-retry-mask", ambiguity_and_retry_mask_work());
    passed &= run_case("observation-yaw", observation_yaw_matches_four_directions());
    passed &= run_case("classifier-consecutive", classifier_requires_consecutive_confident_samples());
    passed &= run_case("car-on-target", car_on_target_is_collected());
    passed &= run_case("binding-sets", bindings_are_unique_and_sets_match());
    passed &= run_case("track-active-box", active_box_identity_tracks_one_move());
    passed &= run_case("track-ambiguous", multiple_changes_are_ambiguous());
    passed &= run_case("center-map-same-cell", center_map_same_cell_is_preserved());
    passed &= run_case("center-map-neighbor", center_map_neighbor_preserves_targets());
    passed &= run_case("center-map-invalid", center_map_invalid_reference_is_rejected());
    passed &= run_case("sync-one-box-move", reconcile_one_box_move_preserves_identity());
    passed &= run_case("sync-box-map-order", reconcile_box_move_keeps_map_order());
    passed &= run_case("sync-ambiguous-moves", reconcile_ambiguous_moves_only_invalidates_boxes());
    passed &= run_case("sync-multi-complete", reconcile_multiple_completions_preserves_bindings());
    passed &= run_case("sync-ambiguous-update", ambiguous_sync_does_not_publish_partial_update());

    return (0u != passed) ? 0 : 1;
}
