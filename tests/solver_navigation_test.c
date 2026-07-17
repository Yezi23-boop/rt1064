#include <stdio.h>
#include <string.h>
#include "map_utils.h"
#include "solver.h"

typedef struct
{
    char storage[MAP_ROWS][MAP_COLS + 1];
    map_source_struct source;
} navigation_test_map_struct;

static void init_map(navigation_test_map_struct *map, uint8 car_row, uint8 car_col)
{
    uint8 row;
    uint8 col;

    memset(map, 0, sizeof(*map));
    map->source.name = "navigation-test";
    for(row = 0; row < MAP_ROWS; row++)
    {
        for(col = 0; col < MAP_COLS; col++)
        {
            map->storage[row][col] = ((0 == row) || (MAP_ROWS - 1 == row) ||
                                      (0 == col) || (MAP_COLS - 1 == col)) ? '#' : '.';
        }
        map->storage[row][MAP_COLS] = '\0';
        map->source.rows[row] = map->storage[row];
    }
    map->storage[car_row][car_col] = 'C';
}

static uint8 expect_path(navigation_test_map_struct *map,
                         uint8 target_row,
                         uint8 target_col,
                         uint8 should_solve,
                         uint8 zero_path)
{
    solve_result_struct result;
    uint8 solved = solve_navigation_path(&map->source,
                                         target_row,
                                         target_col,
                                         &result);

    if(solved != should_solve)
    {
        return 0;
    }
    if(0 == solved)
    {
        return 1;
    }
    if(0 != zero_path)
    {
        return ((0 == result.action_count) && (0 == result.waypoint_count)) ? 1u : 0u;
    }
    if(0 == result.waypoint_count)
    {
        return 0;
    }
    return ((target_row == result.waypoints[result.waypoint_count - 1u].row) &&
            (target_col == result.waypoints[result.waypoint_count - 1u].col)) ? 1u : 0u;
}

static uint8 run_case(const char *name, uint8 passed)
{
    printf("%-24s %s\n", name, (0 != passed) ? "PASS" : "FAIL");
    return passed;
}

static uint8 each_push_action_is_a_waypoint(void)
{
    navigation_test_map_struct map;
    solve_result_struct result;
    uint16 index;

    init_map(&map, 1u, 1u);
    map.storage[1][2] = 'B';
    map.storage[1][4] = 'T';
    if(0 == solve_map(&map.source, &result))
    {
        return 0;
    }
    if((2u != result.action_count) || (2u != result.waypoint_count))
    {
        return 0;
    }
    for(index = 0; index < result.waypoint_count; index++)
    {
        if(('R' != result.waypoints[index].action) ||
           ((result.waypoints[index].action_end - result.waypoints[index].action_start) != 1u))
        {
            return 0;
        }
    }
    return ((1u == result.waypoints[0].center_correct_before) &&
            (0u == result.waypoints[1].center_correct_before)) ? 1u : 0u;
}

static uint8 same_direction_push_is_not_marked(void)
{
    navigation_test_map_struct map;
    solve_result_struct result;

    init_map(&map, 1u, 1u);
    map.storage[1][4] = 'B';
    map.storage[1][6] = 'T';
    if(0 == solve_map(&map.source, &result))
    {
        return 0;
    }

    return ((0 == strcmp(result.actions, "rrRR")) &&
            (4u == result.waypoint_count) &&
            (2u == result.waypoints[0].col) &&
            (0u == result.waypoints[0].near_box_axis_lock) &&
            (0u == result.waypoints[0].center_correct_before) &&
            ('r' == result.waypoints[1].action) &&
            (1u == result.waypoints[1].near_box_axis_lock) &&
            (1u == result.waypoints[1].center_correct_before) &&
            ('R' == result.waypoints[2].action) &&
            (1u == result.waypoints[2].pre_push_extra_gap) &&
            (0u == result.waypoints[2].center_correct_before) &&
            (0u == result.waypoints[3].pre_push_extra_gap) &&
            (0u == result.waypoints[3].center_correct_before)) ? 1u : 0u;
}

static uint8 navigation_splits_near_box_boundary(void)
{
    navigation_test_map_struct map;
    solve_result_struct result;

    init_map(&map, 2u, 1u);
    map.storage[1][3] = 'B';
    if(0 == solve_navigation_path(&map.source, 2u, 6u, &result))
    {
        return 0u;
    }

    return ((0 == strcmp(result.actions, "rrrrr")) &&
            (2u == result.waypoint_count) &&
            (1u == result.waypoints[0].near_box_axis_lock) &&
            (5u == result.waypoints[0].col) &&
            (0u == result.waypoints[1].near_box_axis_lock) &&
            (6u == result.waypoints[1].col)) ? 1u : 0u;
}

static uint8 blocked_extra_gap_case(char blocker)
{
    navigation_test_map_struct map;
    solve_result_struct result;
    uint16 index;

    init_map(&map, 2u, 1u);
    map.storage[1][2] = blocker;
    map.storage[1][4] = 'B';
    map.storage[1][5] = 'T';
    if('B' == blocker)
    {
        map.storage[3][2] = 'T';
    }
    if(0 == solve_bound_box_path(&map.source,
                                 map_cell_index(1u, 4u),
                                 map_cell_index(1u, 5u),
                                 &result))
    {
        return 0u;
    }
    for(index = 0u; index < result.waypoint_count; index++)
    {
        if('R' == result.waypoints[index].action)
        {
            return (0u == result.waypoints[index].pre_push_extra_gap) ? 1u : 0u;
        }
    }
    return 0u;
}

static uint8 blocked_extra_gap_is_not_marked(void)
{
    return ((0u != blocked_extra_gap_case('#')) &&
            (0u != blocked_extra_gap_case('X')) &&
            (0u != blocked_extra_gap_case('B'))) ? 1u : 0u;
}

static uint8 navigation_blocks_box_on_target(void)
{
    navigation_test_map_struct map;
    solve_result_struct result;
    uint16 index;

    init_map(&map, 2u, 1u);
    map.storage[2][2] = MAP_BOX_ON_TARGET;
    if(0u == solve_navigation_path(&map.source, 2u, 3u, &result))
    {
        return 0u;
    }
    for(index = 0u; index < result.waypoint_count; index++)
    {
        if((2u == result.waypoints[index].row) &&
           (2u == result.waypoints[index].col))
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8 navigation_accepts_car_on_target(void)
{
    navigation_test_map_struct map;

    init_map(&map, 2u, 1u);
    map.storage[2][1] = '+';
    return expect_path(&map, 2u, 1u, 1u, 1u);
}

static uint8 turn_into_push_is_marked(void)
{
    navigation_test_map_struct map;
    solve_result_struct result;

    init_map(&map, 1u, 2u);
    map.storage[2][3] = 'B';
    map.storage[2][4] = 'T';
    if(0 == solve_map(&map.source, &result))
    {
        return 0;
    }

    return ((0 == strcmp(result.actions, "dR")) &&
            (2u == result.waypoint_count) &&
            ('d' == result.waypoints[0].action) &&
            (1u == result.waypoints[0].center_correct_before) &&
            ('R' == result.waypoints[1].action) &&
            (0u == result.waypoints[1].center_correct_before)) ? 1u : 0u;
}

static uint8 merged_turn_marks_center(void)
{
    navigation_test_map_struct map;
    solve_result_struct result;

    init_map(&map, 1u, 1u);
    if(0 == solve_navigation_path(&map.source, 3u, 3u, &result))
    {
        return 0u;
    }

    return ((0 == strcmp(result.actions, "ddrr")) &&
            (2u == result.waypoint_count) &&
            ('d' == result.waypoints[0].action) &&
            (0u == result.waypoints[0].center_correct_before) &&
            ('r' == result.waypoints[1].action) &&
            (1u == result.waypoints[1].center_correct_before)) ? 1u : 0u;
}

static uint8 merged_straight_does_not_mark_center(void)
{
    navigation_test_map_struct map;
    solve_result_struct result;

    init_map(&map, 1u, 1u);
    if(0 == solve_navigation_path(&map.source, 1u, 4u, &result))
    {
        return 0u;
    }

    return ((0 == strcmp(result.actions, "rrr")) &&
            (1u == result.waypoint_count) &&
            (0u == result.waypoints[0].center_correct_before)) ? 1u : 0u;
}

static uint8 merged_push_approach_marks_turn(void)
{
    navigation_test_map_struct map;
    solve_result_struct result;

    init_map(&map, 1u, 1u);
    map.storage[3][4] = 'B';
    map.storage[3][5] = 'T';
    if(0 == solve_map(&map.source, &result))
    {
        return 0u;
    }

    return ((0 == strcmp(result.actions, "ddrrR")) &&
            (4u == result.waypoint_count) &&
            ('d' == result.waypoints[0].action) &&
            (0u == result.waypoints[0].center_correct_before) &&
            ('r' == result.waypoints[1].action) &&
            (1u == result.waypoints[1].center_correct_before) &&
            ('r' == result.waypoints[2].action) &&
            (0u == result.waypoints[2].center_correct_before) &&
            ('R' == result.waypoints[3].action) &&
            (0u == result.waypoints[3].center_correct_before)) ? 1u : 0u;
}

static uint8 bound_box_uses_requested_target(void)
{
    navigation_test_map_struct map;
    solve_result_struct result;

    init_map(&map, 1u, 1u);
    map.storage[1][3] = 'B';
    map.storage[1][4] = 'T';
    map.storage[1][6] = 'T';
    map.storage[3][3] = 'B';

    if(0 == solve_bound_box_path(&map.source,
                                 map_cell_index(1u, 3u),
                                 map_cell_index(1u, 6u),
                                 &result))
    {
        return 0u;
    }

    return ((0 == strcmp(result.actions, "rRRR")) &&
            (1u == result.task_count) &&
            (1u == result.waypoints[result.waypoint_count - 1u].task_end) &&
            (1u == result.waypoints[result.waypoint_count - 1u].row) &&
            (5u == result.waypoints[result.waypoint_count - 1u].col)) ? 1u : 0u;
}

static uint8 bound_box_rejects_missing_cells(void)
{
    navigation_test_map_struct map;
    solve_result_struct result;

    init_map(&map, 1u, 1u);
    map.storage[1][3] = 'B';
    map.storage[1][6] = 'T';

    if(0 != solve_bound_box_path(&map.source,
                                 map_cell_index(2u, 3u),
                                 map_cell_index(1u, 6u),
                                 &result))
    {
        return 0u;
    }
    return (0 == solve_bound_box_path(&map.source,
                                      map_cell_index(1u, 3u),
                                      map_cell_index(2u, 6u),
                                      &result)) ? 1u : 0u;
}

static uint8 car_on_target_is_supported(void)
{
    navigation_test_map_struct map;
    solve_result_struct result;

    init_map(&map, 2u, 2u);
    map.storage[2][2] = '+';
    if(0 == solve_navigation_path(&map.source, 2u, 4u, &result))
    {
        return 0u;
    }

    init_map(&map, 2u, 2u);
    map.storage[2][2] = '+';
    map.storage[2][4] = 'B';
    return solve_map(&map.source, &result);
}

int main(void)
{
    navigation_test_map_struct map;
    uint8 passed = 1;

    init_map(&map, 1, 1);
    passed &= run_case("straight", expect_path(&map, 1, 4, 1, 0));

    init_map(&map, 1, 1);
    passed &= run_case("turn", expect_path(&map, 3, 3, 1, 0));

    init_map(&map, 1, 1);
    map.storage[1][2] = 'X';
    map.storage[2][2] = 'B';
    passed &= run_case("wall-x-box-detour", expect_path(&map, 1, 4, 1, 0));

    init_map(&map, 1, 1);
    map.storage[1][4] = 'B';
    passed &= run_case("blocked-target", expect_path(&map, 1, 4, 0, 0));

    init_map(&map, 1, 1);
    map.storage[1][2] = '#';
    map.storage[2][1] = '#';
    passed &= run_case("unreachable", expect_path(&map, 3, 3, 0, 0));

    init_map(&map, 1, 1);
    passed &= run_case("start-equals-target", expect_path(&map, 1, 1, 1, 1));

    passed &= run_case("each-push-is-waypoint", each_push_action_is_a_waypoint());
    passed &= run_case("same-dir-push-no-center", same_direction_push_is_not_marked());
    passed &= run_case("near-box-split", navigation_splits_near_box_boundary());
    passed &= run_case("blocked-extra-gap", blocked_extra_gap_is_not_marked());
    passed &= run_case("turn-into-push-center", turn_into_push_is_marked());
    passed &= run_case("merged-turn-center", merged_turn_marks_center());
    passed &= run_case("merged-straight-no-center", merged_straight_does_not_mark_center());
    passed &= run_case("merged-push-turn-center", merged_push_approach_marks_turn());
    passed &= run_case("bound-requested-target", bound_box_uses_requested_target());
    passed &= run_case("bound-missing-cells", bound_box_rejects_missing_cells());
    passed &= run_case("car-on-target", car_on_target_is_supported());
    passed &= run_case("box-on-target-blocked", navigation_blocks_box_on_target());
    passed &= run_case("navigation-car-on-target", navigation_accepts_car_on_target());

    return (0 != passed) ? 0 : 1;
}
