#include <stdio.h>
#include <string.h>
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
    {
        uint16 index;
        for(index = 0; index < result.waypoint_count; index++)
        {
            if(0u != result.waypoints[index].center_correct_before)
            {
                return 0;
            }
        }
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

static uint8 first_push_is_marked_after_merged_approach(void)
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
            (3u == result.waypoint_count) &&
            (3u == result.waypoints[0].col) &&
            (0u == result.waypoints[0].center_correct_before) &&
            ('R' == result.waypoints[1].action) &&
            (1u == result.waypoints[1].center_correct_before) &&
            (0u == result.waypoints[2].center_correct_before)) ? 1u : 0u;
}

static uint8 diagonal_approach_marks_first_push(void)
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
            (0u == result.waypoints[0].center_correct_before) &&
            ('R' == result.waypoints[1].action) &&
            (1u == result.waypoints[1].center_correct_before)) ? 1u : 0u;
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
    passed &= run_case("mark-first-push", first_push_is_marked_after_merged_approach());
    passed &= run_case("diagonal-first-push", diagonal_approach_marks_first_push());

    return (0 != passed) ? 0 : 1;
}
