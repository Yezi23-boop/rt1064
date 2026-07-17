#include <stdio.h>
#include <string.h>
#include "map_utils.h"
#include "solver.h"

typedef struct
{
    char rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct source;
} bomb_test_map_struct;

static void init_map(bomb_test_map_struct *map, uint8 car_row, uint8 car_col)
{
    uint8 row;
    uint8 col;

    memset(map, 0, sizeof(*map));
    map->source.name = "Bomb";
    for(row = 0u; row < MAP_ROWS; row++)
    {
        map->source.rows[row] = map->rows[row];
        for(col = 0u; col < MAP_COLS; col++)
        {
            map->rows[row][col] = ((0u == row) || ((MAP_ROWS - 1u) == row) ||
                                   (0u == col) || ((MAP_COLS - 1u) == col)) ? '#' : '.';
        }
        map->rows[row][MAP_COLS] = '\0';
    }
    map->rows[car_row][car_col] = 'C';
}

static uint8 run_case(const char *name, uint8 passed)
{
    printf("%s: %s\n", name, (0u != passed) ? "PASS" : "FAIL");
    return passed;
}

static uint8 straight_push(void)
{
    bomb_test_map_struct map;
    solve_result_struct result;
    uint16 pushes = 0u;
    uint16 turns = 0u;
    uint16 final_car = INVALID_STATE;

    init_map(&map, 5u, 2u);
    map.rows[5][4] = 'X';
    map.rows[5][7] = '#';
    if(0u == solve_bomb_path(&map.source,
                             map_cell_index(5u, 4u),
                             map_cell_index(5u, 7u),
                             &result, &pushes, &turns, &final_car))
    {
        return 0u;
    }
    return ((0 == strcmp(result.actions, "rrrr")) &&
            (3u == pushes) && (0u == turns) &&
            (map_cell_index(5u, 6u) == final_car) &&
            (2u == result.waypoint_count) &&
            (1u == result.waypoints[1].center_correct_before) &&
            (0u == result.waypoints[1].task_end)) ? 1u : 0u;
}

static uint8 detour_to_push(void)
{
    bomb_test_map_struct map;
    solve_result_struct result;
    uint16 pushes;
    uint16 turns;

    init_map(&map, 5u, 3u);
    map.rows[5][4] = 'X';
    map.rows[5][2] = '#';
    if(0u == solve_bomb_path(&map.source,
                             map_cell_index(5u, 4u),
                             map_cell_index(5u, 2u),
                             &result, &pushes, &turns, 0))
    {
        return 0u;
    }
    return ((2u == pushes) && (0u < turns) &&
            (0u < result.action_count)) ? 1u : 0u;
}

static uint8 blockers_are_respected(void)
{
    bomb_test_map_struct map;
    solve_result_struct result;

    init_map(&map, 5u, 2u);
    map.rows[5][4] = 'X';
    map.rows[5][5] = 'B';
    map.rows[5][6] = '#';
    map.rows[4][4] = '#';
    map.rows[6][4] = '#';
    return (0u == solve_bomb_path(&map.source,
                                  map_cell_index(5u, 4u),
                                  map_cell_index(5u, 6u),
                                  &result, 0, 0, 0)) ? 1u : 0u;
}

static uint8 target_and_outer_wall_rejected(void)
{
    bomb_test_map_struct map;
    solve_result_struct result;

    init_map(&map, 5u, 2u);
    map.rows[5][4] = 'X';
    map.rows[5][5] = 'T';
    map.rows[5][6] = '#';
    map.rows[4][4] = '#';
    map.rows[4][5] = '#';
    map.rows[6][4] = '#';
    map.rows[6][5] = '#';
    if(0u != solve_bomb_path(&map.source,
                             map_cell_index(5u, 4u),
                             map_cell_index(5u, 6u),
                             &result, 0, 0, 0))
    {
        return 0u;
    }
    return (0u == solve_bomb_path(&map.source,
                                  map_cell_index(5u, 4u),
                                  map_cell_index(5u, 0u),
                                  &result, 0, 0, 0)) ? 1u : 0u;
}

static uint8 player_can_cross_target(void)
{
    bomb_test_map_struct map;
    solve_result_struct result;
    uint8 col;

    init_map(&map, 5u, 2u);
    for(col = 1u; col <= 7u; col++)
    {
        map.rows[4][col] = '#';
        map.rows[6][col] = '#';
    }
    map.rows[5][3] = 'T';
    map.rows[5][5] = 'X';
    map.rows[5][7] = '#';
    if(0u == solve_bomb_path(&map.source,
                             map_cell_index(5u, 5u),
                             map_cell_index(5u, 7u),
                             &result, 0, 0, 0))
    {
        return 0u;
    }
    return (0 == strcmp(result.actions, "rrrr")) ? 1u : 0u;
}

int main(void)
{
    uint8 passed = 1u;

    passed &= run_case("straight-push", straight_push());
    passed &= run_case("detour-push", detour_to_push());
    passed &= run_case("blockers", blockers_are_respected());
    passed &= run_case("target-outer-wall", target_and_outer_wall_rejected());
    passed &= run_case("player-cross-target", player_can_cross_target());
    return (0u != passed) ? 0 : 1;
}
