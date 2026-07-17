#include <stdio.h>
#include <string.h>
#include "map_utils.h"
#include "subject3_logic.h"

typedef struct
{
    char rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct source;
} test_map_struct;

static void init_map(test_map_struct *map)
{
    uint8 row;
    uint8 col;

    memset(map, 0, sizeof(*map));
    map->source.name = "S3";
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
    map->rows[5][2] = 'C';
    map->rows[5][4] = 'X';
    map->rows[5][8] = 'X';
    map->rows[4][6] = 'B';
    map->rows[4][10] = 'T';
    map->rows[5][6] = '#';
    map->rows[4][5] = '#';
}

static uint8 run_case(const char *name, uint8 passed)
{
    printf("%s: %s\n", name, (0u != passed) ? "PASS" : "FAIL");
    return passed;
}

static uint8 expected_map_is_complete(void)
{
    test_map_struct before;
    char rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct expected;

    init_map(&before);
    if(0u == subject3_build_expected_map(
                  &before.source, map_cell_index(5u, 4u),
                  map_cell_index(5u, 6u), map_cell_index(5u, 5u),
                  rows, &expected))
    {
        return 0u;
    }
    return (('.' == rows[5][4]) && ('C' == rows[5][5]) &&
            ('.' == rows[5][6]) && ('.' == rows[4][5]) &&
            ('X' == rows[5][8]) && ('B' == rows[4][6]) &&
            ('T' == rows[4][10]) && ('#' == rows[0][6])) ? 1u : 0u;
}

static uint8 blast_confirmation_checks_sets(void)
{
    test_map_struct before;
    char rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct after;

    init_map(&before);
    if(0u == subject3_build_expected_map(
                  &before.source, map_cell_index(5u, 4u),
                  map_cell_index(5u, 6u), map_cell_index(5u, 5u),
                  rows, &after))
    {
        return 0u;
    }
    if(0u == subject3_blast_map_matches(
                  &before.source, &after, map_cell_index(5u, 4u),
                  map_cell_index(5u, 6u)))
    {
        return 0u;
    }
    rows[5][8] = '.';
    return (0u == subject3_blast_map_matches(
                  &before.source, &after, map_cell_index(5u, 4u),
                  map_cell_index(5u, 6u))) ? 1u : 0u;
}

static uint8 candidate_order_is_deterministic(void)
{
    subject3_candidate_score_struct best = {5u, 20u, 0u, 10u, 2u, 3u};
    subject3_candidate_score_struct candidate = best;

    candidate.action_count = 9u;
    if(0u == subject3_candidate_is_better(&candidate, &best, 1u)) return 0u;
    candidate = best;
    candidate.push_count = 1u;
    if(0u == subject3_candidate_is_better(&candidate, &best, 1u)) return 0u;
    candidate = best;
    candidate.bomb_cell = 6u;
    return (0u == subject3_candidate_is_better(&candidate, &best, 1u)) ? 1u : 0u;
}

static uint8 edge_blast_keeps_outer_wall(void)
{
    test_map_struct before;
    char rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct after;

    init_map(&before);
    before.rows[5][6] = '.';
    before.rows[4][5] = '.';
    before.rows[1][4] = '#';
    if(0u == subject3_build_expected_map(
                  &before.source, map_cell_index(5u, 4u),
                  map_cell_index(1u, 4u), map_cell_index(2u, 4u),
                  rows, &after))
    {
        return 0u;
    }
    return subject3_blast_map_matches(
        &before.source, &after, map_cell_index(5u, 4u),
        map_cell_index(1u, 4u));
}

static uint8 unexploded_wall_change_is_rejected(void)
{
    test_map_struct before;
    test_map_struct after;

    init_map(&before);
    init_map(&after);
    after.rows[3][3] = '#';
    return (0u == subject3_unexploded_map_matches(
                      &before.source, &after.source,
                      map_cell_index(5u, 4u))) ? 1u : 0u;
}

static uint8 interrupted_bomb_may_move(void)
{
    test_map_struct before;
    test_map_struct after;

    init_map(&before);
    init_map(&after);
    after.rows[5][2] = '.';
    after.rows[5][4] = 'C';
    after.rows[5][5] = 'X';
    if(0u == subject3_unexploded_map_matches(
                  &before.source, &after.source,
                  map_cell_index(5u, 4u)))
    {
        return 0u;
    }
    after.rows[5][8] = '.';
    after.rows[5][9] = 'X';
    return (0u == subject3_unexploded_map_matches(
                      &before.source, &after.source,
                      map_cell_index(5u, 4u))) ? 1u : 0u;
}

static uint8 blast_rejects_wall_change_outside_area(void)
{
    test_map_struct before;
    char rows[MAP_ROWS][MAP_COLS + 1];
    map_source_struct after;

    init_map(&before);
    before.rows[2][12] = '#';
    if(0u == subject3_build_expected_map(
                  &before.source, map_cell_index(5u, 4u),
                  map_cell_index(5u, 6u), map_cell_index(5u, 5u),
                  rows, &after))
    {
        return 0u;
    }
    rows[2][12] = '.';
    return (0u == subject3_blast_map_matches(
                      &before.source, &after,
                      map_cell_index(5u, 4u),
                      map_cell_index(5u, 6u))) ? 1u : 0u;
}

int main(void)
{
    uint8 passed = 1u;

    passed &= run_case("expected-map", expected_map_is_complete());
    passed &= run_case("blast-confirm", blast_confirmation_checks_sets());
    passed &= run_case("candidate-order", candidate_order_is_deterministic());
    passed &= run_case("edge-blast", edge_blast_keeps_outer_wall());
    passed &= run_case("unexploded-wall", unexploded_wall_change_is_rejected());
    passed &= run_case("interrupted-bomb-move", interrupted_bomb_may_move());
    passed &= run_case("outside-wall-change", blast_rejects_wall_change_outside_area());
    return (0u != passed) ? 0 : 1;
}
