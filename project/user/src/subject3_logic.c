#include "subject3_logic.h"
#include "map_utils.h"
#include <string.h>

static uint8 subject3_source_valid(const map_source_struct *source)
{
    uint8 row;

    if(0 == source)
    {
        return 0u;
    }
    for(row = 0u; row < MAP_ROWS; row++)
    {
        if((0 == source->rows[row]) ||
           (MAP_COLS != strlen(source->rows[row])))
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8 subject3_is_inner_wall(const map_source_struct *source,
                                    uint16 cell)
{
    uint8 row = map_cell_row(cell);
    uint8 col = map_cell_col(cell);

    return ((0u < row) && (row < (MAP_ROWS - 1u)) &&
            (0u < col) && (col < (MAP_COLS - 1u)) &&
            ('#' == source->rows[row][col])) ? 1u : 0u;
}

static uint8 subject3_symbol_at(const map_source_struct *source,
                                uint16 cell,
                                char symbol)
{
    char value = source->rows[map_cell_row(cell)][map_cell_col(cell)];

    if('B' == symbol)
    {
        return (('B' == value) || (MAP_BOX_ON_TARGET == value)) ? 1u : 0u;
    }
    if('T' == symbol)
    {
        return (('T' == value) || ('+' == value) ||
                (MAP_BOX_ON_TARGET == value)) ? 1u : 0u;
    }
    return (value == symbol) ? 1u : 0u;
}

static uint8 subject3_symbol_sets_equal(const map_source_struct *left,
                                        const map_source_struct *right,
                                        char symbol,
                                        uint16 excluded_cell)
{
    uint16 cell;

    for(cell = 0u; cell < MAP_CELLS; cell++)
    {
        uint8 left_has = subject3_symbol_at(left, cell, symbol);
        uint8 right_has = subject3_symbol_at(right, cell, symbol);

        if((cell == excluded_cell) && ('X' == symbol))
        {
            if((0u == left_has) || (0u != right_has))
            {
                return 0u;
            }
        }
        else if(left_has != right_has)
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8 subject3_interrupted_bombs_match(
    const map_source_struct *before,
    const map_source_struct *after,
    uint16 selected_bomb_cell)
{
    uint16 cell;
    uint8 added_count = 0u;
    uint8 selected_still_present;

    if((MAP_CELLS <= selected_bomb_cell) ||
       (0u == subject3_symbol_at(before, selected_bomb_cell, 'X')))
    {
        return 0u;
    }
    selected_still_present =
        subject3_symbol_at(after, selected_bomb_cell, 'X');
    for(cell = 0u; cell < MAP_CELLS; cell++)
    {
        uint8 before_has;
        uint8 after_has;

        if(cell == selected_bomb_cell)
        {
            continue;
        }
        before_has = subject3_symbol_at(before, cell, 'X');
        after_has = subject3_symbol_at(after, cell, 'X');
        if((0u != before_has) && (0u == after_has))
        {
            return 0u;
        }
        if((0u == before_has) && (0u != after_has))
        {
            added_count++;
        }
    }
    return (0u != selected_still_present) ?
           ((0u == added_count) ? 1u : 0u) :
           ((1u == added_count) ? 1u : 0u);
}

uint8 subject3_build_expected_map(
    const map_source_struct *source,
    uint16 bomb_cell,
    uint16 blast_wall_cell,
    uint16 final_car_cell,
    char rows[MAP_ROWS][MAP_COLS + 1],
    map_source_struct *expected)
{
    uint8 row;
    uint8 col;
    uint8 car_count = 0u;
    char final_value;

    if((0u == subject3_source_valid(source)) || (0 == rows) ||
       (0 == expected) || (MAP_CELLS <= bomb_cell) ||
       (MAP_CELLS <= blast_wall_cell) || (MAP_CELLS <= final_car_cell) ||
       ('X' != source->rows[map_cell_row(bomb_cell)][map_cell_col(bomb_cell)]) ||
       (0u == subject3_is_inner_wall(source, blast_wall_cell)))
    {
        return 0u;
    }

    map_source_snapshot(expected, rows, source);
    for(row = 0u; row < MAP_ROWS; row++)
    {
        for(col = 0u; col < MAP_COLS; col++)
        {
            if('C' == rows[row][col])
            {
                rows[row][col] = '.';
                car_count++;
            }
            else if('+' == rows[row][col])
            {
                rows[row][col] = 'T';
                car_count++;
            }
        }
    }
    if(1u != car_count)
    {
        return 0u;
    }

    rows[map_cell_row(bomb_cell)][map_cell_col(bomb_cell)] = '.';
    for(row = (uint8)(map_cell_row(blast_wall_cell) - 1u);
        row <= (uint8)(map_cell_row(blast_wall_cell) + 1u); row++)
    {
        for(col = (uint8)(map_cell_col(blast_wall_cell) - 1u);
            col <= (uint8)(map_cell_col(blast_wall_cell) + 1u); col++)
        {
            if((0u < row) && (row < (MAP_ROWS - 1u)) &&
               (0u < col) && (col < (MAP_COLS - 1u)) &&
               ('#' == rows[row][col]))
            {
                rows[row][col] = '.';
            }
        }
    }

    final_value = rows[map_cell_row(final_car_cell)][map_cell_col(final_car_cell)];
    if('T' == final_value)
    {
        rows[map_cell_row(final_car_cell)][map_cell_col(final_car_cell)] = '+';
    }
    else if('.' == final_value)
    {
        rows[map_cell_row(final_car_cell)][map_cell_col(final_car_cell)] = 'C';
    }
    else
    {
        return 0u;
    }
    return 1u;
}

uint8 subject3_blast_map_matches(const map_source_struct *before,
                                 const map_source_struct *after,
                                 uint16 bomb_cell,
                                 uint16 blast_wall_cell)
{
    uint8 row;
    uint8 col;
    map_scan_stats_struct stats;

    if((0u == subject3_source_valid(before)) ||
       (0u == subject3_source_valid(after)) ||
       (MAP_CELLS <= bomb_cell) || (MAP_CELLS <= blast_wall_cell) ||
       (0u == subject3_is_inner_wall(before, blast_wall_cell)) ||
       (0u == subject3_symbol_sets_equal(before, after, 'X', bomb_cell)) ||
       (0u == subject3_symbol_sets_equal(before, after, 'B', INVALID_STATE)) ||
       (0u == subject3_symbol_sets_equal(before, after, 'T', INVALID_STATE)))
    {
        return 0u;
    }

    map_scan_stats(after, &stats);
    if(1u != stats.car_count)
    {
        return 0u;
    }
    for(col = 0u; col < MAP_COLS; col++)
    {
        if((before->rows[0][col] != after->rows[0][col]) ||
           (before->rows[MAP_ROWS - 1u][col] !=
            after->rows[MAP_ROWS - 1u][col]))
        {
            return 0u;
        }
    }
    for(row = 0u; row < MAP_ROWS; row++)
    {
        if((before->rows[row][0] != after->rows[row][0]) ||
           (before->rows[row][MAP_COLS - 1u] !=
            after->rows[row][MAP_COLS - 1u]))
        {
            return 0u;
        }
    }
    for(row = 0u; row < MAP_ROWS; row++)
    {
        for(col = 0u; col < MAP_COLS; col++)
        {
            uint8 inside_blast =
                ((0u < row) && (row < (MAP_ROWS - 1u)) &&
                 (0u < col) && (col < (MAP_COLS - 1u)) &&
                 (row + 1u >= map_cell_row(blast_wall_cell)) &&
                 (row <= map_cell_row(blast_wall_cell) + 1u) &&
                 (col + 1u >= map_cell_col(blast_wall_cell)) &&
                 (col <= map_cell_col(blast_wall_cell) + 1u)) ? 1u : 0u;
            uint8 before_wall =
                ('#' == before->rows[row][col]) ? 1u : 0u;
            uint8 after_wall =
                ('#' == after->rows[row][col]) ? 1u : 0u;

            if(((0u != inside_blast) && (0u != after_wall)) ||
               ((0u == inside_blast) && (before_wall != after_wall)))
            {
                return 0u;
            }
        }
    }
    return 1u;
}

uint8 subject3_candidate_is_better(
    const subject3_candidate_score_struct *candidate,
    const subject3_candidate_score_struct *best,
    uint8 best_valid)
{
    if(0 == candidate)
    {
        return 0u;
    }
    if((0u == best_valid) || (0 == best))
    {
        return 1u;
    }
    if(candidate->action_count != best->action_count)
        return (candidate->action_count < best->action_count) ? 1u : 0u;
    if(candidate->push_count != best->push_count)
        return (candidate->push_count < best->push_count) ? 1u : 0u;
    if(candidate->turn_count != best->turn_count)
        return (candidate->turn_count < best->turn_count) ? 1u : 0u;
    if(candidate->bomb_cell != best->bomb_cell)
        return (candidate->bomb_cell < best->bomb_cell) ? 1u : 0u;
    return (candidate->blast_wall_cell < best->blast_wall_cell) ? 1u : 0u;
}

uint8 subject3_unexploded_map_matches(const map_source_struct *before,
                                      const map_source_struct *after,
                                      uint16 selected_bomb_cell)
{
    uint8 row;
    uint8 col;
    map_scan_stats_struct stats;

    if((0u == subject3_source_valid(before)) ||
       (0u == subject3_source_valid(after)) ||
       (0u == subject3_interrupted_bombs_match(
                  before, after, selected_bomb_cell)) ||
       (0u == subject3_symbol_sets_equal(before, after, 'B', INVALID_STATE)) ||
       (0u == subject3_symbol_sets_equal(before, after, 'T', INVALID_STATE)))
    {
        return 0u;
    }
    map_scan_stats(after, &stats);
    if(1u != stats.car_count)
    {
        return 0u;
    }
    for(row = 0u; row < MAP_ROWS; row++)
    {
        for(col = 0u; col < MAP_COLS; col++)
        {
            if((('#' == before->rows[row][col]) ? 1u : 0u) !=
               (('#' == after->rows[row][col]) ? 1u : 0u))
            {
                return 0u;
            }
        }
    }
    for(col = 0u; col < MAP_COLS; col++)
    {
        if((before->rows[0][col] != after->rows[0][col]) ||
           (before->rows[MAP_ROWS - 1u][col] !=
            after->rows[MAP_ROWS - 1u][col]))
        {
            return 0u;
        }
    }
    for(row = 0u; row < MAP_ROWS; row++)
    {
        if((before->rows[row][0] != after->rows[row][0]) ||
           (before->rows[row][MAP_COLS - 1u] !=
            after->rows[row][MAP_COLS - 1u]))
        {
            return 0u;
        }
    }
    return 1u;
}
