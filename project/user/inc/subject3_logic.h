#ifndef _subject3_logic_h_
#define _subject3_logic_h_

#include "map_types.h"

typedef struct
{
    uint16 bomb_cell;
    uint16 blast_wall_cell;
    uint16 final_car_cell;
    uint16 action_count;
    uint16 push_count;
    uint16 turn_count;
} subject3_candidate_score_struct;

uint8 subject3_build_expected_map(
    const map_source_struct *source,
    uint16 bomb_cell,
    uint16 blast_wall_cell,
    uint16 final_car_cell,
    char rows[MAP_ROWS][MAP_COLS + 1],
    map_source_struct *expected);

uint8 subject3_blast_map_matches(const map_source_struct *before,
                                 const map_source_struct *after,
                                 uint16 bomb_cell,
                                 uint16 blast_wall_cell);

uint8 subject3_unexploded_map_matches(const map_source_struct *before,
                                      const map_source_struct *after,
                                      uint16 selected_bomb_cell);

uint8 subject3_candidate_is_better(
    const subject3_candidate_score_struct *candidate,
    const subject3_candidate_score_struct *best,
    uint8 best_valid);

#endif
