#ifndef _sokoban_solver_h_
#define _sokoban_solver_h_

#include "sokoban_types.h"

void sokoban_result_clear(sokoban_result_struct *result);
uint8 sokoban_solve_map_multi_box(const offline_map_struct *source, sokoban_result_struct *result);

#endif
