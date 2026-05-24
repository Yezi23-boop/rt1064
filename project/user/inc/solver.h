#ifndef _solver_h_
#define _solver_h_

#include "map_types.h"

void clear_result(solve_result_struct *result);
uint8 solve_map(const map_source_struct *source, solve_result_struct *result);

#endif
