#ifndef _sokoban_maps_h_
#define _sokoban_maps_h_

#include "sokoban_types.h"

uint8 offline_map_count_get(void);
const offline_map_struct *offline_map_get(uint8 index);

#endif
