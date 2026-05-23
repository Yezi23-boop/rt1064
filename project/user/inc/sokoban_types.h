#ifndef _sokoban_types_h_
#define _sokoban_types_h_

#include "zf_common_typedef.h"

#define SOKOBAN_MAP_ROWS                (12)
#define SOKOBAN_MAP_COLS                (16)
#define SOKOBAN_MAP_CELLS               (SOKOBAN_MAP_ROWS * SOKOBAN_MAP_COLS)
#define SOKOBAN_STATE_COUNT             (SOKOBAN_MAP_CELLS * SOKOBAN_MAP_CELLS)
#define SOKOBAN_STATE_NONE              (0xFFFF)
#define SOKOBAN_MAX_BOXES               (8)
#define SOKOBAN_MAX_SINGLE_PATH         (512)
#define SOKOBAN_MAX_TOTAL_ACTIONS       (1024)
#define SOKOBAN_MAX_WAYPOINTS           (256)
#define SOKOBAN_MAP_FORMAT_RT           (0)
#define SOKOBAN_MAP_FORMAT_SEEKFREE     (1)

typedef struct
{
    const char *name;
    uint8 format;
    const char *rows[SOKOBAN_MAP_ROWS];
} offline_map_struct;

typedef struct
{
    char grid[SOKOBAN_MAP_ROWS][SOKOBAN_MAP_COLS];
    uint16 player;
    uint16 boxes[SOKOBAN_MAX_BOXES];
    uint16 targets[SOKOBAN_MAX_BOXES];
    uint8 box_count;
    uint8 target_count;
} sokoban_map_struct;

typedef struct
{
    uint8 row;
    uint8 col;
    char action;
} motion_waypoint_struct;

typedef struct
{
    uint8 solved;
    uint8 task_count;
    uint16 action_count;
    uint16 waypoint_count;
    char actions[SOKOBAN_MAX_TOTAL_ACTIONS + 1];
    motion_waypoint_struct waypoints[SOKOBAN_MAX_WAYPOINTS];
    char message[48];
} sokoban_result_struct;

#endif
