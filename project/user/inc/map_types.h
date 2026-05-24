#ifndef _map_types_h_
#define _map_types_h_

#include "zf_common_typedef.h"

#define MAP_ROWS                (12)
#define MAP_COLS                (16)
#define MAP_CELLS               (MAP_ROWS * MAP_COLS)
#define SEARCH_STATE_COUNT             (MAP_CELLS * MAP_CELLS)
#define INVALID_STATE              (0xFFFF)
#define MAX_BOXES               (8)
#define MAX_SINGLE_PATH         (512)
#define MAX_TOTAL_ACTIONS       (1024)
#define MAX_WAYPOINTS           (256)
#define MAP_FORMAT_RT           (0)
#define MAP_FORMAT_SEEKFREE     (1)

typedef struct
{
    const char *name;
    uint8 format;
    const char *rows[MAP_ROWS];
} map_source_struct;

typedef struct
{
    char grid[MAP_ROWS][MAP_COLS];
    uint16 player;
    uint16 boxes[MAX_BOXES];
    uint16 targets[MAX_BOXES];
    uint8 box_count;
    uint8 target_count;
} map_state_struct;

typedef struct
{
    uint8 row;
    uint8 col;
    char action;
} waypoint_struct;

typedef struct
{
    uint8 solved;
    uint8 task_count;
    uint16 action_count;
    uint16 waypoint_count;
    char actions[MAX_TOTAL_ACTIONS + 1];
    waypoint_struct waypoints[MAX_WAYPOINTS];
    char message[48];
} solve_result_struct;

#endif
