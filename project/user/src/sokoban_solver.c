#include "sokoban_solver.h"
#include <string.h>

static uint8 bfs_visited[SOKOBAN_STATE_COUNT];
static uint16 bfs_parent[SOKOBAN_STATE_COUNT];
static uint16 bfs_queue[SOKOBAN_STATE_COUNT];
static char bfs_action[SOKOBAN_STATE_COUNT];
static char single_path[SOKOBAN_MAX_SINGLE_PATH + 1];

static uint16 cell_index(uint8 row, uint8 col)
{
    return (uint16)(row * SOKOBAN_MAP_COLS + col);
}

static uint8 cell_row(uint16 cell)
{
    return (uint8)(cell / SOKOBAN_MAP_COLS);
}

static uint8 cell_col(uint16 cell)
{
    return (uint8)(cell % SOKOBAN_MAP_COLS);
}

static void set_message(sokoban_result_struct *result, const char *message)
{
    uint32 i = 0;

    while((i < (sizeof(result->message) - 1)) && ('\0' != message[i]))
    {
        result->message[i] = message[i];
        i++;
    }
    result->message[i] = '\0';
}

void sokoban_result_clear(sokoban_result_struct *result)
{
    memset(result, 0, sizeof(*result));
    set_message(result, "Not solved");
}

static uint8 map_load(const offline_map_struct *source, sokoban_map_struct *map, sokoban_result_struct *result)
{
    uint8 row;
    uint8 col;
    char value;

    memset(map, 0, sizeof(*map));
    map->player = SOKOBAN_STATE_NONE;

    for(row = 0; row < SOKOBAN_MAP_ROWS; row++)
    {
        if(SOKOBAN_MAP_COLS != strlen(source->rows[row]))
        {
            set_message(result, "Bad map width");
            return 0;
        }

        for(col = 0; col < SOKOBAN_MAP_COLS; col++)
        {
            value = source->rows[row][col];
            map->grid[row][col] = '.';

            if(SOKOBAN_MAP_FORMAT_SEEKFREE == source->format)
            {
                if('#' == value)
                {
                    map->grid[row][col] = '#';
                }
                else if('C' == value)
                {
                    map->player = cell_index(row, col);
                }
                else if('$' == value)
                {
                    if(SOKOBAN_MAX_BOXES <= map->box_count)
                    {
                        set_message(result, "Too many boxes");
                        return 0;
                    }
                    map->boxes[map->box_count++] = cell_index(row, col);
                }
                else if('.' == value)
                {
                    if(SOKOBAN_MAX_BOXES <= map->target_count)
                    {
                        set_message(result, "Too many targets");
                        return 0;
                    }
                    map->targets[map->target_count++] = cell_index(row, col);
                }
                else if('*' == value)
                {
                    map->grid[row][col] = '#';
                }
                else if('-' != value)
                {
                    set_message(result, "Bad map char");
                    return 0;
                }
            }
            else if('#' == value)
            {
                map->grid[row][col] = '#';
            }
            else if('C' == value)
            {
                map->player = cell_index(row, col);
            }
            else if('B' == value)
            {
                if(SOKOBAN_MAX_BOXES <= map->box_count)
                {
                    set_message(result, "Too many boxes");
                    return 0;
                }
                map->boxes[map->box_count++] = cell_index(row, col);
            }
            else if('T' == value)
            {
                if(SOKOBAN_MAX_BOXES <= map->target_count)
                {
                    set_message(result, "Too many targets");
                    return 0;
                }
                map->targets[map->target_count++] = cell_index(row, col);
            }
            else if('.' != value)
            {
                set_message(result, "Bad map char");
                return 0;
            }
        }
    }

    if(SOKOBAN_STATE_NONE == map->player)
    {
        set_message(result, "No car");
        return 0;
    }
    if((0 == map->box_count) || (map->box_count != map->target_count))
    {
        set_message(result, "Box/target mismatch");
        return 0;
    }
    return 1;
}

static uint8 cell_has_other_box(const sokoban_map_struct *map, uint16 cell, uint8 selected_box)
{
    uint8 i;

    for(i = 0; i < map->box_count; i++)
    {
        if((i != selected_box) && (cell == map->boxes[i]))
        {
            return 1;
        }
    }
    return 0;
}

static uint8 cell_is_free_for_player(const sokoban_map_struct *map, uint16 cell, uint16 selected_box_cell, uint8 selected_box)
{
    if('#' == map->grid[cell_row(cell)][cell_col(cell)])
    {
        return 0;
    }
    if(cell == selected_box_cell)
    {
        return 0;
    }
    if(cell_has_other_box(map, cell, selected_box))
    {
        return 0;
    }
    return 1;
}

static uint8 cell_is_free_for_box(const sokoban_map_struct *map, uint16 cell, uint8 selected_box)
{
    if('#' == map->grid[cell_row(cell)][cell_col(cell)])
    {
        return 0;
    }
    if(cell_has_other_box(map, cell, selected_box))
    {
        return 0;
    }
    return 1;
}

static uint8 step_cell(uint16 cell, int8 dr, int8 dc, uint16 *next_cell)
{
    int16 row = (int16)cell_row(cell) + dr;
    int16 col = (int16)cell_col(cell) + dc;

    if((0 > row) || (SOKOBAN_MAP_ROWS <= row) || (0 > col) || (SOKOBAN_MAP_COLS <= col))
    {
        return 0;
    }

    *next_cell = cell_index((uint8)row, (uint8)col);
    return 1;
}

static uint8 solve_single_box(const sokoban_map_struct *map, uint8 box_index, uint8 target_index, char *path, uint16 *path_len)
{
    static const int8 dr[4] = {-1, 1, 0, 0};
    static const int8 dc[4] = {0, 0, -1, 1};
    static const char move_action[4] = {'u', 'd', 'l', 'r'};
    static const char push_action[4] = {'U', 'D', 'L', 'R'};

    uint16 start_state;
    uint16 read_index = 0;
    uint16 write_index = 0;
    uint16 found_state = SOKOBAN_STATE_NONE;
    uint16 state;
    uint16 player;
    uint16 box;
    uint16 next_player;
    uint16 next_box;
    uint16 next_state;
    uint16 reverse_len = 0;
    uint8 dir;
    char reverse_path[SOKOBAN_MAX_SINGLE_PATH + 1];

    memset(bfs_visited, 0, sizeof(bfs_visited));

    start_state = (uint16)(map->player * SOKOBAN_MAP_CELLS + map->boxes[box_index]);
    bfs_visited[start_state] = 1;
    bfs_parent[start_state] = SOKOBAN_STATE_NONE;
    bfs_queue[write_index++] = start_state;

    while(read_index < write_index)
    {
        state = bfs_queue[read_index++];
        player = (uint16)(state / SOKOBAN_MAP_CELLS);
        box = (uint16)(state % SOKOBAN_MAP_CELLS);

        if(box == map->targets[target_index])
        {
            found_state = state;
            break;
        }

        for(dir = 0; dir < 4; dir++)
        {
            if(0 == step_cell(player, dr[dir], dc[dir], &next_player))
            {
                continue;
            }

            next_box = box;
            if(next_player == box)
            {
                if(0 == step_cell(box, dr[dir], dc[dir], &next_box))
                {
                    continue;
                }
                if(0 == cell_is_free_for_box(map, next_box, box_index))
                {
                    continue;
                }
                next_state = (uint16)(next_player * SOKOBAN_MAP_CELLS + next_box);
                if(0 == bfs_visited[next_state])
                {
                    bfs_visited[next_state] = 1;
                    bfs_parent[next_state] = state;
                    bfs_action[next_state] = push_action[dir];
                    bfs_queue[write_index++] = next_state;
                }
            }
            else
            {
                if(0 == cell_is_free_for_player(map, next_player, box, box_index))
                {
                    continue;
                }
                next_state = (uint16)(next_player * SOKOBAN_MAP_CELLS + box);
                if(0 == bfs_visited[next_state])
                {
                    bfs_visited[next_state] = 1;
                    bfs_parent[next_state] = state;
                    bfs_action[next_state] = move_action[dir];
                    bfs_queue[write_index++] = next_state;
                }
            }
        }
    }

    if(SOKOBAN_STATE_NONE == found_state)
    {
        return 0;
    }

    state = found_state;
    while(SOKOBAN_STATE_NONE != bfs_parent[state])
    {
        if(SOKOBAN_MAX_SINGLE_PATH <= reverse_len)
        {
            return 0;
        }
        reverse_path[reverse_len++] = bfs_action[state];
        state = bfs_parent[state];
    }

    *path_len = reverse_len;
    for(state = 0; state < reverse_len; state++)
    {
        path[state] = reverse_path[reverse_len - 1 - state];
    }
    path[reverse_len] = '\0';
    return 1;
}

static uint8 result_append_action(sokoban_result_struct *result, char action)
{
    if(SOKOBAN_MAX_TOTAL_ACTIONS <= result->action_count)
    {
        set_message(result, "Action overflow");
        return 0;
    }

    result->actions[result->action_count++] = action;
    result->actions[result->action_count] = '\0';
    return 1;
}

static uint8 result_append_waypoint(sokoban_result_struct *result, uint16 player_cell, char action)
{
    if(SOKOBAN_MAX_WAYPOINTS <= result->waypoint_count)
    {
        set_message(result, "Waypoint overflow");
        return 0;
    }

    result->waypoints[result->waypoint_count].row = cell_row(player_cell);
    result->waypoints[result->waypoint_count].col = cell_col(player_cell);
    result->waypoints[result->waypoint_count].action = action;
    result->waypoint_count++;
    return 1;
}

static void remove_cell_from_list(uint16 *list, uint8 *count, uint8 index)
{
    uint8 i;

    for(i = index; (i + 1) < *count; i++)
    {
        list[i] = list[i + 1];
    }
    (*count)--;
}

static uint8 apply_path_to_runtime(sokoban_map_struct *map, uint8 box_index, uint8 target_index, const char *path, uint16 path_len, sokoban_result_struct *result)
{
    uint16 player = map->player;
    uint16 box = map->boxes[box_index];
    uint16 next_player;
    uint16 next_box;
    uint16 i;
    int8 row_delta;
    int8 col_delta;
    char action;

    for(i = 0; i < path_len; i++)
    {
        action = path[i];
        row_delta = 0;
        col_delta = 0;

        if(('u' == action) || ('U' == action))
        {
            row_delta = -1;
        }
        else if(('d' == action) || ('D' == action))
        {
            row_delta = 1;
        }
        else if(('l' == action) || ('L' == action))
        {
            col_delta = -1;
        }
        else if(('r' == action) || ('R' == action))
        {
            col_delta = 1;
        }

        if(0 == step_cell(player, row_delta, col_delta, &next_player))
        {
            set_message(result, "Apply path failed");
            return 0;
        }

        if((action >= 'A') && (action <= 'Z'))
        {
            if(0 == step_cell(box, row_delta, col_delta, &next_box))
            {
                set_message(result, "Apply push failed");
                return 0;
            }
            box = next_box;
        }

        player = next_player;
        if(0 == result_append_action(result, action))
        {
            return 0;
        }
        if(0 == result_append_waypoint(result, player, action))
        {
            return 0;
        }
    }

    if(box != map->targets[target_index])
    {
        set_message(result, "Target not reached");
        return 0;
    }

    map->player = player;
    remove_cell_from_list(map->boxes, &map->box_count, box_index);
    remove_cell_from_list(map->targets, &map->target_count, target_index);
    result->task_count++;
    return 1;
}

uint8 sokoban_solve_map_multi_box(const offline_map_struct *source, sokoban_result_struct *result)
{
    sokoban_map_struct map;
    char best_path[SOKOBAN_MAX_SINGLE_PATH + 1];
    uint16 best_len;
    uint8 best_box;
    uint8 best_target;
    uint8 found;
    uint8 box_i;
    uint8 target_i;
    uint16 trial_len;

    sokoban_result_clear(result);
    if(0 == map_load(source, &map, result))
    {
        return 0;
    }

    while(0 < map.box_count)
    {
        found = 0;
        best_len = 0xFFFF;
        best_box = 0;
        best_target = 0;

        for(box_i = 0; box_i < map.box_count; box_i++)
        {
            for(target_i = 0; target_i < map.target_count; target_i++)
            {
                if(0 != solve_single_box(&map, box_i, target_i, single_path, &trial_len))
                {
                    if((0 == found) || (trial_len < best_len))
                    {
                        found = 1;
                        best_len = trial_len;
                        best_box = box_i;
                        best_target = target_i;
                        memcpy(best_path, single_path, trial_len + 1);
                    }
                }
            }
        }

        if(0 == found)
        {
            set_message(result, "No BFS path");
            return 0;
        }

        if(0 == apply_path_to_runtime(&map, best_box, best_target, best_path, best_len, result))
        {
            return 0;
        }

        if(0 < map.box_count)
        {
            if(0 == result_append_action(result, '|'))
            {
                return 0;
            }
        }
    }

    result->solved = 1;
    set_message(result, "Solved");
    return 1;
}
