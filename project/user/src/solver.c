#include "solver.h"
#include "map_utils.h"
#include <string.h>

// 单箱 BFS 状态编码为 player_cell * MAP_CELLS + box_cell。
// 12x16 地图下状态总数可静态分配，避免在嵌入式运行时引入堆内存碎片风险。
static uint8 bfs_visited[SEARCH_STATE_COUNT];   // 单箱 BFS 访问标记；每次搜索前清零，避免不同 box-target 尝试互相污染。
static uint16 bfs_parent[SEARCH_STATE_COUNT];  // 父状态用于终点回溯；INVALID_STATE 同时表示起点。
static uint16 bfs_queue[SEARCH_STATE_COUNT];   // FIFO 队列容量等于状态数，正常 BFS 不会重复入队溢出。
static char bfs_action[SEARCH_STATE_COUNT];    // 小写为小车移动，大写为推箱动作。
static char single_path[MAX_SINGLE_PATH + 1];  // 每个 box-target 尝试复用同一临时缓冲，结果会复制到 best_path。

static void set_message(solve_result_struct *result, const char *message)
{
    uint32 i = 0;

    while((i < (sizeof(result->message) - 1)) && ('\0' != message[i]))
    {
        result->message[i] = message[i];
        i++;
    }
    result->message[i] = '\0';
}

void clear_result(solve_result_struct *result)
{
    memset(result, 0, sizeof(*result));
    set_message(result, "Not solved");
}

static uint8 map_load(const map_source_struct *source, map_state_struct *map, solve_result_struct *result)
{
    uint8 row;
    uint8 col;
    char value;

    memset(map, 0, sizeof(*map));
    map->player = INVALID_STATE;

    // 求解器和 OpenART/离线地图共用字符协议：+ 表示小车站在目标上。
    // 动态对象解析到 player/boxes/targets 后，grid 只保留静态障碍，便于推箱过程中更新箱子数组。
    for(row = 0; row < MAP_ROWS; row++)
    {
        if(MAP_COLS != strlen(source->rows[row]))
        {
            set_message(result, "Bad map width");
            return 0;
        }

        for(col = 0; col < MAP_COLS; col++)
        {
            value = source->rows[row][col];
            map->grid[row][col] = '.';

            if(('#' == value) || ('X' == value))
            {
                // X 在比赛语义里是炸弹/障碍；对 BFS 来说和墙一样不可进入。
                map->grid[row][col] = '#';
            }
            else if(('C' == value) || ('+' == value))
            {
                map->player = map_cell_index(row, col);
                if('+' == value)
                {
                    if(MAX_BOXES <= map->target_count)
                    {
                        set_message(result, "Too many targets");
                        return 0;
                    }
                    map->targets[map->target_count++] = map_cell_index(row, col);
                }
            }
            else if('B' == value)
            {
                if(MAX_BOXES <= map->box_count)
                {
                    set_message(result, "Too many boxes");
                    return 0;
                }
                map->boxes[map->box_count++] = map_cell_index(row, col);
            }
            else if('T' == value)
            {
                if(MAX_BOXES <= map->target_count)
                {
                    set_message(result, "Too many targets");
                    return 0;
                }
                map->targets[map->target_count++] = map_cell_index(row, col);
            }
            else if('.' != value)
            {
                set_message(result, "Bad map char");
                return 0;
            }
        }
    }

    if(INVALID_STATE == map->player)
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

static uint8 cell_has_other_box(const map_state_struct *map, uint16 cell, uint8 selected_box)
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

static uint8 cell_is_free_for_player(const map_state_struct *map, uint16 cell, uint16 selected_box_cell, uint8 selected_box)
{
    // 玩家移动时，当前正在求解的箱子和其他箱子都必须视为占用格。
    if('#' == map->grid[map_cell_row(cell)][map_cell_col(cell)])
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

static uint8 cell_is_free_for_box(const map_state_struct *map, uint16 cell, uint8 selected_box)
{
    // 箱子可被推入目标点或空地，但不能穿墙，也不能穿过尚未处理的其他箱子。
    if('#' == map->grid[map_cell_row(cell)][map_cell_col(cell)])
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
    int16 row = (int16)map_cell_row(cell) + dr;
    int16 col = (int16)map_cell_col(cell) + dc;

    if((0 > row) || (MAP_ROWS <= row) || (0 > col) || (MAP_COLS <= col))
    {
        return 0;
    }

    *next_cell = map_cell_index((uint8)row, (uint8)col);
    return 1;
}

static uint8 solve_single_box(const map_state_struct *map, uint8 box_index, uint8 target_index, char *path, uint16 *path_len)
{
    static const int8 dr[4] = {-1, 1, 0, 0};
    static const int8 dc[4] = {0, 0, -1, 1};
    static const char move_action[4] = {'u', 'd', 'l', 'r'};
    static const char push_action[4] = {'U', 'D', 'L', 'R'};

    uint16 start_state;
    uint16 read_index = 0;
    uint16 write_index = 0;
    uint16 found_state = INVALID_STATE;
    uint16 state;
    uint16 player;
    uint16 box;
    uint16 next_player;
    uint16 next_box;
    uint16 next_state;
    uint16 reverse_len = 0;
    uint8 dir;
    char reverse_path[MAX_SINGLE_PATH + 1];

    memset(bfs_visited, 0, sizeof(bfs_visited)); // 每个 box-target 尝试独立 BFS，不能复用上一次访问状态。

    start_state = (uint16)(map->player * MAP_CELLS + map->boxes[box_index]);
    bfs_visited[start_state] = 1;
    bfs_parent[start_state] = INVALID_STATE;
    bfs_queue[write_index++] = start_state;

    // 队列按先进先出扩展；在单个 box-target 子问题中，第一次到达目标就是最短动作数。
    // 多箱全局只做贪心拆解，不保证整张地图的全局最优，这是容量和实现复杂度之间的取舍。
    while(read_index < write_index)
    {
        state = bfs_queue[read_index++];
        player = (uint16)(state / MAP_CELLS);
        box = (uint16)(state % MAP_CELLS);

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
                next_state = (uint16)(next_player * MAP_CELLS + next_box);
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
                next_state = (uint16)(next_player * MAP_CELLS + box);
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

    if(INVALID_STATE == found_state)
    {
        return 0;
    }

    state = found_state;
    while(INVALID_STATE != bfs_parent[state])
    {
        if(MAX_SINGLE_PATH <= reverse_len)
        {
            return 0;
        }
        reverse_path[reverse_len++] = bfs_action[state]; // 从终点沿父节点回溯，路径先得到反序。
        state = bfs_parent[state];
    }

    *path_len = reverse_len;
    for(state = 0; state < reverse_len; state++)
    {
        path[state] = reverse_path[reverse_len - 1 - state]; // 翻转为从起点到目标的执行顺序。
    }
    path[reverse_len] = '\0';
    return 1;
}

static uint8 result_append_action(solve_result_struct *result, char action)
{
    if(MAX_TOTAL_ACTIONS <= result->action_count)
    {
        set_message(result, "Action overflow");
        return 0;
    }

    result->actions[result->action_count++] = action;
    result->actions[result->action_count] = '\0';
    return 1;
}

static char waypoint_action_dir(char action)
{
    if(('u' == action) || ('U' == action))
    {
        return 'u';
    }
    if(('d' == action) || ('D' == action))
    {
        return 'd';
    }
    if(('l' == action) || ('L' == action))
    {
        return 'l';
    }
    if(('r' == action) || ('R' == action))
    {
        return 'r';
    }
    return action;
}

static uint8 action_is_push(char action)
{
    return ((action >= 'A') && (action <= 'Z')) ? 1u : 0u;
}

static uint8 action_changes_direction(char previous_action, char action)
{
    return (waypoint_action_dir(previous_action) != waypoint_action_dir(action)) ? 1u : 0u;
}

static uint8 same_waypoint_run(char previous_action, char action)
{
    // 大写动作保持独立 waypoint，保留虚拟推箱动作和单箱任务结束点语义。
    if((0 != action_is_push(previous_action)) || (0 != action_is_push(action)))
    {
        return 0u;
    }

    return (waypoint_action_dir(previous_action) == waypoint_action_dir(action)) ? 1u : 0u;
}

static uint8 action_range_has_separator(const solve_result_struct *result, uint16 start, uint16 end)
{
    uint16 i;

    for(i = start; i < end; i++)
    {
        if('|' == result->actions[i])
        {
            // `|` 不是运动动作，只标记两个单箱任务的边界，waypoint 合并不能跨过它。
            return 1u;
        }
    }
    return 0u;
}

static uint8 result_append_waypoint(solve_result_struct *result,
                                    uint16 player_cell,
                                    char action,
                                    uint8 center_correct_before)
{
    uint16 last_index;
    uint16 action_index;

    if(0 == result->action_count)
    {
        set_message(result, "Waypoint action missing");
        return 0;
    }
    action_index = (uint16)(result->action_count - 1u);

    if(0 < result->waypoint_count)
    {
        last_index = (uint16)(result->waypoint_count - 1u);
        // 连续同方向、同类型动作合并为一个 waypoint，减少屏幕回放点数和后续底盘路径点压力。
        // 合并前检查 action 区间，保证 `|` 分隔的任务边界不会被吞掉。
        if((0 == center_correct_before) &&
           (0 == action_range_has_separator(result, result->waypoints[last_index].action_end, action_index)) &&
           (0 != same_waypoint_run(result->waypoints[last_index].action, action)))
        {
            result->waypoints[last_index].row = map_cell_row(player_cell);
            result->waypoints[last_index].col = map_cell_col(player_cell);
            result->waypoints[last_index].action = action;
            result->waypoints[last_index].action_end = result->action_count;
            return 1;
        }
    }

    if(MAX_WAYPOINTS <= result->waypoint_count)
    {
        set_message(result, "Waypoint overflow");
        return 0;
    }

    result->waypoints[result->waypoint_count].row = map_cell_row(player_cell);
    result->waypoints[result->waypoint_count].col = map_cell_col(player_cell);
    result->waypoints[result->waypoint_count].action = action;
    result->waypoints[result->waypoint_count].action_start = action_index;
    result->waypoints[result->waypoint_count].action_end = result->action_count;
    result->waypoints[result->waypoint_count].task_end = 0;
    result->waypoints[result->waypoint_count].center_correct_before = center_correct_before;
    result->waypoint_count++;
    return 1;
}

static uint8 result_mark_last_waypoint_task_end(solve_result_struct *result)
{
    if(0 == result->waypoint_count)
    {
        set_message(result, "Task waypoint missing");
        return 0;
    }

    result->waypoints[result->waypoint_count - 1u].task_end = 1;
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

static uint8 apply_path_to_runtime(map_state_struct *map, uint8 box_index, uint8 target_index, const char *path, uint16 path_len, solve_result_struct *result)
{
    uint16 player = map->player;
    uint16 box = map->boxes[box_index];
    uint16 next_player;
    uint16 next_box;
    uint16 i;
    int8 row_delta;
    int8 col_delta;
    char action;
    uint8 center_correct_before;

    // BFS 只返回动作串；这里把动作重放到运行态地图，作为多箱拆解后下一轮 BFS 的新起点。
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
            // 大写动作表示本步车进入箱子原位置，同时箱子沿相同方向前进一格。
            if(0 == step_cell(box, row_delta, col_delta, &next_box))
            {
                set_message(result, "Apply push failed");
                return 0;
            }
            box = next_box;
        }

        /* 普通转向仍在新段前校正；推箱链则提前到最后一个普通靠近动作前，
         * 避免车已到箱子相邻格后才做视觉小修。 */
        center_correct_before = ((0u < i) &&
                                 (0 != action_changes_direction(path[i - 1u], action))) ? 1u : 0u;
        if((0 == action_is_push(action)) &&
           ((i + 1u) < path_len) &&
           (0 != action_is_push(path[i + 1u])))
        {
            center_correct_before = 1u;
        }
        else if(0 != action_is_push(action))
        {
            if((0u == i) ||
               ((0 != action_is_push(path[i - 1u])) &&
                (0 != action_changes_direction(path[i - 1u], action))))
            {
                center_correct_before = 1u;
            }
            else if(0 == action_is_push(path[i - 1u]))
            {
                /* 上一个普通动作已承担本推箱链的视觉准备，避免大写动作重复等待。 */
                center_correct_before = 0u;
            }
        }
        player = next_player;
        if(0 == result_append_action(result, action))
        {
            return 0;
        }
        if(0 == result_append_waypoint(result, player, action, center_correct_before))
        {
            return 0;
        }
    }

    if(box != map->targets[target_index])
    {
        // 理论上只有路径缓冲被截断或状态重放逻辑出错才会触发，保留检查便于定位容量问题。
        set_message(result, "Target not reached");
        return 0;
    }

    if(0 == result_mark_last_waypoint_task_end(result))
    {
        return 0;
    }

    map->player = player;
    // 已完成的箱子和目标从后续子问题中移除，剩余箱子继续作为障碍参与 BFS。
    remove_cell_from_list(map->boxes, &map->box_count, box_index);
    remove_cell_from_list(map->targets, &map->target_count, target_index);
    result->task_count++;
    return 1;
}

uint8 solve_map(const map_source_struct *source, solve_result_struct *result)
{
    map_state_struct map;
    char best_path[MAX_SINGLE_PATH + 1];
    uint16 best_len;
    uint8 best_box;
    uint8 best_target;
    uint8 found;
    uint8 box_i;
    uint8 target_i;
    uint16 trial_len;

    clear_result(result);
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
                // 每轮枚举所有剩余 box-target 对，选择当前动作数最短的一对先完成。
                // 这是可解释的贪心策略，不等价于完整多箱 Sokoban 搜索。
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
            // 分隔符标记一次单箱任务完成；屏幕回放会跳过它，统计和 waypoint 合并会保留边界。
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

uint8 solve_bound_box_path(const map_source_struct *source,
                           uint16 box_cell,
                           uint16 target_cell,
                           solve_result_struct *result)
{
    map_state_struct map;
    uint16 path_len;
    uint8 box_index = 0u;
    uint8 target_index = 0u;
    uint8 box_found = 0u;
    uint8 target_found = 0u;
    uint8 index;

    if((0 == source) || (0 == result))
    {
        return 0u;
    }
    clear_result(result);
    if(0 == map_load(source, &map, result))
    {
        return 0u;
    }

    for(index = 0u; index < map.box_count; index++)
    {
        if(box_cell == map.boxes[index])
        {
            box_index = index;
            box_found = 1u;
            break;
        }
    }
    for(index = 0u; index < map.target_count; index++)
    {
        if(target_cell == map.targets[index])
        {
            target_index = index;
            target_found = 1u;
            break;
        }
    }
    if((0u == box_found) || (0u == target_found))
    {
        set_message(result, "Bound cell missing");
        return 0u;
    }
    if(0 == solve_single_box(&map, box_index, target_index, single_path, &path_len))
    {
        set_message(result, "No bound BFS path");
        return 0u;
    }
    if(0 == apply_path_to_runtime(&map, box_index, target_index,
                                  single_path, path_len, result))
    {
        return 0u;
    }

    result->solved = 1u;
    set_message(result, "Solved bound task");
    return 1u;
}

uint8 solve_navigation_path(const map_source_struct *source,
                            uint8 target_row,
                            uint8 target_col,
                            solve_result_struct *result)
{
    static const int8 dr[4] = {-1, 1, 0, 0};
    static const int8 dc[4] = {0, 0, -1, 1};
    static const char move_action[4] = {'u', 'd', 'l', 'r'};
    char reverse_path[MAP_CELLS];
    uint16 start_cell = INVALID_STATE;
    uint16 target_cell;
    uint16 current_cell;
    uint16 next_cell;
    uint16 read_index = 0;
    uint16 write_index = 0;
    uint16 reverse_len = 0;
    uint16 path_index;
    uint8 car_count = 0;
    uint8 row;
    uint8 col;
    uint8 dir;
    char value;
    char action;
    char previous_action = '\0';

    if((0 == source) || (0 == result))
    {
        return 0;
    }
    clear_result(result);
    if((target_row >= MAP_ROWS) || (target_col >= MAP_COLS))
    {
        set_message(result, "Bad navigation target");
        return 0;
    }

    for(row = 0; row < MAP_ROWS; row++)
    {
        if(MAP_COLS != strlen(source->rows[row]))
        {
            set_message(result, "Bad map width");
            return 0;
        }
        for(col = 0; col < MAP_COLS; col++)
        {
            value = source->rows[row][col];
            if(('C' == value) || ('+' == value))
            {
                start_cell = map_cell_index(row, col);
                car_count++;
            }
            else if(('#' != value) && ('.' != value) && ('B' != value) &&
                    ('T' != value) && ('+' != value) && ('X' != value))
            {
                set_message(result, "Bad map char");
                return 0;
            }
        }
    }
    if((1u != car_count) || (INVALID_STATE == start_cell))
    {
        set_message(result, "Bad car count");
        return 0;
    }

    value = source->rows[target_row][target_col];
    if(('#' == value) || ('X' == value) || ('B' == value))
    {
        set_message(result, "Navigation target blocked");
        return 0;
    }
    target_cell = map_cell_index(target_row, target_col);
    if(start_cell == target_cell)
    {
        result->solved = 1;
        set_message(result, "Navigation ready");
        return 1;
    }

    memset(bfs_visited, 0, MAP_CELLS * sizeof(bfs_visited[0]));
    bfs_visited[start_cell] = 1;
    bfs_parent[start_cell] = INVALID_STATE;
    bfs_queue[write_index++] = start_cell;

    while(read_index < write_index)
    {
        current_cell = bfs_queue[read_index++];
        if(current_cell == target_cell)
        {
            break;
        }
        for(dir = 0; dir < 4; dir++)
        {
            if(0 == step_cell(current_cell, dr[dir], dc[dir], &next_cell))
            {
                continue;
            }
            value = source->rows[map_cell_row(next_cell)][map_cell_col(next_cell)];
            if(('#' == value) || ('X' == value) || ('B' == value) ||
               (0 != bfs_visited[next_cell]))
            {
                continue;
            }
            bfs_visited[next_cell] = 1;
            bfs_parent[next_cell] = current_cell;
            bfs_action[next_cell] = move_action[dir];
            bfs_queue[write_index++] = next_cell;
        }
    }
    if(0 == bfs_visited[target_cell])
    {
        set_message(result, "No navigation path");
        return 0;
    }

    current_cell = target_cell;
    while(INVALID_STATE != bfs_parent[current_cell])
    {
        reverse_path[reverse_len++] = bfs_action[current_cell];
        current_cell = bfs_parent[current_cell];
    }

    current_cell = start_cell;
    for(path_index = 0; path_index < reverse_len; path_index++)
    {
        action = reverse_path[reverse_len - 1u - path_index];
        for(dir = 0; dir < 4; dir++)
        {
            if(action == move_action[dir])
            {
                break;
            }
        }
        if((dir >= 4u) ||
           (0 == step_cell(current_cell, dr[dir], dc[dir], &next_cell)) ||
           (0 == result_append_action(result, action)) ||
           (0 == result_append_waypoint(
               result, next_cell, action,
               ((0u < path_index) &&
                (0 != action_changes_direction(previous_action, action))) ? 1u : 0u)))
        {
            set_message(result, "Navigation output failed");
            return 0;
        }
        previous_action = action;
        current_cell = next_cell;
    }

    result->solved = 1;
    set_message(result, "Navigation solved");
    return 1;
}
