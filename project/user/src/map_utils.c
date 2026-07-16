#include "map_utils.h"
#include <string.h>

uint16 map_cell_index(uint8 row, uint8 col)
{
    return (uint16)(row * MAP_COLS + col);
}

uint8 map_cell_row(uint16 cell)
{
    return (uint8)(cell / MAP_COLS);
}

uint8 map_cell_col(uint16 cell)
{
    return (uint8)(cell % MAP_COLS);
}

void map_copy_rows(char dst[MAP_ROWS][MAP_COLS + 1], const map_source_struct *source)
{
    uint8 row;
    uint8 col;

    for(row = 0; row < MAP_ROWS; row++)
    {
        for(col = 0; col < MAP_COLS; col++)
        {
            dst[row][col] = source->rows[row][col];
        }
        // 快照行缓存按 C 字符串暴露给屏幕和日志，额外的 NUL 不属于地图协议。
        dst[row][MAP_COLS] = '\0';
    }
}

uint8 map_rows_equal(const char rows[MAP_ROWS][MAP_COLS + 1], const map_source_struct *source)
{
    uint8 row;
    uint8 col;

    for(row = 0; row < MAP_ROWS; row++)
    {
        for(col = 0; col < MAP_COLS; col++)
        {
            if(rows[row][col] != source->rows[row][col])
            {
                return 0;
            }
        }
    }
    return 1;
}

void map_source_snapshot(map_source_struct *dst,
                         char rows[MAP_ROWS][MAP_COLS + 1],
                         const map_source_struct *source)
{
    uint8 row;

    // map_source_struct 只保存行指针，必须让调用方持有真实行缓存，避免指向临时数组。
    map_copy_rows(rows, source);
    dst->name = source->name;
    for(row = 0; row < MAP_ROWS; row++)
    {
        dst->rows[row] = rows[row];
    }
}

void map_scan_stats(const map_source_struct *source, map_scan_stats_struct *stats)
{
    uint8 row;
    uint8 col;
    char value;

    stats->car_count = 0;
    stats->box_count = 0;
    stats->target_count = 0;
    stats->car_row = 0xFFu;
    stats->car_col = 0xFFu;

    // 统计函数给菜单/调试提示使用；NULL 地图按“无数据”处理，避免上层额外分支。
    if(0 == source)
    {
        return;
    }

    for(row = 0; row < MAP_ROWS; row++)
    {
        for(col = 0; col < MAP_COLS; col++)
        {
            value = source->rows[row][col];
            if(('C' == value) || ('+' == value))
            {
                stats->car_count++;
                if(1u == stats->car_count)
                {
                    stats->car_row = row;
                    stats->car_col = col;
                }
                else
                {
                    stats->car_row = 0xFFu;
                    stats->car_col = 0xFFu;
                }
            }
            if(('B' == value) || (MAP_BOX_ON_TARGET == value))
            {
                stats->box_count++;
            }
            if(('T' == value) || ('+' == value) ||
               (MAP_BOX_ON_TARGET == value))
            {
                stats->target_count++;
            }
        }
    }
}

uint8 map_find_car(const map_source_struct *source, uint8 *row, uint8 *col, uint8 *count)
{
    map_scan_stats_struct stats;

    map_scan_stats(source, &stats);

    if(0 != row)
    {
        *row = stats.car_row;
    }
    if(0 != col)
    {
        *col = stats.car_col;
    }
    if(0 != count)
    {
        *count = stats.car_count;
    }

    return (1u == stats.car_count) ? 1u : 0u;
}

uint8 map_validate_player_center(const map_source_struct *source,
                                 uint16 center_col_q,
                                 uint16 center_row_q,
                                 uint8 *car_row,
                                 uint8 *car_col)
{
    uint8 row;
    uint8 col;

    if((0 == car_row) || (0 == car_col) ||
       (center_col_q >= (MAP_COLS * 100u)) ||
       (center_row_q >= (MAP_ROWS * 100u)) ||
       (0 == map_find_car(source, &row, &col, 0)) ||
       ((uint8)(center_col_q / 100u) != col) ||
       ((uint8)(center_row_q / 100u) != row))
    {
        return 0u;
    }
    *car_row = row;
    *car_col = col;
    return 1u;
}

void map_stability_tracker_reset(map_stability_tracker_struct *tracker,
                                 uint32 current_frame)
{
    if(0 == tracker)
    {
        return;
    }
    memset(tracker->candidate_rows, 0, sizeof(tracker->candidate_rows));
    tracker->last_frame = current_frame;
    tracker->candidate_valid = 0u;
    tracker->stable_count = 0u;
}

void map_stability_tracker_reset_candidate(map_stability_tracker_struct *tracker)
{
    if(0 == tracker)
    {
        return;
    }
    tracker->candidate_valid = 0u;
    tracker->stable_count = 0u;
}

map_stability_result_enum map_stability_tracker_push(
    map_stability_tracker_struct *tracker,
    uint32 frame,
    const map_source_struct *source,
    uint8 required_frames)
{
    if((0 == tracker) || (0 == source) || (0u == frame) ||
       (frame == tracker->last_frame))
    {
        return MAP_STABILITY_NO_NEW_FRAME;
    }
    tracker->last_frame = frame;
    if((0u == tracker->candidate_valid) ||
       (0u == map_rows_equal(tracker->candidate_rows, source)))
    {
        map_copy_rows(tracker->candidate_rows, source);
        tracker->candidate_valid = 1u;
        tracker->stable_count = 1u;
    }
    else if(tracker->stable_count < required_frames)
    {
        tracker->stable_count++;
    }
    return (tracker->stable_count >= required_frames) ?
           MAP_STABILITY_READY : MAP_STABILITY_PENDING;
}
