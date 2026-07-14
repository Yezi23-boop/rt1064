#include "map_utils.h"

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
    stats->car_row = 0;
    stats->car_col = 0;

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
                if(0 == stats->car_count)
                {
                    // 多个 C 是非法地图，但保留第一个位置方便屏幕定位和错误提示。
                    stats->car_row = row;
                    stats->car_col = col;
                }
                stats->car_count++;
            }
            if('B' == value)
            {
                stats->box_count++;
            }
            if(('T' == value) || ('+' == value))
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
