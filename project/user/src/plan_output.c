#include "zf_common_headfile.h"
#include "plan_output.h"

void print_plan(const solve_result_struct *result)
{
    uint16 i;

    if(0 == result->solved)
    {
        printf("MOTION_PLAN_NOT_READY\r\n");
        return;
    }

    printf("MOTION_PLAN_BEGIN count=%d\r\n", result->waypoint_count);
    for(i = 0; i < result->waypoint_count; i++)
    {
        printf("WP %03d row=%02d col=%02d action=%c\r\n",
            i,
            result->waypoints[i].row,
            result->waypoints[i].col,
            result->waypoints[i].action);
    }
    printf("MOTION_PLAN_END\r\n");
}
