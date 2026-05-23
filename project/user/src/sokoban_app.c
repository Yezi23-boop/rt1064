#include "zf_common_headfile.h"
#include "sokoban_app.h"
#include "sokoban_maps.h"
#include "sokoban_solver.h"
#include "sokoban_motion.h"

#define IPS200_TYPE             (IPS200_TYPE_SPI)
#define UI_LINE_H               (16)
#define UI_MAP_X                (0)
#define UI_MAP_Y                (UI_LINE_H * 2)
#define UI_STATUS_Y             (UI_LINE_H * 15)
#define UI_PATH_Y               (UI_LINE_H * 16)
#define UI_KEYS_Y               (UI_LINE_H * 18)

static sokoban_result_struct last_result;
static uint8 selected_map = 0;
static uint8 need_redraw = 1;

static void ui_show_map_rows(const offline_map_struct *source)
{
    uint8 row;

    for(row = 0; row < SOKOBAN_MAP_ROWS; row++)
    {
        ips200_show_string(UI_MAP_X, (uint16)(UI_MAP_Y + row * UI_LINE_H), source->rows[row]);
    }
}

static void ui_show_result(const sokoban_result_struct *result)
{
    char line[64];
    char preview[52];
    uint8 i;
    uint8 max_copy = 48;

    snprintf(line, sizeof(line), "%s task=%d act=%d wp=%d",
        result->message,
        result->task_count,
        result->action_count,
        result->waypoint_count);
    ips200_show_string(0, UI_STATUS_Y, line);

    if(0 == result->action_count)
    {
        ips200_show_string(0, UI_PATH_Y, "Path: <empty>");
        return;
    }

    if(result->action_count < max_copy)
    {
        max_copy = (uint8)result->action_count;
    }

    preview[0] = 'P';
    preview[1] = ':';
    preview[2] = ' ';
    for(i = 0; i < max_copy; i++)
    {
        preview[i + 3] = result->actions[i];
    }
    if(result->action_count > max_copy)
    {
        preview[max_copy + 3] = '.';
        preview[max_copy + 4] = '.';
        preview[max_copy + 5] = '.';
        preview[max_copy + 6] = '\0';
    }
    else
    {
        preview[max_copy + 3] = '\0';
    }
    ips200_show_string(0, UI_PATH_Y, preview);
}

static void ui_redraw(void)
{
    char line[40];
    const offline_map_struct *source = offline_map_get(selected_map);

    ips200_clear();
    snprintf(line, sizeof(line), "Map %d/%d: %s", selected_map + 1, offline_map_count_get(), source->name);
    ips200_show_string(0, 0, "RT1064 Sokoban BFS");
    ips200_show_string(0, UI_LINE_H, line);
    ui_show_map_rows(source);
    ui_show_result(&last_result);
    ips200_show_string(0, UI_KEYS_Y, "K1 Prev K2 Next K3 BFS K4 Emit");
}

static void ui_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    ips200_init(IPS200_TYPE);
    ips200_clear();
}

static void solve_selected_map(void)
{
    const offline_map_struct *source = offline_map_get(selected_map);

    printf("SOLVE_BEGIN map=%d %s\r\n", selected_map + 1, source->name);
    if(0 != sokoban_solve_map_multi_box(source, &last_result))
    {
        printf("SOLVE_OK tasks=%d actions=%d waypoints=%d\r\n",
            last_result.task_count,
            last_result.action_count,
            last_result.waypoint_count);
        printf("ACTIONS=%s\r\n", last_result.actions);
    }
    else
    {
        printf("SOLVE_FAIL %s\r\n", last_result.message);
    }
    need_redraw = 1;
}

static void handle_keys(void)
{
    key_scanner();

    if(KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        key_clear_state(KEY_1);
        selected_map = (0 == selected_map) ? (offline_map_count_get() - 1) : (selected_map - 1);
        sokoban_result_clear(&last_result);
        need_redraw = 1;
    }

    if(KEY_SHORT_PRESS == key_get_state(KEY_2))
    {
        key_clear_state(KEY_2);
        selected_map = (uint8)((selected_map + 1) % offline_map_count_get());
        sokoban_result_clear(&last_result);
        need_redraw = 1;
    }

    if(KEY_SHORT_PRESS == key_get_state(KEY_3))
    {
        key_clear_state(KEY_3);
        solve_selected_map();
    }

    if(KEY_SHORT_PRESS == key_get_state(KEY_4))
    {
        key_clear_state(KEY_4);
        sokoban_motion_submit_plan(&last_result);
    }

    if((KEY_LONG_PRESS == key_get_state(KEY_1)) ||
       (KEY_LONG_PRESS == key_get_state(KEY_2)) ||
       (KEY_LONG_PRESS == key_get_state(KEY_3)) ||
       (KEY_LONG_PRESS == key_get_state(KEY_4)))
    {
        key_clear_all_state();
    }
}

void sokoban_app_init(void)
{
    key_init(5);
    sokoban_result_clear(&last_result);
    ui_init();
    ui_redraw();
}

void sokoban_app_poll(void)
{
    handle_keys();

    if(0 != need_redraw)
    {
        ui_redraw();
        need_redraw = 0;
    }
}
