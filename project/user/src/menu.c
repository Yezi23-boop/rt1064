#include "zf_common_headfile.h"
#include "menu.h"
#include "menu_key.h"
#include "maps.h"
#include "solver.h"
#include "settings.h"
#include "screen.h"
#include "timebase.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "openart_uart.h"

#define MENU_KEY_SCAN_PERIOD_MS (5)
#define PLAYBACK_STEP_MS        (300u)
#define HOME_DYNAMIC_REFRESH_MS (500u)
#define HOME_ITEM_COUNT         (3)
#define RUN_ITEM_COUNT          (3)
#define ROOT_PAGE_COUNT         (4)

typedef enum
{
    MENU_PAGE_HOME         = 0,
    MENU_PAGE_RUN          = 1,
    MENU_PAGE_DEBUG        = 2,
    MENU_PAGE_INFO         = 3,
    MENU_PAGE_RUN_MAP      = 11,
    MENU_PAGE_RUN_MODE     = 12,
    MENU_PAGE_RUN_EXECUTE  = 13,
    MENU_PAGE_RUN_PLAYBACK = 14,
    MENU_PAGE_RUN_DEMO     = 15,
} menu_page_id_enum;

typedef void (*menu_page_draw_func)(void);
typedef void (*menu_page_refresh_func)(void);
typedef void (*menu_page_key_func)(menu_key_event_enum event);

typedef struct
{
    menu_page_id_enum id;
    menu_page_id_enum parent;
    uint16 refresh_ms;
    menu_page_draw_func draw_full;
    menu_page_refresh_func refresh_dynamic;
    menu_page_key_func on_key;
} menu_page_def_struct;

static const char *const home_items[HOME_ITEM_COUNT] =
{
    "Run",
    "Debug",
    "Info",
};

static const menu_page_id_enum home_item_pages[HOME_ITEM_COUNT] =
{
    MENU_PAGE_RUN,
    MENU_PAGE_DEBUG,
    MENU_PAGE_INFO,
};

static const char *const run_items[RUN_ITEM_COUNT] =
{
    "Map",
    "Mode",
    "Execute",
};

static const menu_page_id_enum run_item_pages[RUN_ITEM_COUNT] =
{
    MENU_PAGE_RUN_MAP,
    MENU_PAGE_RUN_MODE,
    MENU_PAGE_RUN_EXECUTE,
};

static menu_page_id_enum current_page;
static uint8 cursor_row;
static uint8 previous_cursor_row;
static uint8 page_cursor[ROOT_PAGE_COUNT];
static uint8 current_map;
static uint8 candidate_map;
static run_mode_enum run_mode;
static run_mode_enum candidate_mode;
static solve_result_struct last_result;
static uint32 last_elapsed_ms;
static uint16 playback_step;
static playback_state_enum playback_state;
static uint32 playback_last_ms;
static uint32 dynamic_last_ms;
static uint8 need_redraw;
static uint8 demo_active;
static uint8 demo_index;
static uint8 demo_ok_count;
static uint8 demo_fail_count;
static uint32 demo_last_elapsed_ms;
static const char *run_state = "Idle";

static void draw_current_page(void);
static void draw_home_page(void);
static void draw_run_page(void);
static void draw_run_map_page(void);
static void draw_mode_page(void);
static void draw_playback_page(void);
static void draw_demo_page(void);
static void draw_debug_page(void);
static void draw_info_page(void);
static void refresh_home_page(void);
static void refresh_debug_page(void);
static void handle_home_key(menu_key_event_enum event);
static void handle_run_key(menu_key_event_enum event);
static void handle_debug_key(menu_key_event_enum event);
static void handle_info_key(menu_key_event_enum event);
static void handle_map_event(menu_key_event_enum event);
static void handle_mode_event(menu_key_event_enum event);
static void handle_playback_event(menu_key_event_enum event);
static void handle_demo_event(menu_key_event_enum event);
static void execute_current_selection(void);
static void stop_demo(void);

static const menu_page_def_struct menu_pages[] =
{
    { MENU_PAGE_HOME,         MENU_PAGE_HOME, HOME_DYNAMIC_REFRESH_MS, draw_home_page,     refresh_home_page,  handle_home_key     },
    { MENU_PAGE_RUN,          MENU_PAGE_HOME, 0,   draw_run_page,      0,                  handle_run_key      },
    { MENU_PAGE_RUN_MAP,      MENU_PAGE_RUN,  0,   draw_run_map_page,  0,                  handle_map_event    },
    { MENU_PAGE_RUN_MODE,     MENU_PAGE_RUN,  0,   draw_mode_page,     0,                  handle_mode_event   },
    { MENU_PAGE_RUN_PLAYBACK, MENU_PAGE_RUN,  0,   draw_playback_page, 0,                  handle_playback_event },
    { MENU_PAGE_RUN_DEMO,     MENU_PAGE_RUN,  0,   draw_demo_page,     0,                  handle_demo_event   },
    { MENU_PAGE_DEBUG,        MENU_PAGE_HOME, HOME_DYNAMIC_REFRESH_MS, draw_debug_page,    refresh_debug_page, handle_debug_key    },
    { MENU_PAGE_INFO,         MENU_PAGE_HOME, 0,   draw_info_page,     0,                  handle_info_key     },
};

static void mark_redraw(void)
{
    need_redraw = 1;
}

static const menu_page_def_struct *find_page(menu_page_id_enum page)
{
    uint8 i;

    for(i = 0; i < (uint8)(sizeof(menu_pages) / sizeof(menu_pages[0])); i++)
    {
        if(menu_pages[i].id == page)
        {
            return &menu_pages[i];
        }
    }
    return &menu_pages[0];
}

static uint8 prev_index(uint8 value, uint8 count)
{
    return (0 == value) ? (uint8)(count - 1u) : (uint8)(value - 1u);
}

static uint8 next_index(uint8 value, uint8 count)
{
    return (uint8)((value + 1u) % count);
}

static uint8 page_root(menu_page_id_enum page)
{
    if(page >= 10)
    {
        return (uint8)(page / 10);
    }
    return (uint8)page;
}

static uint8 page_item_count(menu_page_id_enum page)
{
    if(MENU_PAGE_HOME == page)
    {
        return HOME_ITEM_COUNT;
    }
    if(MENU_PAGE_RUN == page)
    {
        return RUN_ITEM_COUNT;
    }
    if(MENU_PAGE_RUN_MODE == page)
    {
        return RUN_MODE_COUNT;
    }
    return 0;
}

static void save_page_cursor(void)
{
    uint8 root = page_root(current_page);

    if(root < ROOT_PAGE_COUNT)
    {
        page_cursor[root] = cursor_row;
    }
}

static void enter_page(menu_page_id_enum page)
{
    uint8 root;

    save_page_cursor();
    current_page = page;
    root = page_root(page);

    if(root < ROOT_PAGE_COUNT)
    {
        cursor_row = page_cursor[root];
    }
    else
    {
        cursor_row = 0;
    }

    if(cursor_row >= page_item_count(page))
    {
        cursor_row = 0;
    }

    if(MENU_PAGE_RUN_MAP == page)
    {
        candidate_map = current_map;
    }
    else if(MENU_PAGE_RUN_MODE == page)
    {
        candidate_mode = run_mode;
    }

    previous_cursor_row = cursor_row;
    mark_redraw();
}

static void go_home(void)
{
    candidate_map = current_map;
    candidate_mode = run_mode;
    stop_demo();
    enter_page(MENU_PAGE_HOME);
}

static void go_parent(void)
{
    const menu_page_def_struct *page = find_page(current_page);

    if(MENU_PAGE_HOME != current_page)
    {
        enter_page(page->parent);
    }
}

static void clear_result_state(void)
{
    clear_result(&last_result);
    last_elapsed_ms = 0;
    playback_step = 0;
    playback_state = PLAYBACK_STATE_PAUSED;
    run_state = "Idle";
}

static void solve_current_map(void)
{
    const map_source_struct *source = map_get(current_map);
    uint32 start_ms;

    run_state = "Running";
    mark_redraw();
    draw_current_page();
    need_redraw = 0;

    start_ms = time_ms();
    if(0 != solve_map(source, &last_result))
    {
        last_elapsed_ms = time_ms() - start_ms;
        playback_state = PLAYBACK_STATE_PAUSED;
        run_state = "OK";
        printf("SOLVE_OK map=%d tasks=%d actions=%d time=%lu\r\n",
            current_map + 1,
            last_result.task_count,
            last_result.action_count,
            (unsigned long)last_elapsed_ms);
    }
    else
    {
        last_elapsed_ms = time_ms() - start_ms;
        playback_state = PLAYBACK_STATE_FAIL;
        run_state = "Fail";
        printf("SOLVE_FAIL map=%d %s time=%lu\r\n",
            current_map + 1,
            last_result.message,
            (unsigned long)last_elapsed_ms);
    }

    playback_step = 0;
    playback_last_ms = time_ms();
}

static void start_demo(void)
{
    demo_active = 1;
    demo_index = 0;
    demo_ok_count = 0;
    demo_fail_count = 0;
    demo_last_elapsed_ms = 0;
    run_state = "Running";
    clear_result(&last_result);
    mark_redraw();
}

static void stop_demo(void)
{
    if(0 != demo_active)
    {
        demo_active = 0;
        run_state = "Stop";
        mark_redraw();
    }
}

static void demo_tick(void)
{
    const map_source_struct *source;
    uint32 start_ms;

    if((MENU_PAGE_RUN_DEMO != current_page) || (0 == demo_active))
    {
        return;
    }

    if(demo_index >= map_count())
    {
        demo_active = 0;
        run_state = "Done";
        mark_redraw();
        return;
    }

    source = map_get(demo_index);
    start_ms = time_ms();
    if(0 != solve_map(source, &last_result))
    {
        demo_ok_count++;
    }
    else
    {
        demo_fail_count++;
    }
    demo_last_elapsed_ms = time_ms() - start_ms;
    demo_index++;

    if(demo_index >= map_count())
    {
        demo_active = 0;
        run_state = "Done";
    }
    mark_redraw();
}

static void move_cursor(int8 delta)
{
    uint8 count = page_item_count(current_page);

    if(0 == count)
    {
        return;
    }

    previous_cursor_row = cursor_row;
    if(delta < 0)
    {
        cursor_row = prev_index(cursor_row, count);
    }
    else
    {
        cursor_row = next_index(cursor_row, count);
    }

    save_page_cursor();
    screen_draw_nav_cursor(previous_cursor_row, cursor_row);
}

static void enter_selected_item(void)
{
    if(MENU_PAGE_HOME == current_page)
    {
        enter_page(home_item_pages[cursor_row]);
        return;
    }

    if(MENU_PAGE_RUN == current_page)
    {
        if(MENU_PAGE_RUN_EXECUTE == run_item_pages[cursor_row])
        {
            execute_current_selection();
        }
        else
        {
            enter_page(run_item_pages[cursor_row]);
        }
    }
}

static void handle_list_event(menu_key_event_enum event)
{
    if(MENU_KEY_EVENT_K1_SHORT == event)
    {
        move_cursor(-1);
    }
    else if(MENU_KEY_EVENT_K2_SHORT == event)
    {
        move_cursor(1);
    }
    else if(MENU_KEY_EVENT_K3_SHORT == event)
    {
        enter_selected_item();
    }
    else if(MENU_KEY_EVENT_K4_SHORT == event)
    {
        if(MENU_PAGE_HOME == current_page)
        {
            settings_save();
            mark_redraw();
        }
        else
        {
            go_parent();
        }
    }
}

static void handle_home_key(menu_key_event_enum event)
{
    handle_list_event(event);
}

static void handle_map_event(menu_key_event_enum event)
{
    if(MENU_KEY_EVENT_K1_SHORT == event)
    {
        candidate_map = prev_index(candidate_map, map_count());
        run_state = (candidate_map == current_map) ? "Idle" : "Pick Map";
        mark_redraw();
    }
    else if(MENU_KEY_EVENT_K2_SHORT == event)
    {
        candidate_map = next_index(candidate_map, map_count());
        run_state = (candidate_map == current_map) ? "Idle" : "Pick Map";
        mark_redraw();
    }
    else if(MENU_KEY_EVENT_K3_SHORT == event)
    {
        current_map = candidate_map;
        settings_set_runtime(current_map, run_mode);
        clear_result_state();
        enter_page(MENU_PAGE_RUN);
    }
    else if(MENU_KEY_EVENT_K4_SHORT == event)
    {
        candidate_map = current_map;
        run_state = "Idle";
        enter_page(MENU_PAGE_RUN);
    }
}

static void select_run_map(uint8 map)
{
    current_map = map;
    candidate_map = current_map;
    settings_set_runtime(current_map, run_mode);
    clear_result_state();
    run_state = "Pick Map";
    mark_redraw();
}

static void handle_run_event(menu_key_event_enum event)
{
    if(MENU_KEY_EVENT_K1_SHORT == event)
    {
        select_run_map(prev_index(current_map, map_count()));
    }
    else if(MENU_KEY_EVENT_K2_SHORT == event)
    {
        select_run_map(next_index(current_map, map_count()));
    }
    else if(MENU_KEY_EVENT_K3_SHORT == event)
    {
        execute_current_selection();
    }
    else if(MENU_KEY_EVENT_K3_LONG == event)
    {
        enter_page(MENU_PAGE_RUN_MODE);
    }
    else if(MENU_KEY_EVENT_K4_SHORT == event)
    {
        go_home();
    }
}

static void handle_run_key(menu_key_event_enum event)
{
    handle_run_event(event);
}

static void handle_mode_event(menu_key_event_enum event)
{
    if(MENU_KEY_EVENT_K1_SHORT == event)
    {
        candidate_mode = (run_mode_enum)prev_index((uint8)candidate_mode, RUN_MODE_COUNT);
        mark_redraw();
    }
    else if(MENU_KEY_EVENT_K2_SHORT == event)
    {
        candidate_mode = (run_mode_enum)next_index((uint8)candidate_mode, RUN_MODE_COUNT);
        mark_redraw();
    }
    else if(MENU_KEY_EVENT_K3_SHORT == event)
    {
        run_mode = candidate_mode;
        settings_set_runtime(current_map, run_mode);
        run_state = "Idle";
        enter_page(MENU_PAGE_RUN);
    }
    else if(MENU_KEY_EVENT_K4_SHORT == event)
    {
        candidate_mode = run_mode;
        enter_page(MENU_PAGE_RUN);
    }
}

static void pause_playback_if_ready(void)
{
    if(PLAYBACK_STATE_FAIL != playback_state)
    {
        playback_state = PLAYBACK_STATE_PAUSED;
    }
}

static void playback_step_prev(void)
{
    if(0 < playback_step)
    {
        playback_step--;
    }
    while((0 < playback_step) && ('|' == last_result.actions[playback_step - 1u]))
    {
        playback_step--;
    }
}

static void playback_step_next(void)
{
    if(playback_step < last_result.action_count)
    {
        playback_step++;
    }
    while((playback_step < last_result.action_count) && ('|' == last_result.actions[playback_step - 1u]))
    {
        playback_step++;
    }
}

static void handle_playback_event(menu_key_event_enum event)
{
    if(MENU_KEY_EVENT_K1_SHORT == event)
    {
        playback_step_prev();
        pause_playback_if_ready();
        mark_redraw();
    }
    else if(MENU_KEY_EVENT_K2_SHORT == event)
    {
        playback_step_next();
        pause_playback_if_ready();
        mark_redraw();
    }
    else if((MENU_KEY_EVENT_K3_SHORT == event) && (PLAYBACK_STATE_FAIL != playback_state))
    {
        if(PLAYBACK_STATE_PLAYING == playback_state)
        {
            playback_state = PLAYBACK_STATE_PAUSED;
        }
        else if(playback_step < last_result.action_count)
        {
            playback_state = PLAYBACK_STATE_PLAYING;
            playback_last_ms = time_ms();
        }
        mark_redraw();
    }
    else if(MENU_KEY_EVENT_K4_SHORT == event)
    {
        enter_page(MENU_PAGE_RUN);
    }
}

static void handle_demo_event(menu_key_event_enum event)
{
    if(MENU_KEY_EVENT_K4_SHORT == event)
    {
        stop_demo();
        enter_page(MENU_PAGE_RUN);
    }
}

static void handle_debug_key(menu_key_event_enum event)
{
    if(MENU_KEY_EVENT_K4_SHORT == event)
    {
        go_home();
    }
}

static void handle_info_key(menu_key_event_enum event)
{
    if(MENU_KEY_EVENT_K4_SHORT == event)
    {
        go_home();
    }
}

static void playback_tick(void)
{
    uint32 now_ms;

    if((MENU_PAGE_RUN_PLAYBACK != current_page) || (PLAYBACK_STATE_PLAYING != playback_state))
    {
        return;
    }

    now_ms = time_ms();
    if((now_ms - playback_last_ms) >= PLAYBACK_STEP_MS)
    {
        playback_last_ms = now_ms;
        playback_step_next();
        if(playback_step >= last_result.action_count)
        {
            playback_state = PLAYBACK_STATE_DONE;
        }
        mark_redraw();
    }
}

static void execute_current_selection(void)
{
    candidate_map = current_map;
    candidate_mode = run_mode;

    if(RUN_MODE_BROWSE == run_mode)
    {
        clear_result_state();
        run_state = "Browse";
        enter_page(MENU_PAGE_RUN);
        return;
    }

    if(RUN_MODE_DEMO == run_mode)
    {
        start_demo();
        enter_page(MENU_PAGE_RUN_DEMO);
        return;
    }

    solve_current_map();
    enter_page(MENU_PAGE_RUN_PLAYBACK);
}

static void dispatch_key_event(menu_key_event_enum event)
{
    const menu_page_def_struct *page;

    if(MENU_KEY_EVENT_NONE == event)
    {
        return;
    }

    if(MENU_KEY_EVENT_K4_LONG == event)
    {
        go_home();
        return;
    }

    page = find_page(current_page);
    if(0 != page->on_key)
    {
        page->on_key(event);
    }
}

static void draw_home_page(void)
{
    const control_status_struct *status = get_control_status();
    const drive_pose_struct *pose = drive_pose_get();

    screen_draw_home(home_items,
        HOME_ITEM_COUNT,
        cursor_row,
        current_map,
        run_mode,
        settings_get_save_state(),
        status->wheel_feedback_count,
        status->current_roll,
        status->current_pitch,
        status->current_yaw,
        status->target_yaw,
        status->yaw_error,
        status->vz,
        status->vzt,
        pose->x_cm,
        pose->y_cm,
        openart_uart_get_frame_count());
}

static void draw_debug_page(void)
{
    const drive_pose_struct *pose = drive_pose_get();

    screen_draw_debug(current_map, map_get(current_map), pose->x_cm, pose->y_cm);
}

static void draw_run_page(void)
{
    screen_draw_run_workbench(current_map, run_mode, settings_get_save_state(), run_state, last_elapsed_ms);
}

static void draw_run_map_page(void)
{
    screen_draw_map_select(current_map, candidate_map, settings_get_save_state(), run_state);
}

static void draw_mode_page(void)
{
    screen_draw_mode_page(candidate_mode);
}

static void draw_info_page(void)
{
    screen_draw_info(map_count(), settings_get_save_state());
}

static void refresh_home_page(void)
{
    const control_status_struct *status = get_control_status();
    const drive_pose_struct *pose = drive_pose_get();

    screen_draw_home_status(status->wheel_feedback_count,
        status->current_roll,
        status->current_pitch,
        status->current_yaw,
        status->target_yaw,
        status->yaw_error,
        status->vz,
        status->vzt,
        pose->x_cm,
        pose->y_cm,
        openart_uart_get_frame_count());
}

static void refresh_debug_page(void)
{
    draw_debug_page();
}

static void refresh_current_page_dynamic(void)
{
    const menu_page_def_struct *page;
    uint32 now_ms;

    page = find_page(current_page);
    if((0 == page->refresh_ms) || (0 == page->refresh_dynamic))
    {
        return;
    }

    now_ms = time_ms();
    if((now_ms - dynamic_last_ms) >= page->refresh_ms)
    {
        dynamic_last_ms = now_ms;
        page->refresh_dynamic();
    }
}

static void draw_playback_page(void)
{
    screen_draw_playback(current_map, map_get(current_map), &last_result, playback_step, last_elapsed_ms, playback_state);
}

static void draw_demo_page(void)
{
    uint8 total = map_count();
    uint8 display_index = (demo_index >= total) ? total : (uint8)(demo_index + 1u);

    screen_draw_demo(display_index, total, demo_ok_count, demo_fail_count, demo_last_elapsed_ms, run_state);
}

static void draw_current_page(void)
{
    const menu_page_def_struct *page = find_page(current_page);

    if(page->id != current_page)
    {
        current_page = MENU_PAGE_HOME;
        page = find_page(current_page);
    }

    if(0 != page->draw_full)
    {
        page->draw_full();
    }
}

void menu_init(void)
{
    uint8 i;

    menu_key_init();
    pit_ms_init(PIT_CH2, MENU_KEY_SCAN_PERIOD_MS);

    current_map = settings_get_map();
    candidate_map = current_map;
    run_mode = settings_get_mode();
    candidate_mode = run_mode;
    current_page = MENU_PAGE_HOME;
    cursor_row = 0;
    previous_cursor_row = 0;
    for(i = 0; i < ROOT_PAGE_COUNT; i++)
    {
        page_cursor[i] = 0;
    }

    last_elapsed_ms = 0;
    playback_step = 0;
    playback_state = PLAYBACK_STATE_PAUSED;
    playback_last_ms = 0;
    dynamic_last_ms = time_ms();
    demo_active = 0;
    demo_index = 0;
    demo_ok_count = 0;
    demo_fail_count = 0;
    demo_last_elapsed_ms = 0;
    run_state = "Idle";
    clear_result(&last_result);
    mark_redraw();
    draw_current_page();
    need_redraw = 0;
}

void menu_poll(void)
{
    menu_key_event_enum event = menu_key_read_event();

    if(MENU_KEY_EVENT_NONE != event)
    {
        dispatch_key_event(event);
    }

    demo_tick();
    playback_tick();

    if(0 != need_redraw)
    {
        draw_current_page();
        dynamic_last_ms = time_ms();
        need_redraw = 0;
    }

    refresh_current_page_dynamic();
}
