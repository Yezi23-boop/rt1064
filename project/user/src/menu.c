#include "zf_common_headfile.h"
#include "menu.h"
#include "maps.h"
#include "solver.h"
#include "settings.h"
#include "screen.h"
#include "timebase.h"

#define KEY_SCAN_PERIOD_MS      (5)     // 按键扫描间隔，单位 ms；与逐飞 key_scanner 消抖节拍保持一致。
#define PLAYBACK_STEP_MS        (300u)  // 回放自动步进间隔，单位 ms；只影响屏幕演示速度。
#define HOME_ITEM_COUNT         (3)     // Home 一级菜单数量，必须与 home_item_names/home_item_pages 同步。

typedef enum
{
    MENU_PAGE_HOME = 0,
    MENU_PAGE_RUN,
    MENU_PAGE_MODE_SELECT,
    MENU_PAGE_PLAYBACK,
    MENU_PAGE_DEBUG,
    MENU_PAGE_INFO,
} menu_page_enum;

typedef enum
{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_K1_SHORT,
    KEY_EVENT_K2_SHORT,
    KEY_EVENT_K3_SHORT,
    KEY_EVENT_K4_SHORT,
    KEY_EVENT_K1_LONG,
    KEY_EVENT_K2_LONG,
    KEY_EVENT_K3_LONG,
    KEY_EVENT_K4_LONG,
} key_event_enum;

static const char *const home_item_names[HOME_ITEM_COUNT] =
{
    "Run",
    "Debug",
    "Info",
};

static const menu_page_enum home_item_pages[HOME_ITEM_COUNT] =
{
    MENU_PAGE_RUN,
    MENU_PAGE_DEBUG,
    MENU_PAGE_INFO,
};

static menu_page_enum current_page;     // 菜单主状态，只在主循环 menu_poll() 中写入。
static uint8 home_cursor;
static uint8 current_map;               // 已确认地图编号；Flash 保存的也是这个值。
static uint8 candidate_map;             // Run 页面候选地图，按 K3 后才写入 current_map。
static run_mode_enum run_mode;          // 已确认运行模式；切换模式后只标记 Dirty，不立即写 Flash。
static run_mode_enum candidate_mode;    // Mode Select 页面候选模式，按 K3 后才确认。
static solve_result_struct last_result;
static uint32 last_elapsed_ms;          // 最近一次 solve_map 耗时，单位 ms。
static uint16 playback_step;            // 当前回放动作下标，包含内部任务分隔符的原始步数。
static playback_state_enum playback_state;
static uint32 playback_last_ms;         // 上一次自动回放步进时间，单位 ms。
static uint8 need_redraw;               // 缓存刷新标志，避免每次轮询都重画整页。
static uint8 key_long_used[KEY_NUMBER]; // 长按已消费标志，避免长按期间反复触发事件。
static uint32 key_last_scan_ms;         // 上一次按键扫描时间，单位 ms。
static uint8 demo_active;
static uint8 demo_visible;
static uint8 demo_index;
static uint8 demo_ok_count;
static uint8 demo_fail_count;
static uint32 demo_last_elapsed_ms;     // Demo 最近一张地图求解耗时，单位 ms。
static const char *run_state = "Idle";

static void draw_current_page(void);

static void mark_redraw(void)
{
    need_redraw = 1;
}

static uint8 prev_index(uint8 value, uint8 count)
{
    return (0 == value) ? (uint8)(count - 1u) : (uint8)(value - 1u);
}

static uint8 next_index(uint8 value, uint8 count)
{
    return (uint8)((value + 1u) % count);
}

static key_event_enum make_short_event(key_index_enum key)
{
    switch(key)
    {
        case KEY_1: return KEY_EVENT_K1_SHORT;
        case KEY_2: return KEY_EVENT_K2_SHORT;
        case KEY_3: return KEY_EVENT_K3_SHORT;
        case KEY_4: return KEY_EVENT_K4_SHORT;
        default:    return KEY_EVENT_NONE;
    }
}

static key_event_enum make_long_event(key_index_enum key)
{
    switch(key)
    {
        case KEY_1: return KEY_EVENT_K1_LONG;
        case KEY_2: return KEY_EVENT_K2_LONG;
        case KEY_3: return KEY_EVENT_K3_LONG;
        case KEY_4: return KEY_EVENT_K4_LONG;
        default:    return KEY_EVENT_NONE;
    }
}

static key_event_enum key_read_event(void)
{
    key_index_enum key;
    key_state_enum state;
    uint32 now_ms = time_ms();

    if((now_ms - key_last_scan_ms) < KEY_SCAN_PERIOD_MS)
    {
        return KEY_EVENT_NONE;
    }
    key_last_scan_ms = now_ms;
    key_scanner();

    // 先处理长按，避免同一次按住 K4 同时触发短按保存和长按返回 Home。
    for(key = KEY_1; key < KEY_NUMBER; key++)
    {
        state = key_get_state(key);
        if(KEY_RELEASE == state)
        {
            key_long_used[key] = 0;
        }
        else if((KEY_LONG_PRESS == state) && (0 == key_long_used[key]))
        {
            key_long_used[key] = 1;
            key_clear_state(key);
            return make_long_event(key);
        }
    }

    for(key = KEY_1; key < KEY_NUMBER; key++)
    {
        state = key_get_state(key);
        if(KEY_SHORT_PRESS == state)
        {
            key_clear_state(key);
            if(0 == key_long_used[key])
            {
                return make_short_event(key);
            }
        }
    }

    return KEY_EVENT_NONE;
}

static void go_home(void)
{
    candidate_map = current_map;
    candidate_mode = run_mode;
    demo_active = 0;
    demo_visible = 0;
    current_page = MENU_PAGE_HOME;
    mark_redraw();
}

static void enter_run(void)
{
    candidate_map = current_map;
    current_page = MENU_PAGE_RUN;
    mark_redraw();
}

static void solve_current_map(void)
{
    const map_source_struct *source = map_get(current_map);
    uint32 start_ms;

    run_state = "Running";
    mark_redraw();
    draw_current_page();                // 先显示 Running，避免 BFS 耗时时屏幕仍停在旧状态。
    start_ms = time_ms();
    if(0 != solve_map(source, &last_result))
    {
        last_elapsed_ms = time_ms() - start_ms;
        playback_step = 0;
        playback_state = PLAYBACK_STATE_PAUSED;
        playback_last_ms = time_ms();
        current_page = MENU_PAGE_PLAYBACK;
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
        playback_step = 0;
        playback_state = PLAYBACK_STATE_FAIL;
        current_page = MENU_PAGE_PLAYBACK;
        run_state = "Fail";
        printf("SOLVE_FAIL map=%d %s time=%lu\r\n",
            current_map + 1,
            last_result.message,
            (unsigned long)last_elapsed_ms);
    }
    mark_redraw();
}

static void start_demo(void)
{
    demo_active = 1;
    demo_visible = 1;
    demo_index = 0;
    demo_ok_count = 0;
    demo_fail_count = 0;
    demo_last_elapsed_ms = 0;
    run_state = "Running";
    clear_result(&last_result);
    mark_redraw();
    draw_current_page();                // Demo 会连续求解多张图，先刷新页面提示当前进入批量验证。
}

static void demo_tick(void)
{
    const map_source_struct *source;
    uint32 start_ms;

    if(0 == demo_active)
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

static void handle_home(key_event_enum event)
{
    if(KEY_EVENT_K1_SHORT == event)
    {
        home_cursor = prev_index(home_cursor, HOME_ITEM_COUNT);
        mark_redraw();
    }
    else if(KEY_EVENT_K2_SHORT == event)
    {
        home_cursor = next_index(home_cursor, HOME_ITEM_COUNT);
        mark_redraw();
    }
    else if(KEY_EVENT_K3_SHORT == event)
    {
        current_page = home_item_pages[home_cursor];
        if(MENU_PAGE_RUN == current_page)
        {
            candidate_map = current_map;
        }
        mark_redraw();
    }
    else if(KEY_EVENT_K4_SHORT == event)
    {
        settings_save();
        mark_redraw();
    }
}

static void confirm_candidate_map(void)
{
    current_map = candidate_map;
    settings_set_runtime(current_map, run_mode);
    clear_result(&last_result);
    last_elapsed_ms = 0;
    run_state = "Idle";
    mark_redraw();
}

static void browse_current_map(void)
{
    clear_result(&last_result);
    last_elapsed_ms = 0;
    run_state = "Browse";
    mark_redraw();
}

static void confirm_run_action(void)
{
    if(candidate_map != current_map)
    {
        confirm_candidate_map();
        return;
    }

    if(RUN_MODE_BROWSE == run_mode)
    {
        browse_current_map();
        return;
    }

    if(RUN_MODE_DEMO == run_mode)
    {
        start_demo();
        return;
    }

    solve_current_map();
}

static void handle_run(key_event_enum event)
{
    if(0 != demo_visible)
    {
        if(KEY_EVENT_K4_SHORT == event)
        {
            demo_active = 0;
            demo_visible = 0;
            run_state = "Stop";
            mark_redraw();
        }
        return;
    }

    if(KEY_EVENT_K1_SHORT == event)
    {
        candidate_map = prev_index(candidate_map, map_count());
        run_state = (candidate_map == current_map) ? "Idle" : "Pick Map";
        mark_redraw();
    }
    else if(KEY_EVENT_K2_SHORT == event)
    {
        candidate_map = next_index(candidate_map, map_count());
        run_state = (candidate_map == current_map) ? "Idle" : "Pick Map";
        mark_redraw();
    }
    else if(KEY_EVENT_K2_LONG == event)
    {
        candidate_mode = run_mode;
        current_page = MENU_PAGE_MODE_SELECT;
        mark_redraw();
    }
    else if(KEY_EVENT_K3_SHORT == event)
    {
        confirm_run_action();
    }
    else if(KEY_EVENT_K4_SHORT == event)
    {
        go_home();
    }
}

static void handle_mode_select(key_event_enum event)
{
    if(KEY_EVENT_K1_SHORT == event)
    {
        candidate_mode = (run_mode_enum)prev_index((uint8)candidate_mode, RUN_MODE_COUNT);
        mark_redraw();
    }
    else if(KEY_EVENT_K2_SHORT == event)
    {
        candidate_mode = (run_mode_enum)next_index((uint8)candidate_mode, RUN_MODE_COUNT);
        mark_redraw();
    }
    else if(KEY_EVENT_K3_SHORT == event)
    {
        run_mode = candidate_mode;
        settings_set_runtime(current_map, run_mode);
        current_page = MENU_PAGE_RUN;
        run_state = "Idle";
        mark_redraw();
    }
    else if(KEY_EVENT_K4_SHORT == event)
    {
        candidate_mode = run_mode;
        current_page = MENU_PAGE_RUN;
        mark_redraw();
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
        playback_step--;                // `|` 只是多箱任务分隔符，屏幕回放不把它显示为一步。
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
        playback_step++;                // 跳过任务分隔符，保持 K2/自动播放只推进真实动作。
    }
}

static void handle_playback(key_event_enum event)
{
    if(KEY_EVENT_K1_SHORT == event)
    {
        playback_step_prev();
        pause_playback_if_ready();
        mark_redraw();
    }
    else if(KEY_EVENT_K2_SHORT == event)
    {
        playback_step_next();
        pause_playback_if_ready();
        mark_redraw();
    }
    else if((KEY_EVENT_K3_SHORT == event) && (PLAYBACK_STATE_FAIL != playback_state))
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
    else if(KEY_EVENT_K4_SHORT == event)
    {
        enter_run();
    }
}

static void playback_tick(void)
{
    uint32 now_ms;

    if((MENU_PAGE_PLAYBACK != current_page) || (PLAYBACK_STATE_PLAYING != playback_state))
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

static void handle_simple_back(key_event_enum event)
{
    if(KEY_EVENT_K4_SHORT == event)
    {
        go_home();
    }
}

static void dispatch_key_event(key_event_enum event)
{
    if(KEY_EVENT_K4_LONG == event)
    {
        go_home();
        return;
    }

    switch(current_page)
    {
        case MENU_PAGE_HOME:        handle_home(event); break;
        case MENU_PAGE_RUN:         handle_run(event); break;
        case MENU_PAGE_MODE_SELECT: handle_mode_select(event); break;
        case MENU_PAGE_PLAYBACK:    handle_playback(event); break;
        case MENU_PAGE_DEBUG:
        case MENU_PAGE_INFO:        handle_simple_back(event); break;
        default:                    go_home(); break;
    }
}

static void draw_current_page(void)
{
    if(0 == need_redraw)
    {
        return;
    }

    switch(current_page)
    {
        case MENU_PAGE_HOME:
            screen_draw_home(home_item_names, HOME_ITEM_COUNT, home_cursor, current_map, run_mode, settings_get_save_state());
            break;

        case MENU_PAGE_RUN:
            if(0 != demo_visible)
            {
                uint8 display_index = (demo_index >= map_count()) ? map_count() : (uint8)(demo_index + 1u);
                screen_draw_demo(display_index, map_count(), demo_ok_count, demo_fail_count, demo_last_elapsed_ms, run_state);
            }
            else
            {
                screen_draw_run(current_map, candidate_map, run_mode, settings_get_save_state(), run_state, last_elapsed_ms);
            }
            break;

        case MENU_PAGE_MODE_SELECT:
            screen_draw_mode_select(candidate_mode);
            break;

        case MENU_PAGE_PLAYBACK:
            screen_draw_playback(current_map, map_get(current_map), &last_result, playback_step, last_elapsed_ms, playback_state);
            break;

        case MENU_PAGE_DEBUG:
            screen_draw_debug();
            break;

        case MENU_PAGE_INFO:
            screen_draw_info(map_count(), settings_get_save_state());
            break;

        default:
            current_page = MENU_PAGE_HOME;
            screen_draw_home(home_item_names, HOME_ITEM_COUNT, home_cursor, current_map, run_mode, settings_get_save_state());
            break;
    }
    need_redraw = 0;
}

void menu_init(void)
{
    key_init(KEY_SCAN_PERIOD_MS);
    current_map = settings_get_map();
    candidate_map = current_map;
    run_mode = settings_get_mode();
    candidate_mode = run_mode;
    current_page = MENU_PAGE_HOME;
    home_cursor = 0;
    last_elapsed_ms = 0;
    playback_step = 0;
    playback_state = PLAYBACK_STATE_PAUSED;
    key_last_scan_ms = time_ms();
    demo_active = 0;
    demo_visible = 0;
    run_state = "Idle";
    clear_result(&last_result);
    mark_redraw();
    draw_current_page();
}

void menu_poll(void)
{
    key_event_enum event = key_read_event();

    dispatch_key_event(event);

    demo_tick();
    playback_tick();
    draw_current_page();
}
