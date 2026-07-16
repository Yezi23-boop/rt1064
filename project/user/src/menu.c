#include "zf_common_headfile.h"
#include "menu.h"
#include "menu_key.h"
#include "maps.h"
#include "solver.h"
#include "map_utils.h"
#include "settings.h"
#include "screen.h"
#include "timebase.h"
#include "drive_control.h"
#include "drive_pose.h"
#include "openart_uart.h"
#include "executor.h"
#include "art_replan.h"
#include "competition_flow.h"
#include "drive_config.h"
#include "subject2.h"
#include "vision_uart.h"

#define MENU_KEY_SCAN_PERIOD_MS (5)
#define PLAYBACK_STEP_MS        (300u)
#define HOME_DYNAMIC_REFRESH_MS (100u)
#define HOME_ITEM_COUNT         (4)
#define ROOT_PAGE_COUNT         (5)

typedef enum
{
    MENU_PAGE_HOME         = 0,  /**< 顶层菜单页。 */
    MENU_PAGE_RUN          = 1,  /**< 运行工作台页，选择地图来源并发起求解/执行。 */
    MENU_PAGE_ART_MAP      = 2,  /**< OpenART 最近完整帧预览页。 */
    MENU_PAGE_DEBUG        = 3,  /**< 本地里程计映射到地图的调试页。 */
    MENU_PAGE_INFO         = 4,  /**< 固件状态信息页。 */
    MENU_PAGE_RUN_MODE     = 12, /**< Run 子页：候选运行模式选择。 */
    MENU_PAGE_RUN_EXECUTE  = 13, /**< Run 子页：executor 执行态显示。 */
    MENU_PAGE_RUN_PLAYBACK = 14, /**< Run 子页：BFS 动作回放。 */
} menu_page_id_enum;

typedef void (*menu_page_lifecycle_func)(void);
typedef void (*menu_page_key_func)(menu_key_event_enum event);

typedef struct
{
    menu_page_id_enum id;                /**< 页面唯一 ID，供 `find_page()` 查表。 */
    menu_page_id_enum parent;            /**< K4 短按返回目标；顶层页的 parent 指向自身。 */
    uint16 refresh_ms;                   /**< 动态局部刷新周期，单位 ms；0 表示只在事件触发时重绘。 */
    menu_page_lifecycle_func on_enter;   /**< 进入页面时运行的钩子，可为 NULL。 */
    menu_page_lifecycle_func on_exit;    /**< 离开页面时运行的钩子，可为 NULL。 */
    menu_page_lifecycle_func on_draw;    /**< 整页绘制函数，不应阻塞等待按键或 ART 帧。 */
    menu_page_lifecycle_func on_refresh; /**< 动态局部刷新函数，可为 NULL。 */
    menu_page_key_func on_key;           /**< 页面私有按键处理函数，可为 NULL。 */
} menu_page_def_struct;

static const char *const home_items[HOME_ITEM_COUNT] =
{
    "Run",
    "ART Map",
    "Debug",
    "Info",
};

static const menu_page_id_enum home_item_pages[HOME_ITEM_COUNT] =
{
    MENU_PAGE_RUN,
    MENU_PAGE_ART_MAP,
    MENU_PAGE_DEBUG,
    MENU_PAGE_INFO,
};

static menu_page_id_enum current_page;                       // 主循环唯一写入的当前页；按键/刷新都以它查表分发。
static uint8 cursor_row;                                     // 当前页面光标行，Home 页用于一级菜单。
static uint8 previous_cursor_row;                            // 上一次光标行，用于局部擦除旧光标。
static uint8 page_cursor[ROOT_PAGE_COUNT];                   // 顶层页光标记忆，返回 Home 后不丢用户上次位置。
static uint8 current_map;                                    // 当前离线地图索引，Flash 保存的是这个索引而不是地图内容。
static run_mode_enum run_mode;                               // 已确认运行模式；Run 页 K3 执行时读取。
static run_mode_enum candidate_mode;                         // Mode 子页临时选择，K3 确认前不写入 `run_mode`。
static solve_result_struct last_result;                      // 屏幕回放和 executor 共用的最近一次求解结果。
static uint32 last_elapsed_ms;                               // 最近一次 solve_map 耗时，单位 ms，仅用于显示。
static uint16 playback_step;                                 // 当前回放 action 下标；播放 `|` 时会跳过可见动作。
static playback_state_enum playback_state;                   // 回放页状态，和 executor 状态相互独立。
static uint32 playback_last_ms;                              // 自动回放上一帧推进时间，单位 ms。
static uint32 dynamic_last_ms;                               // 当前动态页上一轮局部刷新时间，单位 ms。
static uint8 need_redraw;                                    // 页面级重绘请求，主循环消费后清零。
static const char *run_state = "Idle";                       // Run 页短状态文本，指向常量字符串。
static uint8 exec_start_row = 0;                             // executor 启动时的格点行，屏幕用来把 pose 映射回地图。
static uint8 exec_start_col = 0;                             // executor 启动时的格点列，ART 重解算成功后会更新。
static float competition_launch_yaw_deg = 0.0f;              // 当前科目发车校准后的 IMU 目标航向，供科目二 HYaw 回正。
static char last_solve_rows[MAP_ROWS][MAP_COLS + 1];         // 最近求解地图快照的行缓存，避免 OpenART 实时帧覆盖执行底图。
static map_source_struct last_solve_source =
{
    "Run",
    {
        last_solve_rows[0],
        last_solve_rows[1],
        last_solve_rows[2],
        last_solve_rows[3],
        last_solve_rows[4],
        last_solve_rows[5],
        last_solve_rows[6],
        last_solve_rows[7],
        last_solve_rows[8],
        last_solve_rows[9],
        last_solve_rows[10],
        last_solve_rows[11],
    },
};
static uint8 last_solve_source_valid = 0;                    // 1 表示 `last_solve_source` 可用于 Run/Execute/Playback 显示。
static uint8 subject2_active = 0;                            // 1 表示科目二接管 executor 和 ART 段末事件。

static void draw_current_page(void);
static void enter_run_mode_page(void);
static void draw_home_page(void);
static void draw_run_page(void);
static void draw_mode_page(void);
static void draw_playback_page(void);
static void draw_debug_page(void);
static void draw_info_page(void);
static void draw_art_map_page(void);
static void refresh_home_page(void);
static void refresh_debug_page(void);
static void refresh_art_map_page(void);
static void handle_home_key(menu_key_event_enum event);
static void handle_run_key(menu_key_event_enum event);
static void handle_debug_key(menu_key_event_enum event);
static void handle_info_key(menu_key_event_enum event);
static void handle_art_map_key(menu_key_event_enum event);
static void handle_mode_event(menu_key_event_enum event);
static void handle_playback_event(menu_key_event_enum event);
static void draw_execute_page(void);
static void refresh_execute_page(void);
static void handle_execute_event(menu_key_event_enum event);
static void execute_current_selection(void);
static void build_art_replan_context(art_replan_context_struct *context);
static void apply_art_replan_update(const art_replan_update_struct *update);

static const menu_page_def_struct menu_pages[] =
{
    // 页面表把“如何画、如何刷新、如何处理按键”集中声明，新增页面时不需要改主轮询。
    // refresh_ms 为 0 表示静态页；动态页只局部刷新，减少 IPS200 整屏闪烁。
    { MENU_PAGE_HOME,         MENU_PAGE_HOME, HOME_DYNAMIC_REFRESH_MS, 0,                   0, draw_home_page,     refresh_home_page,    handle_home_key      },
    { MENU_PAGE_RUN,          MENU_PAGE_HOME, 0,                       0,                   0, draw_run_page,      0,                    handle_run_key       },
    { MENU_PAGE_RUN_MODE,     MENU_PAGE_RUN,  0,                       enter_run_mode_page, 0, draw_mode_page,     0,                    handle_mode_event    },
    { MENU_PAGE_RUN_PLAYBACK, MENU_PAGE_RUN,  0,                       0,                   0, draw_playback_page, 0,                    handle_playback_event },
    { MENU_PAGE_RUN_EXECUTE,  MENU_PAGE_RUN,  500,                     0,                   0, draw_execute_page,  refresh_execute_page, handle_execute_event  },
    { MENU_PAGE_DEBUG,        MENU_PAGE_HOME, 500,                     0,                   0, draw_debug_page,    refresh_debug_page,   handle_debug_key     },
    { MENU_PAGE_INFO,         MENU_PAGE_HOME, 0,                       0,                   0, draw_info_page,     0,                    handle_info_key      },
    { MENU_PAGE_ART_MAP,      MENU_PAGE_HOME, 500,                     0,                   0, draw_art_map_page,  refresh_art_map_page, handle_art_map_key   },
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
    const menu_page_def_struct *old_page;
    const menu_page_def_struct *new_page;
    uint8 root;

    old_page = find_page(current_page);
    if(0 != old_page->on_exit)
    {
        old_page->on_exit();
    }

    save_page_cursor();
    current_page = page;
    new_page = find_page(current_page);
    root = page_root(current_page);

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

    if(0 != new_page->on_enter)
    {
        new_page->on_enter();
    }

    previous_cursor_row = cursor_row;
    mark_redraw();
}

static void enter_run_mode_page(void)
{
    candidate_mode = run_mode;
}

static void go_home(void)
{
    candidate_mode = run_mode;
    enter_page(MENU_PAGE_HOME);
}

static void safe_go_home(void)
{
    // K4 长按是全局安全出口：无论当前页面在哪，都取消 ART 等待并停止底盘执行。
    // 这条路径不能写 Flash，避免紧急退出时被慢速擦写拖住。
    art_replan_cancel();
    subject2_cancel();
    vision_uart_cancel();
    competition_flow_cancel();
    subject2_active = 0u;
    if(EXEC_STATE_IDLE != executor_get_state())
    {
        executor_stop();
    }
    run_state = "Idle";
    candidate_mode = run_mode;
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
    art_replan_cancel();
    subject2_cancel();
    vision_uart_cancel();
    competition_flow_cancel();
    subject2_active = 0u;
    if(EXEC_STATE_IDLE != executor_get_state())
    {
        executor_stop();
    }
    clear_result(&last_result);
    last_elapsed_ms = 0;
    playback_step = 0;
    playback_state = PLAYBACK_STATE_PAUSED;
    last_solve_source_valid = 0;
    run_state = "Idle";
}

static const map_source_struct *selected_map_source(void)
{
    if(MAP_SOURCE_ART == settings_get_source())
    {
        return openart_map_get();
    }
    return map_get(current_map);
}

static void save_solve_source_snapshot(const map_source_struct *source)
{
    // 求解和回放必须使用同一份地图；ART 来源会持续更新，所以求解前先冻结快照。
    map_source_snapshot(&last_solve_source, last_solve_rows, source);
    last_solve_source_valid = 1;
}

static const map_source_struct *last_or_selected_map_source(void)
{
    if(0 != last_solve_source_valid)
    {
        return &last_solve_source;
    }
    return selected_map_source();
}

static void solve_current_map(void)
{
    const map_source_struct *source;
    uint32 start_ms;

    source = selected_map_source();

    if(0 == source)
    {
        clear_result(&last_result);
        last_elapsed_ms = 0;
        playback_step = 0;
        playback_state = PLAYBACK_STATE_FAIL;
        last_solve_source_valid = 0;
        run_state = "No Map";
        mark_redraw();
        return;
    }
    save_solve_source_snapshot(source);
    source = &last_solve_source;

    run_state = "Running";
    mark_redraw();
    // BFS 可能让主循环短暂停住，先把 Running 状态画出来，避免用户误判按键无响应。
    draw_current_page();
    need_redraw = 0;

    start_ms = time_ms();
    if(0 != solve_map(source, &last_result))
    {
        last_elapsed_ms = time_ms() - start_ms;
        playback_state = PLAYBACK_STATE_PAUSED;
        run_state = "OK";
    }
    else
    {
        last_elapsed_ms = time_ms() - start_ms;
        playback_state = PLAYBACK_STATE_FAIL;
        run_state = "Fail";
    }

    playback_step = 0;
    playback_last_ms = time_ms();
}

static void build_art_replan_context(art_replan_context_struct *context)
{
    context->result = &last_result;
    context->snapshot = &last_solve_source;
    context->snapshot_rows = last_solve_rows;
    context->snapshot_valid = &last_solve_source_valid;
    context->elapsed_ms = &last_elapsed_ms;
    context->start_row = &exec_start_row;
    context->start_col = &exec_start_col;
    context->run_mode = run_mode;
}

static void build_subject2_context(subject2_context_struct *context)
{
    context->result = &last_result;
    context->snapshot = &last_solve_source;
    context->snapshot_rows = last_solve_rows;
    context->snapshot_valid = &last_solve_source_valid;
    context->elapsed_ms = &last_elapsed_ms;
    context->start_row = &exec_start_row;
    context->start_col = &exec_start_col;
    context->run_mode = run_mode;
    context->launch_yaw_deg = competition_launch_yaw_deg;
    context->art_yaw_bias_valid =
        art_replan_get_launch_yaw_bias(&context->art_yaw_bias_deg);
}

static void apply_subject2_update(const subject2_update_struct *update)
{
    if(0 == update)
    {
        return;
    }
    if(0 != update->run_state)
    {
        run_state = update->run_state;
    }
    if(0 != update->enter_execute)
    {
        enter_page(MENU_PAGE_RUN_EXECUTE);
    }
    else if(0 != update->redraw)
    {
        mark_redraw();
    }
}

static void start_competition_action(competition_action_enum action,
                                     art_replan_update_struct *update)
{
    subject2_active = 0u;
    if(COMPETITION_ACTION_START_SUBJECT1 == action)
    {
        art_replan_begin_initial(update);
    }
    else if(COMPETITION_ACTION_START_SUBJECT2 == action)
    {
        art_replan_begin_subject2(update);
    }
    else
    {
        memset(update, 0, sizeof(*update));
        if(COMPETITION_ACTION_FINISH == action)
        {
            update->playback = ART_REPLAN_PLAYBACK_DONE;
            update->run_state = "Done";
            update->redraw = 1u;
        }
    }
}

static void apply_art_replan_update(const art_replan_update_struct *update)
{
    if(0 == update)
    {
        return;
    }

    if(0 != update->run_state)
    {
        run_state = update->run_state;
    }

    if(0 != update->reset_playback_step)
    {
        playback_step = 0;
        playback_last_ms = time_ms();
    }

    if(ART_REPLAN_PLAYBACK_PAUSED == update->playback)
    {
        playback_state = PLAYBACK_STATE_PAUSED;
    }
    else if(ART_REPLAN_PLAYBACK_DONE == update->playback)
    {
        playback_state = PLAYBACK_STATE_DONE;
    }
    else if(ART_REPLAN_PLAYBACK_FAIL == update->playback)
    {
        playback_state = PLAYBACK_STATE_FAIL;
    }

    if(0 != update->enter_execute)
    {
        enter_page(MENU_PAGE_RUN_EXECUTE);
    }
    else if(0 != update->redraw)
    {
        mark_redraw();
    }
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
            // Flash 保存只在 Home 页 K4 短按触发，避免每次切换地图/模式都擦写。
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

static void select_run_map(uint8 map)
{
    current_map = map;
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
        if(EXEC_STATE_PAUSED == executor_get_state())
        {
            executor_resume();
            run_state = "Running";
        }
        else if(EXEC_STATE_IDLE == executor_get_state())
        {
            execute_current_selection();
        }
    }
    else if(MENU_KEY_EVENT_K3_LONG == event)
    {
        enter_page(MENU_PAGE_RUN_MODE);
    }
    else if(MENU_KEY_EVENT_K4_SHORT == event)
    {
        if(EXEC_STATE_ERROR == executor_get_state() ||
           EXEC_STATE_DONE == executor_get_state())
        {
            executor_stop();
            run_state = "Idle";
        }
        else
        {
            map_source_enum src = settings_get_source();
            settings_set_source((MAP_SOURCE_OFFLINE == src) ? MAP_SOURCE_ART : MAP_SOURCE_OFFLINE);
            mark_redraw();
        }
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
    art_replan_update_struct update;
    competition_action_enum action;

#if VISION_UART_BOARD_TEST_ENABLE
    run_state = "V4 Test";
    mark_redraw();
    return;
#endif

    candidate_mode = run_mode;
    art_replan_cancel();
    subject2_cancel();
    vision_uart_cancel();
    competition_flow_cancel();
    subject2_active = 0u;

    if((MAP_SOURCE_ART == settings_get_source()) && (RUN_MODE_SOLVE != run_mode))
    {
        clear_result(&last_result);
        last_elapsed_ms = 0;
        playback_step = 0;
        playback_state = PLAYBACK_STATE_PAUSED;
        last_solve_source_valid = 0;
        executor_stop();
        competition_launch_yaw_deg = get_control_status()->current_yaw;
        art_replan_reset_competition_yaw();
        competition_flow_start(COMPETITION_MODE);
        action = competition_flow_take_action();
        start_competition_action(action, &update);
        apply_art_replan_update(&update);
        enter_page(MENU_PAGE_RUN_EXECUTE);
        return;
    }

    solve_current_map();

    if(last_result.solved)
    {
        if(RUN_MODE_SOLVE == run_mode)
        {
            enter_page(MENU_PAGE_RUN_PLAYBACK);
        }
        else
        {
            uint8 single_step = (RUN_MODE_STEP == run_mode) ? 1u : 0u;

            if(0 != last_solve_source_valid)
            {
                if(0 == map_find_car(&last_solve_source, &exec_start_row, &exec_start_col, 0))
                {
                    return;
                }
            }

            executor_start(last_result.waypoints, last_result.waypoint_count,
                           exec_start_row, exec_start_col,
                           0.0f, 0.0f, single_step,
                           0u);
            enter_page(MENU_PAGE_RUN_EXECUTE);
        }
    }
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
        // K4 长按不交给具体页面，保证任何页面下都能走同一条安全退出路径。
        safe_go_home();
        return;
    }

    page = find_page(current_page);
    if(0 != page->on_key)
    {
        page->on_key(event);
    }
}

static void build_home_view(screen_home_view_struct *view)
{
    const control_status_struct *status = get_control_status();
    const drive_pose_struct *pose = drive_pose_get();
    vision_uart_board_test_status_struct vision_test;

    view->items = home_items;
    view->item_count = HOME_ITEM_COUNT;
    view->cursor = cursor_row;
    view->current_map = current_map;
    view->mode = run_mode;
    view->save_state = settings_get_save_state();
    view->encoder_count = status->wheel_feedback_count;
    view->imu_yaw = status->current_yaw;
    view->target_yaw = status->target_yaw;
    view->yaw_error = status->yaw_error;
    view->vz = status->vz;
    view->vzt = status->vzt;
    view->pose_x_cm = pose->x_cm;
    view->pose_y_cm = pose->y_cm;
    view->openart_frame_count = openart_uart_get_frame_count();
    vision_uart_board_test_get_status(&vision_test);
    view->vision_test_enabled =
        (VISION_UART_BOARD_TEST_DISABLED != vision_test.state) ? 1u : 0u;
    view->vision_test_state = vision_test.state;
    view->vision_test_samples = vision_test.sample_count;
    view->vision_test_class = vision_test.class_id;
    view->vision_test_confidence_q = vision_test.confidence_q;
}

static void build_run_view(screen_run_view_struct *view)
{
    const drive_pose_struct *pose = drive_pose_get();

    view->current_map = current_map;
    view->mode = run_mode;
    view->source_type = settings_get_source();
    view->save_state = settings_get_save_state();
    view->state_text = run_state;
    view->elapsed_ms = last_elapsed_ms;
    view->source = selected_map_source();
    view->result = &last_result;
    view->executor_active = (EXEC_STATE_IDLE != executor_get_state()) ? 1u : 0u;
    view->executor_state = executor_get_state();
    view->executor_error = executor_get_error();
    view->current_step = executor_get_current_step();
    view->total_steps = executor_get_total_steps();
    view->pose_x_cm = pose->x_cm;
    view->pose_y_cm = pose->y_cm;
}

static void build_execute_view(screen_execute_view_struct *view)
{
    const drive_pose_struct *pose = drive_pose_get();
    uint8 art_row = 0;
    uint8 art_col = 0;
    uint8 art_count = 0;

    view->current_map = current_map;
    view->source = last_or_selected_map_source();
    view->result = &last_result;
    view->state_text = run_state;
    view->current_step = executor_get_current_step();
    view->state = executor_get_state();
    view->error = executor_get_error();
    view->start_row = exec_start_row;
    view->start_col = exec_start_col;
    view->pose_x_cm = pose->x_cm;
    view->pose_y_cm = pose->y_cm;
    view->recognition_valid =
        subject2_get_last_recognition(&view->recognition_is_target,
                                      &view->recognition_class);
    view->art_player_enabled = (MAP_SOURCE_ART == settings_get_source()) ? 1u : 0u;
    if(0 != view->art_player_enabled)
    {
        view->art_player_valid = openart_find_player_cell(&art_row, &art_col, &art_count);
    }
    else
    {
        view->art_player_valid = 0;
    }
    view->art_player_count = art_count;
    view->art_row = art_row;
    view->art_col = art_col;
}

static void draw_home_page(void)
{
    screen_home_view_struct view;

    build_home_view(&view);
    screen_draw_home(&view);
}

static void draw_debug_page(void)
{
    const drive_pose_struct *pose = drive_pose_get();

    screen_draw_debug(current_map, map_get(current_map), pose->x_cm, pose->y_cm);
}

static void draw_run_page(void)
{
    screen_run_view_struct view;

    build_run_view(&view);
    screen_draw_run_workbench(&view);
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
    screen_home_view_struct view;

    build_home_view(&view);
    screen_draw_home_status(&view);
}

static void refresh_debug_page(void)
{
    draw_debug_page();
}

static uint32 art_map_age_seconds(void)
{
    uint32 last_ms = openart_last_rx_ms();

    if(0 == last_ms)
    {
        return 0;
    }
    return (time_ms() - last_ms) / 1000u;
}

static void draw_art_map_page(void)
{
    const map_source_struct *source = openart_map_get();

    screen_draw_art_map(openart_uart_get_frame_count(), art_map_age_seconds(), source);
}

static void refresh_art_map_page(void)
{
    draw_art_map_page();
}

static void handle_art_map_key(menu_key_event_enum event)
{
    if(MENU_KEY_EVENT_K4_SHORT == event)
    {
        go_home();
    }
}

static void refresh_current_page_dynamic(void)
{
    const menu_page_def_struct *page;
    uint32 now_ms;

    page = find_page(current_page);
    if((0 == page->refresh_ms) || (0 == page->on_refresh))
    {
        return;
    }

    now_ms = time_ms();
    if((now_ms - dynamic_last_ms) >= page->refresh_ms)
    {
        // 用无符号差值判断周期，即使 time_ms() 溢出也能继续刷新动态页面。
        dynamic_last_ms = now_ms;
        page->on_refresh();
    }
}

static void draw_playback_page(void)
{
    screen_draw_playback(current_map, last_or_selected_map_source(), &last_result, playback_step, last_elapsed_ms, playback_state);
}

static void draw_execute_page(void)
{
    screen_execute_view_struct view;

    build_execute_view(&view);
    screen_draw_execute(&view);
}

static void refresh_execute_page(void)
{
    draw_execute_page();
}

static void handle_execute_event(menu_key_event_enum event)
{
    switch(event)
    {
        case MENU_KEY_EVENT_K3_SHORT:
            if(EXEC_STATE_PAUSED == executor_get_state())
            {
                executor_resume();
                run_state = "Running";
                mark_redraw();
            }
            break;
            
        case MENU_KEY_EVENT_K4_SHORT:
            // 执行页 K4 是人工停止路径：同时取消 ART 等待和 executor，防止返回后后台继续跑。
            art_replan_cancel();
            subject2_cancel();
            vision_uart_cancel();
            competition_flow_cancel();
            subject2_active = 0u;
            executor_stop();
            go_parent();
            break;
            
        default:
            break;
    }
}

static void draw_current_page(void)
{
    const menu_page_def_struct *page = find_page(current_page);

    if(page->id != current_page)
    {
        current_page = MENU_PAGE_HOME;
        page = find_page(current_page);
    }

    if(0 != page->on_draw)
    {
        page->on_draw();
    }
}

void menu_init(void)
{
    uint8 i;

    menu_key_init();
    pit_ms_init(PIT_CH2, MENU_KEY_SCAN_PERIOD_MS);

    current_map = settings_get_map();
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
    run_state = "Idle";
    clear_result(&last_result);
    mark_redraw();
    draw_current_page();
    need_redraw = 0;
}

void menu_poll(void)
{
    menu_key_event_enum event = menu_key_read_event();
    art_replan_context_struct art_context;
    art_replan_update_struct art_update;
    subject2_context_struct subject2_context;
    subject2_update_struct subject2_update;
    competition_action_enum action;

    if(MENU_KEY_EVENT_NONE != event)
    {
        dispatch_key_event(event);
    }

    playback_tick();
    build_art_replan_context(&art_context);
    if(0u != subject2_active)
    {
        build_subject2_context(&subject2_context);
        subject2_tick(&subject2_context, &subject2_update);
        apply_subject2_update(&subject2_update);
        if(0u != subject2_update.return_requested)
        {
            if(0 != art_replan_begin_return_home(&art_context,
                                                  subject2_update.return_pose_x_cm,
                                                  subject2_update.return_pose_y_cm,
                                                  &art_update))
            {
                subject2_cancel();
                subject2_active = 0u;
                apply_art_replan_update(&art_update);
            }
            else
            {
                subject2_cancel();
                subject2_active = 0u;
                executor_set_error(EXEC_ERROR_ART_SYNC);
                run_state = "RetErr";
                mark_redraw();
            }
        }
    }
    else
    {
        art_replan_tick(&art_context,
                        (MAP_SOURCE_ART == settings_get_source()) ? 1u : 0u,
                        &art_update);
        apply_art_replan_update(&art_update);
        if(0u != art_update.subject2_map_ready)
        {
            competition_launch_yaw_deg = get_control_status()->target_yaw;
            build_subject2_context(&subject2_context);
            subject2_begin(&subject2_context,
                           art_update.initial_pose_x_cm,
                           art_update.initial_pose_y_cm,
                           &subject2_update);
            subject2_active = (SUBJECT2_ERROR != subject2_get_state()) ? 1u : 0u;
            apply_subject2_update(&subject2_update);
        }
        if(0u != art_update.return_complete)
        {
            competition_flow_on_return_complete();
            action = competition_flow_take_action();
            start_competition_action(action, &art_update);
            apply_art_replan_update(&art_update);
        }
    }

    if(0 != need_redraw)
    {
        draw_current_page();
        dynamic_last_ms = time_ms();
        need_redraw = 0;
    }

    refresh_current_page_dynamic();
}
