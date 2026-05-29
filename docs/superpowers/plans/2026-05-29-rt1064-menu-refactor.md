# RT1064 Menu Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 RT1064 当前菜单重构为页面码驱动的层级菜单，并把按键扫描移到 `PIT_CH2` 5ms 节拍，解决菜单按键不响应和屏幕卡顿。

**Architecture:** `menu_key.c/h` 独立负责按键扫描和一次性事件队列，`menu.c` 只消费事件并维护页面码、光标、候选值和刷新脏标志，`screen.c/h` 只负责页面绘制和局部光标刷新。菜单页面切换和内容变化才重绘，Home/Debug 不再周期刷新。

**Tech Stack:** Embedded C, SeekFree RT1064 library, Keil uVision project, IPS200 screen, PIT timer interrupt.

---

## Scope And Rules

- 本计划只改应用层文件和 Keil 工程文件，不改 `libraries/` 下逐飞库源码。
- `menu.h` 公开接口保持 `menu_init()` 和 `menu_poll()`。
- `menu.c` 不再调用 `key_init/key_scanner/key_get_state/key_clear_state`。
- `PIT_CH2` 只调用 `menu_key_tick_5ms()`，不放耗时逻辑。
- Flash 保存语义保持不变：只有 K3 确认后的 `current_map/run_mode` 才进入 Dirty 状态，Home K4 保存。
- VOFA 曲线输出默认关闭，避免 `printf` 影响无线串口和菜单响应。

## File Map

- Create: `project/user/inc/menu_key.h`
  - 定义菜单按键事件枚举和 `menu_key_init/menu_key_tick_5ms/menu_key_read_event`。
- Create: `project/user/src/menu_key.c`
  - 包装逐飞按键库，5ms 扫描 K1-K4，短按/长按转换为环形队列事件。
- Modify: `project/user/src/isr.c`
  - 引入 `menu_key.h`，在 `PIT_CH2` 分支调用 `menu_key_tick_5ms()`。
- Modify: `project/user/src/menu.c`
  - 删除本地按键扫描逻辑，改为页面码状态机和统一列表导航。
- Modify: `project/user/inc/screen.h`
  - 增加局部光标绘制接口和 Run 根页、Map 页、Mode 页绘制接口。
- Modify: `project/user/src/screen.c`
  - 增加新页面绘制函数，保留 Playback/Demo/Debug/Info 等现有绘制能力。
- Modify: `project/user/src/vofa.c`
  - 确认 `VOFA_CURVE_OUTPUT_ENABLE` 为 `0`。
- Modify: `project/mdk/rt1064.uvprojx`
  - 在 user 源文件组加入 `menu_key.c`。

---

## Task 1: Baseline Snapshot And Guard Checks

**Files:**
- Read: `project/user/src/menu.c`
- Read: `project/user/src/screen.c`
- Read: `project/user/src/isr.c`
- Read: `project/user/src/vofa.c`
- Read: `project/mdk/rt1064.uvprojx`

- [ ] **Step 1: Record current dirty state**

Run:

```powershell
git status --short
```

Expected:

- Worktree may already contain user/session changes.
- Do not revert unrelated files.
- Confirm `libraries/` files are not modified before starting this refactor.

- [ ] **Step 2: Confirm `PIT_CH2` is free**

Run:

```powershell
Select-String -Path "project\user\src\*.c","project\user\inc\*.h" -Pattern "PIT_CH2"
```

Expected:

- `project/user/src/isr.c` has an empty `PIT_CH2` branch.
- No existing application module depends on `PIT_CH2`.

- [ ] **Step 3: Confirm current menu still scans keys directly**

Run:

```powershell
Select-String -Path "project\user\src\menu.c" -Pattern "key_init|key_scanner|key_get_state|key_clear_state"
```

Expected:

- Finds direct key scanning in `menu.c`.
- These calls must be gone after Task 4.

---

## Task 2: Add `menu_key` Public Interface

**Files:**
- Create: `project/user/inc/menu_key.h`

- [ ] **Step 1: Create `menu_key.h`**

Add this file:

```c
#ifndef _menu_key_h_
#define _menu_key_h_

#include "zf_common_typedef.h"

typedef enum
{
    MENU_KEY_EVENT_NONE = 0,
    MENU_KEY_EVENT_K1_SHORT,
    MENU_KEY_EVENT_K2_SHORT,
    MENU_KEY_EVENT_K3_SHORT,
    MENU_KEY_EVENT_K4_SHORT,
    MENU_KEY_EVENT_K1_LONG,
    MENU_KEY_EVENT_K2_LONG,
    MENU_KEY_EVENT_K3_LONG,
    MENU_KEY_EVENT_K4_LONG,
} menu_key_event_enum;

void menu_key_init(void);
void menu_key_tick_5ms(void);
menu_key_event_enum menu_key_read_event(void);

#endif
```

- [ ] **Step 2: Verify the header is standalone**

Run:

```powershell
Select-String -Path "project\user\inc\menu_key.h" -Pattern "menu_key_event_enum|menu_key_tick_5ms"
```

Expected:

- Both symbols are found.
- Header does not include `menu.h`, `screen.h`, or other high-level modules.

---

## Task 3: Implement 5ms Key Event Queue

**Files:**
- Create: `project/user/src/menu_key.c`

- [ ] **Step 1: Add event queue implementation**

Add this file:

```c
#include "zf_common_headfile.h"
#include "menu_key.h"

#define MENU_KEY_SCAN_PERIOD_MS (5)
#define MENU_KEY_QUEUE_SIZE     (8)

static volatile menu_key_event_enum event_queue[MENU_KEY_QUEUE_SIZE];
static volatile uint8 queue_head;
static volatile uint8 queue_tail;
static volatile uint8 queue_count;
static uint8 long_used[KEY_NUMBER];

static menu_key_event_enum short_event_from_index(uint8 index)
{
    switch(index)
    {
        case 0: return MENU_KEY_EVENT_K1_SHORT;
        case 1: return MENU_KEY_EVENT_K2_SHORT;
        case 2: return MENU_KEY_EVENT_K3_SHORT;
        case 3: return MENU_KEY_EVENT_K4_SHORT;
        default: return MENU_KEY_EVENT_NONE;
    }
}

static menu_key_event_enum long_event_from_index(uint8 index)
{
    switch(index)
    {
        case 0: return MENU_KEY_EVENT_K1_LONG;
        case 1: return MENU_KEY_EVENT_K2_LONG;
        case 2: return MENU_KEY_EVENT_K3_LONG;
        case 3: return MENU_KEY_EVENT_K4_LONG;
        default: return MENU_KEY_EVENT_NONE;
    }
}

static void push_event(menu_key_event_enum event)
{
    if(event == MENU_KEY_EVENT_NONE)
    {
        return;
    }

    if(queue_count >= MENU_KEY_QUEUE_SIZE)
    {
        queue_tail = (uint8)((queue_tail + 1) % MENU_KEY_QUEUE_SIZE);
        queue_count--;
    }

    event_queue[queue_head] = event;
    queue_head = (uint8)((queue_head + 1) % MENU_KEY_QUEUE_SIZE);
    queue_count++;
}

void menu_key_init(void)
{
    uint8 i;

    key_init(MENU_KEY_SCAN_PERIOD_MS);
    queue_head = 0;
    queue_tail = 0;
    queue_count = 0;

    for(i = 0; i < KEY_NUMBER; i++)
    {
        long_used[i] = 0;
        key_clear_state((key_index_enum)i);
    }
}

void menu_key_tick_5ms(void)
{
    uint8 i;

    key_scanner();

    for(i = 0; i < KEY_NUMBER; i++)
    {
        key_state_enum state = key_get_state((key_index_enum)i);

        if(state == KEY_RELEASE)
        {
            long_used[i] = 0;
            continue;
        }

        if((state == KEY_LONG_PRESS) && (long_used[i] == 0))
        {
            long_used[i] = 1;
            push_event(long_event_from_index(i));
            key_clear_state((key_index_enum)i);
            continue;
        }

        if((state == KEY_SHORT_PRESS) && (long_used[i] == 0))
        {
            push_event(short_event_from_index(i));
            key_clear_state((key_index_enum)i);
        }
    }
}

menu_key_event_enum menu_key_read_event(void)
{
    menu_key_event_enum event;

    if(queue_count == 0)
    {
        return MENU_KEY_EVENT_NONE;
    }

    event = event_queue[queue_tail];
    queue_tail = (uint8)((queue_tail + 1) % MENU_KEY_QUEUE_SIZE);
    queue_count--;

    return event;
}
```

- [ ] **Step 2: Static check key library usage is isolated**

Run:

```powershell
Select-String -Path "project\user\src\menu_key.c" -Pattern "key_init|key_scanner|key_get_state|key_clear_state"
```

Expected:

- All four key library calls are found only in `menu_key.c`.

---

## Task 4: Wire `PIT_CH2` And Menu Initialization

**Files:**
- Modify: `project/user/src/isr.c`
- Modify: `project/user/src/menu.c`

- [ ] **Step 1: Add ISR include and tick call**

In `project/user/src/isr.c`, add:

```c
#include "menu_key.h"
```

Update the `PIT_CH2` branch to:

```c
    if(pit_flag_get(PIT_CH2))
    {
        pit_flag_clear(PIT_CH2);
        menu_key_tick_5ms();
    }
```

- [ ] **Step 2: Change `menu_init()` ownership of key scanning**

In `project/user/src/menu.c`, add:

```c
#include "menu_key.h"
```

Remove local key scanning state and helpers:

```c
static uint32 key_last_scan_ms;
static uint8 key_long_used[KEY_NUMBER];
static key_event_enum key_read_event(void);
```

Update `menu_init()` so key setup becomes:

```c
    menu_key_init();
    pit_ms_init(PIT_CH2, 5);
```

- [ ] **Step 3: Verify direct key calls are gone from `menu.c`**

Run:

```powershell
Select-String -Path "project\user\src\menu.c" -Pattern "key_init|key_scanner|key_get_state|key_clear_state"
```

Expected:

- No matches.

---

## Task 5: Define Page Codes And Cursor Memory In `menu.c`

**Files:**
- Modify: `project/user/src/menu.c`

- [ ] **Step 1: Replace enum page model with numeric page codes**

Use these definitions near the top of `menu.c`:

```c
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

#define HOME_ITEM_COUNT (3)
#define RUN_ITEM_COUNT  (3)
#define ROOT_PAGE_COUNT (4)

static const char *const home_items[HOME_ITEM_COUNT] = {"Run", "Debug", "Info"};
static const menu_page_id_enum home_item_pages[HOME_ITEM_COUNT] = {
    MENU_PAGE_RUN,
    MENU_PAGE_DEBUG,
    MENU_PAGE_INFO,
};

static const char *const run_items[RUN_ITEM_COUNT] = {"Map", "Mode", "Execute"};
static const menu_page_id_enum run_item_pages[RUN_ITEM_COUNT] = {
    MENU_PAGE_RUN_MAP,
    MENU_PAGE_RUN_MODE,
    MENU_PAGE_RUN_EXECUTE,
};
```

- [ ] **Step 2: Add page state**

Use this state set:

```c
static menu_page_id_enum current_page;
static uint8 cursor_row;
static uint8 previous_cursor_row;
static uint8 page_cursor[ROOT_PAGE_COUNT];
static uint8 need_redraw;
```

Keep existing domain state:

```c
static uint8 current_map;
static uint8 candidate_map;
static run_mode_enum run_mode;
static run_mode_enum candidate_mode;
static save_state_enum save_state;
static solve_result_struct last_result;
static uint32 last_solve_elapsed_ms;
static const char *run_state;
```

- [ ] **Step 3: Add page helper functions**

Add helpers with this behavior:

```c
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
    if(page == MENU_PAGE_HOME)
    {
        return HOME_ITEM_COUNT;
    }
    if(page == MENU_PAGE_RUN)
    {
        return RUN_ITEM_COUNT;
    }
    if(page == MENU_PAGE_RUN_MODE)
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

    previous_cursor_row = cursor_row;
    need_redraw = 1;
}
```

---

## Task 6: Refactor Menu Event Dispatch

**Files:**
- Modify: `project/user/src/menu.c`

- [ ] **Step 1: Add unified list movement**

Add:

```c
static void move_cursor(int8 delta)
{
    uint8 count = page_item_count(current_page);

    if(count == 0)
    {
        return;
    }

    previous_cursor_row = cursor_row;

    if(delta < 0)
    {
        cursor_row = (cursor_row == 0) ? (uint8)(count - 1) : (uint8)(cursor_row - 1);
    }
    else
    {
        cursor_row = (uint8)((cursor_row + 1) % count);
    }

    save_page_cursor();
    screen_draw_nav_cursor(previous_cursor_row, cursor_row);
}
```

- [ ] **Step 2: Add enter and back behavior**

Add:

```c
static void go_home(void)
{
    enter_page(MENU_PAGE_HOME);
}

static void go_parent(void)
{
    if(current_page >= 10)
    {
        enter_page((menu_page_id_enum)page_root(current_page));
    }
    else if(current_page != MENU_PAGE_HOME)
    {
        enter_page(MENU_PAGE_HOME);
    }
}

static void enter_selected_item(void)
{
    if(current_page == MENU_PAGE_HOME)
    {
        enter_page(home_item_pages[cursor_row]);
        return;
    }

    if(current_page == MENU_PAGE_RUN)
    {
        if(run_item_pages[cursor_row] == MENU_PAGE_RUN_EXECUTE)
        {
            execute_current_selection();
        }
        else
        {
            enter_page(run_item_pages[cursor_row]);
        }
    }
}
```

- [ ] **Step 3: Replace top-level dispatcher**

Use this dispatch shape:

```c
static void dispatch_key_event(menu_key_event_enum event)
{
    if(event == MENU_KEY_EVENT_NONE)
    {
        return;
    }

    if(event == MENU_KEY_EVENT_K4_LONG)
    {
        stop_demo();
        go_home();
        return;
    }

    switch(current_page)
    {
        case MENU_PAGE_HOME:
        case MENU_PAGE_RUN:
            handle_list_event(event);
            break;

        case MENU_PAGE_RUN_MAP:
            handle_map_event(event);
            break;

        case MENU_PAGE_RUN_MODE:
            handle_mode_event(event);
            break;

        case MENU_PAGE_RUN_PLAYBACK:
            handle_playback_event(event);
            break;

        case MENU_PAGE_RUN_DEMO:
            handle_demo_event(event);
            break;

        case MENU_PAGE_DEBUG:
        case MENU_PAGE_INFO:
            if(event == MENU_KEY_EVENT_K4_SHORT)
            {
                go_home();
            }
            break;

        default:
            go_home();
            break;
    }
}
```

---

## Task 7: Implement Run Subpages

**Files:**
- Modify: `project/user/src/menu.c`

- [ ] **Step 1: Map page behavior**

Add:

```c
static void handle_map_event(menu_key_event_enum event)
{
    if(event == MENU_KEY_EVENT_K1_SHORT)
    {
        candidate_map = (candidate_map == 0) ? (uint8)(map_count() - 1) : (uint8)(candidate_map - 1);
        need_redraw = 1;
    }
    else if(event == MENU_KEY_EVENT_K2_SHORT)
    {
        candidate_map = (uint8)((candidate_map + 1) % map_count());
        need_redraw = 1;
    }
    else if(event == MENU_KEY_EVENT_K3_SHORT)
    {
        current_map = candidate_map;
        settings_set_runtime(current_map, run_mode);
        save_state = SAVE_STATE_DIRTY;
        clear_result_state();
        enter_page(MENU_PAGE_RUN);
    }
    else if(event == MENU_KEY_EVENT_K4_SHORT)
    {
        candidate_map = current_map;
        enter_page(MENU_PAGE_RUN);
    }
}
```

- [ ] **Step 2: Mode page behavior**

Add:

```c
static void handle_mode_event(menu_key_event_enum event)
{
    if(event == MENU_KEY_EVENT_K1_SHORT)
    {
        candidate_mode = (candidate_mode == 0) ? (run_mode_enum)(RUN_MODE_COUNT - 1) : (run_mode_enum)(candidate_mode - 1);
        need_redraw = 1;
    }
    else if(event == MENU_KEY_EVENT_K2_SHORT)
    {
        candidate_mode = (run_mode_enum)((candidate_mode + 1) % RUN_MODE_COUNT);
        need_redraw = 1;
    }
    else if(event == MENU_KEY_EVENT_K3_SHORT)
    {
        run_mode = candidate_mode;
        settings_set_runtime(current_map, run_mode);
        save_state = SAVE_STATE_DIRTY;
        enter_page(MENU_PAGE_RUN);
    }
    else if(event == MENU_KEY_EVENT_K4_SHORT)
    {
        candidate_mode = run_mode;
        enter_page(MENU_PAGE_RUN);
    }
}
```

- [ ] **Step 3: Execute behavior**

Add:

```c
static void execute_current_selection(void)
{
    candidate_map = current_map;
    candidate_mode = run_mode;

    if(run_mode == RUN_MODE_BROWSE)
    {
        run_state = "Browse";
        enter_page(MENU_PAGE_RUN_MAP);
        return;
    }

    if(run_mode == RUN_MODE_DEMO)
    {
        start_demo();
        enter_page(MENU_PAGE_RUN_DEMO);
        return;
    }

    solve_current_map();
    enter_page(MENU_PAGE_RUN_PLAYBACK);
}
```

Acceptance:

- Browse 不执行 BFS，只进入地图预览页。
- Solve/Run 执行 BFS，进入 Playback。
- Demo 进入 Demo 页面，由 `demo_tick()` 分批推进，避免一次性卡住屏幕。

---

## Task 8: Refresh Strategy In `menu_poll()`

**Files:**
- Modify: `project/user/src/menu.c`

- [ ] **Step 1: Read queued events only**

Use this `menu_poll()` structure:

```c
void menu_poll(void)
{
    menu_key_event_enum event;

    event = menu_key_read_event();
    if(event != MENU_KEY_EVENT_NONE)
    {
        dispatch_key_event(event);
    }

    demo_tick();
    playback_tick();

    if(need_redraw)
    {
        draw_current_page();
        need_redraw = 0;
    }
}
```

- [ ] **Step 2: Draw only when dirty**

Keep this rule:

- `enter_page()` sets `need_redraw = 1`。
- Map/Mode candidate value changes set `need_redraw = 1`。
- Playback step or state changes set `need_redraw = 1`。
- Demo index/count/state changes set `need_redraw = 1`。
- Home/Debug do not refresh on timer.
- List cursor movement calls `screen_draw_nav_cursor()` directly and does not redraw the whole page.

---

## Task 9: Add Screen Interfaces

**Files:**
- Modify: `project/user/inc/screen.h`
- Modify: `project/user/src/screen.c`

- [ ] **Step 1: Add prototypes to `screen.h`**

Add:

```c
void screen_draw_nav_cursor(uint8 previous_cursor, uint8 cursor);
void screen_draw_run_root(const char *const *items, uint8 item_count, uint8 cursor, uint8 current_map, run_mode_enum mode, save_state_enum save_state, const char *state, uint32 elapsed_ms);
void screen_draw_map_select(uint8 current_map, uint8 candidate_map, save_state_enum save_state, const char *state);
void screen_draw_mode_page(run_mode_enum candidate_mode);
```

Keep existing:

```c
void screen_draw_home(...);
void screen_draw_playback(...);
void screen_draw_demo(...);
void screen_draw_debug(...);
void screen_draw_info(...);
```

- [ ] **Step 2: Implement local cursor drawing**

In `screen.c`, use the existing line height convention:

```c
#define MENU_CURSOR_X (0)
#define MENU_ROW_Y(row) ((uint16)((row) * LINE_HEIGHT + LINE_HEIGHT))

void screen_draw_nav_cursor(uint8 previous_cursor, uint8 cursor)
{
    ips200_show_string(MENU_CURSOR_X, MENU_ROW_Y(previous_cursor), " ");
    ips200_show_string(MENU_CURSOR_X, MENU_ROW_Y(cursor), ">");
}
```

- [ ] **Step 3: Implement Run root drawing**

Add:

```c
void screen_draw_run_root(const char *const *items, uint8 item_count, uint8 cursor, uint8 current_map, run_mode_enum mode, save_state_enum save_state, const char *state, uint32 elapsed_ms)
{
    uint8 i;

    begin_page(SCREEN_PAGE_RUN_ROOT);
    show_line(0, "Run");

    for(i = 0; i < item_count; i++)
    {
        clear_line((uint16)((i + 1) * LINE_HEIGHT));
        ips200_show_string(0, (uint16)((i + 1) * LINE_HEIGHT), (i == cursor) ? ">" : " ");
        ips200_show_string(16, (uint16)((i + 1) * LINE_HEIGHT), items[i]);
    }

    show_line(5, "Map:%02d Mode:%s", current_map + 1, run_mode_name(mode));
    show_line(6, "Save:%s State:%s", save_state_name(save_state), state);
    show_line(7, "Last:%lums", elapsed_ms);
    show_hint("K1/K2 Move  K3 Enter");
}
```

If existing `run_mode_name()` or `save_state_name()` are `static`, keep them in `screen.c` and reuse them.

- [ ] **Step 4: Implement Map and Mode pages**

Add:

```c
void screen_draw_map_select(uint8 current_map, uint8 candidate_map, save_state_enum save_state, const char *state)
{
    begin_page(SCREEN_PAGE_RUN_MAP);
    show_line(0, "Run/Map");
    show_line(1, "Current:%02d", current_map + 1);
    show_line(2, "Select :%02d", candidate_map + 1);
    show_line(3, "Save:%s", save_state_name(save_state));
    show_line(4, "State:%s", state);
    draw_color_map(map_get(candidate_map), 0, 0);
    show_hint("K1/K2 Map  K3 OK  K4 Back");
}

void screen_draw_mode_page(run_mode_enum candidate_mode)
{
    screen_draw_mode_select(candidate_mode);
}
```

Acceptance:

- `screen_draw_map_select()` shows candidate map preview.
- `screen_draw_mode_page()` preserves existing Mode UI if the old function is already readable.
- No periodic redraw is introduced inside `screen.c`。

---

## Task 10: Update `draw_current_page()`

**Files:**
- Modify: `project/user/src/menu.c`

- [ ] **Step 1: Route every page code to one draw function**

Use:

```c
static void draw_current_page(void)
{
    switch(current_page)
    {
        case MENU_PAGE_HOME:
            draw_home_page();
            break;

        case MENU_PAGE_RUN:
            screen_draw_run_root(run_items, RUN_ITEM_COUNT, cursor_row, current_map, run_mode, save_state, run_state, last_solve_elapsed_ms);
            break;

        case MENU_PAGE_RUN_MAP:
            screen_draw_map_select(current_map, candidate_map, save_state, run_state);
            break;

        case MENU_PAGE_RUN_MODE:
            screen_draw_mode_page(candidate_mode);
            break;

        case MENU_PAGE_RUN_PLAYBACK:
            draw_playback_page();
            break;

        case MENU_PAGE_RUN_DEMO:
            draw_demo_page();
            break;

        case MENU_PAGE_DEBUG:
            draw_debug_page();
            break;

        case MENU_PAGE_INFO:
            screen_draw_info(map_count(), save_state);
            break;

        default:
            current_page = MENU_PAGE_HOME;
            draw_home_page();
            break;
    }
}
```

- [ ] **Step 2: Preserve Home and Debug pose data**

Keep current `draw_home_page()` and `draw_debug_page()` logic that reads:

```c
get_control_status(...);
drive_pose_get(...);
screen_draw_home(...);
screen_draw_debug(...);
```

Acceptance:

- Home and Debug still display status once when entering the page.
- They do not refresh again unless a page transition explicitly redraws them.

---

## Task 11: VOFA Default Off

**Files:**
- Modify: `project/user/src/vofa.c`

- [ ] **Step 1: Ensure curve output is disabled**

Set:

```c
#define VOFA_CURVE_OUTPUT_ENABLE (0)
```

- [ ] **Step 2: Verify no accidental enable**

Run:

```powershell
Select-String -Path "project\user\src\vofa.c" -Pattern "VOFA_CURVE_OUTPUT_ENABLE"
```

Expected:

- The macro is defined as `0`.

---

## Task 12: Add `menu_key.c` To Keil Project

**Files:**
- Modify: `project/mdk/rt1064.uvprojx`

- [ ] **Step 1: Insert file entry next to `menu.c`**

In the same group that currently contains `screen.c`, `menu.c`, and `vofa.c`, add:

```xml
            <File>
              <FileName>menu_key.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\user\src\menu_key.c</FilePath>
            </File>
```

Recommended location: immediately after the existing `menu.c` entry and before `vofa.c`.

- [ ] **Step 2: Verify project entry**

Run:

```powershell
Select-String -Path "project\mdk\rt1064.uvprojx" -Pattern "menu_key.c" -Context 2,2
```

Expected:

- One `menu_key.c` file entry is found.

---

## Task 13: Static Verification

**Files:**
- Read: all modified files

- [ ] **Step 1: Verify `menu.c` no longer scans keys**

Run:

```powershell
Select-String -Path "project\user\src\menu.c" -Pattern "key_init|key_scanner|key_get_state|key_clear_state"
```

Expected:

- No matches.

- [ ] **Step 2: Verify key library calls only exist in `menu_key.c`**

Run:

```powershell
Select-String -Path "project\user\src\*.c" -Pattern "key_init|key_scanner|key_get_state|key_clear_state"
```

Expected:

- Matches only in `project/user/src/menu_key.c`.

- [ ] **Step 3: Verify `PIT_CH2` branch only ticks menu key**

Run:

```powershell
Select-String -Path "project\user\src\isr.c" -Pattern "PIT_CH2|menu_key_tick_5ms" -Context 2,3
```

Expected:

- `PIT_CH2` branch clears flag and calls `menu_key_tick_5ms()` only.

- [ ] **Step 4: Verify `libraries/` has no diff**

Run:

```powershell
git diff -- libraries
```

Expected:

- No output.

- [ ] **Step 5: Verify page constants and item counts**

Run:

```powershell
Select-String -Path "project\user\src\menu.c" -Pattern "MENU_PAGE_HOME|MENU_PAGE_RUN_MAP|HOME_ITEM_COUNT|RUN_ITEM_COUNT|home_item_pages|run_item_pages"
```

Expected:

- Page code constants include `0, 1, 11, 12, 13, 14, 15, 2, 3`.
- `HOME_ITEM_COUNT` is `3`.
- `RUN_ITEM_COUNT` is `3`.

---

## Task 14: Keil Build Verification

**Files:**
- Build target: `project/mdk/rt1064.uvprojx`
- Read log: `project/mdk/Objects/rt1064.build_log.htm`

- [ ] **Step 1: Run full Keil build**

Run:

```powershell
& "D:\Keil_v5\UV4\UV4.exe" -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected:

- Keil exits successfully or writes a build log with final result.

- [ ] **Step 2: Read build result**

Run:

```powershell
Select-String -Path "project\mdk\Objects\rt1064.build_log.htm" -Pattern "Error\\(s\\)|Warning\\(s\\)|Build Time"
```

Expected:

- Final build line reports `0 Error(s), 0 Warning(s)`.

If warnings appear, fix warnings caused by this refactor. Do not fix unrelated pre-existing warnings unless they block the menu build.

---

## Task 15: Board Verification Checklist

**Files:**
- No code edits in this task unless a board test fails and the root cause is identified.

- [ ] **Step 1: Home navigation**

Expected on board:

- K1/K2 moves Home cursor smoothly.
- Cursor movement does not clear and redraw the whole page.
- K3 enters selected root page.
- K4 on Home saves confirmed runtime settings.

- [ ] **Step 2: Global return**

Expected on board:

- K4 long from any menu page returns Home.
- Demo is stopped before returning Home.

- [ ] **Step 3: Run root**

Expected on board:

- K3 from Home Run enters Run root.
- Run root shows `Map / Mode / Execute`.
- Returning to Run root restores its last cursor position.

- [ ] **Step 4: Map page**

Expected on board:

- K1/K2 changes candidate map and redraws preview only when candidate changes.
- K3 confirms candidate to `current_map`, sets Save Dirty, returns Run root.
- K4 cancels candidate change and returns Run root.

- [ ] **Step 5: Mode page**

Expected on board:

- K1/K2 changes candidate mode.
- K3 confirms `run_mode`, sets Save Dirty, returns Run root.
- K4 cancels and returns Run root.

- [ ] **Step 6: Execute page behavior**

Expected on board:

- Browse: enters map preview without BFS.
- Solve/Run: solves current map and enters Playback.
- Demo: enters Demo page and progresses without blocking the main loop for long periods.

- [ ] **Step 7: Low-refresh pages**

Expected on board:

- Home and Debug do not stutter from periodic redraw.
- Wireless UART output is quieter because VOFA curve output is off.

---

## Commit Plan

Commit only after build and static checks pass.

Recommended commit:

```powershell
git add project/user/inc/menu_key.h project/user/src/menu_key.c project/user/src/isr.c project/user/src/menu.c project/user/inc/screen.h project/user/src/screen.c project/user/src/vofa.c project/mdk/rt1064.uvprojx
git commit -m "重构 RT1064 菜单按键与页面结构"
```

Do not stage unrelated user changes unless they are part of this refactor and were intentionally modified.

---

## Self-Review

- Spec coverage: 按键 5ms 扫描、页面码、Run 层级菜单、刷新策略、VOFA 默认关闭、Keil 工程加入新文件、静态和构建验证均有对应任务。
- Placeholder scan: 本计划没有使用 TBD/TODO/implement later；每个新增接口和核心状态机都有具体代码形状。
- Type consistency: `menu_key_event_enum`、`menu_page_id_enum`、`screen_draw_*` 函数名在任务间保持一致。
- Risk note: `menu_key_read_event()` 与 ISR 共享环形队列，当前计划使用 `volatile` 和短临界操作；如果上板出现偶发丢键，再加极短全局中断保护包住 pop/push，而不改变公开接口。
