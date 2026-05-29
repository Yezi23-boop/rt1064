# Menu Page Table Refresh Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the menu refresh structure so pages are table-driven, dynamic values refresh locally, and future pages or printed variables are easy to add.

**Architecture:** `menu.c` owns page state, page table lookup, key dispatch, and refresh timing. `screen.c` owns direct IPS200 drawing helpers and keeps full-page draw separate from dynamic value refresh. Numeric dynamic values overwrite in fixed width; text dynamic values clear only the value area before drawing.

**Tech Stack:** RT1064 firmware C, SeekFree IPS200 API, existing `menu_key`, `settings`, `maps`, `solver`, `drive_control`, and `drive_pose` modules.

---

### Task 1: Add Small Screen Value Helpers

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\screen.c`

- [ ] **Step 1: Replace line-wide clearing for value text with a small value-area helper**

Add these helpers near `clear_line()`:

```c
static void clear_text_area(uint16 x, uint16 y, uint8 char_count)
{
    fill_rect(x, y, (uint16)(char_count * 8), LINE_H, SCREEN_BG_COLOR);
}

static void show_text_value(uint16 x, uint16 y, const char *text, uint8 max_chars)
{
    clear_text_area(x, y, max_chars);
    ips200_show_string(x, y, text);
}
```

Keep numeric fields using `ips200_show_int`, `ips200_show_uint`, or `ips200_show_float` directly with fixed width.

- [ ] **Step 2: Update dynamic text fields to use `show_text_value()`**

Change dynamic text values such as save state, run state, playback state, and flash state to clear only their value area:

```c
ips200_show_string(0, LINE_H * 3, "State:");
show_text_value(48, LINE_H * 3, state, 10);
```

Do not use `snprintf`. Do not add another formatting abstraction for numbers.

- [ ] **Step 3: Reduce Home dynamic refresh clearing**

In `draw_home_status_lines()`, keep labels if this function remains shared by full draw and dynamic refresh, but avoid clearing rows for pure numeric fields unless the row includes variable-length text. Numeric fields should directly overwrite:

```c
ips200_show_int(56, LINE_H * 8, (int16)encoder_count[WHEEL_LF], 5);
ips200_show_int(136, LINE_H * 8, (int16)encoder_count[WHEEL_RF], 5);
```

### Task 2: Introduce Page Definition Table

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\menu.c`

- [ ] **Step 1: Add page callback types**

Add after the page enum:

```c
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
```

- [ ] **Step 2: Add per-page key wrapper functions**

Keep existing handlers where possible, but add wrappers so the table has one key handler per page:

```c
static void handle_home_key(menu_key_event_enum event)
{
    handle_list_event(event);
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
```

- [ ] **Step 3: Add page table**

Add after draw and handler forward declarations are available:

```c
static const menu_page_def_struct menu_pages[] =
{
    { MENU_PAGE_HOME,         MENU_PAGE_HOME, 500, draw_home_page,     refresh_home_page,     handle_home_key     },
    { MENU_PAGE_RUN,          MENU_PAGE_HOME, 0,   draw_run_page,      0,                     handle_run_key      },
    { MENU_PAGE_RUN_MAP,      MENU_PAGE_RUN,  0,   draw_run_map_page,  0,                     handle_map_event    },
    { MENU_PAGE_RUN_MODE,     MENU_PAGE_RUN,  0,   draw_mode_page,     0,                     handle_mode_event   },
    { MENU_PAGE_RUN_PLAYBACK, MENU_PAGE_RUN,  0,   draw_playback_page, 0,                     handle_playback_event },
    { MENU_PAGE_RUN_DEMO,     MENU_PAGE_RUN,  0,   draw_demo_page,     0,                     handle_demo_event   },
    { MENU_PAGE_DEBUG,        MENU_PAGE_HOME, 500, draw_debug_page,    refresh_debug_page,    handle_debug_key    },
    { MENU_PAGE_INFO,         MENU_PAGE_HOME, 0,   draw_info_page,     0,                     handle_info_key     },
};
```

Use the existing page IDs. Do not add new pages in this refactor.

- [ ] **Step 4: Add page lookup helper**

```c
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
```

### Task 3: Replace Switch-Based Draw and Refresh Dispatch

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\menu.c`

- [ ] **Step 1: Split current draw cases into page draw functions**

Create small wrappers:

```c
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
```

Keep `draw_home_page()`, `draw_debug_page()`, `draw_playback_page()`, and `draw_demo_page()`.

- [ ] **Step 2: Rename Home dynamic refresh wrapper**

Rename `draw_home_status()` to:

```c
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
        pose->y_cm);
}
```

- [ ] **Step 3: Add Debug dynamic refresh**

Add a debug refresh wrapper that redraws only the debug dynamic page for now:

```c
static void refresh_debug_page(void)
{
    draw_debug_page();
}
```

This preserves behavior and gives Debug a page-table refresh hook. A later pass can split `screen_draw_debug_status()` if screen tearing is visible.

- [ ] **Step 4: Replace `draw_current_page()` with table dispatch**

```c
static void draw_current_page(void)
{
    const menu_page_def_struct *page = find_page(current_page);

    if(page->id != current_page)
    {
        current_page = MENU_PAGE_HOME;
    }

    if(0 != page->draw_full)
    {
        page->draw_full();
    }
}
```

- [ ] **Step 5: Replace `dynamic_refresh_tick()` with table-driven refresh**

```c
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
```

### Task 4: Replace Key Dispatch With Page Table Dispatch

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\menu.c`

- [ ] **Step 1: Keep global K4 long handling**

In `dispatch_key_event()`, keep:

```c
if(MENU_KEY_EVENT_K4_LONG == event)
{
    go_home();
    return;
}
```

- [ ] **Step 2: Dispatch page key through table**

Replace the large switch with:

```c
const menu_page_def_struct *page = find_page(current_page);

if(0 != page->on_key)
{
    page->on_key(event);
}
```

The existing `handle_run_event`, `handle_map_event`, `handle_mode_event`, `handle_playback_event`, and `handle_demo_event` remain the actual page behavior.

- [ ] **Step 3: Reset refresh timer after full redraw**

In `menu_poll()`, after `draw_current_page()`:

```c
dynamic_last_ms = time_ms();
```

Then call `refresh_current_page_dynamic()` instead of `dynamic_refresh_tick()`.

### Task 5: Verify Structure and Build

**Files:**
- Verify: `C:\Users\ye\Desktop\rt1064\project\user\src\menu.c`
- Verify: `C:\Users\ye\Desktop\rt1064\project\user\src\screen.c`
- Verify: `C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx`

- [ ] **Step 1: Static checks**

Run:

```powershell
Select-String -Path "C:\Users\ye\Desktop\rt1064\project\user\src\menu.c" -Pattern "key_scanner|key_get_state|key_clear_state"
Select-String -Path "C:\Users\ye\Desktop\rt1064\project\user\src\screen.c" -Pattern "snprintf|show_line\("
git -C "C:\Users\ye\Desktop\rt1064" diff -- libraries
```

Expected:

```text
No key library calls in menu.c
No snprintf/show_line in screen.c
No libraries diff
```

- [ ] **Step 2: Keil build**

Run:

```powershell
& "D:\Keil_v5\UV4\UV4.exe" -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected build log:

```text
0 Error(s), 0 Warning(s)
```

- [ ] **Step 3: Manual board checks**

Check on board:

```text
Home data refreshes at about 500ms.
Home cursor movement only updates the cursor.
Run map switching redraws map immediately.
Run does not periodically redraw the map.
Debug refreshes while staying on Debug.
Text fields such as State do not leave old characters.
Numeric fields overwrite without full-line clearing.
K4 long still returns Home from all pages.
```

---

## Self-Review

- Spec coverage: The plan covers page table dispatch, page-level dynamic refresh, local text clearing, direct numeric IPS200 output, and no vendor library changes.
- Placeholder scan: No placeholder steps are left.
- Scope check: This is one focused refactor across `menu.c` and `screen.c`; no new menu features are introduced.
