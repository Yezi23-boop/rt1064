# Menu Page Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the RT1064 menu into a unified page-controller model, keep current behavior mostly unchanged, and make `menu.c` own business data while `screen.c` only draws.

**Architecture:** Keep the existing table-driven menu design, but expand each page entry with `on_enter` and `on_exit` lifecycle hooks. Move direct business-state reads out of `screen.c` into `menu.c`, then pass compact view structs to screen drawing functions. Unify map drawing through one internal render input so Run, Playback, and Execute share state simulation.

**Tech Stack:** C for RT1064, Keil MDK project at `project/mdk/rt1064.uvprojx`, IPS200 drawing through existing `zf_common_headfile.h` APIs.

---

## File Structure

- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\menu.c`
  - Own page lifecycle dispatch.
  - Own business data reads from settings, executor, OpenART, drive pose, and control status.
  - Build screen view structs before calling `screen_draw_*`.
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\inc\screen.h`
  - Add view structs for Home, Run, Execute, Playback, Debug, and ART Map where useful.
  - Replace the longest drawing signatures with struct-based signatures.
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\screen.c`
  - Remove direct business includes and getters.
  - Keep only rendering helpers and `screen_draw_*` functions.
  - Consolidate map rendering input.
- Reference only: `C:\Users\ye\Desktop\rt1064\docs\superpowers\specs\2026-05-30-menu-page-controller-design.md`
  - Design source of truth.
- Verify: `C:\Users\ye\Desktop\rt1064\project\mdk\build_log.txt`
  - Keil build output.

## Task 1: Add Page Lifecycle Hooks

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\menu.c`
- Verify: `C:\Users\ye\Desktop\rt1064\project\mdk\build_log.txt`

- [ ] **Step 1: Rename page callback typedefs**

In `menu.c`, replace:

```c
typedef void (*menu_page_draw_func)(void);
typedef void (*menu_page_refresh_func)(void);
typedef void (*menu_page_key_func)(menu_key_event_enum event);
```

with:

```c
typedef void (*menu_page_lifecycle_func)(void);
typedef void (*menu_page_key_func)(menu_key_event_enum event);
```

- [ ] **Step 2: Extend the page definition struct**

Replace the current `menu_page_def_struct` with:

```c
typedef struct
{
    menu_page_id_enum id;
    menu_page_id_enum parent;
    uint16 refresh_ms;
    menu_page_lifecycle_func on_enter;
    menu_page_lifecycle_func on_exit;
    menu_page_lifecycle_func on_draw;
    menu_page_lifecycle_func on_refresh;
    menu_page_key_func on_key;
} menu_page_def_struct;
```

- [ ] **Step 3: Add page enter hook declarations**

Near the existing static declarations, add:

```c
static void enter_run_map_page(void);
static void enter_run_mode_page(void);
static void exit_noop_page(void);
```

- [ ] **Step 4: Update `menu_pages[]` entries**

Replace each row with the new field order. Use `0` for pages without hooks:

```c
static const menu_page_def_struct menu_pages[] =
{
    { MENU_PAGE_HOME,         MENU_PAGE_HOME, HOME_DYNAMIC_REFRESH_MS, 0,                   0, draw_home_page,     refresh_home_page,    handle_home_key      },
    { MENU_PAGE_RUN,          MENU_PAGE_HOME, 0,                       0,                   0, draw_run_page,      0,                    handle_run_key       },
    { MENU_PAGE_RUN_MAP,      MENU_PAGE_RUN,  0,                       enter_run_map_page,  0, draw_run_map_page,  0,                    handle_map_event     },
    { MENU_PAGE_RUN_MODE,     MENU_PAGE_RUN,  0,                       enter_run_mode_page, 0, draw_mode_page,     0,                    handle_mode_event    },
    { MENU_PAGE_RUN_PLAYBACK, MENU_PAGE_RUN,  0,                       0,                   0, draw_playback_page, 0,                    handle_playback_event },
    { MENU_PAGE_RUN_EXECUTE,  MENU_PAGE_RUN,  500,                     0,                   0, draw_execute_page,  refresh_execute_page, handle_execute_event  },
    { MENU_PAGE_DEBUG,        MENU_PAGE_HOME, HOME_DYNAMIC_REFRESH_MS, 0,                   0, draw_debug_page,    refresh_debug_page,   handle_debug_key     },
    { MENU_PAGE_INFO,         MENU_PAGE_HOME, 0,                       0,                   0, draw_info_page,     0,                    handle_info_key      },
    { MENU_PAGE_ART_MAP,      MENU_PAGE_HOME, HOME_DYNAMIC_REFRESH_MS, 0,                   0, draw_art_map_page,  refresh_art_map_page, handle_art_map_key   },
};
```

- [ ] **Step 5: Implement page enter hooks**

Add these functions near `enter_page()`:

```c
static void enter_run_map_page(void)
{
    candidate_map = current_map;
}

static void enter_run_mode_page(void)
{
    candidate_mode = run_mode;
}

static void exit_noop_page(void)
{
}
```

If `exit_noop_page()` remains unused after final implementation, remove its declaration and function before compiling.

- [ ] **Step 6: Move special enter logic out of `enter_page()`**

In `enter_page()`, remove:

```c
if(MENU_PAGE_RUN_MAP == page)
{
    candidate_map = current_map;
}
else if(MENU_PAGE_RUN_MODE == page)
{
    candidate_mode = run_mode;
}
```

Then call lifecycle hooks after cursor restoration:

```c
if(0 != page_def->on_enter)
{
    page_def->on_enter();
}
```

Use this full `enter_page()` shape:

```c
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

    if(0 != new_page->on_enter)
    {
        new_page->on_enter();
    }

    previous_cursor_row = cursor_row;
    mark_redraw();
}
```

- [ ] **Step 7: Update draw and refresh dispatch fields**

In `draw_current_page()`, replace `page->draw_full` with `page->on_draw`:

```c
if(0 != page->on_draw)
{
    page->on_draw();
}
```

In `refresh_current_page_dynamic()`, replace `page->refresh_dynamic` with `page->on_refresh`:

```c
if((0 == page->refresh_ms) || (0 == page->on_refresh))
{
    return;
}
...
page->on_refresh();
```

- [ ] **Step 8: Build**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected in `project/mdk/build_log.txt`:

```text
0 Error(s), 0 Warning(s)
```

- [ ] **Step 9: Commit task 1**

Run:

```powershell
git add -- project/user/src/menu.c
git commit -m "refactor: add menu page lifecycle hooks"
```

## Task 2: Introduce Screen View Structs

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\inc\screen.h`
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\screen.c`
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\menu.c`

- [ ] **Step 1: Add `screen_home_view_struct`**

In `screen.h`, after `playback_state_enum`, add:

```c
typedef struct
{
    const char *const *items;
    uint8 item_count;
    uint8 cursor;
    uint8 current_map;
    run_mode_enum mode;
    save_state_enum save_state;
    const float *encoder_count;
    float imu_roll;
    float imu_pitch;
    float imu_yaw;
    float target_yaw;
    float yaw_error;
    float vz;
    float vzt;
    float pose_x_cm;
    float pose_y_cm;
    uint32 openart_frame_count;
} screen_home_view_struct;
```

- [ ] **Step 2: Add `screen_run_view_struct`**

In `screen.h`, add:

```c
typedef struct
{
    uint8 current_map;
    run_mode_enum mode;
    map_source_enum source_type;
    save_state_enum save_state;
    const char *state_text;
    uint32 elapsed_ms;
    const map_source_struct *source;
    uint8 executor_active;
    executor_state_enum executor_state;
    executor_error_enum executor_error;
    uint16 current_step;
    uint16 total_steps;
    uint16 current_box;
    uint16 total_boxes;
    float pose_x_cm;
    float pose_y_cm;
} screen_run_view_struct;
```

- [ ] **Step 3: Add `screen_execute_view_struct`**

In `screen.h`, add:

```c
typedef struct
{
    uint8 current_map;
    const map_source_struct *source;
    const solve_result_struct *result;
    uint16 current_step;
    executor_state_enum state;
    executor_error_enum error;
    uint16 current_box;
    uint16 total_boxes;
    uint8 start_row;
    uint8 start_col;
    float pose_x_cm;
    float pose_y_cm;
} screen_execute_view_struct;
```

- [ ] **Step 4: Replace Home function declarations**

In `screen.h`, replace:

```c
void screen_draw_home(const char *const *items, uint8 item_count, uint8 cursor, uint8 current_map, run_mode_enum mode, save_state_enum save_state, const float encoder_count[WHEEL_COUNT], float imu_roll, float imu_pitch, float imu_yaw, float target_yaw, float yaw_error, float vz, float vzt, float pose_x_cm, float pose_y_cm, uint32 openart_frame_count);
void screen_draw_home_status(const float encoder_count[WHEEL_COUNT], float imu_roll, float imu_pitch, float imu_yaw, float target_yaw, float yaw_error, float vz, float vzt, float pose_x_cm, float pose_y_cm, uint32 openart_frame_count);
```

with:

```c
void screen_draw_home(const screen_home_view_struct *view);
void screen_draw_home_status(const screen_home_view_struct *view);
```

- [ ] **Step 5: Replace Run and Execute declarations**

In `screen.h`, replace:

```c
void screen_draw_run_workbench(uint8 current_map, run_mode_enum mode, map_source_enum source, save_state_enum save_state, const char *state, uint32 elapsed_ms);
```

with:

```c
void screen_draw_run_workbench(const screen_run_view_struct *view);
```

Replace the current `screen_draw_execute(...)` declaration with:

```c
void screen_draw_execute(const screen_execute_view_struct *view);
```

- [ ] **Step 6: Update Home drawing implementation**

In `screen.c`, change `screen_draw_home()` to:

```c
void screen_draw_home(const screen_home_view_struct *view)
{
    uint8 i;

    begin_page(SCREEN_PAGE_HOME);
    ips200_show_string(0, 0, "Home");
    for(i = 0; i < view->item_count; i++)
    {
        ips200_show_string(0, (uint16)(LINE_H * (i + 1)), (i == view->cursor) ? ">" : " ");
        ips200_show_string(16, (uint16)(LINE_H * (i + 1)), view->items[i]);
    }
    ips200_show_string(0, LINE_H * 5, "Map : V");
    ips200_show_uint(56, LINE_H * 5, view->current_map + 1, 2);
    ips200_show_string(0, LINE_H * 6, "Mode: ");
    show_text_value(48, LINE_H * 6, mode_name(view->mode), 8);
    ips200_show_string(0, LINE_H * 7, "Save: ");
    show_text_value(48, LINE_H * 7, save_state_name(view->save_state), 12);
    draw_home_status_labels();
    draw_home_status_values(view->encoder_count,
        view->imu_roll,
        view->imu_pitch,
        view->imu_yaw,
        view->target_yaw,
        view->yaw_error,
        view->vz,
        view->vzt,
        view->pose_x_cm,
        view->pose_y_cm,
        view->openart_frame_count);
    show_hint("K1/K2 Move  K3 Enter", "K4 Save  K4L Home");
}
```

Change `screen_draw_home_status()` to:

```c
void screen_draw_home_status(const screen_home_view_struct *view)
{
    begin_page(SCREEN_PAGE_HOME);
    draw_home_status_values(view->encoder_count,
        view->imu_roll,
        view->imu_pitch,
        view->imu_yaw,
        view->target_yaw,
        view->yaw_error,
        view->vz,
        view->vzt,
        view->pose_x_cm,
        view->pose_y_cm,
        view->openart_frame_count);
}
```

- [ ] **Step 7: Add Home view builder in `menu.c`**

Add this helper near `draw_home_page()`:

```c
static void build_home_view(screen_home_view_struct *view)
{
    const control_status_struct *status = get_control_status();
    const drive_pose_struct *pose = drive_pose_get();

    view->items = home_items;
    view->item_count = HOME_ITEM_COUNT;
    view->cursor = cursor_row;
    view->current_map = current_map;
    view->mode = run_mode;
    view->save_state = settings_get_save_state();
    view->encoder_count = status->wheel_feedback_count;
    view->imu_roll = status->current_roll;
    view->imu_pitch = status->current_pitch;
    view->imu_yaw = status->current_yaw;
    view->target_yaw = status->target_yaw;
    view->yaw_error = status->yaw_error;
    view->vz = status->vz;
    view->vzt = status->vzt;
    view->pose_x_cm = pose->x_cm;
    view->pose_y_cm = pose->y_cm;
    view->openart_frame_count = openart_uart_get_frame_count();
}
```

- [ ] **Step 8: Update Home page calls**

Replace `draw_home_page()` with:

```c
static void draw_home_page(void)
{
    screen_home_view_struct view;

    build_home_view(&view);
    screen_draw_home(&view);
}
```

Replace `refresh_home_page()` with:

```c
static void refresh_home_page(void)
{
    screen_home_view_struct view;

    build_home_view(&view);
    screen_draw_home_status(&view);
}
```

- [ ] **Step 9: Build**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected:

```text
0 Error(s), 0 Warning(s)
```

- [ ] **Step 10: Commit task 2**

Run:

```powershell
git add -- project/user/inc/screen.h project/user/src/screen.c project/user/src/menu.c
git commit -m "refactor: pass home screen view data"
```

## Task 3: Move Run Screen Business Reads Into Menu

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\menu.c`
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\screen.c`

- [ ] **Step 1: Add Run view builder**

In `menu.c`, add:

```c
static void build_run_view(screen_run_view_struct *view)
{
    const drive_pose_struct *pose = drive_pose_get();
    map_source_enum source_type = settings_get_source();

    view->current_map = current_map;
    view->mode = run_mode;
    view->source_type = source_type;
    view->save_state = settings_get_save_state();
    view->state_text = run_state;
    view->elapsed_ms = last_elapsed_ms;
    view->source = selected_map_source();
    view->executor_active = (EXEC_STATE_IDLE != executor_get_state()) ? 1u : 0u;
    view->executor_state = executor_get_state();
    view->executor_error = executor_get_error();
    view->current_step = executor_get_current_step();
    view->total_steps = executor_get_total_steps();
    view->current_box = executor_get_current_box();
    view->total_boxes = executor_get_total_boxes();
    view->pose_x_cm = pose->x_cm;
    view->pose_y_cm = pose->y_cm;
}
```

- [ ] **Step 2: Update Run page call**

Replace `draw_run_page()` with:

```c
static void draw_run_page(void)
{
    screen_run_view_struct view;

    build_run_view(&view);
    screen_draw_run_workbench(&view);
}
```

- [ ] **Step 3: Replace `draw_executor_status()` signature**

In `screen.c`, replace:

```c
static void draw_executor_status(void)
```

with:

```c
static void draw_executor_status(const screen_run_view_struct *view)
```

Inside that function, replace executor and pose getters with view fields:

```c
ips200_show_string(16, y0, executor_state_name_from_value(view->executor_state));
ips200_show_uint(124, y0, view->current_step, 3);
ips200_show_uint(156, y0, view->total_steps, 3);
ips200_show_uint(16, y1, view->current_box, 2);
ips200_show_uint(40, y1, view->total_boxes, 2);
ips200_show_float(76, y1, (double)view->pose_x_cm, 3, 1);
ips200_show_float(132, y1, (double)view->pose_y_cm, 3, 1);
```

If no value-based name helper exists, add this local helper in `screen.c`:

```c
static const char *executor_state_text(executor_state_enum state)
{
    switch(state)
    {
        case EXEC_STATE_IDLE:    return "IDLE";
        case EXEC_STATE_RUNNING: return "RUN";
        case EXEC_STATE_PAUSED:  return "PAUSE";
        case EXEC_STATE_DONE:    return "DONE";
        case EXEC_STATE_ERROR:   return "ERROR";
        default:                 return "?";
    }
}
```

Then use:

```c
ips200_show_string(16, y0, executor_state_text(view->executor_state));
```

- [ ] **Step 4: Replace error getter use**

Inside `draw_executor_status()`, replace:

```c
if(executor_get_state() == EXEC_STATE_ERROR)
{
    ...
    switch(executor_get_error())
```

with:

```c
if(EXEC_STATE_ERROR == view->executor_state)
{
    const char *err_msg = "E:?";
    switch(view->executor_error)
```

- [ ] **Step 5: Update `screen_draw_run_workbench()` signature and body**

Replace the function signature with:

```c
void screen_draw_run_workbench(const screen_run_view_struct *view)
```

Replace local field use:

```c
ips200_show_uint(56, LINE_H, view->current_map + 1, 2);
show_text_value(128, LINE_H, source_name(view->source_type), 8);
show_text_value(48, LINE_H * 2, mode_name(view->mode), 8);
show_text_value(48, LINE_H * 3, save_state_name(view->save_state), 12);
show_text_value(48, LINE_H * 4, view->state_text, 10);
ips200_show_uint(160, LINE_H * 4, view->elapsed_ms, 5);
```

Replace map selection with:

```c
if(0 != view->source)
{
    draw_color_map(view->source, 0, 0);
}
```

Replace executor status condition with:

```c
if(0 != view->executor_active)
{
    draw_executor_status(view);
}
else
{
    show_hint("K1/K2 Map  K3 Run", "K3L Mode  K4 Src");
}
```

- [ ] **Step 6: Remove business includes from `screen.c` if unused**

Remove these includes from `screen.c`:

```c
#include "openart_uart.h"
#include "drive_pose.h"
```

Keep `executor.h` only if `screen.h` still exposes executor enum types and `screen.c` needs those definitions through `screen.h`. Do not call `executor_get_*()` from `screen.c`.

- [ ] **Step 7: Verify no forbidden getter remains in `screen.c`**

Run:

```powershell
Select-String -Path 'C:\Users\ye\Desktop\rt1064\project\user\src\screen.c' -Pattern 'drive_pose_get|openart_map_get|executor_get_'
```

Expected: no matches.

- [ ] **Step 8: Build**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected:

```text
0 Error(s), 0 Warning(s)
```

- [ ] **Step 9: Commit task 3**

Run:

```powershell
git add -- project/user/src/menu.c project/user/src/screen.c
git commit -m "refactor: keep run screen drawing data-driven"
```

## Task 4: Convert Execute Page To View Struct

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\menu.c`
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\screen.c`
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\inc\screen.h`

- [ ] **Step 1: Add Execute view builder**

In `menu.c`, add:

```c
static void build_execute_view(screen_execute_view_struct *view)
{
    const drive_pose_struct *pose = drive_pose_get();

    view->current_map = current_map;
    view->source = last_or_selected_map_source();
    view->result = &last_result;
    view->current_step = executor_get_current_step();
    view->state = executor_get_state();
    view->error = executor_get_error();
    view->current_box = executor_get_current_box();
    view->total_boxes = executor_get_total_boxes();
    view->start_row = exec_start_row;
    view->start_col = exec_start_col;
    view->pose_x_cm = pose->x_cm;
    view->pose_y_cm = pose->y_cm;
}
```

- [ ] **Step 2: Update Execute page call**

Replace `draw_execute_page()` with:

```c
static void draw_execute_page(void)
{
    screen_execute_view_struct view;

    build_execute_view(&view);
    screen_draw_execute(&view);
}
```

- [ ] **Step 3: Update Execute screen signature**

In `screen.c`, replace:

```c
void screen_draw_execute(uint8 current_map,
                         const map_source_struct *source,
                         const solve_result_struct *result,
                         uint16 current_step,
                         executor_state_enum state,
                         uint16 current_box,
                         uint16 total_boxes,
                         uint8 start_row,
                         uint8 start_col)
```

with:

```c
void screen_draw_execute(const screen_execute_view_struct *view)
```

- [ ] **Step 4: Replace Execute local reads with view fields**

In `screen_draw_execute()`, remove:

```c
const drive_pose_struct *pose;
pose = drive_pose_get();
```

Replace all parameter reads with `view->` fields:

```c
ips200_show_uint(32, LINE_H, view->current_map + 1, 2);
if(0 == view->source)
...
draw_pose_map_from_start(view->source,
                         view->result,
                         view->current_step,
                         view->pose_x_cm,
                         view->pose_y_cm,
                         view->start_row,
                         view->start_col,
                         &pose_row,
                         &pose_col);
show_text_value(16, LINE_H * 2, executor_state_text(view->state), 7);
ips200_show_uint(104, LINE_H * 2, view->current_step, 3);
ips200_show_uint(136, LINE_H * 2, view->result->waypoint_count, 3);
ips200_show_uint(16, LINE_H * 3, view->current_box, 2);
ips200_show_uint(40, LINE_H * 3, view->total_boxes, 2);
```

For hint logic, use:

```c
if(EXEC_STATE_PAUSED == view->state)
{
    show_hint("K3 Resume", "K4 Stop");
}
else if((EXEC_STATE_ERROR == view->state) || (EXEC_STATE_DONE == view->state))
{
    show_hint("K4 Back", "K4L Home");
}
else
{
    show_hint("", "K4 Stop");
}
```

- [ ] **Step 5: Build**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected:

```text
0 Error(s), 0 Warning(s)
```

- [ ] **Step 6: Commit task 4**

Run:

```powershell
git add -- project/user/inc/screen.h project/user/src/menu.c project/user/src/screen.c
git commit -m "refactor: pass execute screen view data"
```

## Task 5: Unify Internal Map Render Input

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\screen.c`

- [ ] **Step 1: Add internal map render struct**

In `screen.c`, after `screen_page_enum`, add:

```c
typedef struct
{
    const map_source_struct *source;
    const solve_result_struct *result;
    uint16 step;
    uint8 use_pose;
    uint8 start_row;
    uint8 start_col;
    float pose_x_cm;
    float pose_y_cm;
    uint8 *pose_row;
    uint8 *pose_col;
} screen_map_render_struct;
```

- [ ] **Step 2: Add unified map renderer**

Add this function near existing map draw helpers:

```c
static void draw_map_render(const screen_map_render_struct *render)
{
    char grid[MAP_ROWS][MAP_COLS];
    uint16 car;
    uint16 pose_car;
    uint16 boxes[MAX_BOXES];
    uint16 targets[MAX_BOXES];
    uint8 box_count;
    uint8 target_count;
    uint8 row;
    uint8 col;
    int16 pose_row_value;
    int16 pose_col_value;
    uint16 cell;
    uint16 color;
    uint16 action_step;

    parse_source(render->source, grid, &car, boxes, &box_count, targets, &target_count);

    if((0 != render->result) && (0 != render->result->solved) && (0 < render->step))
    {
        if(0 != render->use_pose)
        {
            action_step = action_step_from_waypoint_step(render->result, render->step);
        }
        else
        {
            action_step = render->step;
        }
        apply_actions(&car, boxes, &box_count, targets, &target_count, render->result->actions, action_step);
    }

    if(0 != render->use_pose)
    {
        pose_col_value = (int16)render->start_col + round_cm_to_grid_delta(render->pose_x_cm);
        pose_row_value = (int16)render->start_row - round_cm_to_grid_delta(render->pose_y_cm);
        *render->pose_row = clamp_grid_index(pose_row_value, MAP_ROWS);
        *render->pose_col = clamp_grid_index(pose_col_value, MAP_COLS);
        pose_car = cell_index_local(*render->pose_row, *render->pose_col);
    }
    else
    {
        pose_car = car;
    }

    for(row = 0; row < MAP_ROWS; row++)
    {
        for(col = 0; col < MAP_COLS; col++)
        {
            cell = cell_index_local(row, col);
            if(0 != render->use_pose)
            {
                color = color_for_pose_cell(grid[row][col], cell, pose_car, boxes, box_count, targets, target_count);
            }
            else
            {
                color = color_for_cell(grid[row][col], cell, car, boxes, box_count, targets, target_count);
            }
            fill_rect((uint16)(MAP_X + col * CELL_SIZE),
                      (uint16)(MAP_Y + row * CELL_SIZE),
                      (CELL_SIZE - 1),
                      (CELL_SIZE - 1),
                      color);
        }
    }
}
```

- [ ] **Step 3: Replace `draw_color_map()` body**

Make `draw_color_map()` a thin wrapper:

```c
static void draw_color_map(const map_source_struct *source, const solve_result_struct *result, uint16 step)
{
    screen_map_render_struct render;

    render.source = source;
    render.result = result;
    render.step = step;
    render.use_pose = 0;
    render.start_row = 0;
    render.start_col = 0;
    render.pose_x_cm = 0.0f;
    render.pose_y_cm = 0.0f;
    render.pose_row = 0;
    render.pose_col = 0;
    draw_map_render(&render);
}
```

- [ ] **Step 4: Replace `draw_pose_map_from_start()` body**

Make `draw_pose_map_from_start()` a thin wrapper:

```c
static void draw_pose_map_from_start(const map_source_struct *source, const solve_result_struct *result, uint16 step, float pose_x_cm, float pose_y_cm, uint8 start_row, uint8 start_col, uint8 *pose_row, uint8 *pose_col)
{
    screen_map_render_struct render;

    render.source = source;
    render.result = result;
    render.step = step;
    render.use_pose = 1;
    render.start_row = start_row;
    render.start_col = start_col;
    render.pose_x_cm = pose_x_cm;
    render.pose_y_cm = pose_y_cm;
    render.pose_row = pose_row;
    render.pose_col = pose_col;
    draw_map_render(&render);
}
```

- [ ] **Step 5: Remove unused map helper if compiler reports it**

If `draw_pose_map()` becomes unused and Keil warns about it, delete the whole `draw_pose_map()` function. If Keil does not warn and it is still used by Debug, keep it.

- [ ] **Step 6: Build**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected:

```text
0 Error(s), 0 Warning(s)
```

- [ ] **Step 7: Commit task 5**

Run:

```powershell
git add -- project/user/src/screen.c
git commit -m "refactor: unify screen map rendering"
```

## Task 6: Clean Up Drawing Helpers and Obsolete Interfaces

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\inc\screen.h`
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\screen.c`
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\menu.c`

- [ ] **Step 1: Find obsolete screen functions**

Run:

```powershell
Select-String -Path 'C:\Users\ye\Desktop\rt1064\project\user\src\*.c','C:\Users\ye\Desktop\rt1064\project\user\inc\*.h' -Pattern 'screen_draw_run\(|screen_draw_demo\(|screen_draw_mode_select\('
```

Expected likely active uses:

```text
screen_draw_mode_select may be wrapped by screen_draw_mode_page
screen_draw_run may be unused
screen_draw_demo may be unused
```

- [ ] **Step 2: Delete unused declarations only after confirming no callers**

If `screen_draw_run()` has no caller, remove its declaration from `screen.h` and its implementation from `screen.c`.

If `screen_draw_demo()` has no caller, remove its declaration from `screen.h` and its implementation from `screen.c`.

Keep `screen_draw_mode_select()` if it has any caller or if `screen_draw_mode_page()` delegates to it.

- [ ] **Step 3: Add simple row helpers**

In `screen.c`, add near `show_text_value()`:

```c
static void show_uint_value(uint16 x, uint16 y, uint16 value, uint8 digits)
{
    ips200_show_uint(x, y, value, digits);
}

static void show_float_value(uint16 x, uint16 y, float value, uint8 int_digits, uint8 frac_digits)
{
    ips200_show_float(x, y, (double)value, int_digits, frac_digits);
}
```

Use these only where they make a repeated status row clearer. Do not force one-off layouts through these helpers.

- [ ] **Step 4: Build**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected:

```text
0 Error(s), 0 Warning(s)
```

- [ ] **Step 5: Commit task 6**

Run:

```powershell
git add -- project/user/inc/screen.h project/user/src/screen.c project/user/src/menu.c
git commit -m "refactor: clean screen drawing interfaces"
```

## Task 7: Final Verification

**Files:**
- Verify: `C:\Users\ye\Desktop\rt1064\project\mdk\build_log.txt`
- Verify: `C:\Users\ye\Desktop\rt1064\project\user\src\screen.c`
- Verify: `C:\Users\ye\Desktop\rt1064\project\user\src\menu.c`

- [ ] **Step 1: Search for forbidden screen business reads**

Run:

```powershell
Select-String -Path 'C:\Users\ye\Desktop\rt1064\project\user\src\screen.c' -Pattern 'drive_pose_get|openart_map_get|openart_uart_get|openart_last_rx_ms|executor_get_'
```

Expected: no matches.

- [ ] **Step 2: Search for page-specific enter logic in `enter_page()`**

Run:

```powershell
Select-String -Path 'C:\Users\ye\Desktop\rt1064\project\user\src\menu.c' -Pattern 'MENU_PAGE_RUN_MAP == page|MENU_PAGE_RUN_MODE == page|if\\(MENU_PAGE_'
```

Expected: no matches inside `enter_page()`. Matches in key handlers and page functions are acceptable.

- [ ] **Step 3: Full Keil build**

Run:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected in `project/mdk/build_log.txt`:

```text
0 Error(s), 0 Warning(s)
```

- [ ] **Step 4: Board smoke test checklist**

On board, verify:

```text
Home: K1/K2 move cursor, dynamic encoder/IMU/ART count refresh.
Run: K1/K2 switch map, K4 switches source, K3 starts execution or playback according to mode.
Run/Map: entering page starts from current map, K3 confirms, K4 cancels.
Run/Mode: entering page starts from current mode, K3 confirms, K4 cancels.
Playback: K1/K2 step, K3 play/pause, completed boxes disappear after target.
Execute: map uses solve snapshot, car position updates, boxes move and disappear after target.
ART Map: latest OpenART map still appears.
K4 long: returns Home from every page.
```

- [ ] **Step 5: Final commit if smoke test changes were needed**

If board smoke test required fixes, commit them:

```powershell
git add -- project/user/inc/screen.h project/user/src/menu.c project/user/src/screen.c
git commit -m "fix: stabilize menu page controller refactor"
```

If no fixes were needed, do not create an empty commit.

## Self-Review

- Spec coverage: lifecycle hooks are covered in Task 1; view structs and business data ownership are covered in Tasks 2-4; map rendering reuse is covered in Task 5; cleanup and verification are covered in Tasks 6-7.
- Red-flag scan: each task has explicit files, snippets, commands, and expected results.
- Type consistency: all new screen view structs are declared in `screen.h`, built in `menu.c`, and consumed by `screen.c`. The plan uses existing project enum types: `run_mode_enum`, `save_state_enum`, `map_source_enum`, `executor_state_enum`, and `executor_error_enum`.
