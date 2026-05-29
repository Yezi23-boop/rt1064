# RT1064 Run Page Workbench Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 Run 根页改成“地图预览 + 选图 + 执行”的直接工作台，进入 Run 后立即看到地图并能直接执行当前地图。

**Architecture:** 保留现有页面码和按键事件队列，只改变 `MENU_PAGE_RUN` 的交互和绘制。Run 页不再使用 `Map / Mode / Execute` 列表光标，K1/K2 直接更新 `current_map` 并标记 Dirty，K3 短按执行，K3 长按进入 Mode 页面。

**Tech Stack:** Embedded C, SeekFree RT1064 library, Keil uVision, IPS200 screen.

---

## Task 1: Change Run Key Semantics

**Files:**
- Modify: `project/user/src/menu.c`

- [ ] **Step 1: Stop treating Run as a list page**

In `dispatch_key_event()`, route `MENU_PAGE_RUN` to a new `handle_run_event()` instead of `handle_list_event()`:

```c
case MENU_PAGE_HOME:
    handle_list_event(event);
    break;

case MENU_PAGE_RUN:
    handle_run_event(event);
    break;
```

- [ ] **Step 2: Add direct map selection on Run**

Add:

```c
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
```

Expected behavior:

- K1/K2 changes the visible map immediately and marks settings Dirty through `settings_set_runtime()`。
- K3 short executes the currently visible `current_map`。
- K3 long opens Mode page。
- K4 returns Home。

---

## Task 2: Draw Run As Map Workbench

**Files:**
- Modify: `project/user/inc/screen.h`
- Modify: `project/user/src/screen.c`
- Modify: `project/user/src/menu.c`

- [ ] **Step 1: Add workbench draw interface**

In `screen.h`, add:

```c
void screen_draw_run_workbench(uint8 current_map, run_mode_enum mode, save_state_enum save_state, const char *state, uint32 elapsed_ms);
```

- [ ] **Step 2: Implement workbench drawing**

In `screen.c`, add:

```c
void screen_draw_run_workbench(uint8 current_map, run_mode_enum mode, save_state_enum save_state, const char *state, uint32 elapsed_ms)
{
    char line[48];

    begin_page(SCREEN_PAGE_RUN);
    show_line(0, "Run");
    snprintf(line, sizeof(line), "Map : V%02d  Mode:%s", current_map + 1, mode_name(mode));
    show_line(LINE_H, line);
    snprintf(line, sizeof(line), "Save: %s  State:%s", save_state_name(save_state), state);
    show_line(LINE_H * 2u, line);
    snprintf(line, sizeof(line), "Last: %lu ms", (unsigned long)elapsed_ms);
    show_line(LINE_H * 3u, line);
    draw_color_map(map_get(current_map), 0, 0);
    show_hint("K1/K2 Map  K3 Run", "K3L Mode  K4 Home");
}
```

- [ ] **Step 3: Route Run drawing**

In `draw_current_page()`, replace `MENU_PAGE_RUN` drawing with:

```c
screen_draw_run_workbench(current_map, run_mode, settings_get_save_state(), run_state, last_elapsed_ms);
```

Expected behavior:

- Entering Run immediately shows the current map preview。
- Run page no longer draws `Map / Mode / Execute` list。

---

## Task 3: Keep Existing Subpages Working

**Files:**
- Modify: `project/user/src/menu.c`

- [ ] **Step 1: Preserve Mode page**

Keep `MENU_PAGE_RUN_MODE` and `handle_mode_event()` unchanged:

- K1/K2 changes candidate mode.
- K3 confirms mode and returns Run.
- K4 cancels and returns Run.

- [ ] **Step 2: Preserve Playback/Demo behavior**

Keep `execute_current_selection()` behavior:

- Browse: stays on Run and only displays the map.
- Solve/Run: solves current map and enters Playback.
- Demo: enters Demo page.

If Browse currently enters `MENU_PAGE_RUN_MAP`, change it to:

```c
clear_result_state();
run_state = "Browse";
enter_page(MENU_PAGE_RUN);
```

---

## Task 4: Verification

**Files:**
- Read: modified files
- Build: `project/mdk/rt1064.uvprojx`

- [ ] **Step 1: Static checks**

Run:

```powershell
Select-String -Path "project\user\src\menu.c" -Pattern "handle_run_event|MENU_KEY_EVENT_K3_LONG|screen_draw_run_workbench"
Select-String -Path "project\user\src\screen.c","project\user\inc\screen.h" -Pattern "screen_draw_run_workbench"
git diff -- libraries
```

Expected:

- `handle_run_event` exists.
- K3 long enters Mode page.
- Run workbench draw API exists in header/source.
- `libraries/` diff is empty.

- [ ] **Step 2: Keil build**

Run:

```powershell
& "D:\Keil_v5\UV4\UV4.exe" -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
Get-Content -Path "project\mdk\Objects\rt1064.build_log.htm" -Tail 50
```

Expected:

- Build log reports `0 Error(s), 0 Warning(s)`。
