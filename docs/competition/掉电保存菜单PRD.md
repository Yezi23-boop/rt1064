# PRD: RT1064 Push Box 掉电保存菜单系统

## Problem Statement

当前 Push Box 固件已经具备离线地图、BFS 求解和基础 IPS200 显示能力，但执行入口仍偏临时验证：选图、选模式、运行算法、查看结果、掉电恢复和后续调试入口没有形成稳定菜单系统。

用户需要一个可以在车模上直接操作的四按键菜单：开机进入 Home，进入 Run 后选择地图和运行模式，按键启动路径算法，在屏幕上用色块显示地图和最短路径回放，并能把已确认的地图编号和模式保存到 Flash，断电后恢复。该系统必须保持轻量、非阻塞、易扩展，并且不能把调试功能、显示逻辑、Flash 保存和 BFS 执行继续挤在一个应用文件里。

## Solution

实现一个基于 IPS200 和 KEY_1~KEY_4 的 Push Box 菜单系统。

第一版菜单包含 Home、Run、Mode Select、Playback、Debug、Info 六类页面。Home 是开机主页面；Run 是执行入口；Mode Select 只负责候选模式确认；Playback 用色块显示算法最短路径；Debug 是预留调试入口；Info 显示地图数量、Flash 状态和保存状态。

菜单使用静态表和 enum 页面状态机，不使用链表、动态菜单树或堆内存。Home 子页面后续通过静态表扩展。按键统一转换成一次性短按/长按事件，页面只消费 key event，不直接消费底层按键状态。

Flash 第一版只保存已确认的 current_map 和 run_mode，不保存地图本体、BFS 结果、候选值、临时页面状态或回放进度。保存记录使用 magic、version、checksum 做基本完整性保护，固定写入保留 Flash 区域的单页。

## User Stories

1. As a driver, I want the board to enter Home after boot, so that I can start from a predictable menu state.
2. As a driver, I want Home to show Run, Debug, and Info entries, so that execution, future debugging, and status viewing are separated.
3. As a driver, I want K1/K2 on Home to move the selected entry, so that I can navigate with the four-key board.
4. As a driver, I want K3 on Home to enter the selected page, so that page entry behavior is consistent.
5. As a driver, I want K4 short press on Home to save the confirmed map and mode, so that my normal competition setup can survive power loss.
6. As a driver, I want K4 long press on every page to return Home, so that there is always a universal escape action.
7. As a driver, I want Run to show the confirmed map and candidate map separately, so that I can see whether a map change still needs confirmation.
8. As a driver, I want K1/K2 on Run to change only the candidate map, so that accidental presses do not immediately change the execution target.
9. As a driver, I want K3 on Run to confirm the candidate map when it differs from the confirmed map, so that map changes require an explicit commit.
10. As a driver, I want K3 on Run to execute only when the candidate map already matches the confirmed map, so that map confirmation and algorithm execution do not get confused.
11. As a driver, I want leaving Run to discard unconfirmed candidate maps, so that stale candidate selections do not persist unexpectedly.
12. As a driver, I want K2 long press on Run to enter Mode Select, so that mode changes are available without cluttering Run.
13. As a driver, I want Mode Select to use a candidate mode, so that mode browsing does not immediately change the confirmed run mode.
14. As a driver, I want K3 on Mode Select to confirm the candidate mode, so that the confirmed run mode changes only by explicit action.
15. As a driver, I want K4 on Mode Select to cancel and return, so that I can back out without changing mode.
16. As a driver, I want Browse mode to show the current map without running BFS, so that I can inspect maps quickly.
17. As a driver, I want Solve mode to run BFS and enter Playback, so that I can verify the computed shortest path.
18. As a driver, I want Run mode to behave like Solve in the first version, so that the UI can be validated before physical chassis execution is connected.
19. As a driver, I want Demo mode to validate all offline maps in order, so that I can check algorithm coverage quickly.
20. As a driver, I want Demo to process at most one map per menu poll, so that K4 can stop Demo between maps.
21. As a driver, I want BFS total elapsed time shown on screen, so that I can compare traversal performance between maps.
22. As a driver, I want Playback to start paused at step 0, so that I can inspect the initial state before animation.
23. As a driver, I want K1/K2 in Playback to step backward and forward, so that I can manually inspect the computed path.
24. As a driver, I want K3 in Playback to play or pause the path, so that I can watch the shortest path automatically.
25. As a driver, I want Playback to advance at a fixed interval, so that the animation is simple and predictable.
26. As a driver, I want the map displayed as color blocks, so that walls, boxes, targets, and the car are readable on IPS200.
27. As a driver, I want only the car position shown in the first version, so that heading and real motion execution do not block algorithm validation.
28. As a driver, I want failure cases to enter Playback(Fail), so that success and failure share one result page layout.
29. As a driver, I want Info to show map count, Flash state, save state, and version, so that I can diagnose saved configuration state without running the algorithm.
30. As a driver, I want Debug reserved under Home, so that future wheel tests, encoder direction checks, and PID views have a clear location.
31. As a developer, I want menu state separate from screen drawing, so that UI rendering can change without breaking navigation.
32. As a developer, I want settings persistence separate from menu behavior, so that Flash details are isolated behind a small interface.
33. As a developer, I want a unified millisecond timebase, so that BFS timing and Playback timing do not use blocking delays.
34. As a developer, I want key events wrapped once, so that long press does not trigger multiple page actions.
35. As a developer, I want app to remain glue code, so that main stays lightweight and modules are testable in isolation.
36. As a developer, I want Keil project and linker configuration included in the work, so that the new files build and the Flash config page is not overwritten by program image.

## Implementation Decisions

- Build or modify five deep modules: settings, timebase, screen, menu, and app glue.
- Settings owns default map/mode values, Flash initialization, Flash read/write, checksum validation, and save state.
- Settings only persists confirmed current_map and run_mode.
- Settings does not persist candidate_map, candidate_mode, BFS results, actions, cursor positions, page state, Demo results, or Playback progress.
- The Flash record contains magic, version, checksum, current_map, and run_mode.
- Checksum is a lightweight uint16 byte-sum style integrity check. It is used only to reject empty, half-written, or obviously damaged records.
- No sequence number and no double-page rotation in the first version.
- The saved configuration uses one fixed Flash page inside a reserved Flash sector.
- Business code uses the direct Flash page read/write functions with local uint32 buffers.
- Business code does not use the driver’s global Flash union buffer helper for save success decisions.
- Write success is accepted only after writing, reading back, and validating magic, version, checksum, and field values.
- Saving always writes when Home/K4 short press is used. First version does not skip writes when content is unchanged.
- Timebase exposes only initialization and millisecond read. It uses the GPT millisecond timer and must be non-blocking.
- Screen owns IPS200 initialization, text rendering, color-block map rendering, and Playback visual rendering.
- Screen does not read keys, run BFS, mutate menu state, or access Flash.
- Menu owns page state, Home static entries, key event adaptation, current/candidate map behavior, run/candidate mode behavior, solver invocation, Demo progress, and Playback progress.
- Menu uses enum page state plus static tables. It does not use linked lists, heap allocation, or runtime menu node creation.
- Home entries are table-driven so future pages can be added by adding a page enum, a static table item, and page handlers.
- Key handling converts raw driver states into one-shot key events.
- Long press produces one event and does not produce an extra short press on release.
- Run page owns candidate_map. Leaving Run resets candidate_map to current_map.
- Mode Select owns candidate_mode. Canceling or leaving resets candidate_mode to run_mode.
- K4 long press always returns Home.
- Demo mode validates all offline maps in order but only one map per menu poll.
- Solve and Run mode are equivalent in the first version. Physical chassis execution remains a later integration.
- Playback uses solver output actions and does not rerun BFS during animation.
- Playback redraws the full 12x16 color-block map area per step in the first version.
- App initializes and polls the subsystem modules. Main remains limited to board init, drive control init, app init, and app poll.

## Testing Decisions

- Tests and checks should focus on externally visible behavior: page transitions, saved state, restored state, map/mode confirmation, Demo progress, and Playback rendering state.
- Do not test private helper implementation details such as exact switch layout or static table variable names.
- Settings is the highest-value AFK test target. It can be tested with valid record, empty record, bad magic, bad version, bad checksum, out-of-range map, out-of-range mode, write failure, and write-then-readback validation cases.
- Timebase can be validated by build and simple monotonic runtime observation. It should not be mocked into menu tests unless a host-test harness is added later.
- Menu state can be tested as a pure event reducer if the implementation keeps rendering and Flash behind small interfaces.
- Screen rendering is mainly HITL in the first version because IPS200 output needs visual confirmation.
- Solver behavior is existing functionality; this PRD only verifies that menu calls it correctly and displays results.
- Demo acceptance requires checking that K4 can stop between maps.
- Flash persistence acceptance requires board testing: save, power cycle, restore map/mode.
- Build acceptance requires a full Keil rebuild with no errors and no warnings.

## Acceptance Criteria

- Firmware boots to Home.
- Home shows Run, Debug, and Info.
- Home K4 short press saves confirmed map and mode.
- K4 long press returns Home from every menu page.
- Run K1/K2 changes candidate map only.
- Run K3 confirms candidate map when candidate differs from current.
- Run K3 executes current map and current mode when candidate equals current.
- Mode Select K1/K2 changes candidate mode only.
- Mode Select K3 confirms run_mode and marks state Dirty.
- Mode Select K4 cancels candidate mode.
- Browse mode displays the current map without running BFS.
- Solve mode runs BFS and enters Playback.
- Run mode behaves like Solve in the first version.
- Demo mode validates offline maps sequentially and can be stopped with K4 between maps.
- Playback shows current step, action, elapsed BFS time, state, car, boxes, targets, walls, and empty cells.
- Playback K1/K2 single-step without starting automatic playback.
- Playback K3 toggles play/pause.
- Playback completion stops at the final step and shows Done.
- BFS failure enters Playback(Fail) and shows the initial map, reason, and elapsed time.
- Info shows map count, Flash state, save state, and version.
- Debug shows Reserved and returns with K4.
- Flash empty or invalid data falls back to default map 0 and Solve mode.
- Flash save writes only confirmed current_map and run_mode.
- Flash save readback validation must pass before showing Saved.
- The program image does not occupy the reserved Flash page used for settings.
- The Keil project includes all new source files.
- Full Keil rebuild completes with 0 errors and 0 warnings.

## Out of Scope

- Saving map body data to Flash.
- Editing maps on the board.
- Saving BFS results, action strings, Demo results, cursor state, Playback step, or page state.
- Runtime menu node insertion or linked-list menu structures.
- Dynamic allocation.
- Complex CRC32, wear leveling, sequence counters, or double-page rotation.
- Real chassis motion execution from Run mode.
- PID tuning UI.
- Single-wheel test UI implementation.
- Encoder direction calibration UI implementation.
- Incremental multi-frame BFS for a single map.
- Dirty-rectangle rendering optimization.
- Car heading rendering.
- Changing IPS200 orientation from portrait to landscape.

## Further Notes

The existing framework document remains the lower-level source of truth for screen layout, color choices, Flash driver constraints, key semantics, and file split details. This PRD defines the executable product boundary and acceptance criteria for an implementation agent.

