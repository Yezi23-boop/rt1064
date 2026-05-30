# 菜单统一页面控制器设计

## 背景

当前菜单已经有 `menu_pages[]` 页面表，能统一保存页面 ID、父页面、刷新周期、完整绘制、动态刷新和按键处理函数。这是一个好的基础。

随着 OpenART 地图页、Run 工作台、Playback、Execute 执行页加入，菜单和屏幕层的边界开始变松：

- `screen.c` 直接包含并读取 `executor`、`openart_uart`、`drive_pose` 等业务模块。
- `screen.h` 部分接口参数很长，新增字段时需要同步改很多调用点。
- Run、Playback、Execute 都在处理“地图源、求解结果、动作步、小车位置、箱子状态”这类相似显示输入。
- `enter_page()` 中混有少量具体页面逻辑，例如进入 Run/Map、Run/Mode 时同步候选值。

本次目标是做统一页面控制器型整理，使菜单调度、业务数据聚合、屏幕绘制职责更清楚，同时尽量不改变现有用户体验。

## 目标

1. 统一页面生命周期：页面进入、退出、绘制、刷新、按键处理都由页面表声明。
2. 收紧职责边界：
   - `menu.c` 负责页面状态机、按键分发、业务数据读取、组装显示数据。
   - `screen.c` 只负责绘制，不主动读取业务模块状态。
3. 减少长参数：用小型 view struct 传递页面显示数据。
4. 复用地图显示输入：普通地图、回放地图、执行地图尽量共享同一套内部绘制路径。
5. 保持现有页面层级和主要按键语义，允许小幅修正明显不一致的提示文字、清屏策略和重复接口。

## 非目标

- 不重写求解器、执行器、OpenART UART 协议、地图格式。
- 不重新设计屏幕 UI 风格。
- 不把菜单做成复杂通用 UI 框架。
- 不引入动态分配、复杂对象系统或库依赖。

## 页面控制器结构

将当前页面表从：

```c
id, parent, refresh_ms, draw_full, refresh_dynamic, on_key
```

扩展为：

```c
id, parent, refresh_ms, on_enter, on_exit, on_draw, on_refresh, on_key
```

建议类型：

```c
typedef void (*menu_page_lifecycle_func)(void);
typedef void (*menu_page_key_func)(menu_key_event_enum event);

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
} menu_page_controller_struct;
```

调度规则：

```text
enter_page(new_page)
  -> current.on_exit()
  -> save_page_cursor()
  -> current_page = new_page
  -> restore_page_cursor()
  -> new.on_enter()
  -> mark_redraw()

menu_poll()
  -> dispatch_key_event()
  -> playback_tick()
  -> if need_redraw: current.on_draw()
  -> current.on_refresh() by refresh_ms
```

页面自己的进入准备放进 `on_enter()`，例如：

- Run/Map：`candidate_map = current_map`
- Run/Mode：`candidate_mode = run_mode`
- Execute：准备执行起点、显示状态快照

这样 `enter_page()` 不再累积具体页面判断。

## 显示数据结构

`screen.h` 增加小型 view struct，替代过长参数列表。

建议第一批只整理最容易膨胀的页面：

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

具体字段可以在实现时按实际需要微调，但原则是：业务模块读取都发生在 `menu.c`，`screen.c` 不调用 getter。

## 地图绘制复用

保留 `screen.c` 内部的地图解析和绘制函数，但把输入统一成“源地图 + 可选求解结果 + 当前步 + 可选位姿”。

建议内部模型：

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
} screen_map_render_struct;
```

普通预览、Playback、Execute 都可基于该结构绘制：

- 普通预览：只有 `source`
- Playback：`source + result + step`
- Execute：`source + result + current_step + pose + start_row/start_col`

箱子推入目标后消失的逻辑继续由 `apply_actions()` 处理 `|`，显示层不额外保留已完成箱子。

## 按键和刷新约定

保留当前主要语义：

- K1/K2：列表移动、地图切换或回放步进。
- K3：确认、进入、开始执行、暂停/继续。
- K4 短按：返回父页面或当前页特定操作。
- K4 长按：统一回 Home。

允许的小幅修正：

- 页面提示文案与实际行为保持一致。
- 文本状态使用固定区域清除，避免短文本覆盖长文本后残留。
- 动态刷新只刷新必要内容，避免不必要整屏刷新。

## 实施分步

### 第一步：页面控制器骨架

- 扩展页面表字段。
- 增加默认空生命周期函数或空指针判断。
- 把 `enter_page()` 内具体页面判断迁移到对应 `on_enter()`。
- 保持现有 draw/refresh/key 函数主体基本不变。

### 第二步：屏幕层业务依赖下沉

- 从 `screen.c` 移除 `openart_uart.h`、`executor.h`、`drive_pose.h` 等业务 include。
- `menu.c` 读取 OpenART、executor、pose 状态后传给 `screen.c`。
- `screen.c` 只使用入参绘制。

### 第三步：view struct 化接口

- 优先整理 `screen_draw_execute()` 和 `screen_draw_run_workbench()`。
- 再整理 `screen_draw_home()`。
- 保留简单页面接口，例如 Info、Mode Select 可以暂时不改。

### 第四步：地图绘制输入统一

- 提取内部 `screen_map_render_struct`。
- 让预览、Playback、Execute 复用同一条绘制路径。
- 删除过时或重复的绘制函数。

## 验证计划

每一步后都运行 Keil 编译：

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

期望：

```text
0 Error(s), 0 Warning(s)
```

板上验证：

- Home 页面菜单移动、动态状态刷新正常。
- Run 页面能切换地图源、进入执行、切换模式。
- Run/Map、Run/Mode 进入时候选值正确同步。
- Playback 回放步进和自动播放正常。
- Execute 页面地图、小车位置、箱子消失规则正常。
- ART Map 页面仍显示实时 OpenART 地图。
- K4 短按和 K4 长按行为与提示一致。

## 风险和控制

- 风险：页面生命周期重构可能导致进入页面时状态未初始化。
  - 控制：先只迁移 `candidate_map/candidate_mode` 等已知进入逻辑，每一步编译并板上验证。
- 风险：view struct 改接口时漏字段。
  - 控制：先改 Execute 和 Run 两个重点页面，编译器会暴露签名遗漏。
- 风险：`screen.c` 去业务依赖后某些状态显示缺字段。
  - 控制：所有 getter 调用集中搬到 `menu.c`，字段缺失时补到 view struct。
- 风险：过度抽象导致比赛现场不好改。
  - 控制：只使用 C 结构体和函数指针，不引入动态分配，不做通用控件系统。

## 成功标准

- `screen.c` 不再主动读取 executor、OpenART、drive pose 业务状态。
- 页面进入逻辑不再散落在 `enter_page()` 的具体页面判断中。
- Execute/Run/Home 的长参数接口明显减少。
- 现有菜单功能和按键体验保持基本一致。
- Keil 编译通过，板上关键页面行为正常。
