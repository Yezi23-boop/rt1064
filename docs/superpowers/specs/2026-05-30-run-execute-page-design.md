# Run/Step 执行页面设计文档

## 概述

为 Run 和 Step 模式添加独立的执行页面（MENU_PAGE_RUN_EXECUTE），模仿 Solve 回放页面的显示方式，支持实时更新小车位置到地图上。

## 需求总结

| 需求 | 决策 |
|------|------|
| 页面 ID | 复用 MENU_PAGE_RUN_EXECUTE (ID=13) |
| 显示方式 | 模仿 Solve 回放页面（地图 + 状态） |
| 小车位置 | 实时更新到地图上（物理坐标→网格坐标） |
| 刷新频率 | 500ms (2Hz) |
| 按键处理 | K3 恢复单步，K4 停止返回 |

## 架构设计

### 页面结构

```
MENU_PAGE_RUN (工作台)
    ├── MENU_PAGE_RUN_MAP (地图选择)
    ├── MENU_PAGE_RUN_MODE (模式选择)
    ├── MENU_PAGE_RUN_PLAYBACK (Solve 回放)
    └── MENU_PAGE_RUN_EXECUTE (Run/Step 执行) ← 本设计
```

### 数据流

```
Run 页面按 K3
    ↓
execute_current_selection()
    ↓
solve_current_map() → last_result.waypoints[]
    ↓
executor_start(...) → 启动执行器
    ↓
enter_page(MENU_PAGE_RUN_EXECUTE) → 进入执行页面
    ↓
refresh_execute_page() 每 500ms 调用
    ↓
drive_pose_get() → 获取小车物理坐标
    ↓
物理坐标 → 网格坐标 → 绘制到地图
```

## 页面注册

在 `menu_pages[]` 数组中添加：

```c
{ MENU_PAGE_RUN_EXECUTE, MENU_PAGE_RUN, 500, draw_execute_page, refresh_execute_page, handle_execute_event }
```

- `parent = MENU_PAGE_RUN`：按 K4 返回 Run 页面
- `refresh_ms = 500`：每 500ms 刷新一次（2Hz）
- `draw_full = draw_execute_page`：首次进入全量绘制
- `refresh_dynamic = refresh_execute_page`：定时刷新小车位置
- `on_key = handle_execute_event`：按键处理

## 页面流程

### 进入执行页面

```c
// execute_current_selection() 中
if(RUN_MODE_SOLVE == run_mode) {
    enter_page(MENU_PAGE_RUN_PLAYBACK);
} else {
    // Run 或 Step 模式
    executor_start(...);
    enter_page(MENU_PAGE_RUN_EXECUTE);  // ← 进入执行页面
}
```

### 按键处理

```c
static void handle_execute_event(menu_key_event_enum event)
{
    switch(event) {
        case K3_SHORT:
            // 单步模式下恢复执行
            if(executor_get_state() == EXEC_STATE_PAUSED) {
                executor_resume();
            }
            break;
            
        case K4_SHORT:
            // 停止执行，返回 Run 页面
            executor_stop();
            go_parent();
            break;
            
        default:
            break;
    }
}
```

### 页面退出条件

- executor 状态变为 DONE → 停留显示结果，按 K4 返回
- executor 状态变为 ERROR → 停留显示错误，按 K4 返回
- 用户按 K4 → 强制停止，返回 Run 页面

## 屏幕显示

### 显示布局

```
┌────────────────────────────────┐
│ Map: 03  Mode: Run  St:3/10    │  ← 顶部信息栏
├────────────────────────────────┤
│ ████████████████████████████   │
│ █ · · · · · · · · · · · · █   │
│ █ · · C · · · · · · · · · █   │  ← C = 小车实时位置（绿色）
│ █ · · · · B · · · · · · · █   │  ← B = 箱子（原有颜色）
│ █ · · · · · · · · · · · · █   │
│ █ · · · · · · · T · · · · █   │  ← T = 目标（原有颜色）
│ █ · · · · · · · · · · · · █   │  ← * = 当前目标格子（黄色高亮）
│ ████████████████████████████   │
├────────────────────────────────┤
│ S:Running  B:1/2               │  ← 状态信息
│ X:40.0 Y:-20.0                 │  ← 小车位置
└────────────────────────────────┘
```

### 绘制函数

```c
void screen_draw_execute(uint8 current_map,
                         const map_source_struct *source,
                         const solve_result_struct *result,
                         uint16 current_step,
                         executor_state_enum state,
                         uint16 current_box,
                         uint16 total_boxes,
                         uint8 start_row,      // 起点行号
                         uint8 start_col)      // 起点列号
```

### 物理坐标→网格坐标转换

```c
// 以起点为原点，反算网格坐标
int car_col = start_col + (int)(pose->x_cm / 20.0f + 0.5f);
int car_row = start_row - (int)(pose->y_cm / 20.0f + 0.5f);
```

- `x_cm / 20.0f`：物理X转列偏移（每格20cm）
- `+ 0.5f`：四舍五入
- `start_col +`：加上起始列
- `- (row)`：行增大=向下=物理Y负方向

## 文件变更

| 文件 | 操作 | 说明 |
|------|------|------|
| `project/user/src/menu.c` | 修改 | 注册页面、添加 draw/key/refresh 函数 |
| `project/user/src/screen.c` | 修改 | 添加 screen_draw_execute() |
| `project/user/inc/screen.h` | 修改 | 添加 screen_draw_execute() 声明 |

## 实现步骤

### 1. menu.c 修改

1. 在 `menu_pages[]` 中注册 `MENU_PAGE_RUN_EXECUTE`
2. 添加 `draw_execute_page()` 函数
3. 添加 `refresh_execute_page()` 函数
4. 添加 `handle_execute_event()` 函数
5. 修改 `execute_current_selection()` 中 Run/Step 的跳转目标

### 2. screen.c 修改

1. 添加 `screen_draw_execute()` 函数
2. 复用 playback 的地图绘制逻辑
3. 添加小车位置和目标位置的高亮绘制

### 3. screen.h 修改

1. 添加 `screen_draw_execute()` 函数声明

## 关键实现细节

### 地图绘制复用

Playback 页面的地图绘制逻辑提取为独立静态函数，供 playback 和 execute 共用：

```c
// screen.c 内部函数
static void draw_map_grid(const map_source_struct *source, int x, int y)
{
    // 绘制墙壁、箱子、目标
    // 从 screen_draw_playback 中提取
}
```

在 `screen_draw_playback()` 和 `screen_draw_execute()` 中都调用此函数。

### 小车位置标记

用 `ips200_fill_rect()` 在小车所在格子绘制绿色背景：

```c
static void draw_cell_highlight(uint8 row, uint8 col, uint16 color)
{
    int x = MAP_START_X + col * CELL_SIZE;
    int y = MAP_START_Y + row * CELL_SIZE;
    ips200_fill_rect(x, y, x + CELL_SIZE - 1, y + CELL_SIZE - 1, color);
}
```

### 刷新优化

每 500ms 刷新时，只更新变化的区域（小车位置和状态信息），避免全屏重绘：

```c
static void refresh_execute_page(void)
{
    // 只更新状态信息区域
    // 地图在 draw_execute_page 中全量绘制
    // 小车位置在 refresh 中增量更新
}
```

## 测试验证

1. 选择 Run 模式，按 K3 启动
2. 观察执行页面是否正确显示
3. 观察小车移动时地图上的标记是否实时更新
4. 测试单步模式：按 K3 走一格
5. 测试按 K4 停止并返回 Run 页面
6. 测试完成后自动显示结果
