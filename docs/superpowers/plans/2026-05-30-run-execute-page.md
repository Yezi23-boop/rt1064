# Run/Step 执行页面实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 Run 和 Step 模式添加独立的执行页面（MENU_PAGE_RUN_EXECUTE），模仿 Solve 回放页面的显示方式，支持实时更新小车位置到地图上。

**Architecture:** 复用已有的 MENU_PAGE_RUN_EXECUTE (ID=13)，在 menu.c 中注册页面并添加 draw/key/refresh 函数，在 screen.c 中添加绘制函数，复用 playback 的地图绘制逻辑。

**Tech Stack:** C (Keil MDK), NXP i.MX RT1064, IPS200 屏幕

---

## 文件结构

| 文件 | 操作 | 职责 |
|------|------|------|
| `project/user/src/menu.c` | 修改 | 注册页面、添加 draw/key/refresh 函数 |
| `project/user/src/screen.c` | 修改 | 添加 screen_draw_execute()、提取 draw_map_grid() |
| `project/user/inc/screen.h` | 修改 | 添加 screen_draw_execute() 声明 |

---

## Task 1: 提取地图绘制函数

**Files:**
- Modify: `project/user/src/screen.c`

**目的：** 将 playback 页面的地图绘制逻辑提取为独立函数，供 playback 和 execute 共用。

- [ ] **Step 1: 读取 screen_draw_playback() 实现**

读取 `project/user/src/screen.c`，找到 `screen_draw_playback()` 函数，理解地图绘制逻辑。

- [ ] **Step 2: 提取 draw_map_grid() 函数**

将地图绘制逻辑提取为独立静态函数：

```c
/**
 * @brief 绘制地图网格（墙壁、箱子、目标）
 * @param[in] source  地图数据源
 * @param[in] x       绘制起始X坐标
 * @param[in] y       绘制起始Y坐标
 */
static void draw_map_grid(const map_source_struct *source, int x, int y)
{
    uint8 row, col;
    char value;
    uint16 color;
    
    for(row = 0; row < MAP_ROWS; row++)
    {
        for(col = 0; col < MAP_COLS; col++)
        {
            value = source->rows[row][col];
            
            if('#' == value || 'X' == value)
            {
                color = COLOR_WALL;
            }
            else if('B' == value)
            {
                color = COLOR_BOX;
            }
            else if('T' == value)
            {
                color = COLOR_TARGET;
            }
            else if('C' == value)
            {
                color = COLOR_EMPTY;
            }
            else
            {
                color = COLOR_EMPTY;
            }
            
            ips200_fill_rect(x + col * CELL_SIZE, 
                            y + row * CELL_SIZE,
                            x + (col + 1) * CELL_SIZE - 1,
                            y + (row + 1) * CELL_SIZE - 1,
                            color);
        }
    }
}
```

- [ ] **Step 3: 修改 screen_draw_playback() 使用 draw_map_grid()**

将 `screen_draw_playback()` 中的地图绘制代码替换为调用 `draw_map_grid()`。

- [ ] **Step 4: 验证编译**

运行 Keil 编译，确认无语法错误。

---

## Task 2: 添加 screen_draw_execute() 函数

**Files:**
- Modify: `project/user/src/screen.c`
- Modify: `project/user/inc/screen.h`

- [ ] **Step 1: 在 screen.c 中添加 screen_draw_execute()**

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
{
    char buf[32];
    int car_col, car_row;
    const drive_pose_struct *pose;
    
    /* 1. 绘制顶部信息栏 */
    ips200_show_string(0, 0, "Map:");
    ips200_show_uint(32, 0, current_map, 2);
    ips200_show_string(56, 0, "Mode:");
    ips200_show_string(88, 0, (RUN_MODE_STEP == settings_get_mode()) ? "Step" : "Run");
    
    /* 2. 绘制原始地图 */
    draw_map_grid(source, MAP_START_X, MAP_START_Y);
    
    /* 3. 计算小车网格坐标 */
    pose = drive_pose_get();
    car_col = start_col + (int)(pose->x_cm / 20.0f + 0.5f);
    car_row = start_row - (int)(pose->y_cm / 20.0f + 0.5f);
    
    /* 边界保护 */
    if(car_col < 0) car_col = 0;
    if(car_col >= MAP_COLS) car_col = MAP_COLS - 1;
    if(car_row < 0) car_row = 0;
    if(car_row >= MAP_ROWS) car_row = MAP_ROWS - 1;
    
    /* 4. 标记小车位置（绿色） */
    draw_cell_highlight(car_row, car_col, COLOR_CAR);
    
    /* 5. 标记当前目标位置（黄色） */
    if(current_step < result->waypoint_count)
    {
        uint8 target_row = result->waypoints[current_step].row;
        uint8 target_col = result->waypoints[current_step].col;
        draw_cell_highlight(target_row, target_col, COLOR_TARGET_HIGHLIGHT);
    }
    
    /* 6. 绘制状态信息 */
    ips200_show_string(0, STATUS_Y, "S:");
    ips200_show_string(16, STATUS_Y, executor_state_name());
    ips200_show_string(80, STATUS_Y, "B:");
    ips200_show_uint(96, STATUS_Y, current_box, 1);
    ips200_show_string(104, STATUS_Y, "/");
    ips200_show_uint(112, STATUS_Y, total_boxes, 1);
    
    /* 7. 绘制小车位置 */
    ips200_show_string(0, STATUS_Y + 16, "X:");
    ips200_show_float(16, STATUS_Y + 16, (double)pose->x_cm, 3, 1);
    ips200_show_string(80, STATUS_Y + 16, "Y:");
    ips200_show_float(96, STATUS_Y + 16, (double)pose->y_cm, 3, 1);
}
```

- [ ] **Step 2: 在 screen.c 中添加 draw_cell_highlight() 辅助函数**

```c
/**
 * @brief 在指定格子绘制高亮背景
 * @param[in] row   行号
 * @param[in] col   列号
 * @param[in] color 颜色
 */
static void draw_cell_highlight(uint8 row, uint8 col, uint16 color)
{
    int x = MAP_START_X + col * CELL_SIZE;
    int y = MAP_START_Y + row * CELL_SIZE;
    ips200_fill_rect(x, y, x + CELL_SIZE - 1, y + CELL_SIZE - 1, color);
}
```

- [ ] **Step 3: 在 screen.h 中添加函数声明**

在 `project/user/inc/screen.h` 中添加：

```c
/**
 * @brief 绘制 Run/Step 执行页面
 * @param[in] current_map   当前地图编号
 * @param[in] source        地图数据源
 * @param[in] result        求解结果
 * @param[in] current_step  当前步骤
 * @param[in] state         执行器状态
 * @param[in] current_box   当前箱子编号
 * @param[in] total_boxes   总箱子数
 * @param[in] start_row     起点行号
 * @param[in] start_col     起点列号
 */
void screen_draw_execute(uint8 current_map,
                         const map_source_struct *source,
                         const solve_result_struct *result,
                         uint16 current_step,
                         executor_state_enum state,
                         uint16 current_box,
                         uint16 total_boxes,
                         uint8 start_row,
                         uint8 start_col);
```

- [ ] **Step 4: 验证编译**

运行 Keil 编译，确认无语法错误。

---

## Task 3: 在 menu.c 注册执行页面

**Files:**
- Modify: `project/user/src/menu.c`

- [ ] **Step 1: 添加静态变量保存起点坐标**

在 `menu.c` 顶部添加：

```c
/* 执行页面起点坐标 */
static uint8 exec_start_row = 0;
static uint8 exec_start_col = 0;
```

- [ ] **Step 2: 在 menu_pages[] 中注册 MENU_PAGE_RUN_EXECUTE**

找到 `menu_pages[]` 数组，在 `MENU_PAGE_RUN_PLAYBACK` 之后添加：

```c
{ MENU_PAGE_RUN_EXECUTE, MENU_PAGE_RUN, 500, draw_execute_page, refresh_execute_page, handle_execute_event },
```

- [ ] **Step 3: 添加 draw_execute_page() 函数**

```c
static void draw_execute_page(void)
{
    const map_source_struct *source;
    
    if(MAP_SOURCE_ART == settings_get_source())
    {
        source = openart_map_get();
    }
    else
    {
        source = map_get(current_map);
    }
    
    screen_draw_execute(current_map,
                        source,
                        &last_result,
                        executor_get_current_step(),
                        executor_get_state(),
                        executor_get_current_box(),
                        executor_get_total_boxes(),
                        exec_start_row,
                        exec_start_col);
}
```

- [ ] **Step 4: 添加 refresh_execute_page() 函数**

```c
static void refresh_execute_page(void)
{
    draw_execute_page();
    
    /* 如果执行完成或出错，可以考虑自动返回 */
    /* 暂时不做，让用户按 K4 手动返回 */
}
```

- [ ] **Step 5: 添加 handle_execute_event() 函数**

```c
static void handle_execute_event(menu_key_event_enum event)
{
    switch(event)
    {
        case K3_SHORT:
            /* 单步模式下恢复执行 */
            if(EXEC_STATE_PAUSED == executor_get_state())
            {
                executor_resume();
            }
            break;
            
        case K4_SHORT:
            /* 停止执行，返回 Run 页面 */
            executor_stop();
            go_parent();
            break;
            
        default:
            break;
    }
}
```

- [ ] **Step 6: 修改 execute_current_selection() 保存起点并跳转执行页面**

找到 `execute_current_selection()` 函数，修改 Run/Step 模式的分支：

```c
else
{
    const map_source_struct *source;
    uint8 single_step = (RUN_MODE_STEP == run_mode) ? 1u : 0u;

    if(MAP_SOURCE_ART == settings_get_source())
    {
        source = openart_map_get();
    }
    else
    {
        source = map_get(current_map);
    }

    if(0 != source)
    {
        find_car_in_source(source, &exec_start_row, &exec_start_col);
    }

    executor_start(last_result.waypoints, last_result.waypoint_count,
                   exec_start_row, exec_start_col, single_step);
    enter_page(MENU_PAGE_RUN_EXECUTE);  /* 进入执行页面 */
}
```

- [ ] **Step 7: 验证编译**

运行 Keil 编译，确认 0 Error, 0 Warning。

---

## Task 4: 测试验证

**Files:**
- 无新增文件

- [ ] **Step 1: 测试 Run 模式执行页面**

1. 选择离线地图（K1/K2）
2. 长按 K3 选择 Run 模式
3. 按 K3 启动
4. 观察：
   - 是否进入执行页面（不是停留在 Run 工作台）
   - 地图是否正确显示
   - 小车位置是否实时更新（绿色标记）
   - 目标位置是否高亮（黄色标记）
   - 状态信息是否正确显示

- [ ] **Step 2: 测试 Step 模式执行页面**

1. 长按 K3 选择 Step 模式
2. 按 K3 启动
3. 观察：
   - 是否进入执行页面
   - 是否立即暂停（显示 "Paused"）
   - 按 K3 后是否恢复执行
   - 走完一格后是否再次暂停

- [ ] **Step 3: 测试按 K4 返回**

1. 执行过程中按 K4
2. 观察：
   - 小车是否停止
   - 是否返回 Run 工作台页面

- [ ] **Step 4: 测试完成后显示**

1. 等待执行完成
2. 观察：
   - 是否显示 "Done" 状态
   - 按 K4 是否返回 Run 工作台页面

---

## 完成

所有任务完成后，Run/Step 模式将拥有独立的执行页面：

1. Run/Step 模式按 K3 启动后进入执行页面
2. 执行页面显示地图 + 小车实时位置 + 状态信息
3. 单步模式下按 K3 恢复执行
4. 按 K4 停止并返回 Run 工作台
