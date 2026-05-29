# 菜单页面和变量扩展指南

本文给用户和后续 agent 使用，说明如何在当前 RT1064 菜单结构中添加新页面和动态变量。

当前菜单采用：

```text
menu.c   负责页面状态、按键分发、刷新节拍、业务数据获取
screen.c 负责 IPS200 绘制、地图绘制、变量显示、文本残留处理
screen.h 只声明 screen.c 对 menu.c 暴露的绘制接口
```

不要修改 `libraries/` 下的逐飞库文件。菜单按键只通过 `menu_key_read_event()` 消费事件，不在 `menu.c` 里直接调用 `key_scanner()`。

## 刷新规则

新增页面或变量时先按这个规则判断：

```text
静态标签：只在完整绘制函数里直接 ips200_show_string()
数字变量：直接 ips200_show_int/uint/float() 固定宽度覆盖
文本变量：用 show_text_value() 清一小块值区域再写
地图/大块图形：只在页面进入、地图切换、Playback 步进时重画
光标：只擦旧 >，画新 >
```

不要为了普通标签和数字清整行。当前 `screen.c` 不保留整行清除函数，也不应重新加回来。

数字一般能覆盖旧值，例如：

```c
ips200_show_uint(56, y, value, 3);
ips200_show_int(56, y, value, 5);
ips200_show_float(56, y, (double)value, 3, 2);
```

文本长度可能变短，必须清值区域，例如 `"Running"` 变 `"OK"`：

```c
ips200_show_string(0, y, "State:");
show_text_value(56, y, state, 10);
```

`show_text_value()` 是 `screen.c` 内部辅助函数，只在 `screen.c` 里用。

## 添加一个一级页面

下面以新增 `Sensor` 页面为例。一级页面是指从 Home 直接进入的页面。

### 1. 在 `menu.c` 增加页面码

在 `menu_page_id_enum` 中增加页面 ID：

```c
typedef enum
{
    MENU_PAGE_HOME         = 0,
    MENU_PAGE_RUN          = 1,
    MENU_PAGE_DEBUG        = 2,
    MENU_PAGE_INFO         = 3,
    MENU_PAGE_SENSOR       = 4,
    MENU_PAGE_RUN_MAP      = 11,
    MENU_PAGE_RUN_MODE     = 12,
    MENU_PAGE_RUN_EXECUTE  = 13,
    MENU_PAGE_RUN_PLAYBACK = 14,
    MENU_PAGE_RUN_DEMO     = 15,
} menu_page_id_enum;
```

一级页面数量增加后，更新：

```c
#define HOME_ITEM_COUNT (4)
#define ROOT_PAGE_COUNT (5)
```

`ROOT_PAGE_COUNT` 要大于最大一级页面编号。例如新增 `MENU_PAGE_SENSOR = 4`，根页面数量至少是 5。

### 2. 在 Home 菜单中加入入口

同步更新 `home_items` 和 `home_item_pages`，两者顺序必须一致：

```c
static const char *const home_items[HOME_ITEM_COUNT] =
{
    "Run",
    "Debug",
    "Info",
    "Sensor",
};

static const menu_page_id_enum home_item_pages[HOME_ITEM_COUNT] =
{
    MENU_PAGE_RUN,
    MENU_PAGE_DEBUG,
    MENU_PAGE_INFO,
    MENU_PAGE_SENSOR,
};
```

### 3. 在 `menu.c` 增加函数声明

在静态函数声明区加入：

```c
static void draw_sensor_page(void);
static void refresh_sensor_page(void);
static void handle_sensor_key(menu_key_event_enum event);
```

如果页面没有动态数据，可以不写 `refresh_sensor_page()`，页面表里填 `0`。

### 4. 在 `menu_pages[]` 增加页面表项

有动态刷新时：

```c
{ MENU_PAGE_SENSOR, MENU_PAGE_HOME, 500, draw_sensor_page, refresh_sensor_page, handle_sensor_key },
```

没有动态刷新时：

```c
{ MENU_PAGE_SENSOR, MENU_PAGE_HOME, 0, draw_sensor_page, 0, handle_sensor_key },
```

字段含义：

```text
id              当前页面 ID
parent          K4 返回的父页面
refresh_ms      动态刷新周期，0 表示不自动刷新
draw_full       页面进入或强制重画时调用
refresh_dynamic 停留页面时周期刷新，只刷新变量
on_key          当前页面按键处理
```

### 5. 写页面按键处理

普通只读页面一般只处理 K4 短按返回父页面：

```c
static void handle_sensor_key(menu_key_event_enum event)
{
    if(MENU_KEY_EVENT_K4_SHORT == event)
    {
        go_parent();
    }
}
```

K4 长按不用在每个页面重复写，`dispatch_key_event()` 已经统一处理：

```text
K4 long -> go_home()
```

### 6. 写 `menu.c` 的页面数据入口

`menu.c` 负责从业务模块取数据，然后交给 `screen.c`：

```c
static void draw_sensor_page(void)
{
    uint16 adc_value = sensor_get_adc();
    float imu_yaw = sensor_get_yaw();
    const char *state = sensor_get_state_name();

    screen_draw_sensor(adc_value, imu_yaw, state);
}

static void refresh_sensor_page(void)
{
    uint16 adc_value = sensor_get_adc();
    float imu_yaw = sensor_get_yaw();
    const char *state = sensor_get_state_name();

    screen_draw_sensor_status(adc_value, imu_yaw, state);
}
```

完整绘制函数 `draw_sensor_page()` 画标题、标签、初始值和按键提示。

动态刷新函数 `refresh_sensor_page()` 只刷新会变的变量，不重画标题、标签、地图或大块图形。

### 7. 在 `screen.h` 增加绘制接口

```c
void screen_draw_sensor(uint16 adc_value, float imu_yaw, const char *state);
void screen_draw_sensor_status(uint16 adc_value, float imu_yaw, const char *state);
```

### 8. 在 `screen.c` 增加屏幕页面枚举

在 `screen_page_enum` 中增加：

```c
SCREEN_PAGE_SENSOR,
```

### 9. 在 `screen.c` 写完整绘制和动态刷新

```c
void screen_draw_sensor(uint16 adc_value, float imu_yaw, const char *state)
{
    begin_page(SCREEN_PAGE_SENSOR);

    ips200_show_string(0, 0, "Sensor");
    ips200_show_string(0, LINE_H, "ADC:");
    ips200_show_string(0, LINE_H * 2, "Yaw:");
    ips200_show_string(0, LINE_H * 3, "State:");

    screen_draw_sensor_status(adc_value, imu_yaw, state);

    show_hint("K4 Back", "K4L Home");
}

void screen_draw_sensor_status(uint16 adc_value, float imu_yaw, const char *state)
{
    begin_page(SCREEN_PAGE_SENSOR);

    ips200_show_uint(40, LINE_H, adc_value, 4);
    ips200_show_float(40, LINE_H * 2, (double)imu_yaw, 3, 1);
    show_text_value(56, LINE_H * 3, state, 10);
}
```

注意：

```text
完整绘制函数负责静态标签。
动态刷新函数只负责值。
数字不清屏，文本只清值区域。
```

## 给已有页面添加变量

下面以给 Home 添加 `Battery` 电压为例。

### 1. 修改 `screen.h` 接口

给 `screen_draw_home()` 和 `screen_draw_home_status()` 增加参数：

```c
float battery_v
```

完整绘制和动态刷新都需要这个参数，因为完整绘制会画初始值，动态刷新会更新值。

### 2. 在 `menu.c` 获取数据并传入 screen

在 `draw_home_page()` 和 `refresh_home_page()` 中获取同一个数据源：

```c
float battery_v = power_get_battery_voltage();
```

然后分别传给：

```c
screen_draw_home(..., battery_v);
screen_draw_home_status(..., battery_v);
```

### 3. 在 `screen.c` 画静态标签

如果变量属于 Home 动态区，标签放到 `draw_home_status_labels()`：

```c
ips200_show_string(0, LINE_H * 15, "Bat:");
```

不要写：

```c
/* 不要先清整行 */
```

### 4. 在 `screen.c` 刷新变量值

数值放到 `draw_home_status_values()`：

```c
ips200_show_float(40, LINE_H * 15, (double)battery_v, 2, 2);
```

如果新增的是文本状态，使用：

```c
show_text_value(56, LINE_H * 15, battery_state, 10);
```

## 添加子页面

子页面不需要加入 Home 菜单，但需要页面码和页面表。

例如新增 Run 下的 `Run/Stats`：

```c
MENU_PAGE_RUN_STATS = 16,
```

页面表：

```c
{ MENU_PAGE_RUN_STATS, MENU_PAGE_RUN, 500, draw_run_stats_page, refresh_run_stats_page, handle_run_stats_key },
```

返回逻辑：

```c
static void handle_run_stats_key(menu_key_event_enum event)
{
    if(MENU_KEY_EVENT_K4_SHORT == event)
    {
        go_parent();
    }
}
```

如果要从 Run 页面按键进入这个子页面，在 `handle_run_event()` 里增加对应按键行为。

## 什么时候设置动态刷新

页面表中的 `refresh_ms` 按下面规则选择：

```text
0    不自动刷新，例如 Info、Mode Select、Run 地图页面
500  普通状态页，例如 Home、Debug、Sensor
200  需要更流畅观察的轻量数据页
```

不要把地图、大面积色块、BFS 求解过程放到高频动态刷新里。地图只在变化时重画。

## Agent 修改检查清单

新增页面后，逐项检查：

```text
[ ] menu_page_id_enum 增加页面码
[ ] 一级页面更新 HOME_ITEM_COUNT
[ ] 一级页面更新 home_items
[ ] 一级页面更新 home_item_pages
[ ] 新一级页面编号需要 ROOT_PAGE_COUNT 足够大
[ ] menu.c 增加 draw/refresh/key 函数声明
[ ] menu_pages[] 增加页面表项
[ ] menu.c 实现 draw_xxx_page()
[ ] 如需动态刷新，实现 refresh_xxx_page()
[ ] menu.c 实现 handle_xxx_key()
[ ] screen_page_enum 增加 SCREEN_PAGE_XXX
[ ] screen.h 增加 screen_draw_xxx() 声明
[ ] 如需动态刷新，screen.h 增加 screen_draw_xxx_status() 声明
[ ] screen.c 实现完整绘制函数
[ ] screen.c 实现动态刷新函数
[ ] 文本值使用 show_text_value()
[ ] 数字值直接使用 ips200_show_int/uint/float()
[ ] 没有新增整行清除函数
[ ] 没有新增字符串拼接显示
[ ] Keil 全量构建 0 Error(s), 0 Warning(s)
```

新增变量后，逐项检查：

```text
[ ] 数据源在 menu.c 获取，不在 screen.c 访问业务模块
[ ] screen.h 的完整绘制和动态刷新接口参数一致
[ ] 静态标签只在完整绘制中画
[ ] 数字值放在动态值刷新函数中固定宽度覆盖
[ ] 文本值用 show_text_value() 指定最大字符数
[ ] 不周期刷新地图或大块图形
```

## 常见错误

不要在 `menu.c` 里直接画屏：

```c
ips200_show_string(...);
```

应该放到 `screen.c`。

不要在 `screen.c` 里读业务状态：

```c
get_control_status();
drive_pose_get();
```

应该在 `menu.c` 获取，再作为参数传给 `screen.c`。

不要为固定标签清整行。

应该直接写：

```c
ips200_show_string(0, LINE_H * 15, "Bat:");
```

不要用字符串拼接函数组显示字符串。数字用 `ips200_show_*`，文本值用 `show_text_value()`。
