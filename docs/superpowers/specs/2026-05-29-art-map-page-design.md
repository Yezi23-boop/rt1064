# ART Map 子页面设计

日期：2026-05-29

## 概述

在 Home 菜单新增 "ART Map" 子页面，用于实时查看 OpenART 传输的 12×16 地图数据、接收帧数和通信状态。

## 菜单结构变更

Home 菜单项从 3 个变为 4 个：

```
Run
ART Map    ← 新增
Debug
Info
```

- `HOME_ITEM_COUNT` 3→4
- `ROOT_PAGE_COUNT` 4→5
- 新增枚举 `MENU_PAGE_ART_MAP = 4`
- `home_items[]` 和 `home_item_pages[]` 在索引 1 插入

## 页面布局

屏幕 240×320，8×16 字体，布局从上到下：

```
OpenART Map              ← 标题，第 0 行
Frames: 123              ← 成功接收帧数，第 1 行
Age: 5s                  ← 距最近一次字节的时间，第 2 行
[12行 × 16列色块地图]     ← MAP_Y=112 起画，CELL_SIZE=14
K4 Home                  ← 底部按键提示
K4L Home
```

### 有地图时

- 显示色块地图，复用 `draw_color_map()` 渲染
- Frames 显示 `openart_uart_get_frame_count()`
- Age 显示 `(time_ms() - openart_last_rx_ms()) / 1000`，后缀 `s`

### 无地图时

- 地图区域显示 "No OpenART map"
- Frames 显示 `0`
- Age 显示 `-`

## Age 计算

- `age_ms = time_ms() - openart_last_rx_ms()`
- 显示为整数秒：`age_ms / 1000`，后缀 `s`
- 若 `openart_last_rx_ms() == 0`（从未收到字节），显示 `-`

## 刷新机制

- `refresh_ms = 500`，与 Home 页面一致
- `draw_full`：整页重绘（标题 + Frames + Age + 地图/提示）
- `refresh_dynamic`：只更新 Frames 和 Age 数值，不重绘地图

## 按键

- K4 短按：返回 Home
- K4 长按：返回 Home（全局 `dispatch_key_event` 已处理）
- K1/K2/K3：无操作

## 涉及文件

| 文件 | 变更 |
|------|------|
| `project/user/src/menu.c` | 新增 `MENU_PAGE_ART_MAP` 枚举、菜单数组、页面定义、按键处理、绘图函数 |
| `project/user/src/screen.c` | 新增 `SCREEN_PAGE_ART_MAP` 枚举、`screen_draw_art_map()` 函数 |
| `project/user/inc/screen.h` | 新增 `screen_draw_art_map()` 声明 |
