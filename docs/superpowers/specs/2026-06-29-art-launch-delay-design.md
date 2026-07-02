# ART 发车延迟设计

日期：2026-06-29

## 背景

当前 ART 来源发车流程需要按两次 K3：第一次触发 ART 求解，第二次确认发车。用户希望改为一次按键后自动等待 5 秒（等人离场），然后取最新地图、求解、直接发车。

OpenART 每 200ms 发送一帧地图，MCU 持续接收。"定期刷新最新 ART 地图"已有机制，不需要改动。

## 需求

1. 待命阶段：没按出发键时，持续接收最新 ART 地图（已有）
2. 发车阶段：按 K3 后等 5 秒，取最新地图作为基准，求解并直接发车
3. 5 秒期间持续接收 ART 地图，不清空 UART 缓冲
4. 不需要二次 K3 确认

## 当前流程

```
K3 → art_replan_begin_initial()
   → openart_uart_discard_pending()
   → 等连续 N 帧完全一致（稳定帧）
   → 求解 → "Ready K3"
   → 用户再按 K3 → executor_start()
```

## 新流程

```
K3 → art_replan_begin_initial()
   → ART_REPLAN_WAIT_LAUNCH（新阶段）
   → 等 5 秒，期间持续接收 ART 地图
   → 5 秒到，丢弃 UART 缓冲，等新帧
   → 连续 N 帧一致确认（复用现有稳定帧机制）
   → 求解 → 直接 executor_start()（跳过 "Ready K3"）
```

## 改动

### art_replan.c / art_replan.h

新增枚举值：

```c
typedef enum
{
    ART_REPLAN_IDLE = 0,
    ART_REPLAN_WAIT_LAUNCH,  // 新增：等人离场延迟阶段
    ART_REPLAN_INITIAL,
    ART_REPLAN_SEGMENT,
} art_replan_phase_enum;
```

新增状态变量：

```c
static uint32 art_launch_delay_start_ms = 0;  // 5 秒延迟起点
```

新增常量：

```c
#define ART_LAUNCH_DELAY_MS  (5000u)  // 发车前等人离场延迟
```

修改 `art_replan_begin_initial()`：

```c
void art_replan_begin_initial(art_replan_update_struct *update)
{
    art_replan_update_reset(update);
    art_replan_phase = ART_REPLAN_WAIT_LAUNCH;  // 先进入等待阶段
    art_launch_delay_start_ms = time_ms();
    // 不丢弃 UART 缓冲，持续接收
    if(0 != update)
    {
        update->run_state = "Wait 5s";
        update->redraw = 1;
    }
}
```

修改 `art_replan_tick()`：

```c
void art_replan_tick(...)
{
    // ...

    // 新增：等待发车延迟阶段
    if(ART_REPLAN_WAIT_LAUNCH == art_replan_phase)
    {
        if((time_ms() - art_launch_delay_start_ms) >= ART_LAUNCH_DELAY_MS)
        {
            // 5 秒到，丢弃旧帧，进入 INITIAL 阶段等新帧
            art_replan_begin(ART_REPLAN_INITIAL, update);
        }
        else
        {
            if(0 != update)
            {
                update->run_state = "Wait 5s";
            }
        }
        return;
    }

    // 现有 INITIAL 和 SEGMENT 逻辑...
}
```

`art_replan_begin(ART_REPLAN_INITIAL)` 内部会调用 `art_replan_wait_fresh_frame()` 丢弃 UART 缓冲，
确保只用 5 秒之后到达的新帧。

修改 INITIAL 阶段成功后的逻辑：跳过 "Ready K3"，直接发车。

在 `art_handle_stable_map()` 中，`ART_REPLAN_INITIAL` 成功后改为直接调用 `art_replan_start_executor()`，不再调用 `art_replan_wait_launch()`：

```c
if(ART_REPLAN_INITIAL == phase)
{
    // 直接发车，不需要二次 K3 确认
    art_replan_start_executor(context, &stats, update);
    art_replan_cancel();
}
```

### menu.c

`execute_current_selection()` 中 ART 路径不再清除 `art_launch_pending` 标志（因为不再使用它）。

`handle_run_event()` 中 K3 的 `confirm_art_launch_if_pending()` 检查保留但不再触发（`art_launch_pending` 始终为 0）。

### 显示

Execute 页面在等待阶段显示 "Wait 5s" 和倒计时秒数。

## 参数

```c
#define ART_LAUNCH_DELAY_MS       (5000u)   // 发车前等人离场延迟
#define EXEC_ART_STABLE_FRAMES    (2u)      // 延迟结束后的稳定帧确认数（复用现有）
```

## 风险

1. **5 秒内 ART 识别到人**：不影响，因为 5 秒后才取地图
2. **5 秒后 ART 地图仍有问题**：稳定帧机制会过滤掉不一致的帧
3. **UART 缓冲满**：5 秒期间持续接收，缓冲 384 字节足够（一帧约 250 字节，200ms 一帧）

## 验收

1. Run 页面选 ART 来源，按 K3
2. 屏幕显示 "Wait 5s" 并倒计时
3. 5 秒后自动取最新地图、求解、启动 executor
4. 车开始执行路径
5. 不需要二次按 K3
