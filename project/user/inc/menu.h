#ifndef _menu_h_
#define _menu_h_

/**
 * @brief 初始化菜单状态机和按键扫描。
 *
 * 初始化后默认停留在 Home 页面，并从掉电设置中恢复当前地图编号、运行模式和地图来源。
 *
 * @pre `timebase_init()` 和全局硬件初始化应已完成，使菜单能读取毫秒时间和按键状态。
 *
 * @note 仅在主循环启动前调用一次；页面结构和按键语义由 `menu_poll()` 维护。
 * @note 本函数会启动 PIT_CH2 的 5ms 菜单按键扫描节拍。
 */
void menu_init(void);

/**
 * @brief 非阻塞轮询菜单状态机。
 *
 * 该函数读取按键事件，推进回放、执行页动态刷新和 ART 重解算状态，并在需要时刷新屏幕。
 *
 * @pre `menu_init()` 已调用。
 *
 * @note 必须在主循环中高频调用；函数本身不得阻塞等待按键或算法执行完成。
 * @note ART 重解算可能触发 BFS 求解，因此不要在 ISR 或严格实时控制环中调用。
 */
void menu_poll(void);

#endif
