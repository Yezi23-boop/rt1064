#ifndef _menu_h_
#define _menu_h_

/**
 * @brief 初始化菜单状态机和按键扫描。
 *
 * 初始化后默认停留在 Home 页面，并从掉电设置中恢复当前地图编号和运行模式。
 *
 * @note 仅在主循环启动前调用一次；页面结构和按键语义由 `menu_poll()` 维护。
 */
void menu_init(void);

/**
 * @brief 非阻塞轮询菜单状态机。
 *
 * 该函数读取按键事件、推进 Demo/Playback 状态，并在需要时刷新屏幕。
 *
 * @note 必须在主循环中高频调用；函数本身不得阻塞等待按键或算法执行完成。
 */
void menu_poll(void);

#endif
