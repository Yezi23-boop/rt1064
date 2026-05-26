#ifndef _app_h_
#define _app_h_

/**
 * @brief 初始化用户应用层。
 *
 * 该入口把菜单、屏幕、掉电设置和 Push Box 应用状态收敛到一起，
 * 让 `main.c` 只负责启动顺序，不直接承载业务逻辑。
 *
 * @note 仅在主程序初始化阶段调用一次；函数内部不应依赖中断已经开始调度。
 */
void app_init(void);

/**
 * @brief 轮询用户应用层任务。
 *
 * 主循环反复调用该函数，用于处理非阻塞菜单、按键和屏幕刷新。
 *
 * @note 该函数必须保持非阻塞，避免影响 5ms/20ms 中断闭环的实时性。
 */
void app_poll(void);

#endif
