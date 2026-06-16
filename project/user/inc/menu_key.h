#ifndef _menu_key_h_
#define _menu_key_h_

#include "zf_common_typedef.h"

/**
 * @brief 菜单层消费的抽象按键事件。
 *
 * 物理 KEY_1..KEY_4 在本模块内转换为短按/长按事件，菜单状态机不直接读取底层
 * key 驱动状态，从而保持 5ms 扫描节拍和页面逻辑解耦。
 */
typedef enum
{
    MENU_KEY_EVENT_NONE = 0,  /**< 当前没有可消费事件。 */
    MENU_KEY_EVENT_K1_SHORT,  /**< K1 短按，通常用于上移或上一步。 */
    MENU_KEY_EVENT_K2_SHORT,  /**< K2 短按，通常用于下移或下一步。 */
    MENU_KEY_EVENT_K3_SHORT,  /**< K3 短按，通常用于确认、启动或继续。 */
    MENU_KEY_EVENT_K4_SHORT,  /**< K4 短按，通常用于返回、保存或停止。 */
    MENU_KEY_EVENT_K1_LONG,   /**< K1 长按，预留给页面扩展。 */
    MENU_KEY_EVENT_K2_LONG,   /**< K2 长按，预留给页面扩展。 */
    MENU_KEY_EVENT_K3_LONG,   /**< K3 长按，当前用于进入运行模式选择。 */
    MENU_KEY_EVENT_K4_LONG,   /**< K4 长按，全局安全返回 Home。 */
} menu_key_event_enum;

/**
 * @brief 初始化菜单按键事件队列和底层按键扫描器。
 *
 * @note 仅在菜单初始化阶段调用一次；会清除底层 key 驱动已有状态，避免上电抖动
 * 或旧状态变成菜单事件。
 */
void menu_key_init(void);

/**
 * @brief 5ms 周期按键扫描入口。
 *
 * @note 由 PIT_CH2 中断按固定周期调用；函数只做扫描、去重和入队，不应加入
 * 屏幕刷新、Flash 写入或求解等耗时操作。
 */
void menu_key_tick_5ms(void);

/**
 * @brief 读取一个菜单按键事件。
 *
 * @return 队列中最早的事件；若无事件则返回 `MENU_KEY_EVENT_NONE`。
 *
 * @note 主循环调用本函数消费事件。事件队列满时会丢弃最旧事件，优先保留最近输入。
 */
menu_key_event_enum menu_key_read_event(void);

#endif
