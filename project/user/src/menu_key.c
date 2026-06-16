#include "zf_common_headfile.h"
#include "menu_key.h"

#define MENU_KEY_SCAN_PERIOD_MS (5)
#define MENU_KEY_QUEUE_SIZE     (8)

static volatile menu_key_event_enum event_queue[MENU_KEY_QUEUE_SIZE]; // PIT_CH2 写入、主循环读取；只存边沿事件，不存按键电平。
static volatile uint8 queue_head;                                     // ISR 写入下标，队列满时会覆盖最旧事件。
static volatile uint8 queue_tail;                                     // 主循环读取下标；弹出事件时在临界区更新。
static volatile uint8 queue_count;                                    // ISR/主循环共享计数，访问时需要关中断保证一致。
static uint8 long_used[KEY_NUMBER];                                   // 每次长按只产生一次长按事件，松手后才允许再次触发。

static menu_key_event_enum short_event_from_index(key_index_enum key)
{
    switch(key)
    {
        case KEY_1: return MENU_KEY_EVENT_K1_SHORT;
        case KEY_2: return MENU_KEY_EVENT_K2_SHORT;
        case KEY_3: return MENU_KEY_EVENT_K3_SHORT;
        case KEY_4: return MENU_KEY_EVENT_K4_SHORT;
        default:    return MENU_KEY_EVENT_NONE;
    }
}

static menu_key_event_enum long_event_from_index(key_index_enum key)
{
    switch(key)
    {
        case KEY_1: return MENU_KEY_EVENT_K1_LONG;
        case KEY_2: return MENU_KEY_EVENT_K2_LONG;
        case KEY_3: return MENU_KEY_EVENT_K3_LONG;
        case KEY_4: return MENU_KEY_EVENT_K4_LONG;
        default:    return MENU_KEY_EVENT_NONE;
    }
}

static void push_event(menu_key_event_enum event)
{
    if(MENU_KEY_EVENT_NONE == event)
    {
        return;
    }

    if(queue_count >= MENU_KEY_QUEUE_SIZE)
    {
        // 按键 ISR 不能等待主循环消费；队列满时丢最旧事件，优先保留用户最近的操作。
        queue_tail = (uint8)((queue_tail + 1u) % MENU_KEY_QUEUE_SIZE);
        queue_count--;
    }

    event_queue[queue_head] = event;
    queue_head = (uint8)((queue_head + 1u) % MENU_KEY_QUEUE_SIZE);
    queue_count++;
}

void menu_key_init(void)
{
    key_index_enum key;

    key_init(MENU_KEY_SCAN_PERIOD_MS);
    queue_head = 0;
    queue_tail = 0;
    queue_count = 0;

    for(key = KEY_1; key < KEY_NUMBER; key++)
    {
        long_used[key] = 0;
        key_clear_state(key);
    }
}

void menu_key_tick_5ms(void)
{
    key_index_enum key;

    key_scanner();

    for(key = KEY_1; key < KEY_NUMBER; key++)
    {
        key_state_enum state = key_get_state(key);

        if(KEY_RELEASE == state)
        {
            long_used[key] = 0;
            continue;
        }

        if((KEY_LONG_PRESS == state) && (0 == long_used[key]))
        {
            // 长按只上报一次，直到释放后才允许产生下一次长按或短按事件。
            long_used[key] = 1;
            push_event(long_event_from_index(key));
            key_clear_state(key);
            continue;
        }

        if((KEY_SHORT_PRESS == state) && (0 == long_used[key]))
        {
            push_event(short_event_from_index(key));
            key_clear_state(key);
        }
    }
}

menu_key_event_enum menu_key_read_event(void)
{
    menu_key_event_enum event;

    if(0 == queue_count)
    {
        return MENU_KEY_EVENT_NONE;
    }

    event = event_queue[queue_tail];
    queue_tail = (uint8)((queue_tail + 1u) % MENU_KEY_QUEUE_SIZE);
    queue_count--;

    return event;
}
