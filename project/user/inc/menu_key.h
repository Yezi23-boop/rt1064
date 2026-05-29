#ifndef _menu_key_h_
#define _menu_key_h_

#include "zf_common_typedef.h"

typedef enum
{
    MENU_KEY_EVENT_NONE = 0,
    MENU_KEY_EVENT_K1_SHORT,
    MENU_KEY_EVENT_K2_SHORT,
    MENU_KEY_EVENT_K3_SHORT,
    MENU_KEY_EVENT_K4_SHORT,
    MENU_KEY_EVENT_K1_LONG,
    MENU_KEY_EVENT_K2_LONG,
    MENU_KEY_EVENT_K3_LONG,
    MENU_KEY_EVENT_K4_LONG,
} menu_key_event_enum;

void menu_key_init(void);
void menu_key_tick_5ms(void);
menu_key_event_enum menu_key_read_event(void);

#endif
