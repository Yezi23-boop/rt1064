#ifndef _settings_h_
#define _settings_h_

#include "zf_common_typedef.h"

typedef enum
{
    RUN_MODE_BROWSE = 0,
    RUN_MODE_SOLVE,
    RUN_MODE_RUN,
    RUN_MODE_DEMO,
    RUN_MODE_COUNT,
} run_mode_enum;

typedef enum
{
    SAVE_STATE_EMPTY = 0,
    SAVE_STATE_SAVED,
    SAVE_STATE_DIRTY,
    SAVE_STATE_ERROR,
    SAVE_STATE_CHECK_ERROR,
    SAVE_STATE_WRITE_ERROR,
} save_state_enum;

void settings_init(void);
uint8 settings_get_map(void);
run_mode_enum settings_get_mode(void);
save_state_enum settings_get_save_state(void);
void settings_set_runtime(uint8 current_map, run_mode_enum run_mode);
uint8 settings_save(void);

const char *mode_name(run_mode_enum mode);
const char *save_state_name(save_state_enum state);
const char *flash_state_name(save_state_enum state);

#endif
