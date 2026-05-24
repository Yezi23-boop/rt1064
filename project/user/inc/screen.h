#ifndef _screen_h_
#define _screen_h_

#include "map_types.h"
#include "settings.h"

typedef enum
{
    PLAYBACK_STATE_PAUSED = 0,
    PLAYBACK_STATE_PLAYING,
    PLAYBACK_STATE_DONE,
    PLAYBACK_STATE_FAIL,
} playback_state_enum;

void screen_init(void);
void screen_draw_home(const char *const *items, uint8 item_count, uint8 cursor, uint8 current_map, run_mode_enum mode, save_state_enum save_state);
void screen_draw_run(uint8 current_map, uint8 candidate_map, run_mode_enum mode, save_state_enum save_state, const char *state, uint32 elapsed_ms);
void screen_draw_mode_select(run_mode_enum candidate_mode);
void screen_draw_playback(uint8 map_index, const map_source_struct *source, const solve_result_struct *result, uint16 step, uint32 elapsed_ms, playback_state_enum state);
void screen_draw_demo(uint8 index, uint8 total, uint8 ok_count, uint8 fail_count, uint32 elapsed_ms, const char *state);
void screen_draw_debug(void);
void screen_draw_info(uint8 map_count_value, save_state_enum save_state);

#endif
