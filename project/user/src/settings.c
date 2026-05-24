#include "zf_common_headfile.h"
#include "settings.h"
#include "maps.h"

#define SETTINGS_SECTOR             (127)
#define SETTINGS_PAGE               (FLASH_PAGE_0)
#define SETTINGS_MAGIC              (0x5058424Du)
#define SETTINGS_VERSION            (1u)

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 checksum;
    uint8 current_map;
    uint8 run_mode;
    uint16 reserved;
} settings_record_struct;

static uint8 current_map;
static run_mode_enum run_mode;
static uint8 flash_ready;
static save_state_enum save_state = SAVE_STATE_EMPTY;

static uint16 checksum_record(const settings_record_struct *record)
{
    const uint8 *bytes = (const uint8 *)record;
    uint16 sum = 0;
    uint32 i;
    settings_record_struct temp = *record;

    temp.checksum = 0;
    bytes = (const uint8 *)&temp;
    for(i = 0; i < sizeof(temp); i++)
    {
        sum = (uint16)(sum + bytes[i]);
    }
    return sum;
}

static uint8 record_valid(const settings_record_struct *record)
{
    if(SETTINGS_MAGIC != record->magic)
    {
        return 0;
    }
    if(SETTINGS_VERSION != record->version)
    {
        return 0;
    }
    if(record->checksum != checksum_record(record))
    {
        return 0;
    }
    if(record->current_map >= map_count())
    {
        return 0;
    }
    if(record->run_mode >= RUN_MODE_COUNT)
    {
        return 0;
    }
    return 1;
}

void settings_init(void)
{
    settings_record_struct record;

    current_map = 0;
    run_mode = RUN_MODE_SOLVE;
    flash_ready = 0;
    save_state = SAVE_STATE_EMPTY;

    if(0 != flash_init())
    {
        save_state = SAVE_STATE_ERROR;
        return;
    }
    flash_ready = 1;

    flash_read_page(SETTINGS_SECTOR, SETTINGS_PAGE, (uint32 *)&record, (uint16)((sizeof(record) + 3u) / 4u));
    if(0 != record_valid(&record))
    {
        current_map = record.current_map;
        run_mode = (run_mode_enum)record.run_mode;
        save_state = SAVE_STATE_SAVED;
    }
    else if(SETTINGS_MAGIC == record.magic)
    {
        save_state = SAVE_STATE_CHECK_ERROR;
    }
}

uint8 settings_get_map(void)
{
    return current_map;
}

run_mode_enum settings_get_mode(void)
{
    return run_mode;
}

save_state_enum settings_get_save_state(void)
{
    return save_state;
}

void settings_set_runtime(uint8 map, run_mode_enum mode)
{
    if(map >= map_count())
    {
        map = 0;
    }
    if(mode >= RUN_MODE_COUNT)
    {
        mode = RUN_MODE_SOLVE;
    }

    current_map = map;
    run_mode = mode;
    if(0 != flash_ready)
    {
        save_state = SAVE_STATE_DIRTY;
    }
}

uint8 settings_save(void)
{
    settings_record_struct record;
    settings_record_struct verify;
    uint16 word_len = (uint16)((sizeof(record) + 3u) / 4u);

    if(0 == flash_ready)
    {
        save_state = SAVE_STATE_ERROR;
        return 0;
    }

    record.magic = SETTINGS_MAGIC;
    record.version = SETTINGS_VERSION;
    record.checksum = 0;
    record.current_map = current_map;
    record.run_mode = (uint8)run_mode;
    record.reserved = 0;
    record.checksum = checksum_record(&record);

    if(0 != flash_write_page(SETTINGS_SECTOR, SETTINGS_PAGE, (const uint32 *)&record, word_len))
    {
        save_state = SAVE_STATE_WRITE_ERROR;
        return 0;
    }

    flash_read_page(SETTINGS_SECTOR, SETTINGS_PAGE, (uint32 *)&verify, word_len);
    if((0 == record_valid(&verify)) ||
       (verify.current_map != record.current_map) ||
       (verify.run_mode != record.run_mode))
    {
        save_state = SAVE_STATE_CHECK_ERROR;
        return 0;
    }

    save_state = SAVE_STATE_SAVED;
    return 1;
}

const char *mode_name(run_mode_enum mode)
{
    static const char *const names[RUN_MODE_COUNT] =
    {
        "Browse",
        "Solve",
        "Run",
        "Demo",
    };

    if(mode >= RUN_MODE_COUNT)
    {
        return "Solve";
    }
    return names[mode];
}

const char *save_state_name(save_state_enum state)
{
    switch(state)
    {
        case SAVE_STATE_SAVED:       return "Saved";
        case SAVE_STATE_DIRTY:       return "Dirty";
        case SAVE_STATE_ERROR:
        case SAVE_STATE_CHECK_ERROR:
        case SAVE_STATE_WRITE_ERROR: return "Error";
        case SAVE_STATE_EMPTY:
        default:                     return "Empty";
    }
}

const char *flash_state_name(save_state_enum state)
{
    switch(state)
    {
        case SAVE_STATE_SAVED:
        case SAVE_STATE_DIRTY:       return "OK";
        case SAVE_STATE_CHECK_ERROR: return "Check Error";
        case SAVE_STATE_WRITE_ERROR: return "Write Error";
        case SAVE_STATE_ERROR:       return "Error";
        case SAVE_STATE_EMPTY:
        default:                     return "Empty";
    }
}
