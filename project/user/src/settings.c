#include "zf_common_headfile.h"
#include "settings.h"
#include "maps.h"

// 用户设置独占一个 Flash 扇区，只保存“索引类”配置，不保存 16x12 地图内容。
// 当前只有一份记录，故意不引入 sequence/多副本选择，降低菜单保存路径的复杂度。
#define SETTINGS_SECTOR             (127)
#define SETTINGS_PAGE               (FLASH_PAGE_0)
#define SETTINGS_MAGIC              (0x5058424Du)  // "PXBM"：区分空 Flash、其他项目数据和本项目设置记录。
#define SETTINGS_VERSION            (2u)           // 字段布局或枚举持久化含义变化时必须递增。

typedef struct
{
    uint32 magic;                       // 记录识别字，避免把空 Flash 当成有效设置。
    uint16 version;                     // 结构版本，防止旧固件格式被误读。
    uint16 checksum;                    // 轻量字节和校验，只用于发现擦写失败或半写入记录。
    uint8 current_map;                  // 保存的地图编号，只是常量表索引。
    uint8 run_mode;                     // 保存的运行模式，取值为 run_mode_enum。
    uint8 map_source;                   // 保存的地图来源，取值为 map_source_enum。
    uint8 reserved;                     // 固定为 0 的对齐字段，避免未初始化字节影响校验。
} settings_record_struct;

static uint8 current_map;               // 主循环菜单写入并读取；不在 ISR 中访问。
static run_mode_enum run_mode;          // Flash 只保存用户显式确认后的最终值。
static map_source_enum map_source;      // 来源切换只影响求解入口，不验证 OpenART 帧是否可用。
static uint8 flash_ready;               // flash_init 成功后置 1，避免初始化失败后继续擦写。
static save_state_enum save_state = SAVE_STATE_EMPTY; // 屏幕提示用状态，不作为控制安全条件。

static uint16 checksum_record(const settings_record_struct *record)
{
    const uint8 *bytes = (const uint8 *)record;
    uint16 sum = 0;
    uint32 i;
    settings_record_struct temp = *record;

    temp.checksum = 0;                  // 校验字段自身不参与求和，否则同一记录无法得到稳定校验值。
    bytes = (const uint8 *)&temp;
    for(i = 0; i < sizeof(temp); i++)
    {
        sum = (uint16)(sum + bytes[i]);
    }
    return sum;
}

static uint8 record_valid(const settings_record_struct *record)
{
    // 先验证魔数和版本，再解释枚举字段，避免旧格式或空 Flash 被当作合法运行配置。
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
        // 地图表数量可能随固件变化；旧记录越界时必须回到默认地图。
        return 0;
    }
    if(record->run_mode >= RUN_MODE_COUNT)
    {
        return 0;
    }
    if(record->map_source >= MAP_SOURCE_COUNT)
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
    map_source = MAP_SOURCE_OFFLINE;
    flash_ready = 0;
    save_state = SAVE_STATE_EMPTY;

    // Flash 初始化失败时只影响掉电保存，运行时菜单仍使用默认 RAM 配置。
    if(0 != flash_init())
    {
        save_state = SAVE_STATE_ERROR;
        return;
    }
    flash_ready = 1;

    // 记录按 word 读取，长度向上取整；结构体尾部不能依赖未定义填充值参与语义。
    flash_read_page(SETTINGS_SECTOR, SETTINGS_PAGE, (uint32 *)&record, (uint16)((sizeof(record) + 3u) / 4u));
    if(0 != record_valid(&record))
    {
        current_map = record.current_map;
        run_mode = (run_mode_enum)record.run_mode;
        map_source = (map_source_enum)record.map_source;
        save_state = SAVE_STATE_SAVED;
    }
    else if(SETTINGS_MAGIC == record.magic)
    {
        // 魔数匹配但保护字段失败，多半是旧版本、半写入或擦写异常，保留默认设置更安全。
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

map_source_enum settings_get_source(void)
{
    return map_source;
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
        // 切换地图或模式只标记 Dirty，真正写 Flash 由 Home 页 K4 短按触发，减少扇区擦写次数。
        save_state = SAVE_STATE_DIRTY;
    }
}

void settings_set_source(map_source_enum source)
{
    if(source >= MAP_SOURCE_COUNT)
    {
        source = MAP_SOURCE_OFFLINE;
    }

    map_source = source;
    if(0 != flash_ready)
    {
        // 来源切换同样延迟保存，避免用户浏览菜单时反复写入 Flash。
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
    record.map_source = (uint8)map_source;
    record.reserved = 0;
    record.checksum = checksum_record(&record);

    // 写入后立即回读校验；菜单显示需要知道“掉电后是否真的可恢复”。
    if(0 != flash_write_page(SETTINGS_SECTOR, SETTINGS_PAGE, (const uint32 *)&record, word_len))
    {
        save_state = SAVE_STATE_WRITE_ERROR;
        return 0;
    }

    flash_read_page(SETTINGS_SECTOR, SETTINGS_PAGE, (uint32 *)&verify, word_len);
    if((0 == record_valid(&verify)) ||
       (verify.current_map != record.current_map) ||
       (verify.run_mode != record.run_mode) ||
       (verify.map_source != record.map_source))
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
        "Solve",
        "Run",
        "Step",
    };

    if(mode >= RUN_MODE_COUNT)
    {
        return "Solve";
    }
    return names[mode];
}

const char *source_name(map_source_enum source)
{
    static const char *const names[MAP_SOURCE_COUNT] =
    {
        "Offline",
        "ART",
    };

    if(source >= MAP_SOURCE_COUNT)
    {
        return "Offline";
    }
    return names[source];
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
