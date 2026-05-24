#include "drive_config.h"

/* 方向符号是上板校正入口：逐轮点动后只修改这里的一份配置，
 * 避免硬件接线差异渗透到混控和 PID 逻辑中。 */
int8 motor_dir_sign[WHEEL_COUNT] =
{
    1,
    1,
    1,
    1,
};

int8 encoder_dir_sign[WHEEL_COUNT] =
{
    1,
    1,
    1,
    1,
};
