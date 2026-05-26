#include "drive_config.h"

/* 电机方向校正入口。
 * 轮位已经对应正确，但某个轮子的“正 PWM”转向和期望正轮速相反时，
 * 只把对应位置从 1 改成 -1。不要通过交换数组元素修正方向问题。 */
int8 motor_dir_sign[WHEEL_COUNT] =
    {
        -1, // LF 左前
        -1, // LB 左后
        -1, // RF 右前
        -1, // RB 右后
};

/* 编码器方向校正入口。
 * Home 页显示的轮位已经对，但手转正方向时 Enc 为负数，就把对应轮位改成 -1。
 * 如果手转 LF 却是 RF 在跳，这是轮位映射错了，应改 base_io.c 的三组 encoder 数组。 */
int8 encoder_dir_sign[WHEEL_COUNT] =
    {
        1,  // LF 左前
        1,  // LB 左后
        -1, // RF 右前
        -1, // RB 右后
};
