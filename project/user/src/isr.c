#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "drive_control.h"
#include "executor.h"
#include "menu_key.h"
#include "openart_uart.h"
#include "isr.h"

void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();
    // NXP 建议在 Cortex-M ISR 退出前做数据同步，避免外设状态写入晚于中断返回。
    __DSB();
}

void PIT_IRQHandler(void)
{
    // PIT_CH1 是运动控制的硬实时边界；不要在该分支加入 printf、屏幕绘制或 BFS 这类不可控耗时任务。
    if(pit_flag_get(PIT_CH1))
    {
        pit_flag_clear(PIT_CH1);
        executor_update_20ms();
        update_control_20ms();
    }

    // 菜单按键用 5ms 节拍做消抖，主循环只消费状态，避免屏幕刷新频率影响按键手感。
    if(pit_flag_get(PIT_CH2))
    {
        pit_flag_clear(PIT_CH2);
        menu_key_tick_5ms();
    }

    if(pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }

    // 同步中断标志清除，降低刚退出 ISR 又因旧 pending 状态重入的风险。
    __DSB();
}

void LPUART1_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        // LPUART1 接 OpenART B12/B13，只推字节给协议解析器；调试 printf 走 LPUART8 无线串口。
        uint8 data = LPUART_ReadByte(LPUART1);
        openart_uart_push_byte(data);
    }

    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);
}

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
    }

    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);
}

void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
    }

    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);
}

void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        flexio_camera_uart_handler();
        gnss_uart_callback();
    }

    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);
}

void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        camera_uart_handler();
    }

    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);
}

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
    }

    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);
}

void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // 无线串口同时承担 printf/VOFA 输出和上位机输入，ISR 只搬运字节，解析放在主循环。
        wireless_module_uart_handler();
    }

    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);
}

void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);
    }
}

void GPIO1_Combined_16_31_IRQHandler(void)
{
    wireless_module_spi_handler();
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16);
    }
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    flexio_camera_vsync_handler();

    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);
    }
}

void GPIO2_Combined_16_31_IRQHandler(void)
{
    tof_module_exti_handler();

    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16);
    }
}

void GPIO3_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(IMU660RC_INT2_PIN))
    {
        exti_flag_clear(IMU660RC_INT2_PIN);
        // 保持 IMU 驱动原始 INT2 回调路径；控制环读取的是驱动维护的最新姿态缓存。
        imu660rc_callback();
    }
    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);
    }
}
