#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "drive_control.h"
#include "isr.h"

void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();
    __DSB();
}

void PIT_IRQHandler(void)
{
    if(pit_flag_get(PIT_CH1))
    {
        pit_flag_clear(PIT_CH1);
        update_control_20ms();   // 20ms 执行姿态环、混控和四轮速度 PID。
    }

    if(pit_flag_get(PIT_CH2))
    {
        pit_flag_clear(PIT_CH2);
    }

    if(pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }

    __DSB();                     // 退出 Cortex-M ISR 前同步总线写入，避免中断标志清除延后生效。
}

void LPUART1_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
    #if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
    #endif
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
        imu660rc_callback();      // 恢复 IMU660RC 驱动原始 INT2 中断触发读取四元数。
    }
    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);
    }
}
