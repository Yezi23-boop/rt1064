#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_

#include <stddef.h>
#include <string.h>
#include "zf_common_typedef.h"
#include "zf_common_interrupt.h"

#define UART_1 (1)
#define UART_4 (4)
#define UART4_TX_C16 (64)
#define UART4_RX_C17 (64)

void uart_write_string(int uartn, const char *str);
void uart_init(int uartn, uint32 baud, int tx_pin, int rx_pin);
void uart_rx_interrupt(int uartn, uint32 status);

#endif
