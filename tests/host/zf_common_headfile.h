#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_

#include <stddef.h>
#include <string.h>
#include "zf_common_typedef.h"
#include "zf_common_interrupt.h"

#define UART_1 (1)

void uart_write_string(int uartn, const char *str);

#endif
