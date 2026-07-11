#ifndef _zf_common_interrupt_h_
#define _zf_common_interrupt_h_

#include "zf_common_typedef.h"

uint32 interrupt_global_disable(void);
void interrupt_global_enable(uint32 primask);

#endif
