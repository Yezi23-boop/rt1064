#ifndef _vision_uart_h_
#define _vision_uart_h_

#include "zf_common_typedef.h"

typedef enum
{
    VISION_MODE_NONE = 0,
    VISION_MODE_BOX,
    VISION_MODE_TARGET
} vision_mode_enum;

typedef struct
{
    uint16 request_id;
    uint16 sample_id;
    uint8 class_id;
    uint16 confidence_q;
} vision_sample_struct;

typedef enum
{
    VISION_UART_BOARD_TEST_DISABLED = 0,
    VISION_UART_BOARD_TEST_WAIT_READY,
    VISION_UART_BOARD_TEST_WAIT_SAMPLE,
    VISION_UART_BOARD_TEST_PASS,
    VISION_UART_BOARD_TEST_FAIL
} vision_uart_board_test_state_enum;

typedef struct
{
    uint8 state;
    uint8 sample_count;
    uint8 class_id;
    uint16 confidence_q;
} vision_uart_board_test_status_struct;

void vision_uart_init(void);
void vision_uart_push_byte(uint8 data);
void vision_uart_poll(void);
void vision_uart_set_mode(vision_mode_enum mode);
uint8 vision_uart_mode_ready(vision_mode_enum mode);
uint16 vision_uart_request_classification(void);
uint8 vision_uart_get_sample(vision_sample_struct *sample);
void vision_uart_ack(uint16 request_id);
void vision_uart_cancel(void);
void vision_uart_board_test_init(void);
void vision_uart_board_test_poll(void);
void vision_uart_board_test_get_status(vision_uart_board_test_status_struct *status);

#endif
