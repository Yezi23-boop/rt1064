#include <stdio.h>
#include <string.h>
#include "openart_uart.h"

static char tx_text[64];
static uint32 fake_time_ms;

uint32 time_ms(void)
{
    return fake_time_ms;
}

uint32 interrupt_global_disable(void)
{
    return 0u;
}

void interrupt_global_enable(uint32 primask)
{
    (void)primask;
}

void uart_write_string(int uartn, const char *str)
{
    (void)uartn;
    strncpy(tx_text, str, sizeof(tx_text) - 1u);
    tx_text[sizeof(tx_text) - 1u] = '\0';
}

static void feed_line(const char *line)
{
    while('\0' != *line)
    {
        openart_uart_push_byte((uint8)*line++);
    }
    openart_uart_push_byte((uint8)'\n');
    openart_uart_poll();
}

static uint8 pop_sample_equals(uint8 expected_index, uint16 expected_col, uint16 expected_row)
{
    uint16 col_q = 0u;
    uint16 row_q = 0u;
    uint8 index = openart_get_requested_center_sample(&col_q, &row_q);

    return ((expected_index == index) &&
            (expected_col == col_q) &&
            (expected_row == row_q)) ? 1u : 0u;
}

static uint8 run_case(const char *name, uint8 passed)
{
    printf("%-28s %s\n", name, (0 != passed) ? "PASS" : "FAIL");
    return passed;
}

int main(void)
{
    uint8 passed = 1u;

    openart_uart_init();
    memset(tx_text, 0, sizeof(tx_text));
    openart_request_player_center();
    passed &= run_case("request-command", 0 == strcmp(tx_text, "CENTER_REQ\n"));
    passed &= run_case("request-clears-samples", pop_sample_equals(0u, 0u, 0u));

    feed_line("CENTER_SAMPLE 1,568,550");
    feed_line("CENTER_SAMPLE 1,999,999");
    feed_line("CENTER_SAMPLE 3,572,551");
    feed_line("CENTER_SAMPLE 2,570,550");
    feed_line("CENTER_SAMPLE 3,572,551");
    passed &= run_case("queued-sample-one", pop_sample_equals(1u, 568u, 550u));
    passed &= run_case("queued-sample-two", pop_sample_equals(2u, 570u, 550u));
    passed &= run_case("queued-sample-three", pop_sample_equals(3u, 572u, 551u));
    passed &= run_case("queue-empty", pop_sample_equals(0u, 0u, 0u));

    openart_request_player_center();
    feed_line("CENTER_SAMPLE 1,600,500");
    openart_request_player_center();
    passed &= run_case("new-request-clears-queue", pop_sample_equals(0u, 0u, 0u));

    return (0 != passed) ? 0 : 1;
}
