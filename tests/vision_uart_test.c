#include <stdio.h>
#include <string.h>
#include "zf_common_headfile.h"
#include "drive_config.h"
#include "vision_uart.h"

static char tx_text[96];
static uint8 init_called;
static uint8 irq_called;
static uint32 test_now_ms;

uint32 time_ms(void)
{
    return test_now_ms;
}

void uart_write_string(int uartn, const char *str)
{
    (void)uartn;
    strncpy(tx_text, str, sizeof(tx_text) - 1u);
    tx_text[sizeof(tx_text) - 1u] = '\0';
}

void uart_init(int uartn, uint32 baud, int tx_pin, int rx_pin)
{
    init_called = ((UART_4 == uartn) && (115200u == baud) &&
                   (UART4_TX_C16 == tx_pin) && (UART4_RX_C17 == rx_pin)) ? 1u : 0u;
}

void uart_rx_interrupt(int uartn, uint32 status)
{
    irq_called = ((UART_4 == uartn) && (1u == status)) ? 1u : 0u;
}

uint32 interrupt_global_disable(void)
{
    return 0u;
}

void interrupt_global_enable(uint32 primask)
{
    (void)primask;
}

static void clear_tx(void)
{
    tx_text[0] = '\0';
}

static void feed_line(const char *line)
{
    while('\0' != *line)
    {
        vision_uart_push_byte((uint8)*line++);
    }
    vision_uart_push_byte((uint8)'\n');
    vision_uart_poll();
}

static uint8 sample_equals(uint16 request_id, uint16 sample_id,
                           uint8 class_id, uint16 confidence_q)
{
    vision_sample_struct sample;

    if(0u == vision_uart_get_sample(&sample))
    {
        return 0u;
    }
    return ((request_id == sample.request_id) &&
            (sample_id == sample.sample_id) &&
            (class_id == sample.class_id) &&
            (confidence_q == sample.confidence_q)) ? 1u : 0u;
}

static uint8 queue_empty(void)
{
    vision_sample_struct sample;
    return (0u == vision_uart_get_sample(&sample)) ? 1u : 0u;
}

static uint8 run_case(const char *name, uint8 passed)
{
    printf("%-28s %s\n", name, (0u != passed) ? "PASS" : "FAIL");
    return passed;
}

int main(void)
{
    uint8 passed = 1u;
    uint16 request_id;
    vision_uart_board_test_status_struct board_status;

    vision_uart_init();
    passed &= run_case("uart4-init", (0u != init_called) && (0u != irq_called));

    clear_tx();
    vision_uart_set_mode(VISION_MODE_BOX);
    passed &= run_case("set-box-mode", 0 == strcmp(tx_text, "VISION_MODE BOX\n"));
    feed_line("VISION_READY TARGET");
    passed &= run_case("wrong-ready-rejected", 0u == vision_uart_mode_ready(VISION_MODE_BOX));
    feed_line("VISION_READY BOX");
    passed &= run_case("box-ready", 0u != vision_uart_mode_ready(VISION_MODE_BOX));

    clear_tx();
    request_id = vision_uart_request_classification();
    passed &= run_case("request-id-one", 1u == request_id);
    passed &= run_case("request-command", 0 == strcmp(tx_text, "VISION_REQ 1\n"));

    feed_line("VISION_SAMPLE 0 1 8 950");
    feed_line("VISION_SAMPLE 1 1 8 932");
    passed &= run_case("stale-sample-filter", sample_equals(1u, 1u, 8u, 932u));

    feed_line("VISION_SAMPLE 1 2 10 900");
    feed_line("VISION_SAMPLE 1 3 8 1001");
    feed_line("VISION_SAMPLE 1 4 8 941");
    passed &= run_case("invalid-fields-filter", sample_equals(1u, 4u, 8u, 941u));
    passed &= run_case("sample-queue-empty", queue_empty());

    clear_tx();
    vision_uart_ack(request_id);
    passed &= run_case("ack-command", 0 == strcmp(tx_text, "VISION_ACK 1\n"));

    clear_tx();
    vision_uart_cancel();
    passed &= run_case("cancel-command", 0 == strcmp(tx_text, "VISION_CANCEL\n"));

    feed_line("VISION_SAMPLE 1 5 8 950");
    passed &= run_case("cancel-rejects-sample", queue_empty());

    clear_tx();
    request_id = vision_uart_request_classification();
    passed &= run_case("request-id-two", 2u == request_id);
    passed &= run_case("request-two-command", 0 == strcmp(tx_text, "VISION_REQ 2\n"));

    vision_uart_init();
    test_now_ms = 0u;
    clear_tx();
    vision_uart_board_test_init();
    vision_uart_board_test_get_status(&board_status);
#if VISION_UART_BOARD_TEST_ENABLE
    passed &= run_case("board-test-mode", (0 == strcmp(tx_text, "VISION_MODE BOX\n")) &&
                       (VISION_UART_BOARD_TEST_WAIT_READY == board_status.state));

    feed_line("VISION_READY BOX");
    clear_tx();
    vision_uart_board_test_poll();
    vision_uart_board_test_get_status(&board_status);
    passed &= run_case("board-test-request", (0 == strcmp(tx_text, "VISION_REQ 1\n")) &&
                       (VISION_UART_BOARD_TEST_WAIT_SAMPLE == board_status.state));

    feed_line("VISION_SAMPLE 1 1 8 910");
    vision_uart_board_test_poll();
    feed_line("VISION_SAMPLE 1 2 8 920");
    vision_uart_board_test_poll();
    feed_line("VISION_SAMPLE 1 3 8 930");
    clear_tx();
    vision_uart_board_test_poll();
    vision_uart_board_test_get_status(&board_status);
    passed &= run_case("board-test-pass", (0 == strcmp(tx_text, "VISION_ACK 1\n")) &&
                       (VISION_UART_BOARD_TEST_PASS == board_status.state) &&
                       (3u == board_status.sample_count) &&
                       (8u == board_status.class_id) &&
                       (930u == board_status.confidence_q));

    vision_uart_init();
    test_now_ms = 0u;
    vision_uart_board_test_init();
    feed_line("VISION_READY BOX");
    vision_uart_board_test_poll();
    test_now_ms = 5001u;
    vision_uart_board_test_poll();
    vision_uart_board_test_get_status(&board_status);
    passed &= run_case("board-test-timeout", VISION_UART_BOARD_TEST_FAIL == board_status.state);
#else
    passed &= run_case("board-test-disabled",
                       VISION_UART_BOARD_TEST_DISABLED == board_status.state);
#endif

    return (0u != passed) ? 0 : 1;
}
