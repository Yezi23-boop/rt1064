#include <stdio.h>
#include <string.h>
#include "map_utils.h"
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
    uint16 yaw_q = 0u;
    uint8 yaw_valid = 0u;
    uint8 index = openart_get_requested_center_sample(&col_q, &row_q,
                                                       &yaw_q, &yaw_valid);

    return ((expected_index == index) &&
            (expected_col == col_q) &&
            (expected_row == row_q)) ? 1u : 0u;
}

static uint8 run_case(const char *name, uint8 passed)
{
    printf("%-28s %s\n", name, (0 != passed) ? "PASS" : "FAIL");
    return passed;
}

static uint8 pop_sample_with_yaw_equals(uint8 expected_index,
                                        uint16 expected_col, uint16 expected_row,
                                        uint16 expected_yaw, uint8 expected_valid)
{
    uint16 col_q = 0u;
    uint16 row_q = 0u;
    uint16 yaw_q = 0u;
    uint8 yaw_valid = 0u;
    uint8 index = openart_get_requested_center_sample(&col_q, &row_q,
                                                       &yaw_q, &yaw_valid);

    return ((expected_index == index) &&
            (expected_col == col_q) && (expected_row == row_q) &&
            (expected_yaw == yaw_q) &&
            (expected_valid == yaw_valid)) ? 1u : 0u;
}

static uint8 pop_observation_equals(uint8 expected_available,
                                    uint16 car_col_q, uint16 car_row_q,
                                    uint16 box_col_q, uint16 box_row_q)
{
    openart_observation_sample_struct sample;
    uint8 available;

    memset(&sample, 0, sizeof(sample));
    available = openart_get_observation_sample(&sample);
    return ((expected_available == available) &&
            ((0u == available) ||
             ((car_col_q == sample.car_col_q) &&
              (car_row_q == sample.car_row_q) &&
              (box_col_q == sample.box_col_q) &&
              (box_row_q == sample.box_row_q)))) ? 1u : 0u;
}

static void feed_map_frame(const char *row_five,
                           uint16 center_col_q,
                           uint16 center_row_q,
                           uint8 center_valid)
{
    char center_line[48];

    feed_line("MAP_BEGIN");
    feed_line("################");
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line(row_five);
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line("################");
    snprintf(center_line, sizeof(center_line),
             "PLAYER_CENTER_GRID %u,%u %u",
             center_col_q, center_row_q, center_valid);
    feed_line(center_line);
    feed_line("MAP_END");
}

static void feed_map_frame_without_center(const char *row_five)
{
    feed_line("MAP_BEGIN");
    feed_line("################");
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line(row_five);
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line("#..............#");
    feed_line("################");
    feed_line("MAP_END");
}

static void feed_center_sample_with_map(uint8 index, uint16 col_q, uint16 row_q)
{
    char sample_line[48];

    feed_map_frame("#....C.........#", col_q, row_q, 1u);
    snprintf(sample_line, sizeof(sample_line),
             "CENTER_SAMPLE %u,%u,%u,18000,1", index, col_q, row_q);
    feed_line(sample_line);
}

static void feed_observation_sample_with_map(uint8 index,
                                             uint16 car_col_q,
                                             uint16 car_row_q,
                                             uint16 box_col_q,
                                             uint16 box_row_q)
{
    char sample_line[48];

    feed_map_frame("#......CB......#", car_col_q, car_row_q, 1u);
    snprintf(sample_line, sizeof(sample_line),
             "OBSERVE_SAMPLE %u,%u,%u,%u,%u",
             index, car_col_q, car_row_q, box_col_q, box_row_q);
    feed_line(sample_line);
}

int main(void)
{
    const map_source_struct *map;
    const map_source_struct *paired_map;
    uint8 player_row = 0u;
    uint8 player_col = 0u;
    uint8 player_count = 0u;
    map_scan_stats_struct stats;
    map_scan_stats_struct paired_stats;
    uint32 frame_before;
    uint8 passed = 1u;

    openart_uart_init();
    memset(tx_text, 0, sizeof(tx_text));
    openart_request_player_center();
    passed &= run_case("request-command", 0 == strcmp(tx_text, "CENTER_REQ\n"));
    passed &= run_case("request-clears-samples", pop_sample_equals(0u, 0u, 0u));

    feed_line("CENTER_SAMPLE 1,568,550,18000,1");
    passed &= run_case("unpaired-sample-rejected", pop_sample_equals(0u, 0u, 0u));
    feed_map_frame("#....C.........#", 568u, 550u, 1u);
    feed_line("CENTER_SAMPLE 1,568,550");
    passed &= run_case("old-center-protocol-rejected", pop_sample_equals(0u, 0u, 0u));
    feed_map_frame("#....C.........#", 568u, 550u, 1u);
    feed_line("CENTER_SAMPLE 1,569,550,18000,1");
    passed &= run_case("mismatched-sample-rejected", pop_sample_equals(0u, 0u, 0u));
    feed_center_sample_with_map(1u, 568u, 550u);
    feed_line("CENTER_SAMPLE 2,568,550,18000,1");
    passed &= run_case("sample-requires-new-map",
        pop_sample_with_yaw_equals(1u, 568u, 550u, 18000u, 1u));
    feed_center_sample_with_map(2u, 570u, 550u);
    feed_center_sample_with_map(3u, 572u, 550u);
    feed_center_sample_with_map(4u, 574u, 550u);
    feed_center_sample_with_map(5u, 576u, 550u);
    passed &= run_case("queued-sample-two", pop_sample_equals(2u, 570u, 550u));
    passed &= run_case("queued-sample-three", pop_sample_equals(3u, 572u, 550u));
    passed &= run_case("queued-sample-four", pop_sample_equals(4u, 574u, 550u));
    passed &= run_case("queued-sample-five", pop_sample_equals(5u, 576u, 550u));
    passed &= run_case("queue-empty", pop_sample_equals(0u, 0u, 0u));

    frame_before = openart_uart_get_frame_count();
    feed_map_frame("#.C..C.........#", 0u, 0u, 0u);
    map = openart_map_get();
    paired_map = openart_get_requested_center_map();
    map_scan_stats(map, &stats);
    map_scan_stats(paired_map, &paired_stats);
    passed &= run_case("multi-car-map-rejected",
        (frame_before == openart_uart_get_frame_count()) &&
        (1u == stats.car_count) &&
        (5u == stats.car_row) && (5u == stats.car_col) &&
        (1u == paired_stats.car_count) &&
        (5u == paired_stats.car_row) && (5u == paired_stats.car_col));

    frame_before = openart_uart_get_frame_count();
    feed_map_frame_without_center("#....C.........#");
    map = openart_map_get();
    passed &= run_case("map-without-center-accepted",
        ((frame_before + 1u) == openart_uart_get_frame_count()) &&
        (0u != map_find_car(map, &player_row, &player_col, &player_count)) &&
        (5u == player_row) && (5u == player_col) && (1u == player_count));

    openart_request_player_center();
    passed &= run_case("new-request-clears-paired-map",
                       NULL == openart_get_requested_center_map());
    feed_line("CENTER_SAMPLE 1,600,500,18000,1");
    openart_request_player_center();
    passed &= run_case("new-request-clears-queue", pop_sample_equals(0u, 0u, 0u));

    memset(tx_text, 0, sizeof(tx_text));
    openart_request_observation(5u, 8u);
    passed &= run_case("observe-request-command",
                       0 == strcmp(tx_text, "OBSERVE_REQ 5,8\n"));
    feed_line("OBSERVE_SAMPLE 1,750,550,850,550");
    passed &= run_case("observe-unpaired-rejected",
        pop_observation_equals(0u, 0u, 0u, 0u, 0u));
    feed_observation_sample_with_map(1u, 750u, 550u, 850u, 550u);
    feed_observation_sample_with_map(3u, 752u, 551u, 852u, 551u);
    feed_observation_sample_with_map(2u, 751u, 550u, 931u, 550u);
    feed_observation_sample_with_map(2u, 751u, 550u, 930u, 550u);
    feed_observation_sample_with_map(3u, 752u, 551u, 852u, 551u);
    passed &= run_case("observe-sample-one",
        pop_observation_equals(1u, 750u, 550u, 850u, 550u));
    passed &= run_case("observe-sample-two",
        pop_observation_equals(1u, 751u, 550u, 930u, 550u));
    passed &= run_case("observe-sample-three",
        pop_observation_equals(1u, 752u, 551u, 852u, 551u));
    passed &= run_case("observe-queue-empty",
        pop_observation_equals(0u, 0u, 0u, 0u, 0u));

    openart_request_observation(5u, 8u);
    feed_observation_sample_with_map(1u, 750u, 550u, 850u, 550u);
    openart_request_observation(6u, 9u);
    passed &= run_case("observe-new-request-clears",
        pop_observation_equals(0u, 0u, 0u, 0u, 0u));

    feed_map_frame("#....C.........#", 550u, 550u, 1u);
    frame_before = openart_uart_get_frame_count();
    feed_map_frame("#.C..C.........#", 550u, 550u, 1u);
    map = openart_map_get();
    passed &= run_case("center-does-not-rewrite-map",
        (frame_before == openart_uart_get_frame_count()) &&
        (0 != map) &&
        ('C' == map->rows[5][5]) &&
        (0 != openart_find_player_cell(&player_row, &player_col, &player_count)) &&
        (5u == player_row) && (5u == player_col) && (1u == player_count));

    frame_before = openart_uart_get_frame_count();
    feed_map_frame("#....C.........#", 650u, 550u, 1u);
    passed &= run_case("center-cell-mismatch-rejected",
                       frame_before == openart_uart_get_frame_count());

    feed_map_frame("#....+.........#", 550u, 550u, 1u);
    map = openart_map_get();
    map_scan_stats(map, &stats);
    passed &= run_case("car-on-target-keeps-target",
        ('+' == map->rows[5][5]) &&
        (1u == stats.car_count) && (1u == stats.target_count) &&
        (5u == stats.car_row) && (5u == stats.car_col));

    feed_map_frame("#....TC........#", 650u, 550u, 1u);
    map = openart_map_get();
    map_scan_stats(map, &stats);
    passed &= run_case("leaving-target-restores-target",
        ('T' == map->rows[5][5]) && ('C' == map->rows[5][6]) &&
        (1u == stats.car_count) && (1u == stats.target_count));

    feed_map_frame("#....+.........#", 0u, 0u, 0u);
    map = openart_map_get();
    map_scan_stats(map, &stats);
    passed &= run_case("coarse-car-keeps-target",
        ('+' == map->rows[5][5]) &&
        (1u == stats.car_count) && (1u == stats.target_count));

    feed_map_frame("#....TC........#", 0u, 0u, 0u);
    map = openart_map_get();
    map_scan_stats(map, &stats);
    passed &= run_case("coarse-car-leaves-target",
        ('T' == map->rows[5][5]) && ('C' == map->rows[5][6]) &&
        (1u == stats.car_count) && (1u == stats.target_count));

    return (0 != passed) ? 0 : 1;
}
