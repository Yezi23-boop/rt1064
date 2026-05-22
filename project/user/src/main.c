#include "zf_common_headfile.h"
#include <string.h>

#define MAP_ROWS 12
#define MAP_COLS 16
#define LINE_BUF_SIZE 64

static void uart_process_line(const uint8 *line, uint32 len, uint8 *in_map, uint8 *map_rows)
{
    if((len == 9) && (0 == memcmp(line, "MAP_BEGIN", 9)))
    {
        *in_map = 1;
        *map_rows = 0;
        debug_send_buffer((const uint8 *)"MAP_RX_BEGIN\r\n", 14);
        return;
    }

    if((len == 7) && (0 == memcmp(line, "MAP_END", 7)))
    {
        if((*in_map) && (*map_rows == MAP_ROWS))
        {
            debug_send_buffer((const uint8 *)"MAP_OK rows=12 cols=16\r\n", 24);
        }
        else
        {
            debug_send_buffer((const uint8 *)"MAP_ERR\r\n", 9);
        }

        *in_map = 0;
        *map_rows = 0;
        return;
    }

    if(*in_map)
    {
        if((*map_rows < MAP_ROWS) && (MAP_COLS == len))
        {
            *map_rows += 1;
            return;
        }

        debug_send_buffer((const uint8 *)"MAP_ERR_LINE\r\n", 14);
        *in_map = 0;
        *map_rows = 0;
        return;
    }

    debug_send_buffer((const uint8 *)"RX:", 3);
    debug_send_buffer(line, len);
    debug_send_buffer((const uint8 *)"\r\n", 2);
}

int main(void)
{
    uint8 rx_buffer[32];
    uint32 rx_len;
    uint8 line_buf[LINE_BUF_SIZE];
    uint32 line_len = 0;
    uint8 in_map = 0;
    uint8 map_rows = 0;
    uint32 i;

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    printf("\r\nRT1064_OPENART_UART_READY\r\n");
    printf("UART_1 TX=B12 RX=B13 BAUD=115200\r\n");

    while(1)
    {
        rx_len = debug_read_ring_buffer(rx_buffer, sizeof(rx_buffer));
        if(0 != rx_len)
        {
            for(i = 0; i < rx_len; i++)
            {
                uint8 ch = rx_buffer[i];

                if('\r' == ch)
                {
                    continue;
                }

                if('\n' == ch)
                {
                    if(line_len > 0)
                    {
                        uart_process_line(line_buf, line_len, &in_map, &map_rows);
                        line_len = 0;
                    }
                    continue;
                }

                if(line_len < (LINE_BUF_SIZE - 1))
                {
                    line_buf[line_len++] = ch;
                }
                else
                {
                    line_len = 0;
                    in_map = 0;
                    map_rows = 0;
                    debug_send_buffer((const uint8 *)"MAP_ERR_OVERFLOW\r\n", 18);
                }
            }
        }

        system_delay_ms(10);
    }
}
