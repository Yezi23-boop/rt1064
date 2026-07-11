#include "zf_common_headfile.h"
#include "vision_uart.h"

#define VISION_UART_INDEX          (UART_4)
#define VISION_RX_BUFFER_SIZE      (256u)
#define VISION_LINE_SIZE           (64u)
#define VISION_SAMPLE_QUEUE_SIZE   (8u)

static volatile uint8 rx_buffer[VISION_RX_BUFFER_SIZE];
static volatile uint16 rx_write_index;
static volatile uint16 rx_read_index;
static char line_buffer[VISION_LINE_SIZE];
static uint16 line_length;

static vision_sample_struct sample_queue[VISION_SAMPLE_QUEUE_SIZE];
static uint8 sample_write_index;
static uint8 sample_read_index;
static uint8 sample_count;

static vision_mode_enum requested_mode;
static vision_mode_enum ready_mode;
static uint16 current_request_id;
static uint8 request_active;

static uint16 next_index(uint16 index, uint16 capacity)
{
    index++;
    return (index >= capacity) ? 0u : index;
}

static void clear_samples(void)
{
    uint32 primask = interrupt_global_disable();
    sample_write_index = 0u;
    sample_read_index = 0u;
    sample_count = 0u;
    interrupt_global_enable(primask);
}

static uint8 parse_uint16_text(const char *text, uint16 *value)
{
    uint32 result = 0u;
    uint8 digit_count = 0u;

    if((0 == text) || (0 == value))
    {
        return 0u;
    }
    while(('0' <= *text) && ('9' >= *text))
    {
        result = result * 10u + (uint32)(*text - '0');
        if(65535u < result)
        {
            return 0u;
        }
        digit_count++;
        text++;
    }
    if((0u == digit_count) || ('\0' != *text))
    {
        return 0u;
    }
    *value = (uint16)result;
    return 1u;
}

static char *next_token(char **text)
{
    char *token;

    while(' ' == **text)
    {
        (*text)++;
    }
    if('\0' == **text)
    {
        return 0;
    }
    token = *text;
    while(('\0' != **text) && (' ' != **text))
    {
        (*text)++;
    }
    if(' ' == **text)
    {
        **text = '\0';
        (*text)++;
    }
    return token;
}

static void append_u16(char *text, uint16 *length, uint16 value)
{
    char reverse[5];
    uint8 count = 0u;

    do
    {
        reverse[count++] = (char)('0' + (value % 10u));
        value = (uint16)(value / 10u);
    } while((0u != value) && (count < sizeof(reverse)));

    while(0u != count)
    {
        text[(*length)++] = reverse[--count];
    }
}

static void send_id_command(const char *prefix, uint16 request_id)
{
    char tx_line[32];
    uint16 length = 0u;

    while(('\0' != *prefix) && (length < (sizeof(tx_line) - 1u)))
    {
        tx_line[length++] = *prefix++;
    }
    append_u16(tx_line, &length, request_id);
    tx_line[length++] = '\n';
    tx_line[length] = '\0';
    uart_write_string(VISION_UART_INDEX, tx_line);
}

static void queue_sample(const vision_sample_struct *sample)
{
    uint32 primask = interrupt_global_disable();

    if(sample_count < VISION_SAMPLE_QUEUE_SIZE)
    {
        sample_queue[sample_write_index] = *sample;
        sample_write_index = (uint8)next_index(sample_write_index,
                                                VISION_SAMPLE_QUEUE_SIZE);
        sample_count++;
    }
    interrupt_global_enable(primask);
}

static void parse_ready_line(void)
{
    if(0 == strcmp(line_buffer, "VISION_READY BOX"))
    {
        ready_mode = VISION_MODE_BOX;
    }
    else if(0 == strcmp(line_buffer, "VISION_READY TARGET"))
    {
        ready_mode = VISION_MODE_TARGET;
    }
}

static void parse_sample_line(void)
{
    char *text = line_buffer;
    char *token;
    uint16 values[4];
    uint8 index;
    vision_sample_struct sample;

    token = next_token(&text);
    if((0 == token) || (0 != strcmp(token, "VISION_SAMPLE")))
    {
        return;
    }
    for(index = 0u; index < 4u; index++)
    {
        token = next_token(&text);
        if((0 == token) || (0 == parse_uint16_text(token, &values[index])))
        {
            return;
        }
    }
    if(0 != next_token(&text))
    {
        return;
    }
    if((0u == request_active) ||
       (current_request_id != values[0]) ||
       (9u < values[2]) ||
       (1000u < values[3]))
    {
        return;
    }

    sample.request_id = values[0];
    sample.sample_id = values[1];
    sample.class_id = (uint8)values[2];
    sample.confidence_q = values[3];
    queue_sample(&sample);
}

static void parse_line(void)
{
    line_buffer[line_length] = '\0';
    if(0 == strncmp(line_buffer, "VISION_READY ", 13u))
    {
        parse_ready_line();
    }
    else if(0 == strncmp(line_buffer, "VISION_SAMPLE ", 14u))
    {
        parse_sample_line();
    }
}

static void parse_byte(uint8 data)
{
    if('\r' == data)
    {
        return;
    }
    if('\n' == data)
    {
        if(0u != line_length)
        {
            parse_line();
            line_length = 0u;
        }
        return;
    }
    if(line_length >= (VISION_LINE_SIZE - 1u))
    {
        line_length = 0u;
        return;
    }
    line_buffer[line_length++] = (char)data;
}

void vision_uart_init(void)
{
    rx_write_index = 0u;
    rx_read_index = 0u;
    line_length = 0u;
    requested_mode = VISION_MODE_NONE;
    ready_mode = VISION_MODE_NONE;
    current_request_id = 0u;
    request_active = 0u;
    clear_samples();
    uart_init(UART_4, 115200u, UART4_TX_C16, UART4_RX_C17);
    uart_rx_interrupt(UART_4, 1u);
}

void vision_uart_push_byte(uint8 data)
{
    uint16 next = next_index(rx_write_index, VISION_RX_BUFFER_SIZE);
    if(next == rx_read_index)
    {
        return;
    }
    rx_buffer[rx_write_index] = data;
    rx_write_index = next;
}

void vision_uart_poll(void)
{
    while(rx_read_index != rx_write_index)
    {
        uint8 data = rx_buffer[rx_read_index];
        rx_read_index = next_index(rx_read_index, VISION_RX_BUFFER_SIZE);
        parse_byte(data);
    }
}

void vision_uart_set_mode(vision_mode_enum mode)
{
    if((VISION_MODE_BOX != mode) && (VISION_MODE_TARGET != mode))
    {
        return;
    }
    requested_mode = mode;
    ready_mode = VISION_MODE_NONE;
    request_active = 0u;
    clear_samples();
    uart_write_string(VISION_UART_INDEX,
        (VISION_MODE_BOX == mode) ? "VISION_MODE BOX\n" : "VISION_MODE TARGET\n");
}

uint8 vision_uart_mode_ready(vision_mode_enum mode)
{
    return ((requested_mode == mode) && (ready_mode == mode)) ? 1u : 0u;
}

uint16 vision_uart_request_classification(void)
{
    current_request_id++;
    if(0u == current_request_id)
    {
        current_request_id = 1u;
    }
    request_active = 1u;
    clear_samples();
    send_id_command("VISION_REQ ", current_request_id);
    return current_request_id;
}

uint8 vision_uart_get_sample(vision_sample_struct *sample)
{
    uint8 available = 0u;
    uint32 primask;

    if(0 == sample)
    {
        return 0u;
    }
    primask = interrupt_global_disable();
    if(0u != sample_count)
    {
        *sample = sample_queue[sample_read_index];
        sample_read_index = (uint8)next_index(sample_read_index,
                                               VISION_SAMPLE_QUEUE_SIZE);
        sample_count--;
        available = 1u;
    }
    interrupt_global_enable(primask);
    return available;
}

void vision_uart_ack(uint16 request_id)
{
    if((0u == request_active) || (current_request_id != request_id))
    {
        return;
    }
    request_active = 0u;
    clear_samples();
    send_id_command("VISION_ACK ", request_id);
}

void vision_uart_cancel(void)
{
    request_active = 0u;
    clear_samples();
    uart_write_string(VISION_UART_INDEX, "VISION_CANCEL\n");
}
