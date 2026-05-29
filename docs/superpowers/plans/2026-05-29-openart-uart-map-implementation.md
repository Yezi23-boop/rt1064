# OpenART UART Map Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add OpenART-to-RT1064 UART map transfer where OpenART sends stable 12x16 text frames and RT1064 receives them on UART1 using interrupt buffering plus main-loop parsing.

**Architecture:** OpenART owns map production and periodically sends `MAP_BEGIN` / 12 map rows / `MAP_END` over `UART(12)`. RT1064 owns UART1 byte reception in `LPUART1_IRQHandler()`, stores bytes in a small ring buffer, and parses full frames in `app_poll()` without doing heavy work in the ISR.

**Tech Stack:** OpenMV MicroPython on OpenART Plus, SeekFree RT1064 C firmware, `UART(12)` on OpenART, `UART_1` B12/B13 on RT1064, PowerShell verification, Keil/uVision build.

---

## File Structure

- Modify `C:/Users/ye/Desktop/rt1064/openmv/openart_plus_grid_recognizer_pure.py`
  - Add UART configuration constants near the top configuration area.
  - Initialize OpenART `UART(12)` once.
  - Send stable `char_matrix` as a text frame on a fixed period.
  - Keep IDE debug printing independent from UART sending.

- Create `C:/Users/ye/Desktop/rt1064/project/user/inc/openart_uart.h`
  - Public RT1064 OpenART UART interface.
  - Expose init, ISR handler, main-loop service, latest map query, and counters.

- Create `C:/Users/ye/Desktop/rt1064/project/user/src/openart_uart.c`
  - Own UART1 ring buffer.
  - Parse line-based map protocol.
  - Store latest valid 12x16 map.
  - Reply `MAP_OK rows=12 cols=16\r\n` after accepting a complete frame.

- Modify `C:/Users/ye/Desktop/rt1064/project/user/src/main.c`
  - Include `openart_uart.h`.
  - Call `openart_uart_init()` during startup.

- Modify `C:/Users/ye/Desktop/rt1064/project/user/src/app.c`
  - Include `openart_uart.h`.
  - Call `openart_uart_service()` in `app_poll()`.

- Modify `C:/Users/ye/Desktop/rt1064/project/user/src/isr.c`
  - Include `openart_uart.h`.
  - Route `LPUART1_IRQHandler()` RX bytes to `openart_uart_handler()` instead of `debug_interrupr_handler()`.

- Modify `C:/Users/ye/Desktop/rt1064/project/mdk/rt1064.uvprojx`
  - Add the new `openart_uart.c` and `openart_uart.h` files to the Keil project.

## Task 1: OpenART UART Frame Sender

**Files:**
- Modify: `C:/Users/ye/Desktop/rt1064/openmv/openart_plus_grid_recognizer_pure.py`

- [ ] **Step 1: Add UART import and configuration constants**

Insert the import after the existing imports:

```python
import sensor
import time
from machine import UART
```

Add this grouped configuration after camera lock switches:

```python
# OpenART -> RT1064 UART 地图发送。使用逐飞 OpenART Plus 的 UART(12) 引脚映射。
UART_ENABLE = True          # True=定时发送字符地图到 RT1064；False=只在 IDE 打印/显示
UART_ID = 12                # OpenART Plus 用户串口使用 UART(12)
UART_BAUDRATE = 115200
UART_SEND_PERIOD_MS = 200   # 地图发送周期；200ms 足够 MCU 更新地图且不挤占视觉帧率
```

- [ ] **Step 2: Add UART initialization helper**

Add these functions before `init_camera()`:

```python
def init_map_uart():
    if not UART_ENABLE:
        print("OPENART_UART_DISABLED")
        return None

    try:
        try:
            import cmm_load
            cmm_load.load()
        except Exception as exc:
            print("cmm_load skipped or failed:", repr(exc))

        uart = UART(UART_ID, baudrate=UART_BAUDRATE)
        print("OPENART_UART_READY UART(%d) %d" % (UART_ID, UART_BAUDRATE))
        return uart
    except Exception as exc:
        print("OPENART_UART_INIT_FAILED:", repr(exc))
        return None
```

- [ ] **Step 3: Add frame sender helper**

Add this function after `print_map()`:

```python
def send_map_frame(uart, char_matrix):
    if uart is None:
        return False

    try:
        uart.write("MAP_BEGIN\r\n")
        for row in char_matrix:
            uart.write("".join(row))
            uart.write("\r\n")
        uart.write("MAP_END\r\n")
        return True
    except Exception as exc:
        print("OPENART_UART_SEND_FAILED:", repr(exc))
        return False
```

- [ ] **Step 4: Hook UART send into `main()`**

In `main()`, after `init_camera()` add:

```python
    map_uart = init_map_uart()
```

After `last_print_ms = time.ticks_ms()` add:

```python
    last_uart_send_ms = time.ticks_ms()
```

After `display_done_us = time.ticks_us()` add:

```python
        if (UART_ENABLE and
                time.ticks_diff(now_ms, last_uart_send_ms) >= UART_SEND_PERIOD_MS):
            send_map_frame(map_uart, char_matrix)
            last_uart_send_ms = now_ms
```

- [ ] **Step 5: Verify OpenART syntax**

Run:

```powershell
uv run python -m py_compile "C:\Users\ye\Desktop\rt1064\openmv\openart_plus_grid_recognizer_pure.py"
```

Expected: no syntax error output.

- [ ] **Step 6: Commit OpenART sender**

Run:

```powershell
git -C "C:\Users\ye\Desktop\rt1064" add -- "openmv/openart_plus_grid_recognizer_pure.py"
git -C "C:\Users\ye\Desktop\rt1064" commit -m "添加 OpenART UART 地图发送"
```

## Task 2: RT1064 OpenART UART Parser Module

**Files:**
- Create: `C:/Users/ye/Desktop/rt1064/project/user/inc/openart_uart.h`
- Create: `C:/Users/ye/Desktop/rt1064/project/user/src/openart_uart.c`

- [ ] **Step 1: Create public header**

Create `project/user/inc/openart_uart.h` with:

```c
#ifndef _openart_uart_h_
#define _openart_uart_h_

#include "zf_common_typedef.h"
#include "map_types.h"

void openart_uart_init(void);
void openart_uart_handler(void);
void openart_uart_service(void);

uint8 openart_uart_has_map(void);
const char (*openart_uart_get_map(void))[MAP_COLS + 1];
uint32 openart_uart_get_frame_count(void);
uint32 openart_uart_get_error_count(void);

#endif
```

- [ ] **Step 2: Create C module skeleton and constants**

Create `project/user/src/openart_uart.c` with:

```c
#include "openart_uart.h"
#include "zf_common_headfile.h"
#include "zf_driver_uart.h"

#define OPENART_UART_INDEX          (UART_1)
#define OPENART_UART_BAUDRATE       (115200)
#define OPENART_UART_TX_PIN         (UART1_TX_B12)
#define OPENART_UART_RX_PIN         (UART1_RX_B13)
#define OPENART_RX_BUFFER_SIZE      (384u)
#define OPENART_LINE_BUFFER_SIZE    (24u)

typedef enum
{
    OPENART_PARSE_WAIT_BEGIN = 0,
    OPENART_PARSE_READ_ROWS,
} openart_parse_state_enum;

static volatile uint8 openart_rx_buffer[OPENART_RX_BUFFER_SIZE];
static volatile uint16 openart_rx_write_index;
static volatile uint16 openart_rx_read_index;
static volatile uint32 openart_rx_overflow_count;

static openart_parse_state_enum parse_state;
static char line_buffer[OPENART_LINE_BUFFER_SIZE];
static uint8 line_length;
static uint8 staging_row_count;
static char staging_map[MAP_ROWS][MAP_COLS + 1];
static char latest_map[MAP_ROWS][MAP_COLS + 1];
static uint8 latest_map_valid;
static uint32 frame_count;
static uint32 error_count;
```

- [ ] **Step 3: Add ring buffer helpers**

Append:

```c
static uint16 next_rx_index(uint16 index)
{
    index++;
    if(OPENART_RX_BUFFER_SIZE <= index)
    {
        index = 0;
    }
    return index;
}

static void openart_rx_push(uint8 data)
{
    uint16 next_index = next_rx_index(openart_rx_write_index);

    if(next_index == openart_rx_read_index)
    {
        openart_rx_overflow_count++;
        error_count++;
        return;
    }

    openart_rx_buffer[openart_rx_write_index] = data;
    openart_rx_write_index = next_index;
}

static uint8 openart_rx_pop(uint8 *data)
{
    if(openart_rx_read_index == openart_rx_write_index)
    {
        return 0;
    }

    *data = openart_rx_buffer[openart_rx_read_index];
    openart_rx_read_index = next_rx_index(openart_rx_read_index);
    return 1;
}
```

- [ ] **Step 4: Add protocol helpers**

Append:

```c
static uint8 str_equal(const char *left, const char *right)
{
    while(('\0' != *left) && ('\0' != *right))
    {
        if(*left != *right)
        {
            return 0;
        }
        left++;
        right++;
    }

    return ('\0' == *left) && ('\0' == *right);
}

static uint8 is_map_char(char ch)
{
    return ('#' == ch) ||
           ('.' == ch) ||
           ('B' == ch) ||
           ('T' == ch) ||
           ('C' == ch) ||
           ('X' == ch);
}

static uint8 is_valid_map_line(const char *line)
{
    uint8 col;

    for(col = 0; col < MAP_COLS; col++)
    {
        if(0 == is_map_char(line[col]))
        {
            return 0;
        }
    }

    return '\0' == line[MAP_COLS];
}

static void reset_parser(void)
{
    parse_state = OPENART_PARSE_WAIT_BEGIN;
    line_length = 0;
    staging_row_count = 0;
}
```

- [ ] **Step 5: Add line parser**

Append:

```c
static void accept_staging_map(void)
{
    uint8 row;

    for(row = 0; row < MAP_ROWS; row++)
    {
        memcpy(latest_map[row], staging_map[row], MAP_COLS + 1);
    }

    latest_map_valid = 1;
    frame_count++;
    uart_write_string(OPENART_UART_INDEX, "MAP_OK rows=12 cols=16\r\n");
}

static void parse_line(const char *line)
{
    if(str_equal(line, "MAP_BEGIN"))
    {
        parse_state = OPENART_PARSE_READ_ROWS;
        staging_row_count = 0;
        return;
    }

    if(OPENART_PARSE_WAIT_BEGIN == parse_state)
    {
        return;
    }

    if(str_equal(line, "MAP_END"))
    {
        if(MAP_ROWS == staging_row_count)
        {
            accept_staging_map();
        }
        else
        {
            error_count++;
        }
        reset_parser();
        return;
    }

    if(MAP_ROWS <= staging_row_count)
    {
        error_count++;
        reset_parser();
        return;
    }

    if(0 == is_valid_map_line(line))
    {
        error_count++;
        reset_parser();
        return;
    }

    memcpy(staging_map[staging_row_count], line, MAP_COLS + 1);
    staging_row_count++;
}
```

- [ ] **Step 6: Add public functions**

Append:

```c
void openart_uart_init(void)
{
    uint8 row;

    openart_rx_write_index = 0;
    openart_rx_read_index = 0;
    openart_rx_overflow_count = 0;
    latest_map_valid = 0;
    frame_count = 0;
    error_count = 0;
    reset_parser();

    for(row = 0; row < MAP_ROWS; row++)
    {
        memset(latest_map[row], 0, MAP_COLS + 1);
        memset(staging_map[row], 0, MAP_COLS + 1);
    }

    uart_init(OPENART_UART_INDEX, OPENART_UART_BAUDRATE,
              OPENART_UART_TX_PIN, OPENART_UART_RX_PIN);
    uart_rx_interrupt(OPENART_UART_INDEX, 1);
}

void openart_uart_handler(void)
{
    uint8 data;

    while(0 != uart_query_byte(OPENART_UART_INDEX, &data))
    {
        openart_rx_push(data);
    }
}

void openart_uart_service(void)
{
    uint8 data;

    while(0 != openart_rx_pop(&data))
    {
        if('\r' == data)
        {
            continue;
        }

        if('\n' == data)
        {
            line_buffer[line_length] = '\0';
            if(0 != line_length)
            {
                parse_line(line_buffer);
            }
            line_length = 0;
            continue;
        }

        if(line_length >= (OPENART_LINE_BUFFER_SIZE - 1u))
        {
            error_count++;
            reset_parser();
            line_length = 0;
            continue;
        }

        line_buffer[line_length] = (char)data;
        line_length++;
    }
}

uint8 openart_uart_has_map(void)
{
    return latest_map_valid;
}

const char (*openart_uart_get_map(void))[MAP_COLS + 1]
{
    return latest_map;
}

uint32 openart_uart_get_frame_count(void)
{
    return frame_count;
}

uint32 openart_uart_get_error_count(void)
{
    return error_count + openart_rx_overflow_count;
}
```

- [ ] **Step 7: Verify C file references**

Run:

```powershell
Select-String -Path "C:\Users\ye\Desktop\rt1064\project\user\src\openart_uart.c" -Pattern "MAP_ROWS|MAP_COLS|uart_init|uart_rx_interrupt" -Encoding UTF8
```

Expected: references to `MAP_ROWS`, `MAP_COLS`, `uart_init`, and `uart_rx_interrupt` are present.

## Task 3: Wire RT1064 Module Into Startup, Polling, ISR, and Keil Project

**Files:**
- Modify: `C:/Users/ye/Desktop/rt1064/project/user/src/main.c`
- Modify: `C:/Users/ye/Desktop/rt1064/project/user/src/app.c`
- Modify: `C:/Users/ye/Desktop/rt1064/project/user/src/isr.c`
- Modify: `C:/Users/ye/Desktop/rt1064/project/mdk/rt1064.uvprojx`

- [ ] **Step 1: Include and initialize OpenART UART in `main.c`**

Modify includes:

```c
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "drive_control.h"
#include "openart_uart.h"
#include "app.h"
```

Modify startup sequence:

```c
    debug_init();
    wireless_uart_init();
    openart_uart_init();
    control_init_state = control_init();
```

- [ ] **Step 2: Poll OpenART parser in `app.c`**

Modify includes:

```c
#include "app.h"
#include "settings.h"
#include "timebase.h"
#include "screen.h"
#include "menu.h"
#include "vofa.h"
#include "drive_test.h"
#include "openart_uart.h"
```

Modify `app_poll()`:

```c
void app_poll(void)
{
    openart_uart_service();
    menu_poll();
    vofa_service();
    drive_test_poll();
}
```

- [ ] **Step 3: Route UART1 ISR to OpenART handler**

Modify `isr.c` includes:

```c
#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "drive_control.h"
#include "menu_key.h"
#include "openart_uart.h"
#include "isr.h"
```

Replace `LPUART1_IRQHandler()` with:

```c
void LPUART1_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        openart_uart_handler();
    }

    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);
}
```

- [ ] **Step 4: Add new files to Keil project**

In `project/mdk/rt1064.uvprojx`, add the source and header near other user files:

```xml
              <File>
                <FileName>openart_uart.c</FileName>
                <FileType>1</FileType>
                <FilePath>..\user\src\openart_uart.c</FilePath>
              </File>
              <File>
                <FileName>openart_uart.h</FileName>
                <FileType>5</FileType>
                <FilePath>..\user\inc\openart_uart.h</FilePath>
              </File>
```

Keep the XML style consistent with the existing project file.

- [ ] **Step 5: Verify project references**

Run:

```powershell
Select-String -Path "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx" -Pattern "openart_uart.c|openart_uart.h" -Encoding UTF8
Select-String -Path "C:\Users\ye\Desktop\rt1064\project\user\src\main.c","C:\Users\ye\Desktop\rt1064\project\user\src\app.c","C:\Users\ye\Desktop\rt1064\project\user\src\isr.c" -Pattern "openart_uart" -Encoding UTF8
```

Expected: both new Keil file entries exist; `main.c`, `app.c`, and `isr.c` each reference `openart_uart`.

- [ ] **Step 6: Build RT1064 project**

Run the local Keil build command:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

Expected: build finishes without compiler errors. If Keil is not installed at `D:\Keil_v5\UV4\UV4.exe`, report that build could not be run and use the local syntax/static searches from Step 5 as partial verification.

- [ ] **Step 7: Commit RT1064 receiver**

Run:

```powershell
git -C "C:\Users\ye\Desktop\rt1064" add -- "project/user/inc/openart_uart.h" "project/user/src/openart_uart.c" "project/user/src/main.c" "project/user/src/app.c" "project/user/src/isr.c" "project/mdk/rt1064.uvprojx"
git -C "C:\Users\ye\Desktop\rt1064" commit -m "添加 RT1064 OpenART UART 接收解析"
```

## Task 4: Integration Smoke Test

**Files:**
- Read-only verification of `C:/Users/ye/Desktop/rt1064/openmv/openart_plus_grid_recognizer_pure.py`
- Read-only verification of `C:/Users/ye/Desktop/rt1064/project/user/src/openart_uart.c`

- [ ] **Step 1: Confirm OpenART sends the agreed protocol**

Run:

```powershell
Select-String -Path "C:\Users\ye\Desktop\rt1064\openmv\openart_plus_grid_recognizer_pure.py" -Pattern "MAP_BEGIN|MAP_END|UART\\(12\\)|UART_ID = 12|UART_SEND_PERIOD_MS|send_map_frame" -Encoding UTF8
```

Expected: all protocol markers and UART(12) configuration are present.

- [ ] **Step 2: Confirm RT1064 parser accepts the same protocol**

Run:

```powershell
Select-String -Path "C:\Users\ye\Desktop\rt1064\project\user\src\openart_uart.c" -Pattern "MAP_BEGIN|MAP_END|MAP_OK rows=12 cols=16|is_map_char|MAP_ROWS|MAP_COLS" -Encoding UTF8
```

Expected: parser uses the same frame markers, dimensions, and reply string.

- [ ] **Step 3: Board-level smoke test**

Run on hardware:

```text
1. Wire OpenART TX -> RT1064 B13, OpenART RX -> RT1064 B12, GND -> GND.
2. Start RT1064 firmware and ensure it is running, not stopped at Keil debug entry.
3. Run OpenART script in OpenMV IDE.
4. Confirm OpenMV IDE prints OPENART_UART_READY UART(12) 115200.
5. Observe RT1064 side frame counter by temporary screen/wireless diagnostic or by adding a short one-time debug print if needed.
6. Confirm OpenART receives no UART send exception.
```

Expected: RT1064 frame counter increases when OpenART is running; disconnecting OpenART stops increments but does not block the main loop.

- [ ] **Step 4: Final status check**

Run:

```powershell
git -C "C:\Users\ye\Desktop\rt1064" status --short
```

Expected: only unrelated pre-existing worktree changes remain. New OpenART UART sender and RT1064 receiver commits are present in `git log --oneline -3`.

## Self-Review

- Spec coverage: OpenART periodic sender is Task 1; RT1064 interrupt reception and main-loop parsing are Tasks 2 and 3; text protocol and `MAP_OK` are Tasks 1 and 2; verification is Task 4.
- Placeholder scan: No unfinished placeholders or unspecified implementation steps are intentionally left in this plan.
- Type consistency: Public RT1064 functions use the same names in header, C module, `main.c`, `app.c`, and `isr.c`. Map dimensions use `MAP_ROWS` and `MAP_COLS` from `map_types.h`.
