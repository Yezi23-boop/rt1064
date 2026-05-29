# File Map

This document helps future agents find the right files in this RT1064 / OpenART intelligent vision project without scanning the whole repository first.

## Start Here

- `AGENTS.md`  
  Repository-level instructions. Read this before changing code.

- `docs/agents/file-map.md`  
  This file. Use it to locate project areas quickly.

- `docs/agents/menu-extension-guide.md`  
  Guide for adding IPS200 menu pages and dynamic variables using the current page-table refresh structure.

- `docs/competition/openart_plus_rt1064_uart_experience.md`  
  Current OpenART Plus to RT1064 UART bring-up notes, Keil command-line notes, and serial / download experience.

- `docs/competition/full.md`  
  Competition-rule and task context gathered for the 21st smart car intelligent vision group.

- `docs/competition/姿态闭环框架.md`  
  Mecanum-wheel attitude-control framework: hardware assumptions, wheel order, coordinate system, motor / encoder mapping, IMU660RC yaw source, PIT control cadence, PD attitude loop, mecanum mixing formula, and wheel-speed PID boundary.

- `docs/competition/掉电保存菜单框架.md`  
  IPS200 + four-key menu contract for Push Box map/mode selection, Flash-backed persistence, and runtime execution pages.

- `docs/competition/掉电保存菜单PRD.md`  
  Executable PRD for the Flash-backed Push Box menu system, including user stories, implementation decisions, testing decisions, and acceptance criteria.

- `docs/competition/push_box_validation_cases.md`  
  Push Box algorithm validation notes and expected cases.

- `docs/competition/push_box_validation_cases.xsb`  
  Push Box map validation data in XSB-like text form.

## Top-Level Layout

- `.gitignore`  
  Local ignore rules. Build outputs, Keil user files, local tool caches, screenshots, and the large OpenART product package should stay out of Git.

- `AGENTS.md`  
  The main Codex / agent entrypoint. Keep durable collaboration rules here.

- `docs/`  
  Project documentation. Prefer adding agent-facing documentation under `docs/agents/` and competition / hardware notes under `docs/competition/`.

- `libraries/`  
  RT1064 SDK, SeekFree common code, drivers, and device libraries. Treat this as vendor / platform support code unless a task explicitly requires changing it.

- `openmv/`  
  OpenMV / OpenART Python scripts and visual-recognition experiments.

- `project/`  
  RT1064 application project, Keil / IAR project files, linker scripts, and user code.

- `OpenART_Plus_Product/`  
  Local vendor product package. It is large and ignored by Git. Use it as local reference material only; do not commit it.

- `wechat_article_snapshot.md`  
  Snapshot of related article content used as background reference.

## RT1064 Firmware Area

Main path:

```text
project/
|-- mdk/
|-- iar/
|-- user/
|   |-- inc/
|   `-- src/
```

Important files:

- `project/mdk/rt1064.uvprojx`  
  Keil MDK project file. Add new C source files here when needed.

- `project/mdk/ini/evkmimxrt1064_flexspi_nor.ini`  
  Keil flash / initialization script used for RT1064 download.

- `project/mdk/scf/MIMXRT1064xxxxx_flexspi_nor.scf`  
  Linker scatter file.

- `project/user/src/main.c`  
  RT1064 application entrypoint.

- `project/user/inc/isr.h` and `project/user/src/isr.c`  
  Interrupt declarations and handlers.

Push Box user modules:

- `project/user/inc/map_types.h`  
  Shared Push Box grid, cell, position, and command types.

- `project/user/inc/maps.h` / `project/user/src/maps.c`  
  Test maps or map fixtures used by the Push Box workflow.

- `project/user/inc/solver.h` / `project/user/src/solver.c`  
  Push Box solver / BFS logic.

- `project/user/inc/plan_output.h` / `project/user/src/plan_output.c`  
  Conversion from solver results to motion-oriented commands.

- `project/user/inc/app.h` / `project/user/src/app.c`  
  Application-level Push Box integration.

Control-framework planning:

- `docs/competition/姿态闭环框架.md`  
  Source-of-truth for the planned mecanum attitude-control structure before C implementation. Check this before adding motor, encoder, IMU, PIT, or wheel PID modules.

Current drive-control modules:

- `project/user/inc/drive_config.h` / `project/user/src/drive_config.c`  
  Wheel order, control limits, yaw PD defaults, wheel PID defaults, and the single definitions of `motor_dir_sign[4]` / `encoder_dir_sign[4]`.

- `project/user/inc/motion_math.h` / `project/user/src/motion_math.c`  
  Pure control math: shortest yaw error, attitude PD, discrete command to `vx/vy/vz`, mecanum mix, wheel normalization, and wheel target count mapping.

- `project/user/inc/wheel_pid.h` / `project/user/src/wheel_pid.c`  
  Four-wheel incremental PID helper.

- `project/user/inc/base_io.h` / `project/user/src/base_io.c`  
  Hardware access boundary for IMU660RC yaw, encoder counts, wheel direction GPIO, and PWM output.

- `project/user/inc/drive_control.h` / `project/user/src/drive_control.c`  
  Drive scheduling and public command API. `PIT_CH0` calls the 5ms IMU update, `PIT_CH1` calls the 20ms attitude / wheel-speed loop.

Relevant driver / device APIs for the planned control framework:

- `libraries/zf_device/zf_device_imu660rc.h` / `libraries/zf_device/zf_device_imu660rc.c`  
  IMU660RC driver. Use `imu660rc_yaw` as the planned yaw source and `imu660rc_gyro_transition()` for gyro debug units.

- `libraries/zf_driver/zf_driver_encoder.h` / `libraries/zf_driver/zf_driver_encoder.c`  
  Quadrature encoder API. Planned wheel feedback unit is `encoder_get_count()` per 20ms cycle.

- `libraries/zf_driver/zf_driver_pwm.h` / `libraries/zf_driver/zf_driver_pwm.c`  
  PWM output API. Planned motor duty range uses `PWM_DUTY_MAX = 10000`.

- `libraries/zf_driver/zf_driver_pit.h` / `libraries/zf_driver/zf_driver_pit.c` and `project/user/src/isr.c`  
  Periodic interrupt path. Planned control cadence is 5ms IMU update and 20ms attitude / wheel-speed control.

Build outputs and local files under `project/mdk/Objects/`, `project/mdk/Listings/`, `project/mdk/.vscode/`, `*.log`, `*.uvoptx`, and `*.uvguix.*` are not source-of-truth.

## OpenMV / OpenART Area

Main path:

```text
openmv/
```

Important files:

- `openmv/openart_plus_uart_smoke.py`  
  Minimal OpenART / OpenMV UART smoke test script.

- `openmv/openart_plus_grid_recognizer.py`  
  OpenART / OpenMV grid recognition script.

- `openmv/openart_plus_grid_recognizer_pure.py`  
  Pure Python or host-checkable variant of the grid recognizer. Use this when logic can be checked without hardware.

- `openmv/fe48eb2150ae5a6eea643c6615610fcf.png`  
  Local visual reference / evidence image. Do not treat screenshots as algorithm source-of-truth.

When writing OpenMV / ART code, consult the official OpenMV library index first:

```text
https://docs.openmv.io/library/index.html
```

Then drill into modules such as `sensor`, `image`, `machine`, `display`, `ml`, `csi`, and `network` before writing new helpers.

## Libraries Area

Main path:

```text
libraries/
|-- components/
|-- doc/
|-- sdk/
|-- zf_common/
|-- zf_components/
|-- zf_device/
`-- zf_driver/
```

Use these areas as follows:

- `libraries/doc/`  
  Library version and vendor documentation notes. Check here when judging library version, provenance, or update state.

- `libraries/components/fatfs/`  
  FatFs filesystem component. Use when investigating SD-card filesystem mounting, file read/write, `ff.c`, `ff.h`, or disk I/O adapter behavior.

- `libraries/components/sdmmc/`  
  SD / MMC stack support. Use with `zf_driver_sdio.*` when SD-card low-level initialization or block access is involved.

- `libraries/components/usb/`  
  NXP USB device / host stack and CDC templates. Use when investigating USB CDC, virtual COM, USB device descriptors, or USB host/device framework behavior.

- `libraries/sdk/`  
  NXP / RT1064 SDK support code. Prefer this when the question is about chip-level registers, clock tree, startup, board muxing, XIP boot, or NXP peripheral drivers.

- `libraries/sdk/board/`  
  Board-level clock and pin mux files: `board.*`, `clock_config.*`, `pin_mux.*`, `RTE_Device.h`.

- `libraries/sdk/CMSIS/`  
  CMSIS headers and ARM core definitions. Use for Cortex-M core types, intrinsics, and CMSIS driver interfaces.

- `libraries/sdk/cmsis_drivers/`  
  CMSIS-style wrappers for selected NXP peripherals such as LPI2C, LPSPI, and LPUART.

- `libraries/sdk/components/`  
  NXP generic components such as lists, OSA abstraction, serial manager, and UART adapter.

- `libraries/sdk/deceive/`  
  Device headers for MIMXRT1064: register definitions, feature macros, and system init code. Use this when checking exact register names or chip feature macros.

- `libraries/sdk/drives/`  
  NXP low-level peripheral drivers: ADC, CSI, FlexIO, FlexSPI, GPIO, GPT, IOMUXC, LPI2C, LPSPI, LPUART, PIT, PWM, SEMC, USDHC, watchdog, and others.

- `libraries/sdk/startup/`  
  Startup assembly files for MDK and IAR.

- `libraries/sdk/utilities/`  
  NXP debug console, shell, assert, notifier, string, memcpy, and sbrk utilities.

- `libraries/sdk/xip/`  
  FlexSPI NOR boot and XIP configuration files. Use when debugging boot, flash layout, or external memory startup behavior.

- `libraries/zf_common/`  
  SeekFree common helpers. Important files include:

  - `zf_common_headfile.h`: common aggregate include.
  - `zf_common_debug.*`: debug / printf retarget and logging path.
  - `zf_common_clock.*`: clock helpers.
  - `zf_common_fifo.*`: FIFO utilities.
  - `zf_common_interrupt.*`: interrupt helper layer.
  - `zf_common_typedef.h`: shared typedefs.
  - `zf_common_function.*`: common utility functions.
  - `zf_common_font.*`: built-in font data.
  - `zf_common_vector.*`: vector / interrupt table related support.

- `libraries/zf_driver/`  
  SeekFree low-level peripheral drivers. Start here when application code needs a board peripheral API rather than raw NXP SDK calls:

  - `zf_driver_gpio.*`: GPIO input/output.
  - `zf_driver_uart.*`: UART serial communication.
  - `zf_driver_usb_cdc.*`: USB CDC serial communication.
  - `zf_driver_delay.*`: delay functions.
  - `zf_driver_timer.*`, `zf_driver_pit.*`: timer / periodic interrupt support.
  - `zf_driver_pwm.*`: PWM output.
  - `zf_driver_encoder.*`: encoder input.
  - `zf_driver_adc.*`: ADC sampling.
  - `zf_driver_spi.*`, `zf_driver_soft_spi.*`: hardware / software SPI.
  - `zf_driver_iic.*`, `zf_driver_soft_iic.*`: hardware / software I2C.
  - `zf_driver_sdio.*`: SDIO / SD-card low-level access.
  - `zf_driver_flash.*`: internal / external flash helper API.
  - `zf_driver_csi.*`, `zf_driver_flexio_csi.*`: camera capture interfaces.
  - `zf_driver_exti.*`: external interrupt support.
  - `zf_driver_romapi.*`: ROM API helpers.

- `libraries/zf_device/`  
  SeekFree device-level drivers for sensors, screens, wireless modules, and external peripherals. Start here when the hardware is a named module:

  - `zf_device_config.h`: device configuration switches and pin / interface configuration.
  - `zf_device_type.*`: common device type definitions.
  - `zf_device_key.*`: key / button handling.
  - `zf_device_ips114.*`, `zf_device_ips200.*`, `zf_device_ips200pro.*`, `zf_device_oled.*`, `zf_device_tft180.*`: display modules.
  - `zf_device_camera.*`: common camera abstraction.
  - `zf_device_mt9v03x.*`, `zf_device_mt9v03x_flexio.*`: MT9V03X camera support.
  - `zf_device_ov7725.*`: OV7725 camera support.
  - `zf_device_scc8660.*`, `zf_device_scc8660_flexio.*`: SCC8660 camera support.
  - `zf_device_imu660ra.*`, `zf_device_imu660rb.*`, `zf_device_imu660rc.*`, `zf_device_imu963ra.*`, `zf_device_icm20602.*`, `zf_device_mpu6050.*`: IMU drivers.
  - `zf_device_absolute_encoder.*`: absolute encoder support.
  - `zf_device_tsl1401.*`: linear CCD sensor.
  - `zf_device_dl1a.*`, `zf_device_dl1b.*`: distance / ranging modules.
  - `zf_device_gnss.*`: GNSS module support.
  - `zf_device_wireless_uart.*`, `zf_device_bluetooth_ch9141.*`, `zf_device_ble6a20.*`: wireless / Bluetooth serial modules.
  - `zf_device_wifi_uart.*`, `zf_device_wifi_spi.*`: Wi-Fi modules.
  - `zf_device_virtual_oscilloscope.*`: virtual oscilloscope / debug visualization support.
  - `zf_device_config.a` and `zf_device_config.lib`: prebuilt device configuration libraries. Treat as binary vendor artifacts; do not edit.

- `libraries/zf_components/`  
  Higher-level SeekFree component code. Current important files:

  - `seekfree_assistant.*`: SeekFree assistant protocol / helper logic.
  - `seekfree_assistant_interface.*`: interface layer used by assistant tooling.

Default rule: avoid modifying `libraries/` unless the bug or hardware requirement is clearly inside library behavior. Prefer implementing project-specific behavior in `project/user/`.

Recommended search order:

1. For project behavior, start in `project/user/`.
2. For board peripheral APIs, check `libraries/zf_driver/`.
3. For named external modules, check `libraries/zf_device/`.
4. For shared debug / clock / FIFO / typedef behavior, check `libraries/zf_common/`.
5. For protocol or assistant tooling, check `libraries/zf_components/`.
6. For chip-level behavior or register-level details, check `libraries/sdk/`.
7. For filesystem / SD / USB stack internals, check `libraries/components/`.

## Documentation Area

- `docs/agents/`  
  Agent workflow configuration and navigation docs.

- `docs/competition/`  
  Competition task, OpenART / RT1064 integration, Push Box algorithm, and hardware notes.

- `docs/tmp_pdf_pages/`  
  Temporary extracted PDF pages if present. Treat as generated / working material, not primary documentation.

## Common Workflows

### Find RT1064 firmware entrypoints

1. Read `project/user/src/main.c`.
2. Follow calls into `app.c`, `solver.c`, and `plan_output.c`.
3. Check headers in `project/user/inc/` for public contracts.
4. Only then inspect `libraries/` if a driver or device API is involved.

### Add a new RT1064 C source file

1. Add `.c` under `project/user/src/` and `.h` under `project/user/inc/` if needed.
2. Add the source file to `project/mdk/rt1064.uvprojx`.
3. Build with Keil command-line build:

```powershell
D:\Keil_v5\UV4\UV4.exe -b D:\rt1064\RT1064_Library\SeekFree_RT1064_Opensource_Library\project\mdk\rt1064.uvprojx
```

### Work on OpenART / OpenMV scripts

1. Start in `openmv/`.
2. Check `docs.openmv.io/library/index.html` before creating helper functions.
3. Keep hardware-dependent logic separate from pure logic where possible.
4. Prefer host-checkable code for grid parsing, map normalization, and algorithm glue.

### Investigate build or download issues

1. Read `docs/competition/openart_plus_rt1064_uart_experience.md`.
2. Check `project/mdk/rt1064.uvprojx`.
3. Check `project/mdk/ini/evkmimxrt1064_flexspi_nor.ini`.
4. Ignore `Objects/`, `Listings/`, and old `.log` files unless the current task is specifically about build evidence.

## Do Not Treat These As Primary Source

- `OpenART_Plus_Product/`  
  Local vendor bundle, ignored by Git.

- `project/mdk/Objects/` and `project/mdk/Listings/`  
  Build outputs.

- `project/mdk/*.log`  
  Local build / flash logs.

- `project/mdk/*.uvoptx`, `project/mdk/*.uvguix.*`  
  Keil user/session files.

- `.playwright-mcp/` and `__pycache__/`  
  Local tool caches.
