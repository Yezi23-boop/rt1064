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

- `tools/smartcar_agent/`
  Read-only Windows observer for the SmartCar VR program. It publishes maps, camera settings, debug-image metadata, and a captured game window as localhost HTTP endpoints plus `runtime/latest_state.json` for Agent-assisted tuning.

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

Competition and Push Box user modules:

- `project/user/inc/map_types.h`  
  Shared 16x12 map, solver result, action, and waypoint types.

- `project/user/inc/map_utils.h` / `project/user/src/map_utils.c`
  Map snapshot, comparison, player lookup, and object-count helpers shared by the UART, menu, solver, and replanning flow.

- `project/user/inc/maps.h` / `project/user/src/maps.c`  
  Test maps or map fixtures used by the Push Box workflow.

- `project/user/inc/solver.h` / `project/user/src/solver.c`  
  Greedy multi-box decomposition, single-box Push Box BFS, and player-only navigation BFS used for return-to-launch routing.

- `project/user/inc/executor.h` / `project/user/src/executor.c`
  Converts solver waypoints into 20 cm physical targets and advances the motion state machine from the 20ms control tick.

- `project/user/inc/art_replan.h` / `project/user/src/art_replan.c`
  ART-source runtime flow: launch delay and center collection, launch movement, stable-map solving, task-end synchronization, replanning, and automatic return to the left launch area.

- `project/user/inc/openart_uart.h` / `project/user/src/openart_uart.c`
  LPUART1 byte buffering and parsing for complete map frames, requested center samples, and subject-two observation samples.

- `project/user/inc/app.h` / `project/user/src/app.c`  
  Non-blocking application initialization and polling for the menu, VOFA service, both OpenART UART parsers, board tests, and drive tests.

- `project/user/inc/menu.h` / `project/user/src/menu.c`
  User workflow, map/source/mode selection, solving, execution-page state, and ART replanning coordination.

- `project/user/inc/screen.h` / `project/user/src/screen.c`
  IPS200 rendering only; business state is assembled by `menu.c` before drawing.

- `project/user/inc/competition_flow.h` / `project/user/src/competition_flow.c`
  Stage coordinator for subject-one debug, subject-two debug, and the full subject-one -> return -> subject-two competition sequence.

- `project/user/inc/subject2.h` / `project/user/src/subject2.c`
  Subject-two runtime state machine: observation-point planning, requested center adjustment, ART #2 classification, class binding, bound pushes, map confirmation, and return requests.

- `project/user/inc/subject2_logic.h` / `project/user/src/subject2_logic.c`
  Host-testable subject-two algorithms for object collection, observation selection, classification stability, box/target binding, and active-box tracking.

- `project/user/inc/vision_uart.h` / `project/user/src/vision_uart.c`
  LPUART4 protocol for ART #2 mode selection, classification requests, samples, acknowledgements, cancellation, and optional board-level smoke testing.

Control-framework planning:

- `docs/competition/姿态闭环框架.md`  
  Design and tuning context for the implemented mecanum attitude-control structure. Check it before changing motor, encoder, IMU, PIT, or wheel PID behavior, then verify details against the current C implementation.

Current drive-control modules:

- `project/user/inc/drive_config.h`
  Wheel order, 20ms control constants, 20 cm grid size, pose calibration, yaw/path PID defaults, ART synchronization settings, and launch/return configuration. This module has no `.c` file.

- `project/user/inc/motion_math.h` / `project/user/src/motion_math.c`  
  Pure control math: shortest yaw error, attitude PD, discrete command to `vx/vy/vz`, mecanum mix, wheel normalization, and wheel target count mapping.

- `project/user/inc/wheel_pid.h` / `project/user/src/wheel_pid.c`  
  Four-wheel incremental PID helper.

- `project/user/inc/base_io.h` / `project/user/src/base_io.c`  
  Hardware access boundary for encoders, motor direction GPIO, PWM output, and the definitions of `motor_dir_sign[4]` / `encoder_dir_sign[4]`.

- `project/user/inc/drive_imu.h` / `project/user/src/drive_imu.c`
  IMU initialization, yaw normalization/locking, and attitude-loop status updates.

- `project/user/inc/drive_output.h` / `project/user/src/drive_output.c`
  Mecanum mixing, wheel target generation, wheel-speed PID updates, startup PWM ramping, and final motor output.

- `project/user/inc/drive_pose.h` / `project/user/src/drive_pose.c`
  Short-distance odometry from four encoder increments and relative IMU yaw, expressed in the executor's local world coordinates.

- `project/user/inc/drive_control.h` / `project/user/src/drive_control.c`  
  Drive scheduling and public motion API. `PIT_CH1` runs the 20ms feedback, executor, attitude, and wheel-speed chain; IMU samples arrive through the IMU INT2 callback, with no PIT-based 5ms IMU loop.

- `project/user/inc/drive_test.h` / `project/user/src/drive_test.c`
  Bench-test and VOFA-driven manual motion/PWM diagnostics that temporarily take ownership of normal control output.

Relevant driver / device APIs for the current control framework:

- `libraries/zf_device/zf_device_imu660rc.h` / `libraries/zf_device/zf_device_imu660rc.c`  
  IMU660RC driver. Use `imu660rc_yaw` as the planned yaw source and `imu660rc_gyro_transition()` for gyro debug units.

- `libraries/zf_driver/zf_driver_encoder.h` / `libraries/zf_driver/zf_driver_encoder.c`  
  Quadrature encoder API. Wheel feedback uses `encoder_get_count()` increments per 20ms cycle.

- `libraries/zf_driver/zf_driver_pwm.h` / `libraries/zf_driver/zf_driver_pwm.c`  
  PWM output API. Motor duty uses the `PWM_DUTY_MAX = 10000` range before project-level limiting.

- `libraries/zf_driver/zf_driver_pit.h` / `libraries/zf_driver/zf_driver_pit.c` and `project/user/src/isr.c`  
  Periodic interrupt path. `PIT_CH1` is the 20ms motion-control boundary and `PIT_CH2` is the 5ms menu-key scan; IMU660RC updates are triggered by the GPIO INT2 interrupt path.

Build outputs and local files under `project/mdk/Objects/`, `project/mdk/Listings/`, `project/mdk/.vscode/`, `*.log`, `*.uvoptx`, and `*.uvguix.*` are not source-of-truth.

## OpenMV / OpenART Area

Main path:

```text
openmv/
```

Important files:

- `openmv/main_see.py`
  OpenART #1 competition entrypoint. It recognizes the 16x12 virtual grid, sends periodic map/player-center frames, and responds to `CENTER_REQ` and `OBSERVE_REQ` geometry requests over UART12 at 115200 baud.

- `openmv/视觉/main.py`
  OpenART #2 competition entrypoint. It loads the cartoon and number models, switches modes on MCU commands, and returns request-scoped classification samples over UART12 at 115200 baud.

- `openmv/视觉/*.tflite` and label files
  ART #2 model assets deployed with `openmv/视觉/main.py`; keep filenames synchronized with the script constants and SD-card layout.

- `openmv/main_model.py`
  Earlier standalone EIQ/TFLite classification experiment. It is not the current ART #2 competition protocol entrypoint.

When writing OpenMV / ART code, consult the official OpenMV library index first:

```text
https://docs.openmv.io/library/index.html
```

Then drill into modules such as `sensor`, `image`, `machine`, `display`, `ml`, `csi`, and `network` before writing new helpers.

## Tests Area

- `tests/run_*.ps1`
  GCC host-test launchers for solver/navigation, executor correction, ART requested-center flow, competition flow, subject-two logic/state, and both UART protocols.

- `tests/*_test.py`
  Host-side protocol and OpenART behavior tests. These scripts execute directly with Python and print a `PASS` marker on success.

- `tests/host/`
  Minimal RT1064 typedef, interrupt, and aggregate-header stubs used only by GCC host tests.

Run the relevant focused test while developing. Before integration, run all PowerShell and Python test entries using the commands in `AGENTS.md`.

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
2. Follow the main-loop path through `app.c` and `menu.c`.
3. Follow stage changes through `competition_flow.c`.
4. For subject one, continue through `openart_uart.c`, `art_replan.c`, `solver.c`, and `executor.c`.
5. For subject two, continue through `subject2.c`, `subject2_logic.c`, `vision_uart.c`, `solver.c`, and `executor.c`.
6. For physical motion, follow `isr.c` into `drive_control.c`, `drive_pose.c`, `drive_imu.c`, and `drive_output.c`.
7. Check headers in `project/user/inc/` for public contracts, then inspect `libraries/` only if a driver or device API is involved.

### Add a new RT1064 C source file

1. Add `.c` under `project/user/src/` and `.h` under `project/user/inc/` if needed.
2. Add the source file to `project/mdk/rt1064.uvprojx`.
3. Build with Keil command-line build:

```powershell
D:\Keil_v5\UV4\UV4.exe -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
```

### Work on OpenART / OpenMV scripts

1. Use `openmv/main_see.py` for ART #1 map/geometry work and `openmv/视觉/main.py` for ART #2 classification work.
2. Check `docs.openmv.io/library/index.html` before creating helper functions.
3. Update the matching MCU protocol module and tests whenever a UART command or response changes.
4. Keep hardware-dependent logic separate from pure logic where possible.
5. Prefer host-checkable code for grid parsing, map normalization, classification filtering, and algorithm glue.

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
