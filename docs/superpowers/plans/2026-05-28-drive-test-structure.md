# Drive Test Structure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move all drive test functions into a dedicated `drive_test.c/h` module without changing motor control behavior.

**Architecture:** `drive_test` owns test switches, test targets, speed-loop test execution, point wheel testing, and main-loop test state machines. `drive_control` still owns real-time motor state and exposes narrow support functions so `drive_test` does not touch `control_status` or `wheel_pid[]` directly.

**Tech Stack:** Embedded C for RT1064, Keil MDK project file, existing ZF/逐飞 headers and PIT/GPT timing.

---

### Task 1: Add `drive_test` Module

**Files:**
- Create: `C:\Users\ye\Desktop\rt1064\project\user\inc\drive_test.h`
- Create: `C:\Users\ye\Desktop\rt1064\project\user\src\drive_test.c`

- [ ] **Step 1: Create `drive_test.h`**

Add:

```c
#ifndef _drive_test_h_
#define _drive_test_h_

#include "zf_common_typedef.h"
#include "drive_config.h"

/**
 * @brief 初始化代码开关式底盘调车测试状态。
 *
 * @note 仅在主循环启动前调用一次；函数不访问电机输出。
 */
void drive_test_init(void);

/**
 * @brief 主循环级调车测试轮询入口。
 *
 * 当前只承载平移保持姿态测试；速度环和姿态环测试仍由 20ms 控制链路实时执行。
 */
void drive_test_poll(void);

/**
 * @brief 执行 20ms 速度环测试链路。
 *
 * @note 仅由 `update_control_20ms()` 调用，保持速度环采样周期固定。
 */
void drive_test_update_speed_loop_20ms(void);

/**
 * @brief 查询速度环测试是否启用。
 * @return 1 表示 20ms 控制链路应进入速度环测试，0 表示走正常链路。
 */
uint8 drive_test_speed_loop_enabled(void);

/**
 * @brief 获取速度环测试目标。
 * @return 四轮共用目标，单位为 encoder count/20ms。
 */
float drive_test_speed_loop_target_count(void);

/**
 * @brief 查询原地姿态测试是否启用。
 * @return 1 表示 yaw 锁定后应设置姿态测试目标，0 表示不设置测试目标。
 */
uint8 drive_test_attitude_loop_enabled(void);

/**
 * @brief 获取姿态测试目标偏移。
 * @return 目标 yaw 相对上电锁定 yaw 的偏移，单位为 degree。
 */
float drive_test_attitude_target_offset_deg(void);

/**
 * @brief 对单个车轮施加低 PWM 点动输出以校验接线方向。
 * @param[in] wheel 待点动的车轮。
 * @param[in] signed_pwm 点动 PWM，正负号表示期望轮速方向。
 */
void test_wheel(wheel_enum wheel, float signed_pwm);

#endif
```

- [ ] **Step 2: Create `drive_test.c`**

Add:

```c
#include "drive_test.h"
#include "drive_config.h"
#include "drive_control.h"
#include "timebase.h"

static uint8 translate_test_started;
static uint8 translate_test_stopped;

void drive_test_init(void)
{
    translate_test_started = 0;
    translate_test_stopped = 0;
}

void drive_test_poll(void)
{
#if DRIVE_TRANSLATE_TEST_ENABLE
    uint32 now_ms = time_ms();

    /* 平移测试只负责发一次上层命令；实际姿态环和速度环仍在 PIT 里按 20ms 执行。 */
    if((0 == translate_test_started) && (now_ms >= DRIVE_TRANSLATE_TEST_START_MS))
    {
        set_motion_command(DRIVE_TRANSLATE_TEST_COMMAND, DRIVE_TRANSLATE_TEST_SPEED, 0.0f);
        translate_test_started = 1;
    }

    if((0 != translate_test_started) &&
       (0 == translate_test_stopped) &&
       (now_ms >= (DRIVE_TRANSLATE_TEST_START_MS + DRIVE_TRANSLATE_TEST_DURATION_MS)))
    {
        stop_motion();
        translate_test_stopped = 1;
    }
#endif
}

uint8 drive_test_speed_loop_enabled(void)
{
#if DRIVE_SPEED_LOOP_TEST_ENABLE
    return 1;
#else
    return 0;
#endif
}

float drive_test_speed_loop_target_count(void)
{
    return DRIVE_SPEED_LOOP_TEST_TARGET_COUNT;
}

uint8 drive_test_attitude_loop_enabled(void)
{
#if DRIVE_ATTITUDE_LOOP_TEST_ENABLE
    return 1;
#else
    return 0;
#endif
}

float drive_test_attitude_target_offset_deg(void)
{
    return DRIVE_ATTITUDE_LOOP_TEST_TARGET_OFFSET_DEG;
}
```

`drive_test_update_speed_loop_20ms()` and `test_wheel()` will be added after Task 3 exposes the required `drive_control` support functions.

- [ ] **Step 3: Verify no behavior is connected yet**

Run:

```powershell
git diff -- project/user/inc/drive_test.h project/user/src/drive_test.c
```

Expected: only the two new files appear; no existing control behavior changes in this task.

---

### Task 2: Move Main-Loop Translation Test Into `drive_test`

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\app.c`
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\main.c`

- [ ] **Step 1: Include and initialize `drive_test` in `app.c`**

Add include:

```c
#include "drive_test.h"
```

Update `app_init()`:

```c
void app_init(void)
{
    // 初始化顺序保持固定：时间基准先于菜单，菜单按键消抖和回放都依赖 time_ms()。
    timebase_init();
    settings_init();
    screen_init();
    vofa_init();
    drive_test_init();
    menu_init();
}
```

Update `app_poll()`:

```c
void app_poll(void)
{
    menu_poll();
    vofa_service();
    drive_test_poll();
}
```

- [ ] **Step 2: Clean `main.c` startup entry**

Remove these includes from `main.c` if they are only used by the old inline test block:

```c
#include "base_io.h"
#include "timebase.h"
```

Remove the commented temporary calls:

```c
//    test_wheel(WHEEL_LF, 500);
//    set_wheel_pwm(...);
//    set_motion_command(...);
```

Remove the old `#if DRIVE_TRANSLATE_TEST_ENABLE` block from the `while(1)` loop.

Keep `main()` shaped like:

```c
int main(void)
{
    uint8 control_init_state;

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    wireless_uart_init();
    control_init_state = control_init();

    // 应用层在底盘初始化后启动；菜单和屏幕只做非阻塞轮询，不影响 PIT 闭环节拍。
    app_init();

    while(1)
    {
        app_poll();
    }
}
```

- [ ] **Step 3: Build-check main-loop wiring**

Run:

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b 'C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx' -o 'C:\Users\ye\Desktop\rt1064\project\mdk\build_drive_test_app_wiring.log'
```

Expected before adding the file to Keil project: if `drive_test.c` is not in the project yet, link may fail with unresolved `drive_test_*`; this is acceptable until Task 4. Header compile errors are not acceptable.

---

### Task 3: Expose Narrow `drive_control` Support Functions

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\inc\drive_control.h`
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\drive_control.c`

- [ ] **Step 1: Add support function prototypes to `drive_control.h`**

Add below `update_control_20ms()`:

```c
void drive_control_set_point_test_enabled(uint8 enabled);
uint8 drive_control_point_test_enabled(void);

void drive_control_clear_motion_outputs(void);
void drive_control_set_all_wheel_target_count(float target_count);
void drive_control_update_wheel_speed_pid(void);
void drive_control_output_wheel_pwm_with_start_ramp(void);
```

- [ ] **Step 2: Add support function implementations to `drive_control.c`**

Add before `control_init()`:

```c
void drive_control_set_point_test_enabled(uint8 enabled)
{
    point_test_enabled = (0 == enabled) ? 0 : 1;
}

uint8 drive_control_point_test_enabled(void)
{
    return point_test_enabled;
}

void drive_control_clear_motion_outputs(void)
{
    uint8 i;

    control_status.vx = 0.0f;
    control_status.vy = 0.0f;
    control_status.vz = 0.0f;
    control_status.vzt = 0.0f;

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        control_status.wheel_norm[i] = 0.0f;
        control_status.wheel_target_count[i] = 0.0f;
        control_status.signed_pwm[i] = 0.0f;
    }
}

void drive_control_set_all_wheel_target_count(float target_count)
{
    uint8 i;

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        control_status.wheel_norm[i] = 0.0f;
        control_status.wheel_target_count[i] = target_count;
    }
}

void drive_control_update_wheel_speed_pid(void)
{
    uint8 i;

    for(i = 0; i < WHEEL_COUNT; i++)
    {
        control_status.signed_pwm[i] = wheel_pid_update(&wheel_pid[i],
                                                        control_status.wheel_target_count[i],
                                                        control_status.wheel_feedback_count[i]);
    }
}

void drive_control_output_wheel_pwm_with_start_ramp(void)
{
    output_wheel_pwm_with_start_ramp();
}
```

- [ ] **Step 3: Replace direct point-test flag access in control APIs**

Replace assignments like:

```c
point_test_enabled = 0;
```

with:

```c
drive_control_set_point_test_enabled(0);
```

Replace the 20ms guard:

```c
if (0 != point_test_enabled)
```

with:

```c
if (0 != drive_control_point_test_enabled())
```

---

### Task 4: Move Test Executors Into `drive_test`

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\inc\drive_test.h`
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\drive_test.c`
- Modify: `C:\Users\ye\Desktop\rt1064\project\user\src\drive_control.c`

- [ ] **Step 1: Implement speed-loop test executor in `drive_test.c`**

Add:

```c
void drive_test_update_speed_loop_20ms(void)
{
    drive_control_clear_motion_outputs();
    drive_control_set_all_wheel_target_count(drive_test_speed_loop_target_count());
    drive_control_update_wheel_speed_pid();
    drive_control_output_wheel_pwm_with_start_ramp();
}
```

- [ ] **Step 2: Implement `test_wheel()` in `drive_test.c`**

Add:

```c
void test_wheel(wheel_enum wheel, float signed_pwm)
{
    if(wheel >= WHEEL_COUNT)
    {
        return;
    }

    drive_control_clear_motion_outputs();
    drive_control_set_point_test_enabled(1);
    stop_wheels();
    set_wheel_pwm(wheel, limit_float(signed_pwm, -(float)MAX_PWM_DUTY, (float)MAX_PWM_DUTY));
}
```

Also add required includes to `drive_test.c`:

```c
#include "base_io.h"
#include "motion_math.h"
```

- [ ] **Step 3: Include `drive_test.h` in `drive_control.c`**

Add near other user includes:

```c
#include "drive_test.h"
```

- [ ] **Step 4: Remove `update_speed_loop_test_20ms()` from `drive_control.c`**

Delete the static speed-loop test function from `drive_control.c`.

- [ ] **Step 5: Replace direct speed test execution in `update_control_20ms()`**

Replace:

```c
#if DRIVE_SPEED_LOOP_TEST_ENABLE
    update_speed_loop_test_20ms();
    return;
#endif
```

with:

```c
    if(0 != drive_test_speed_loop_enabled())
    {
        drive_test_update_speed_loop_20ms();
        return;
    }
```

- [ ] **Step 6: Replace direct attitude test macro in yaw-lock branch**

Replace:

```c
#if DRIVE_ATTITUDE_LOOP_TEST_ENABLE
        set_motion_target(0.0f, 0.0f, control_status.current_yaw + DRIVE_ATTITUDE_LOOP_TEST_TARGET_OFFSET_DEG);
#endif
```

with:

```c
        if(0 != drive_test_attitude_loop_enabled())
        {
            /* 原地姿态闭环测试延迟到 yaw 稳定后再设置目标，避免把启动漂移当成真实姿态误差。 */
            set_motion_target(0.0f,
                              0.0f,
                              control_status.current_yaw + drive_test_attitude_target_offset_deg());
        }
```

- [ ] **Step 7: Remove `test_wheel()` from `drive_control.c`**

Delete the public `test_wheel()` implementation from `drive_control.c`; declaration remains available through `drive_test.h`.

- [ ] **Step 8: Verify test function ownership**

Run:

```powershell
Select-String -Path 'C:\Users\ye\Desktop\rt1064\project\user\src\drive_control.c' -Pattern 'DRIVE_SPEED_LOOP_TEST_ENABLE|DRIVE_ATTITUDE_LOOP_TEST_ENABLE|DRIVE_TRANSLATE_TEST_ENABLE|test_wheel|update_speed_loop_test_20ms'
Select-String -Path 'C:\Users\ye\Desktop\rt1064\project\user\src\drive_test.c' -Pattern 'test_wheel|drive_test_update_speed_loop_20ms'
```

Expected: no matches in `drive_control.c`; both functions appear in `drive_test.c`.

---

### Task 5: Add New Files To Keil Project

**Files:**
- Modify: `C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx`

- [ ] **Step 1: Add `drive_test.c` to `user_c` group**

Insert this `<File>` block near `drive_control.c` / `base_io.c`:

```xml
            <File>
              <FileName>drive_test.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\user\src\drive_test.c</FilePath>
            </File>
```

- [ ] **Step 2: Add `drive_test.h` to `user_h` group**

Insert this `<File>` block near `drive_control.h`:

```xml
            <File>
              <FileName>drive_test.h</FileName>
              <FileType>5</FileType>
              <FilePath>..\user\inc\drive_test.h</FilePath>
            </File>
```

- [ ] **Step 3: Build with Keil**

Run:

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b 'C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx' -o 'C:\Users\ye\Desktop\rt1064\project\mdk\build_drive_test_structure.log'
```

Expected:

```text
".\Objects\rt1064.axf" - 0 Error(s), 0 Warning(s).
```

---

### Task 6: Acceptance Checks

**Files:**
- Inspect: `C:\Users\ye\Desktop\rt1064\project\user\inc\drive_config.h`
- Inspect: `C:\Users\ye\Desktop\rt1064\project\user\src\drive_test.c`
- Inspect: `C:\Users\ye\Desktop\rt1064\project\user\src\drive_control.c`

- [ ] **Step 1: Confirm test macro location**

Run:

```powershell
Select-String -Path 'C:\Users\ye\Desktop\rt1064\project\user\inc\drive_config.h' -Pattern 'DRIVE_SPEED_LOOP_TEST_ENABLE|DRIVE_ATTITUDE_LOOP_TEST_ENABLE|DRIVE_TRANSLATE_TEST_ENABLE'
```

Expected: all three macros remain in `drive_config.h`.

- [ ] **Step 2: Confirm main no longer owns translation test**

Run:

```powershell
Select-String -Path 'C:\Users\ye\Desktop\rt1064\project\user\src\main.c' -Pattern 'DRIVE_TRANSLATE_TEST_ENABLE|set_motion_command|stop_motion|test_wheel|set_wheel_pwm'
Select-String -Path 'C:\Users\ye\Desktop\rt1064\project\user\src\drive_control.c' -Pattern 'test_wheel|update_speed_loop_test_20ms'
Select-String -Path 'C:\Users\ye\Desktop\rt1064\project\user\src\drive_test.c' -Pattern 'test_wheel|drive_test_update_speed_loop_20ms'
```

Expected: no matches for `main.c` and `drive_control.c`; both functions are present in `drive_test.c`.

- [ ] **Step 3: Confirm VOFA and Home fields are unchanged**

Run:

```powershell
Select-String -Path 'C:\Users\ye\Desktop\rt1064\project\user\src\vofa.c' -Pattern 'target_lf,target_rf,target_lb,target_rb|feedback_lf,feedback_rf,feedback_lb,feedback_rb|pwm_lf,pwm_rf,pwm_lb,pwm_rb'
Select-String -Path 'C:\Users\ye\Desktop\rt1064\project\user\src\screen.c' -Pattern 'Enc LF|Enc LB|Yaw T:|E:'
```

Expected: existing VOFA order and Home debug labels are still present.

- [ ] **Step 4: Final build**

Run:

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b 'C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx' -o 'C:\Users\ye\Desktop\rt1064\project\mdk\build_drive_test_structure_final.log'
```

Expected:

```text
".\Objects\rt1064.axf" - 0 Error(s), 0 Warning(s).
```

- [ ] **Step 5: Commit implementation**

Stage only files touched by this plan:

```powershell
git add -- 'project/user/inc/drive_test.h' 'project/user/src/drive_test.c' 'project/user/src/app.c' 'project/user/src/main.c' 'project/user/src/drive_control.c' 'project/mdk/rt1064.uvprojx'
git commit -m 'refactor: 集中管理底盘调车测试'
```
