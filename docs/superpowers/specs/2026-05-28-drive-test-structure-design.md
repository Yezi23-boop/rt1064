# 底盘调车测试结构整理设计

## 背景

当前底盘控制链路已经跑通：速度环、yaw 姿态环、启动安全窗口、Home 调试显示和 VOFA+ FireWater 曲线都能用于调车。接下来需要优化代码结构，让调车入口更集中，同时避免测试逻辑散落在 `main.c` 或实时控制链路里，影响后续比赛代码维护。

本次范围只覆盖底盘控制、调试显示和 VOFA 相关结构；地图、求解器、菜单业务流程不纳入本轮重构。

## 目标

1. 新增 `drive_test.c/h`，集中管理代码开关式调车测试。
2. 保留三个独立测试宏，不新增总开关：
   - `DRIVE_SPEED_LOOP_TEST_ENABLE`
   - `DRIVE_ATTITUDE_LOOP_TEST_ENABLE`
   - `DRIVE_TRANSLATE_TEST_ENABLE`
3. 正式比赛前只需要确认这三个宏为 `0`，测试逻辑不会误启动。
4. 速度环测试仍在 20ms 控制周期内执行，保持采样周期准确。
5. 平移测试仍在主循环中发命令，避免把非实时状态机塞进 PIT 中断。

## 设计

### `drive_test` 模块

新增公开接口：

```c
void drive_test_init(void);
void drive_test_poll(void);

uint8 drive_test_speed_loop_enabled(void);
float drive_test_speed_loop_target_count(void);

uint8 drive_test_attitude_loop_enabled(void);
float drive_test_attitude_target_offset_deg(void);
```

职责划分：

- `drive_test_init()` 初始化测试模块内部状态，例如平移测试是否已经启动/停止。
- `drive_test_poll()` 在主循环中执行平移测试状态机。
- `drive_test_speed_loop_enabled()` 和 `drive_test_speed_loop_target_count()` 提供速度环测试开关和目标值。
- `drive_test_attitude_loop_enabled()` 和 `drive_test_attitude_target_offset_deg()` 提供姿态测试开关和目标偏移。

`drive_test` 只负责“测试是否启用、测试目标是多少、主循环级测试什么时候发命令”，不直接访问 `wheel_pid`、`control_status` 或底层 PWM。

### `drive_control` 调整

`drive_control.c` 继续负责实时控制链路：

- 编码器读取
- 启动安全窗口
- 点动保护
- 速度环测试的 20ms 执行
- 姿态环
- 麦轮混控
- 四轮速度 PID
- PWM 起步限幅

变化点：

- `update_control_20ms()` 不再直接判断 `DRIVE_SPEED_LOOP_TEST_ENABLE`，改为调用 `drive_test_speed_loop_enabled()`。
- `update_speed_loop_test_20ms()` 保留在 `drive_control.c`，但目标值改为参数传入。
- yaw 稳定锁定后，不再直接判断 `DRIVE_ATTITUDE_LOOP_TEST_ENABLE`，改为调用 `drive_test_attitude_loop_enabled()`。
- `drive_control.c` 不管理平移测试状态机。

这样可以把实时执行留在控制层，把测试开关和测试目标集中到 `drive_test`。

### `main` 和 `app` 调整

`main.c` 只保留系统启动入口：

- 时钟初始化
- debug 初始化
- 无线串口初始化
- `control_init()`
- `app_init()`
- `while(1) app_poll()`

删除 `main.c` 中注释掉的临时 `test_wheel()`、`set_wheel_pwm()`、`set_motion_command()` 代码。

`app.c` 增加：

- `app_init()` 调用 `drive_test_init()`
- `app_poll()` 调用 `drive_test_poll()`

调用顺序建议为：

```c
menu_poll();
vofa_service();
drive_test_poll();
```

平移测试在主循环执行，不要求严格 20ms；实际闭环仍由 PIT 控制。

### 配置和调试输出

`drive_config.h` 本轮只轻整理，不拆成多个配置头文件：

- 保留底盘参数、PID 参数、IMU 参数、测试宏、启动限幅宏和枚举。
- 将三个测试宏放在同一块“调车测试开关”区域。
- 不新增 `DRIVE_TEST_ENABLE` 总开关。

Home 显示和 VOFA 输出本轮不改变字段，不改变 FireWater 顺序：

```text
target_lf,target_rf,target_lb,target_rb,
feedback_lf,feedback_rf,feedback_lb,feedback_rb,
pwm_lf,pwm_rf,pwm_lb,pwm_rb
```

## 不做的事

本轮不做以下改动：

- 不调整 PID 参数。
- 不改变麦轮混控公式。
- 不改变 Home 页面显示内容。
- 不改变 VOFA 字段顺序。
- 不拆 `screen.c`、`menu.c` 或 `vofa.c`。
- 不把 `wheel_pid`、`control_status` 或 PWM 输出函数暴露给 `drive_test`。

## 验证标准

1. Keil 构建通过，要求 `0 Error(s), 0 Warning(s)`。
2. 三个测试宏都为 `0` 时，上电后不会自动执行任何测试动作。
3. `DRIVE_TRANSLATE_TEST_ENABLE=1` 时，平移测试仍按启动安全窗口后启动，到时自动停车。
4. `DRIVE_SPEED_LOOP_TEST_ENABLE=1` 时，速度环测试仍保持 20ms 周期输出同目标 count。
5. `DRIVE_ATTITUDE_LOOP_TEST_ENABLE=1` 时，姿态测试仍在 yaw 稳定锁定后设置目标偏移。
6. Home 和 VOFA 仍能观察原来的 encoder、yaw、target、feedback、pwm 调试量。

## 已确认决策

- 使用 `drive_test.c/h` 集中管理测试逻辑。
- 不增加 `DRIVE_TEST_ENABLE` 总开关。
- 速度环测试和姿态环测试的开关/目标放进 `drive_test`，实时执行仍由 `drive_control` 在 20ms 链路中完成。
- 平移测试从 `main.c` 挪到 `drive_test_poll()`。
