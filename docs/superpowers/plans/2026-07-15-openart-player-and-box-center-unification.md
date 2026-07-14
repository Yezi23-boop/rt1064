# OpenART Player and Box Center Unification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 OpenART 只发布唯一 `C/+` 的规范地图，并让 `CENTER_REQ`、`OBSERVE_REQ` 共用同一小车中心与地图配对链路，同时保持精确箱子中心只服务推箱前准备位。

**Architecture:** OpenART 是生成 `C/+` 的唯一权威：普通唯一 player 走快速路径，零个或多个 player 按需运行精确中心恢复，请求式精确中心始终覆盖粗车格。MCU UART 只做原子帧、唯一车格和中心一致性校验，不再重写地图；科目一、科目二、solver、menu 和 screen 只消费已校验的规范地图。箱子 `B` 继续由稳定字符地图决定，指定箱子精确中心通过与新规范地图配套的 `OBSERVE_SAMPLE` 进入共享 executor 三帧中值和二维准备位。

**Tech Stack:** OpenART Plus MicroPython、RT1064 C99、逐飞 UART、现有 map_utils/art_replan/subject2/executor、Python AST/图像主机测试、GCC PowerShell 主机测试、Keil MDK。

---

## File Structure

**OpenART #1**

- Modify: `openmv/main_see.py`
  - 新增规范地图背景、唯一 player 生成、共享中心解析和请求式箱子解析。
  - 将 `CENTER_SAMPLE`、`OBSERVE_SAMPLE` 与同一中心生成的新规范地图配对。
- Create: `tests/openart_single_player_map_test.py`
  - 纯 Python 覆盖唯一 C 快速路径、零/多 C 恢复、`+`/`T` 背景恢复和失败不发送。
- Modify: `tests/openart_player_anchor_test.py`
  - 覆盖统一小车中心入口与历史锚点优先级。
- Modify: `tests/openart_request_flow_test.py`
  - 覆盖请求式中心优先于粗 C、CENTER 配套规范地图。
- Modify: `tests/openart_observation_test.py`
  - 覆盖指定箱子入口、0.8格匹配、同帧车箱中心及 OBSERVE 配套规范地图。
- Modify: `tests/openart_precise_center_test.py`
  - 保留精确中心像素算法回归，并验证通用坐标换算名称。

**MCU 地图与协议**

- Modify: `project/user/inc/drive_config.h`
  - 增加共享 C 端箱子观察样本数和请求格偏差语义常量。
- Modify: `project/user/src/openart_uart.c`
  - 删除接收端移动 C 的行为；严格校验唯一 C、精确中心所在格和观察样本配套帧。
- Modify: `project/user/inc/openart_uart.h`
  - 更新 MAP/CENTER/OBSERVE 的配套语义注释。
- Modify: `tests/openart_center_request_test.c`
  - 覆盖拒收零/多 C、中心与 C 不一致、OBSERVE 无新地图/错误箱格/旧样本。
- Modify: `tests/run_openart_center_request_test.ps1`
  - 仅在新增源依赖导致编译列表变化时同步 GCC 输入；否则不改。

**共享地图与业务消费**

- Modify: `project/user/src/map_utils.c`
- Modify: `project/user/inc/map_utils.h`
  - 非唯一车格时返回 `0xFF/0xFF`，禁止“第一个 C”成为隐式位置。
- Modify: `project/user/src/menu.c`
  - 检查唯一车格查找结果后再启动 executor。
- Modify: `project/user/src/screen.c`
  - 非唯一车格时不叠加 pose 车格，不显示第一个 C 为推算原点。
- Modify: `project/user/src/subject2_logic.c`
- Modify: `project/user/inc/subject2_logic.h`
  - 移除科目二重写车格的 `subject2_normalize_center_map()`。
- Modify: `project/user/src/subject2.c`
  - 使用配套规范地图的唯一 C，仅用精确中心修正 pose。
- Modify: `tests/subject2_logic_test.c`
- Modify: `tests/subject2_scan_test.c`
  - 更新科目二中心配套与唯一车格行为。

**Executor 与流程**

- Modify: `project/user/src/executor.c`
  - 使用配置中的观察样本数和0.8格常量；保留三帧中值、BGeo降级与BTim停车。
- Modify: `project/user/src/art_replan.c`
  - 科目一继续复用 UART+executor，不自行解释箱子中心或重写 C。
- Modify: `tests/executor_pre_push_test.c`
- Modify: `tests/art_replan_requested_center_test.c`
  - 覆盖配套观察样本、边界和失败策略。

**长期文档**

- Modify: `AGENTS.md`
  - 说明 OpenART 是 C/+ 唯一权威，以及 CENTER/OBSERVE 样本前必须有配套规范地图。
- Modify: `docs/competition/openart_plus_rt1064_uart_experience.md`
  - 更新协议时序、非法帧策略和调试判据。

---

### Task 1: Add Pure OpenART Canonical Player Map Logic

**Files:**
- Create: `tests/openart_single_player_map_test.py`
- Modify: `openmv/main_see.py`

- [ ] **Step 1: Write failing canonical-map tests**

使用 AST 只加载纯函数，构造 12x16 element matrix 和背景 matrix，至少写出以下断言：

```python
canonical = build_canonical_player_map(stable, background, None)
assert count_car_chars(canonical) == 1
assert canonical[5][5] == "C"

canonical = build_canonical_player_map(stable_with_two_players,
                                       background, (568, 542))
assert canonical[5][5] == "C"
assert canonical[5][6] == "."

canonical = build_canonical_player_map(stable_on_target,
                                       background_with_target, (550, 550))
assert canonical[5][5] == "+"

assert build_canonical_player_map(two_players_without_background,
                                  empty_background, (550, 550)) is None
```

同时验证：精确中心越界返回 `None`；旧 `+` 离开后恢复 `T`；未选中的 wall/bomb/box 候选恢复背景而不是一律变成 `.`。

- [ ] **Step 2: Run the new test and verify RED**

Run:

```powershell
python tests/openart_single_player_map_test.py
```

Expected: FAIL，因为 `count_player_cells()`、`update_non_player_background()` 和 `build_canonical_player_map()` 尚未定义。

- [ ] **Step 3: Implement the minimum pure helpers**

在 `openmv/main_see.py` 中增加无硬件依赖函数：

```python
def count_player_cells(element_matrix):
    count = 0
    for row in element_matrix:
        for element in row:
            if element == "player":
                count += 1
    return count


def update_non_player_background(element_matrix, background_matrix):
    for row_idx in range(GRID_ROWS):
        for col_idx in range(GRID_COLS):
            element = element_matrix[row_idx][col_idx]
            if element != "player":
                background_matrix[row_idx][col_idx] = element


def build_canonical_player_map(element_matrix, background_matrix,
                               precise_center_grid=None):
    selected_row = -1
    selected_col = -1
    if precise_center_grid is not None:
        col_q, row_q = precise_center_grid
        if not (0 <= col_q < GRID_COLS * 100 and
                0 <= row_q < GRID_ROWS * 100):
            return None
        selected_col = col_q // 100
        selected_row = row_q // 100
    else:
        for row_idx in range(GRID_ROWS):
            for col_idx in range(GRID_COLS):
                if element_matrix[row_idx][col_idx] == "player":
                    if selected_row >= 0:
                        return None
                    selected_row = row_idx
                    selected_col = col_idx
        if selected_row < 0:
            return None

    result = [["" for _ in range(GRID_COLS)]
              for _ in range(GRID_ROWS)]
    for row_idx in range(GRID_ROWS):
        for col_idx in range(GRID_COLS):
            element = element_matrix[row_idx][col_idx]
            background = background_matrix[row_idx][col_idx]
            if row_idx == selected_row and col_idx == selected_col:
                result[row_idx][col_idx] = "+" if background == "goal" else "C"
            elif element == "player":
                if background == "":
                    return None
                result[row_idx][col_idx] = ELEMENT_CHAR[background]
            else:
                result[row_idx][col_idx] = ELEMENT_CHAR[element]
    return result
```

实现中输出新的字符 matrix，不原地污染 `element_matrix`；选中格背景为 `goal` 时写 `+`，否则写 `C`；无法恢复未选 player 的背景时返回 `None`。

- [ ] **Step 4: Run canonical-map test GREEN**

Run:

```powershell
python tests/openart_single_player_map_test.py
```

Expected: `openart-single-player-map PASS`。

- [ ] **Step 5: Commit Task 1**

```powershell
git add openmv/main_see.py tests/openart_single_player_map_test.py
git commit -m "feat: add OpenART canonical player map"
```

### Task 2: Unify OpenART Player Center Resolution and Periodic Sending

**Files:**
- Modify: `openmv/main_see.py`
- Modify: `tests/openart_player_anchor_test.py`
- Modify: `tests/openart_request_flow_test.py`
- Modify: `tests/openart_precise_center_test.py`
- Test: `tests/openart_single_player_map_test.py`

- [ ] **Step 1: Write failing center-resolution tests**

新增测试覆盖：

```python
result = resolve_player_center(image, points, raw_matrix, stable_matrix,
                               previous_precise=(110, 100),
                               previous_cell=(5, 5),
                               rectified=True, raw_transform=None)
pixel_center, grid_q, cell = result
assert pixel_center == (110, 100)
assert grid_q == (550, 500)
assert cell == (5, 5)
```

实际实现可返回 tuple，不要求引入 class；测试还必须验证：

- 请求式中心有效时覆盖粗地图中另一个唯一 player。
- 普通唯一 player 帧不调用 blob 精确检测。
- 零/多 player 帧使用上一次精确中心，其次使用上一次唯一 C 格中心。
- 没有历史锚点且无法识别时返回 `None`。
- `player_center_to_grid_q` 改名后小车与箱子测试都调用 `image_center_to_grid_q()`。

- [ ] **Step 2: Run focused tests and verify RED**

```powershell
python tests/openart_player_anchor_test.py
python tests/openart_request_flow_test.py
python tests/openart_precise_center_test.py
```

Expected: 至少一个测试因缺少 `resolve_player_center()` 或 `image_center_to_grid_q()` 失败。

- [ ] **Step 3: Implement the shared resolver and main-loop gating**

在 `main_see.py` 中实现：

```python
def image_center_to_grid_q(center, rectified=True, raw_transform=None):
    if center is None:
        return None
    if not rectified:
        if raw_transform is None:
            return None
        x, y = center
        a, b, c, d, e, f, g, h = raw_transform
        m00 = a - x * g
        m01 = b - x * h
        m10 = d - y * g
        m11 = e - y * h
        determinant = m00 * m11 - m01 * m10
        if -0.000001 < determinant < 0.000001:
            return None
        u = ((x - c) * m11 - m01 * (y - f)) / determinant
        v = (m00 * (y - f) - (x - c) * m10) / determinant
        col_q = int(u * GRID_COLS * 100 + 0.5)
        row_q = int(v * GRID_ROWS * 100 + 0.5)
    else:
        width = IMG_WIDTH - GRID_LEFT_MARGIN - GRID_RIGHT_MARGIN
        height = IMG_HEIGHT - GRID_TOP_MARGIN - GRID_BOTTOM_MARGIN
        if width <= 0 or height <= 0:
            return None
        col_q = int(((center[0] - GRID_LEFT_MARGIN) * GRID_COLS * 100 /
                     width) + 0.5)
        row_q = int(((center[1] - GRID_TOP_MARGIN) * GRID_ROWS * 100 /
                     height) + 0.5)
    if not (0 <= col_q < GRID_COLS * 100 and
            0 <= row_q < GRID_ROWS * 100):
        return None
    return (col_q, row_q)


def resolve_player_center(img, recognition_points,
                          raw_element_matrix, stable_element_matrix,
                          previous_precise, previous_cell,
                          rectified, raw_transform):
    anchor = previous_precise
    if anchor is None and previous_cell is not None:
        row_idx, col_idx = previous_cell
        anchor = recognition_points[row_idx * GRID_COLS + col_idx]
    if anchor is None:
        anchor = find_player_coarse_center(
            recognition_points, raw_element_matrix, stable_element_matrix)
    blob_center = detect_player_center(
        img, recognition_points, raw_element_matrix,
        stable_element_matrix, anchor)
    precise_anchor = blob_center if blob_center is not None else anchor
    precise_center = detect_player_center_precise(img, precise_anchor)
    grid_q = image_center_to_grid_q(
        precise_center, rectified, raw_transform)
    if grid_q is None:
        return None
    return (precise_center, grid_q,
            (grid_q[1] // 100, grid_q[0] // 100))
```

主循环改为：

```python
player_count = count_player_cells(element_matrix)
need_precise = (center_request_active or observation_request_active or
                DEBUG_PLAYER_CENTER_ENABLE or DEBUG_OBSERVATION_ENABLE or
                player_count != 1)
```

普通 `player_count == 1` 只生成规范地图并更新最后唯一 cell；异常或请求帧才调用 resolver。新的请求 generation 只清请求样本，不清全局历史搜索锚点。周期发送仅在 `canonical_char_matrix is not None` 时调用 `send_map_uart()`；异常恢复成功时携带真实 `grid_q`，唯一粗 C 快速路径携带 `None`。

- [ ] **Step 4: Run all OpenART player/request tests GREEN**

```powershell
python tests/openart_single_player_map_test.py
python tests/openart_player_anchor_test.py
python tests/openart_precise_center_test.py
python tests/openart_request_flow_test.py
```

Expected: 四个测试全部 PASS。

- [ ] **Step 5: Commit Task 2**

```powershell
git add openmv/main_see.py tests/openart_single_player_map_test.py tests/openart_player_anchor_test.py tests/openart_precise_center_test.py tests/openart_request_flow_test.py
git commit -m "feat: unify OpenART player center resolution"
```

### Task 3: Unify Requested Box Center and Pair OBSERVE Samples with Maps

**Files:**
- Modify: `openmv/main_see.py`
- Modify: `tests/openart_observation_test.py`
- Modify: `tests/openart_request_flow_test.py`

- [ ] **Step 1: Write failing requested-box and pairing tests**

扩展 `openart_observation_test.py`：

```python
center = resolve_requested_box_center(adapter, grid_points,
                                      stable_matrix, 5, 8)
assert center is not None
assert image_center_to_grid_q(center) == (875, 535)

resolve_requested_box_center(adapter, grid_points,
                             matrix_without_requested_box, 5, 8)
assert result is None
```

增加相邻箱子最近选择、完全等距拒绝、±80q边界接受、81q拒绝。FakeUart 断言每个样本的输出顺序：

```python
assert uart.tx[0] == "MAP_BEGIN\n"
assert len(uart.tx[1:13]) == 12
assert uart.tx[13] == "PLAYER_CENTER_GRID 750,550 1\n"
assert uart.tx[14] == "MAP_END\n"
assert uart.tx[15] == "OBSERVE_SAMPLE 1,750,550,850,550\n"
```

- [ ] **Step 2: Run observation test RED**

```powershell
python tests/openart_observation_test.py
```

Expected: FAIL，因为正式指定箱子入口尚未验证请求格 `B`，且 `process_observation_request()` 尚未发送配套地图。

- [ ] **Step 3: Implement requested-box resolver and paired send**

在 `main_see.py` 中增加统一语义常量和入口：

```python
BOX_REQUEST_MATCH_MAX_OFFSET_Q = 80

def resolve_requested_box_center(img, recognition_points,
                                 element_matrix, row_idx, col_idx):
    if not (0 <= row_idx < GRID_ROWS and 0 <= col_idx < GRID_COLS):
        return None
    if element_matrix[row_idx][col_idx] != "box":
        return None
    request_matrix = [["space" for _ in range(GRID_COLS)]
                      for _ in range(GRID_ROWS)]
    request_matrix[row_idx][col_idx] = "box"
    centers = detect_box_centers(img, recognition_points, request_matrix)
    return select_box_center(centers, recognition_points,
                             row_idx, col_idx)


def box_center_grid_matches_request(box_center_grid, row_idx, col_idx):
    if box_center_grid is None:
        return False
    col_q, row_q = box_center_grid
    col_offset = col_q - (col_idx * 100 + 50)
    row_offset = row_q - (row_idx * 100 + 50)
    return (-BOX_REQUEST_MATCH_MAX_OFFSET_Q <= col_offset <=
            BOX_REQUEST_MATCH_MAX_OFFSET_Q and
            -BOX_REQUEST_MATCH_MAX_OFFSET_Q <= row_offset <=
            BOX_REQUEST_MATCH_MAX_OFFSET_Q)
```

调试全图中心和正式请求均复用 blob切分、去重和候选选择函数。`select_box_center()` 增加等距歧义标志：出现更近候选时清零，出现相同距离候选时置1，最终歧义时返回 `None`：

```python
if best_distance is None or distance < best_distance:
    best_center = (detected_x, detected_y)
    best_distance = distance
    ambiguous = False
elif distance == best_distance:
    ambiguous = True
return None if ambiguous else best_center
```

箱子中心转换为 q 坐标后必须先通过 `box_center_grid_matches_request()`。修改观察发送接口：

```python
def process_observation_request(uart, canonical_char_matrix,
                                player_center_grid, box_center_grid):
    global observation_request_active
    global observation_request_sample_count

    if (not observation_request_active or uart is None or
            canonical_char_matrix is None or
            player_center_grid is None or box_center_grid is None):
        return False
    if not send_map_uart(uart, canonical_char_matrix, player_center_grid):
        return False
    sample_index = observation_request_sample_count + 1
    uart.write("OBSERVE_SAMPLE %d,%d,%d,%d,%d\n" %
               (sample_index,
                player_center_grid[0], player_center_grid[1],
                box_center_grid[0], box_center_grid[1]))
    observation_request_sample_count = sample_index
    if observation_request_sample_count >= OBSERVATION_SAMPLE_COUNT:
        observation_request_active = False
    return True
```

只有同一循环中的小车精确中心、请求箱子精确中心和规范地图都有效时才发送；新请求清除旧计数，不保存上一只箱子的中心。

- [ ] **Step 4: Run OpenART observation/request tests GREEN**

```powershell
python tests/openart_observation_test.py
python tests/openart_request_flow_test.py
python -m py_compile openmv/main_see.py
```

Expected: 两个测试 PASS，`py_compile` exit 0。

- [ ] **Step 5: Commit Task 3**

```powershell
git add openmv/main_see.py tests/openart_observation_test.py tests/openart_request_flow_test.py
git commit -m "feat: pair requested box centers with canonical maps"
```

### Task 4: Make MCU UART a Strict Canonical-Map Validator

**Files:**
- Modify: `project/user/inc/drive_config.h`
- Modify: `project/user/src/openart_uart.c`
- Modify: `project/user/inc/openart_uart.h`
- Modify: `tests/openart_center_request_test.c`

- [ ] **Step 1: Write failing UART map validation tests**

在 `openart_center_request_test.c` 中将旧的“普通多 C 地图仍发布”预期改为：

```c
frame_before = openart_uart_get_frame_count();
map_copy_rows(saved_rows, openart_map_get());
feed_map_with_two_c_and_invalid_center();
passed &= (frame_before == openart_uart_get_frame_count());
passed &= map_rows_equal(saved_rows, openart_map_get());
```

增加：零 C拒绝；唯一 `+` 接受；`valid=1` 中心格与唯一 C 不一致拒绝；拒绝帧不改变地图、中心和 frame count；合法帧才产生 `MAP_OK`。

- [ ] **Step 2: Run UART test RED**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_openart_center_request_test.ps1
```

Expected: 新增非法帧用例 FAIL，因为当前 `accept_map()` 总会发布并递增 frame count。

- [ ] **Step 3: Implement strict acceptance without C rewriting**

将本地常量移到配置：

```c
#define ART_BOX_OBSERVE_SAMPLE_COUNT (3u)
#define ART_BOX_MATCH_MAX_OFFSET_Q   (80)
```

在 `openart_uart.c` 中将 `normalize_staging_player_from_center()` 替换为只读校验，例如：

```c
static uint8 validate_staging_player(void)
{
    uint8 row;
    uint8 col;
    if(0 == staging_player_center_received) return 0u;
    if(0 == find_unique_staging_player(&row, &col)) return 0u;
    if(0u != staging_player_center_valid)
    {
        if((staging_player_center_col_q >= MAP_COLS * 100u) ||
           (staging_player_center_row_q >= MAP_ROWS * 100u)) return 0u;
        if((staging_player_center_col_q / 100u != col) ||
           (staging_player_center_row_q / 100u != row)) return 0u;
    }
    return 1u;
}
```

`accept_map()` 仅在校验成功后复制 snapshot、发布中心、递增 frame count 并发送 `MAP_OK`；失败时只增加协议错误计数并保留旧状态。

- [ ] **Step 4: Run UART map tests GREEN**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_openart_center_request_test.ps1
```

Expected: 所有 map/center 请求测试 PASS。

- [ ] **Step 5: Commit Task 4**

```powershell
git add project/user/inc/drive_config.h project/user/inc/openart_uart.h project/user/src/openart_uart.c tests/openart_center_request_test.c
git commit -m "feat: validate canonical OpenART maps on MCU"
```

### Task 5: Atomically Pair OBSERVE Samples on MCU

**Files:**
- Modify: `project/user/src/openart_uart.c`
- Modify: `project/user/inc/openart_uart.h`
- Modify: `tests/openart_center_request_test.c`

- [ ] **Step 1: Write failing observation-pairing tests**

增加以下用例：

```c
openart_request_observation(5u, 8u);
feed_line("OBSERVE_SAMPLE 1,750,550,850,550");
passed &= (0u == openart_get_observation_sample(&sample));

feed_valid_paired_map(750u, 550u, 5u, 7u, 5u, 8u);
feed_line("OBSERVE_SAMPLE 1,750,550,850,550");
passed &= (1u == openart_get_observation_sample(&sample));
```

再覆盖：样本小车中心与地图不同、没有唯一 C、请求格不再是 B、箱子中心偏差81q、重复使用同一 frame、序号跳跃和新请求残留。

- [ ] **Step 2: Run UART test RED**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_openart_center_request_test.ps1
```

Expected: 无新地图和错误请求格样本仍被旧解析器接受，测试 FAIL。

- [ ] **Step 3: Add observation request context and frame checks**

在 `openart_uart.c` 保存：

```c
static uint8 observation_request_row;
static uint8 observation_request_col;
static uint32 observation_last_map_frame;
```

`openart_request_observation()` 设置请求格并以当前 frame count 作为起点。`parse_observation_line()` 在入队前验证：活动请求、连续 index、新合法 frame、样本 car 等于 `PLAYER_CENTER_GRID`、请求格为 `B`、box q 在请求格中心 ±`ART_BOX_MATCH_MAX_OFFSET_Q`。接受后更新 `observation_last_map_frame`；新请求清空所有观察队列和帧关联。

- [ ] **Step 4: Run UART test GREEN**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_openart_center_request_test.ps1
```

Expected: 所有 CENTER/OBSERVE 协议测试 PASS。

- [ ] **Step 5: Commit Task 5**

```powershell
git add project/user/inc/openart_uart.h project/user/src/openart_uart.c tests/openart_center_request_test.c
git commit -m "feat: pair box observations with fresh maps"
```

### Task 6: Remove Downstream First-C and Second-Authority Behavior

**Files:**
- Modify: `project/user/src/map_utils.c`
- Modify: `project/user/inc/map_utils.h`
- Modify: `project/user/src/menu.c`
- Modify: `project/user/src/screen.c`
- Modify: `project/user/src/subject2_logic.c`
- Modify: `project/user/inc/subject2_logic.h`
- Modify: `project/user/src/subject2.c`
- Modify: `tests/subject2_logic_test.c`
- Modify: `tests/subject2_scan_test.c`

- [ ] **Step 1: Write failing unique-car consumer tests**

在 `subject2_logic_test.c` 和 `subject2_scan_test.c` 增加/更新断言：

```c
map_scan_stats(&multi_car_map.source, &stats);
passed &= ((2u == stats.car_count) &&
           (0xFFu == stats.car_row) && (0xFFu == stats.car_col));
```

删除“科目二把 paired map 的 C 移到中心格”的预期，改为：OpenART 配套地图本身已在中心格，科目二只计算 pose offset。增加 `subject2_begin()`、扫描规划和推箱重规划遇到非唯一车格时进入现有地图错误，而不是使用第一格。

- [ ] **Step 2: Run subject2 tests RED**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
```

Expected: 旧 `map_scan_stats()` 保留第一个 C，且 subject2 仍调用 `subject2_normalize_center_map()`，测试 FAIL。

- [ ] **Step 3: Implement one-way canonical-map consumption**

修改 `map_scan_stats()`：先统计数量；扫描结束后只在 `car_count == 1` 时保留 row/col，否则写 `0xFF`。`map_find_car()` 继续作为唯一车格接口。

删除 `subject2_normalize_center_map()` 声明、定义和专属测试。`subject2.c` 对请求配套图执行：

```c
if((0 == map_find_car(source, &car_row, &car_col, 0)) ||
   (car_col != center_col_q / 100u) ||
   (car_row != center_row_q / 100u))
{
    subject2_fail(EXEC_ERROR_ART_CENTER, "E:CRef", update);
    return;
}
```

随后只用中心相对该 C 格中心的 dx/dy 修正 pose，不重写 snapshot 字符。

`menu.c` 必须检查 `map_find_car()` 返回值后再 `executor_start()`；`screen.c` 在找不到唯一车格时只画字符地图，不叠加 pose 车格。

- [ ] **Step 4: Run map/subject2 tests GREEN**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_subject2_logic_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_subject2_scan_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_competition_flow_test.ps1
```

Expected: 三个测试脚本全部 PASS。

- [ ] **Step 5: Commit Task 6**

```powershell
git add project/user/inc/map_utils.h project/user/src/map_utils.c project/user/src/menu.c project/user/src/screen.c project/user/inc/subject2_logic.h project/user/src/subject2_logic.c project/user/src/subject2.c tests/subject2_logic_test.c tests/subject2_scan_test.c
git commit -m "refactor: consume only canonical player maps"
```

### Task 7: Align Executor Box Constants and Preserve Failure Policy

**Files:**
- Modify: `project/user/src/executor.c`
- Modify: `project/user/src/art_replan.c`
- Modify: `tests/executor_pre_push_test.c`
- Modify: `tests/art_replan_requested_center_test.c`

- [ ] **Step 1: Write failing shared-constant and flow tests**

更新 executor 测试使用 `ART_BOX_OBSERVE_SAMPLE_COUNT` 和 `ART_BOX_MATCH_MAX_OFFSET_Q`，覆盖：

```c
collect_box_observation(550u, 550u, 730u, 550u);  // 请求格(5,6)中心650q，+80q接受
collect_box_observation(550u, 550u, 731u, 550u);  // +81q拒绝
```

科目一测试增加：只有 UART 已收满3个配套观察样本才进入准备位；几何失败调用 `executor_continue_after_pre_push_center()`，不设置 `E:BGeo`；准备位20ms超时仍保留 `E:BTim`。

- [ ] **Step 2: Run executor/art_replan tests RED**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_requested_center_test.ps1
```

Expected: 本地硬编码 `3u/80` 与配置测试不一致，至少一个新增测试 FAIL。

- [ ] **Step 3: Replace local constants and keep executor as sole median owner**

在 `executor.c` 删除：

```c
#define ART_BOX_OBSERVATION_FILTER_WINDOW (3u)
#define ART_BOX_MATCH_MAX_OFFSET_Q (80)
```

所有观察数组、中值、边界检查改用 `ART_BOX_OBSERVE_SAMPLE_COUNT` 和配置中的 `ART_BOX_MATCH_MAX_OFFSET_Q`。`art_replan.c` 继续只转发 UART 样本，不增加另一套中值。不得重新引入 `E:BGeo` 硬停车；不得改变 BObs 重试、BTim 停车和准备位运动顺序。

- [ ] **Step 4: Run executor/art_replan tests GREEN**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_executor_pre_push_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_art_replan_requested_center_test.ps1
```

Expected: 两个脚本全部 PASS。

- [ ] **Step 5: Commit Task 7**

```powershell
git add project/user/src/executor.c project/user/src/art_replan.c tests/executor_pre_push_test.c tests/art_replan_requested_center_test.c
git commit -m "refactor: unify requested box observation semantics"
```

### Task 8: Update Protocol Documentation and Run Full Verification

**Files:**
- Modify: `AGENTS.md`
- Modify: `docs/competition/openart_plus_rt1064_uart_experience.md`
- Verify: all files changed in Tasks 1-7

- [ ] **Step 1: Update long-lived protocol documentation**

明确写入：

```text
OpenART 是 C/+ 唯一生成者。
普通 MAP 必须恰好一个 C/+。
valid=1 时 PLAYER_CENTER_GRID 所在格必须与 C/+ 一致。
CENTER_SAMPLE 与 OBSERVE_SAMPLE 前必须各有一张新规范 MAP。
精确箱子中心只用于准备位，不修改 B 或触发重规划。
```

同步记录非法帧不会更新 snapshot/frame count/B/T基线，也不会回复 `MAP_OK`。

- [ ] **Step 2: Run all PowerShell host tests**

```powershell
Get-ChildItem tests -Filter "run_*.ps1" | ForEach-Object {
    powershell -ExecutionPolicy Bypass -File $_.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
```

Expected: 所有脚本 exit 0。

- [ ] **Step 3: Run all applicable Python tests and syntax checks**

```powershell
Get-ChildItem tests -Filter "*_test.py" |
    Where-Object { $_.Name -ne "art2_model_sample_test.py" } |
    ForEach-Object {
        python $_.FullName
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }
python -m py_compile openmv/main_see.py "openmv/视觉/main.py"
```

Expected: 所有适用 Python 测试 PASS，`py_compile` exit 0。`art2_model_sample_test.py` 只有在 `openmv/视觉/1/train` 数据集存在时另行运行；缺失数据集不是本功能失败。

- [ ] **Step 4: Build Keil and inspect the build log**

```powershell
& "D:\Keil_v5\UV4\UV4.exe" -b "C:\Users\ye\Desktop\rt1064\project\mdk\rt1064.uvprojx"
Select-String -Path "project/mdk/Objects/rt1064.build_log.htm" -Pattern "Error\(s\)|Warning\(s\)"
```

Expected:

```text
".\Objects\rt1064.axf" - 0 Error(s), 0 Warning(s).
```

- [ ] **Step 5: Run final static checks**

```powershell
if (rg -n "normalize_staging_player_from_center|subject2_normalize_center_map|E:BGeo" project/user) { exit 1 }
git diff --check
git diff --cached --check
```

Expected: 两个旧地图改写函数在生产代码中无结果；生产代码无 `E:BGeo`；diff checks exit 0。测试中可保留 `E:BGeo` 的负向断言。

- [ ] **Step 6: Commit Task 8**

```powershell
git add AGENTS.md docs/competition/openart_plus_rt1064_uart_experience.md
git commit -m "docs: document canonical OpenART center protocol"
```

---

## On-Board Acceptance Checklist

- [ ] 普通地图只有一个 C 时帧率与当前高速路径接近，OpenART 不运行精确中心 blob。
- [ ] 原始零 C 或多 C 时，OpenART 能用历史锚点恢复并只下发一个 C/+。
- [ ] 精确恢复失败时 MCU 保持上一张合法地图，frame count 不增长。
- [ ] `CENTER_REQ` 的每个样本与新规范地图一致，科目一和科目二不再出现跨格二次改 C。
- [ ] `OBSERVE_REQ` 的3个样本分别与新规范地图配套，指定箱子中心不串到相邻箱子。
- [ ] 推箱前仍按 `BGap -> BAlign -> BNear -> Push` 执行，精确箱子中心不修改字符地图 B。
- [ ] 几何失败不出现 `E:BGeo`，观察超时仍执行 BObs 重试，准备位运动超时仍显示 BTim。
- [ ] 推箱完成仍只由稳定地图 B/T 减少确认，科目一重规划、科目二绑定和返航流程保持正常。
