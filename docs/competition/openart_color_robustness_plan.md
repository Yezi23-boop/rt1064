# OpenART 颜色识别鲁棒性优化计划

本文记录 OpenART #1 识别 16x12 地图时，屏幕中心亮、边缘暗、局部模糊导致颜色识别不稳定的分析结论和后续实现计划。目标是给后续 agent 接手 `openmv/openart_plus_grid_recognizer_pure.py` 时使用。

## 当前问题

当前识别代码已经从绝对 RGB 阈值升级为归一化颜色判断：

```python
rn = r * 255 // (r + g + b)
gn = g * 255 // (r + g + b)
bn = b * 255 // (r + g + b)
color_sum = r + g + b
```

其中 `rn/gn/bn` 表示颜色比例，`color_sum` 近似表示总亮度。

当前这种归一化方式是合理的，尤其适合本项目这种固定色块地图。它的优势是：同一种颜色在中心亮区和边缘暗区的 RGB 绝对值可能差很多，但颜色比例通常仍然接近。

例如蓝色空地：

```text
亮处 RGB=(34,34,255)
norm=(26,26,201)
color_sum=323

暗处 RGB=(9,8,64)
norm=(28,25,201)
color_sum=81
```

可以看到亮度差了很多，但 `bn` 仍然稳定在 200 左右，因此继续用 `rn/gn/bn` 做主判断是正确方向。

目标点也类似：

```text
亮粉目标点 RGB=(217,28,217)
norm=(119,15,119)

暗粉目标点 RGB=(96,12,96)
norm=(120,15,120)
```

因此不建议改回纯 RGB 阈值，也不建议在当前阶段上 HSV 或 Lab。RGB 比例归一化计算量低，更适合 OpenART 上实时跑。

归一化的限制也要明确：

1. 极暗噪声会被比例放大，例如 `RGB=(1,0,8)` 归一化后也像蓝色，所以仍需要 `color_sum` 做极暗过滤。
2. 局部过曝会破坏颜色比例，例如接近 `RGB=(255,255,255)` 时归一化会变成近似灰色，代码很难完全救；应优先降低屏幕亮度、缩短曝光或固定增益。

这对“同一种颜色整体变亮或变暗”的情况是有效的。但在中心亮、边缘暗、图像模糊时，仍有两个问题：

1. `color_sum` 门槛过高时，暗边缘的真实颜色会被卡掉。
2. 黑点回看逻辑把极暗点直接返回 `space`，可能把暗墙体误判为空地。

## 暗边缘样本结论

测试图：

```text
openmv/fe48eb2150ae5a6eea643c6615610fcf_center_bright_edge_darkest_blurred.png
```

代表采样结果：

```text
左侧很暗蓝空地：
RGB=(9,8,64)
norm=(28,25,201)
color_sum=81
当前会被 color_sum > 90 卡掉，最终可能识别成 wall。

中心亮蓝空地：
RGB=(34,34,255)
norm=(26,26,201)
color_sum=323
当前可识别为 space。

右上暗粉目标点：
RGB=(96,12,96)
norm=(120,15,120)
color_sum=204
当前可识别为 goal。

暗绿色小车：
RGB=(13,96,16)
norm=(26,195,32)
color_sum=125
当前可识别为 player。

暗青色小车：
RGB=(13,97,97)
norm=(16,119,119)
color_sum=207
当前可识别为 player。

暗墙体：
RGB=(24,23,26)
norm=(83,80,90)
color_sum=73
当前应保持 wall，但黑点回看逻辑存在误判 space 的风险。
```

结论：

```text
归一化颜色方向是正确的。
不应完全移除 color_sum。
color_sum 应从“强亮度门槛”降级为“极暗噪声过滤”。
黑点回看逻辑需要修改，不能在暗图下直接返回 space。
```

## 不建议的做法

不要直接删除所有 `color_sum` 判断。

原因：极暗噪声也可能在归一化后表现出某个通道占比很高，例如：

```text
RGB=(1,0,8)
归一化后 B 占比很高，但它不一定是真实蓝色空地。
```

完全不用 `color_sum` 会增加黑边、阴影、噪声误判为 `space`、`goal` 或 `player` 的风险。

## 推荐实现策略

### 1. 用常量统一亮度门槛

先把散落在分类条件中的亮度门槛抽成常量，便于现场调试：

```python
MIN_VALID_COLOR_SUM = 40
MIN_SPACE_SUM = 45
MIN_BOX_GOAL_SUM = 60
MIN_PLAYER_SUM = 60
DARK_PIXEL_THRESHOLD = 60
```

含义：

- `MIN_VALID_COLOR_SUM`：低于这个值基本认为是黑边、阴影或无效噪声。
- `MIN_SPACE_SUM`：空地识别的最低有效亮度。
- `MIN_BOX_GOAL_SUM`：箱子和目标点识别的最低有效亮度。
- `MIN_PLAYER_SUM`：小车颜色识别的最低有效亮度。
- `DARK_PIXEL_THRESHOLD`：判断中心点是否可能被黑点、阴影或调试覆盖影响。

这些值是起点，不是最终值。后续应结合 OpenMV IDE 真实采样数据继续调。

### 2. 降低 color_sum 的作用

当前逻辑近似为：

```python
box:    color_sum > 120
goal:   color_sum > 120
space:  color_sum > 90
player: color_sum >= 100
```

建议改成：

```python
box:    color_sum > MIN_BOX_GOAL_SUM
goal:   color_sum > MIN_BOX_GOAL_SUM
space:  color_sum > MIN_SPACE_SUM
player: color_sum > MIN_PLAYER_SUM
```

分类仍主要依赖 `rn/gn/bn` 颜色比例，`color_sum` 只排除极暗噪声。

### 3. 修改黑点回看逻辑

当前危险逻辑：

```python
if r < 60 and g < 60 and b < 60:
    r, g, b = get_average_pixel(img, x + 3, y + 3)
    if r < 60 and g < 60 and b < 60:
        return "space"
```

问题：暗墙体、暗边缘也可能满足 `r<60 and g<60 and b<60`，不应直接返回 `space`。

建议改成：

```python
if r < DARK_PIXEL_THRESHOLD and g < DARK_PIXEL_THRESHOLD and b < DARK_PIXEL_THRESHOLD:
    rr, gg, bb = get_average_pixel(img, x + 3, y + 3)
    if rr + gg + bb > r + g + b:
        r, g, b = rr, gg, bb
```

也就是：中心点很暗时，只尝试偏移重采样；如果偏移点更亮，就替换采样值。不要在这里直接判定元素类型。

后续再由归一化颜色分类决定 `space / wall / goal / box / player`。

### 4. 增加失败点调试输出

建议增加开关：

```python
DEBUG_PRINT_UNKNOWN_CELLS = False
```

当内圈格子最后兜底为 `wall` 时，按周期打印：

```text
UNKNOWN row=3 col=5 xy=(123,87) rgb=(9,8,64) norm=(28,25,201) sum=81
```

用途：

- 判断是不是暗蓝空地被误判为墙。
- 判断是不是采样点落到墙体纹理或边界上。
- 判断是不是 `color_sum` 仍然太高。
- 判断是不是 `rn/gn/bn` 阈值需要局部调整。

打印必须受周期限制，避免严重影响帧率。

## 推荐分类顺序

保留当前整体顺序，但调整暗点处理：

```text
1. 外圈强制 wall。
2. 中心点很暗时只做偏移重采样，不直接返回 space。
3. player：归一化颜色 + 多点采样，sum 只排极暗。
4. box：归一化黄色，sum 降低。
5. goal：归一化品红，sum 降低。
6. bomb：默认关闭；后续有炸弹再接入。
7. space：归一化蓝色，sum 降低。
8. 兜底 wall。
```

## 后续实现优先级

第一版实现：

```text
1. 新增亮度阈值常量。
2. 降低 box / goal / space / player 的 color_sum 门槛。
3. 修改黑点回看逻辑，不再直接 return "space"。
4. 保持现有 rn/gn/bn 阈值不大改。
```

第二版实现：

```text
1. 增加 DEBUG_PRINT_UNKNOWN_CELLS。
2. 收集错识别点的 RGB / norm / color_sum。
3. 只针对真实失败样本微调 rn/gn/bn 阈值。
```

第三版实现：

```text
如果同一元素在不同区域的 norm 也明显漂移，再考虑分区域阈值或局部校准。
不要一开始就做分区域阈值，避免参数过多、难以现场调试。
```

## 验收标准

1. 暗边缘蓝色空地，例如 `RGB=(9,8,64), norm=(28,25,201), sum=81`，应能识别为 `space`。
2. 暗墙体，例如 `RGB=(24,23,26), norm=(83,80,90), sum=73`，不应因为黑点回看直接识别成 `space`。
3. 箱子、目标点、小车在中心亮区和边缘暗区都应保持稳定。
4. 调试绘制和打印关闭后，帧率不应因颜色鲁棒性逻辑明显下降。
