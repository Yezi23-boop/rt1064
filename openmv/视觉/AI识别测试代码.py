import sensor, image, time, tf, gc

# ============================================================
# ★★★ 变焦设置 — 你只需要改 ZOOM 这个值 ★★★
# ============================================================

# 【缩放倍率】
#   1.0 = 原图不放大，2.0 = 中心放大2倍，3.0 = 3倍...
#   范围建议 1.0 ~ 4.0（超过 4.0 画面太糊，识别率下降）
ZOOM = 1.0

# ============================================================
# ★★★ 以下不用改 ★★★
# ============================================================

# --- 摄像头初始化（你的原始配置） ---
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_brightness(400)
sensor.set_contrast(2)
sensor.set_vflip(True)         #垂直
# ============================================================
# ★★★ 变焦函数 —— 改 ZOOM 的值就行 ★★★
# ============================================================
def set_zoom(level):
    """
    【教学】数字变焦原理
    sensor.set_windowing((x, y, w, h)) 让 CMOS 只采集画面中心
    w×h 的区域，硬件自动拉伸到全屏 → 看起来就是"放大"效果

    参数: level — 放大倍数，1.0=原图 2.0=2倍 等等
    """
    W, H = 320, 240                         # QVGA 分辨率
    crop_w = int(W / level)
    crop_h = int(H / level)
    offset_x = (W - crop_w) // 2            # 居中裁剪
    offset_y = (H - crop_h) // 2

    sensor.set_windowing((offset_x, offset_y, crop_w, crop_h))
    sensor.skip_frames(time=100)            # 切换后等画面稳定
    print("🔍 变焦 x%.1f（裁剪 %dx%d 居中 — 拉伸到 %dx%d）"
          % (level, crop_w, crop_h, W, H))

# --- 执行变焦（直接调 set_zoom，改 ZOOM 的值即可） ---
set_zoom(ZOOM)

# ★ 镜像和翻转必须在 set_windowing() 之后设置，否则会被覆盖

sensor.set_hmirror(True)       #水平
sensor.skip_frames(time=200)

# --- 加载 AI 模型和标签 ---
net = tf.load("/sd/cartoon_V3.tflite", load_to_fb=True)
labels = [line.rstrip() for line in open("/sd/cartoon_labels.txt")]

print("✅ 模型加载成功，共 %d 类" % len(labels))
print("标签顺序：", labels)
print("=" * 50)

# ============================================================
# ★★★ 主循环 — 你的原始识别逻辑，完全没动 ★★★
# ============================================================
while True:
    img = sensor.snapshot()           # 这里的 img 已是 zoom 后的画面
    result = tf.classify(net, img)[0]
    outputs = result.output()

    # 打印所有类别的置信度
    print("\n[ZOOM: x%.1f] 所有类别置信度：" % ZOOM)
    for i in range(len(labels)):
        print("  class_id=%d %s: %.4f" % (i, labels[i], outputs[i]))

    # 找到最高置信度
    max_idx = outputs.index(max(outputs))
    print("\n最高置信度：class_id=%d %s mapped_digit=%d (%.4f)" %
          (max_idx, labels[max_idx], max_idx, outputs[max_idx]))
    print("-" * 50)

    gc.collect()
