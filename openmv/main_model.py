# OpenMV / Open Art Mini
# AI 图像分类模型（EIQ 训练专用）
import sensor, image, time, tf

# ====================== 摄像头初始化 ======================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)   # 彩色图
sensor.set_framesize(sensor.QVGA)     # 320x240（
sensor.set_brightness(0)              # 亮度默认
sensor.skip_frames(time = 1000)       # 稳定1秒
sensor.set_auto_gain(False)           # 必须关！AI 必加
sensor.set_auto_whitebal(True)        # 自动白平衡
sensor.set_auto_exposure(False, exposure_us=200)
clock = time.clock()

# ====================== 加载模型 ======================
# 你的模型文件（放在 SD 卡根目录）
net = tf.load("a.tflite", load_to_fb=True)

# 你的标签文件（和 EIQ 训练时一样）
labels = [line.rstrip() for line in open("/sd/labels.txt")]

print("✅ 分类模型加载成功！")
print("标签：", labels)

# ====================== 主循环 ======================
while(True):
    clock.tick()
    img = sensor.snapshot()

    # ====================== AI 分类推理 ======================
    # 对整张图进行分类（分类模型就是这么用的）
    result = tf.classify(net, img)[0]

    # 找到置信度最高的类别
    max_idx = result.output().index(max(result.output()))
    label = labels[max_idx]
    confidence = result.output()[max_idx]

    # ====================== 屏幕显示 ======================
    img.draw_string(5, 5,  f"Class: %s" % label,        scale=2, color=(0,255,0))
    img.draw_string(5, 30, f"Conf:  %.2f" % confidence, scale=2, color=(255,255,0))
    img.draw_string(5, 55, f"FPS:   %.1f" % clock.fps(), scale=2, color=(255,255,255))

    # ====================== 串口输出 ======================
    print(f"识别结果：{label} | 置信度：{confidence:.2f}")
