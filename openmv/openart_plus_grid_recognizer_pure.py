import sensor
import time

# ==================== 图像与地图尺寸 ===================
# False: QVGA 320x240，运行速度更高，当前人工标定坐标按此模式填写。
# True : VGA  640x480，细节更多但帧率更低；启用后必须重新填写 MAP_CORNERS。
USE_VGA = False
FRAME_SCALE = 2 if USE_VGA else 1
IMG_WIDTH = 320 * FRAME_SCALE
IMG_HEIGHT = 240 * FRAME_SCALE
GRID_COLS = 16
GRID_ROWS = 12

# ==================== 旧固定矩形采样回退参数 ===================
# 当 USE_QUAD_CALIBRATION=False 时才使用这些参数。
# MAP_ROI 只用于画绿色参考框；采样点由起始点、格距和偏移生成。
MAP_ROI = (25 * FRAME_SCALE, 15 * FRAME_SCALE, 270 * FRAME_SCALE, 210 * FRAME_SCALE)
COL_STEP = 17 * FRAME_SCALE
ROW_STEP = 17 * FRAME_SCALE
DOT_START_X = 35 * FRAME_SCALE
DOT_START_Y = 25 * FRAME_SCALE
SAMPLE_OFFSET_X = -6 * FRAME_SCALE
SAMPLE_OFFSET_Y = -6 * FRAME_SCALE

# ==================== 人工四角标定 ===================
# True: 使用 MAP_CORNERS 表示的屏幕地图外边界，适配相机斜视形成的梯形。
# False: 不做四角映射，退回上面的固定等间距采样参数。
# 四点必须按 左上、右上、右下、左下 填写，指向完整 16x12 地图外边界，
# 不是屏幕黑框、某个格子中心或局部墙体边缘。
# 初始值与当前固定步长网格近似等价；按 QVGA 填写，切换 VGA 后需重标。
USE_QUAD_CALIBRATION = True
MAP_CORNERS = (
    (21, 8),
    (306, 16),
    (300, 210),
    (20, 218),
)

ELEMENT_CHAR = {
    "wall": "#",
    "space": ".",
    "goal": "T",
    "box": "B",
    "bomb": "X",
    "player": "C",
}

DRAW_COLOR = {
    "wall": (128, 128, 128),
    "space": (0, 0, 255),
    "goal": (255, 0, 255),
    "box": (255, 255, 0),
    "bomb": (255, 0, 0),
    "player": (0, 255, 0),
}

PLAYER_SAMPLE_OFFSETS = (
    (0, 0),
    (-5 * FRAME_SCALE, 0),
    (5 * FRAME_SCALE, 0),
    (-7 * FRAME_SCALE, 0),
    (7 * FRAME_SCALE, 0),
    (0, -4 * FRAME_SCALE),
    (0, 4 * FRAME_SCALE),
)

# ==================== 调试、识别与相机开关 ===================
# 总调试开关。关闭后不画框、不画点、不周期打印地图，正式运行可关闭以提升 FPS。
DEBUG_ENABLE = True
# True 时绘制当前显示坐标系的绿色边界：
# 原图显示时为人工四边形；拉正显示时为矫正后图像边框。
DEBUG_DRAW_ROI = True
# True 时绘制 16x12 完整网格线，用来判断采样点是否真的落在格子中心。
# 网格线只用于调试显示，不参与识别；正式跑帧率时可关闭。
DEBUG_DRAW_GRID_LINES = True
# True 时绘制 192 个彩色实心圆点，圆点颜色代表该格子的识别结果。
DEBUG_DRAW_POINTS = True
# 只控制 OpenMV IDE 中看到的图像：
# True 时显示拉正后的地图，方便观察网格与颜色；不会单独决定识别输入。
SHOW_RECTIFIED_VIEW = True
# 只控制取色识别使用的图像：
# True 时先将当前帧拉正，再在规则 16x12 中心点上做 2x2 取色；
# False 时保留原图，通过四角投影得到的 192 个点直接取色，通常更快。
USE_RECTIFIED_RECOGNITION = True
# 地图打印周期，单位毫秒。打印频繁会影响实际帧率。
DEBUG_PRINT_PERIOD_MS = 500
# 当前测试画面没有炸弹；关闭后红色区域不会被分类为 bomb。
ENABLE_BOMB = False
# 固定相机自动项，避免自动曝光把 snapshot() 帧周期拉长到几十毫秒。
CAMERA_MANUAL_EXPOSURE = True
# 仅在 CAMERA_MANUAL_EXPOSURE=True 时生效；先保留当前短曝光，避免中心区域过曝。
CAMERA_EXPOSURE_US = 1000
CAMERA_LOCK_GAIN = True
CAMERA_LOCK_WHITEBAL = True

# 推荐组合：
# 1. 标定和看图：SHOW_RECTIFIED_VIEW=True,  USE_RECTIFIED_RECOGNITION=True
# 2. 正式高速识别：SHOW_RECTIFIED_VIEW=False, USE_RECTIFIED_RECOGNITION=False
# 3. 仅比较显示：SHOW_RECTIFIED_VIEW=True,  USE_RECTIFIED_RECOGNITION=False


def get_average_pixel(img, x, y, size=2):
    # 在格子中心附近做小区域均值采样；2x2 比单点抗噪，也比 4x4 更少混入边缘颜色。
    r_sum = 0
    g_sum = 0
    b_sum = 0
    count = 0

    half = size // 2
    for dy in range(-half, half):
        for dx in range(-half, half):
            px = x + dx
            py = y + dy
            if 0 <= px < IMG_WIDTH and 0 <= py < IMG_HEIGHT:
                pixel = img.get_pixel(px, py)
                r_sum += pixel[0]
                g_sum += pixel[1]
                b_sum += pixel[2]
                count += 1

    if count == 0:
        return (0, 0, 0)
    return (r_sum // count, g_sum // count, b_sum // count)


def build_fixed_grid_points():
    # 旧矩形标定路径：相机与屏幕足够正时，直接按固定步长生成 192 个采样点。
    grid_points = []
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            dot_x = DOT_START_X + col * COL_STEP
            dot_y = DOT_START_Y + row * ROW_STEP
            x = dot_x + SAMPLE_OFFSET_X
            y = dot_y + SAMPLE_OFFSET_Y
            grid_points.append((x, y))
    return grid_points


def build_quad_transform(corners):
    # 由人工四角建立标准矩形 (u, v) 到原相机图像四边形的透视映射参数。
    x0, y0 = corners[0]
    x1, y1 = corners[1]
    x2, y2 = corners[2]
    x3, y3 = corners[3]

    dx1 = x1 - x2
    dx2 = x3 - x2
    dx3 = x0 - x1 + x2 - x3
    dy1 = y1 - y2
    dy2 = y3 - y2
    dy3 = y0 - y1 + y2 - y3
    denominator = dx1 * dy2 - dx2 * dy1

    if abs(denominator) < 0.001:
        return None

    perspective_x = (dx3 * dy2 - dx2 * dy3) / denominator
    perspective_y = (dx1 * dy3 - dx3 * dy1) / denominator

    return (
        x1 - x0 + perspective_x * x1,
        x3 - x0 + perspective_y * x3,
        x0,
        y1 - y0 + perspective_x * y1,
        y3 - y0 + perspective_y * y3,
        y0,
        perspective_x,
        perspective_y,
    )


def project_grid_point(transform, u, v):
    denominator = transform[6] * u + transform[7] * v + 1.0
    x = (transform[0] * u + transform[1] * v + transform[2]) / denominator
    y = (transform[3] * u + transform[4] * v + transform[5]) / denominator
    return (int(x + 0.5), int(y + 0.5))


def build_grid_points():
    # 原图识别路径所用的采样点：开四角标定时，点会随梯形透视落到各格中心。
    if not USE_QUAD_CALIBRATION:
        return build_fixed_grid_points()

    transform = build_quad_transform(MAP_CORNERS)
    if transform is None:
        print("MAP_CORNERS_INVALID_FALLBACK_FIXED")
        return build_fixed_grid_points()

    grid_points = []
    for row in range(GRID_ROWS):
        v = (row + 0.5) / GRID_ROWS
        for col in range(GRID_COLS):
            u = (col + 0.5) / GRID_COLS
            grid_points.append(project_grid_point(transform, u, v))
    return grid_points


def build_rectified_grid_points(img_width, img_height):
    # 拉正图识别路径所用的采样点：地图已成为规则矩形，直接均匀分成 16x12。
    grid_points = []
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            x = int(((col + 0.5) * img_width / GRID_COLS) + 0.5)
            y = int(((row + 0.5) * img_height / GRID_ROWS) + 0.5)
            grid_points.append((x, y))
    return grid_points


def build_quad_corner_list():
    return [
        (int(MAP_CORNERS[0][0] + 0.5), int(MAP_CORNERS[0][1] + 0.5)),
        (int(MAP_CORNERS[1][0] + 0.5), int(MAP_CORNERS[1][1] + 0.5)),
        (int(MAP_CORNERS[2][0] + 0.5), int(MAP_CORNERS[2][1] + 0.5)),
        (int(MAP_CORNERS[3][0] + 0.5), int(MAP_CORNERS[3][1] + 0.5)),
    ]


def draw_calibration_boundary(img, rectified=False):
    if rectified:
        img.draw_rectangle((0, 0, img.width(), img.height()), color=(0, 255, 0), thickness=2)
        return

    if USE_QUAD_CALIBRATION:
        for idx in range(4):
            start = MAP_CORNERS[idx]
            end = MAP_CORNERS[(idx + 1) % 4]
            img.draw_line(
                (int(start[0]), int(start[1]), int(end[0]), int(end[1])),
                color=(0, 255, 0), thickness=2)
    else:
        img.draw_rectangle(MAP_ROI, color=(0, 255, 0), thickness=2)


def draw_grid_lines(img, rectified=False):
    # 网格线跟随当前显示坐标系：拉正图画规则网格，原图画投影后的四边形网格。
    color = (255, 255, 255)

    if rectified:
        width = img.width()
        height = img.height()
        for col in range(GRID_COLS + 1):
            x = int((col * width / GRID_COLS) + 0.5)
            if x >= width:
                x = width - 1
            img.draw_line((x, 0, x, height - 1), color=color, thickness=1)
        for row in range(GRID_ROWS + 1):
            y = int((row * height / GRID_ROWS) + 0.5)
            if y >= height:
                y = height - 1
            img.draw_line((0, y, width - 1, y), color=color, thickness=1)
        return

    if USE_QUAD_CALIBRATION:
        transform = build_quad_transform(MAP_CORNERS)
        if transform is None:
            return
        for col in range(GRID_COLS + 1):
            u = col / GRID_COLS
            x0, y0 = project_grid_point(transform, u, 0.0)
            x1, y1 = project_grid_point(transform, u, 1.0)
            img.draw_line((x0, y0, x1, y1), color=color, thickness=1)
        for row in range(GRID_ROWS + 1):
            v = row / GRID_ROWS
            x0, y0 = project_grid_point(transform, 0.0, v)
            x1, y1 = project_grid_point(transform, 1.0, v)
            img.draw_line((x0, y0, x1, y1), color=color, thickness=1)
        return

    left = DOT_START_X + SAMPLE_OFFSET_X - (COL_STEP / 2.0)
    top = DOT_START_Y + SAMPLE_OFFSET_Y - (ROW_STEP / 2.0)
    right = left + GRID_COLS * COL_STEP
    bottom = top + GRID_ROWS * ROW_STEP
    for col in range(GRID_COLS + 1):
        x = int(left + col * COL_STEP + 0.5)
        img.draw_line((x, int(top), x, int(bottom)), color=color, thickness=1)
    for row in range(GRID_ROWS + 1):
        y = int(top + row * ROW_STEP + 0.5)
        img.draw_line((int(left), y, int(right), y), color=color, thickness=1)


def draw_recognition_points(img, element_matrix, grid_points):
    # 只负责显示识别结论，不参与取色；避免调试圆点污染同一帧的颜色判断。
    for idx, (x, y) in enumerate(grid_points):
        row_idx = idx // GRID_COLS
        col_idx = idx % GRID_COLS
        element = element_matrix[row_idx][col_idx]
        img.draw_circle(x, y, 3, color=DRAW_COLOR[element], fill=True)


def normalize_color(r, g, b):
    color_sum = r + g + b
    if color_sum < 30:
        return (0, 0, 0, color_sum)
    return (
        r * 255 // color_sum,
        g * 255 // color_sum,
        b * 255 // color_sum,
        color_sum,
    )


def is_player_color(rn, gn, bn, color_sum):
    # 小车格子由亮绿色和青色两半组成。参考图中实测主色约为：
    # 绿色半边 RGB=(36,255,42)，青色半边 RGB=(36,255,255)。
    # 这里用归一化比例判断，避免曝光变化导致 RGB 绝对值整体变亮或变暗。
    if color_sum < 100:
        return False

    green_half = (
        rn <= 60 and
        gn >= 165 and
        bn <= 90 and
        gn > rn + 80 and
        gn > bn + 60
    )

    cyan_half = (
        rn <= 60 and
        95 <= gn <= 160 and
        95 <= bn <= 170 and
        gn > rn + 45 and
        bn > rn + 45 and
        abs(gn - bn) <= 55
    )

    # 真实摄像头可能把绿/青边界采成过渡色，保留一个窄的混合区间。
    mixed_half = (
        rn <= 60 and
        gn >= 145 and
        60 <= bn <= 125 and
        gn > rn + 80 and
        gn > bn + 25
    )

    return green_half or cyan_half or mixed_half


def sample_player_color(img, x, y, center_r, center_g, center_b):
    # 小车中心可能落在两色交界或模糊边缘；中心和周围点任一点命中即可。
    center_rn, center_gn, center_bn, center_sum = normalize_color(
        center_r, center_g, center_b)
    if is_player_color(center_rn, center_gn, center_bn, center_sum):
        return True

    for dx, dy in PLAYER_SAMPLE_OFFSETS:
        if dx == 0 and dy == 0:
            continue
        r, g, b = get_average_pixel(img, x + dx, y + dy)
        rn, gn, bn, color_sum = normalize_color(r, g, b)
        if is_player_color(rn, gn, bn, color_sum):
            return True
    return False


def classify_element(img, row_idx, col_idx, x, y):
    # 逐飞地图外圈固定为墙，强制处理可减少外圈纹理和边缘透视带来的误判。
    if (row_idx == 0 or row_idx == GRID_ROWS - 1 or
            col_idx == 0 or col_idx == GRID_COLS - 1):
        return "wall"

    r, g, b = get_average_pixel(img, x, y)

    # 若中心位置被黑色调试点或阴影覆盖，则向右下偏移一次重新读取。
    if r < 60 and g < 60 and b < 60:
        r, g, b = get_average_pixel(img, x + 3, y + 3)
        if r < 60 and g < 60 and b < 60:
            return "space"

    if sample_player_color(img, x, y, r, g, b):
        return "player"

    rn, gn, bn, color_sum = normalize_color(r, g, b)

    # 黄色箱子优先判断，避免高亮黄色被误当作其他高亮色块。
    if (
        color_sum > 120 and
        rn > 90 and gn > 95 and
        bn < 45 and
        abs(rn - gn) < 40
    ):
        return "box"

    # 目标点为品红色：R/B 同时较高且相近，G 明显较低。
    if (
        color_sum > 120 and
        rn > 90 and bn > 90 and
        gn < 45 and
        abs(rn - bn) < 45
    ):
        return "goal"

    # 当前场景没有炸弹，ENABLE_BOMB 默认关闭；以后有炸弹时再启用红色判定。
    if ENABLE_BOMB and r > g + 25 and r > b + 25:
        return "bomb"

    # 蓝色空地要求 B 通道比例明显占优，适配局部偏暗。
    if color_sum > 90 and bn > 105 and bn > rn + 35 and bn > gn + 20:
        return "space"

    return "wall"


def recognize_map(img, grid_points, element_matrix, char_matrix):
    for idx, (x, y) in enumerate(grid_points):
        row_idx = idx // GRID_COLS
        col_idx = idx % GRID_COLS
        element = classify_element(img, row_idx, col_idx, x, y)
        element_matrix[row_idx][col_idx] = element
        char_matrix[row_idx][col_idx] = ELEMENT_CHAR[element]


def print_map(char_matrix, fps, loop_fps, loop_us,
              snapshot_us, rectify_us, recognize_us, display_us, exposure_us):
    print("=" * 40)
    print("当前识别字符地图：")
    for row in char_matrix:
        print("".join(row))
    print("CLK_FPS %.2f LOOP_FPS %.2f LOOP %dus EXP %dus" %
          (fps, loop_fps, loop_us, exposure_us))
    print("TIME CAP %dus RECT %dus REC %dus DISP %dus" %
          (snapshot_us, rectify_us, recognize_us, display_us))


def init_camera():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA if USE_VGA else sensor.QVGA)
    sensor.skip_frames(time=1000)

    if CAMERA_MANUAL_EXPOSURE:
        try:
            sensor.set_auto_exposure(False, exposure_us=CAMERA_EXPOSURE_US)
        except Exception as exc:
            print("set_auto_exposure skipped:", repr(exc))

    if CAMERA_LOCK_GAIN:
        try:
            sensor.set_auto_gain(False)
        except Exception as exc:
            print("set_auto_gain skipped:", repr(exc))

    if CAMERA_LOCK_WHITEBAL:
        try:
            sensor.set_auto_whitebal(False)
        except Exception as exc:
            print("set_auto_whitebal skipped:", repr(exc))

    sensor.skip_frames(time=500)


def main():
    init_camera()
    # 两套点阵均只在启动时生成一次：原图投影点阵与拉正后的规则点阵。
    grid_points = build_grid_points()
    rectified_grid_points = build_rectified_grid_points(IMG_WIDTH, IMG_HEIGHT)
    quad_corners = build_quad_corner_list() if USE_QUAD_CALIBRATION else None
    element_matrix = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
    char_matrix = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]

    clock = time.clock()
    last_print_ms = time.ticks_ms()

    print("OPENART_GRID_RECOGNIZER_READY")
    print("FRAME_MODE=%s" % ("VGA" if USE_VGA else "QVGA"))
    print("GRID_ROWS=%d GRID_COLS=%d" % (GRID_ROWS, GRID_COLS))
    print("MAP_ROI=%s" % str(MAP_ROI))
    print("DOT_START_X=%d DOT_START_Y=%d COL_STEP=%d ROW_STEP=%d OFFSET=(%d,%d)" %
          (DOT_START_X, DOT_START_Y, COL_STEP, ROW_STEP, SAMPLE_OFFSET_X, SAMPLE_OFFSET_Y))
    print("USE_QUAD_CALIBRATION=%s MAP_CORNERS=%s" %
          (str(USE_QUAD_CALIBRATION), str(MAP_CORNERS)))
    print("SHOW_RECTIFIED_VIEW=%s USE_RECTIFIED_RECOGNITION=%s" %
          (str(SHOW_RECTIFIED_VIEW), str(USE_RECTIFIED_RECOGNITION)))
    print("DEBUG_DRAW_ROI=%s DEBUG_DRAW_GRID_LINES=%s DEBUG_DRAW_POINTS=%s" %
          (str(DEBUG_DRAW_ROI), str(DEBUG_DRAW_GRID_LINES), str(DEBUG_DRAW_POINTS)))
    print("CAMERA_MANUAL_EXPOSURE=%s CAMERA_EXPOSURE_US=%d" %
          (str(CAMERA_MANUAL_EXPOSURE), CAMERA_EXPOSURE_US))
    print("CAMERA_LOCK_GAIN=%s CAMERA_LOCK_WHITEBAL=%s" %
          (str(CAMERA_LOCK_GAIN), str(CAMERA_LOCK_WHITEBAL)))

    # 四角标定关闭时，两个拉正模式都自动失效，保持旧固定网格路径可运行。
    rectified_view_active = SHOW_RECTIFIED_VIEW and USE_QUAD_CALIBRATION
    rectified_recognition_active = USE_RECTIFIED_RECOGNITION and USE_QUAD_CALIBRATION
    rectified_view_failed = False
    rectified_recognition_failed = False
    if (SHOW_RECTIFIED_VIEW or USE_RECTIFIED_RECOGNITION) and not USE_QUAD_CALIBRATION:
        print("RECTIFIED_MODE_NEEDS_QUAD_CALIBRATION")

    while True:
        loop_start_us = time.ticks_us()
        clock.tick()
        img = sensor.snapshot()
        snapshot_done_us = time.ticks_us()
        now_ms = time.ticks_ms()

        # 默认路径：原图 + 投影后的采样点，不对 framebuffer 做整图透视变换。
        recognition_img = img
        recognition_points = grid_points
        display_rectified = False

        if (rectified_recognition_active and not rectified_recognition_failed and
                quad_corners is not None):
            try:
                # 若 IDE 也要看拉正图，直接变换 framebuffer，识别和显示只做一次透视变换。
                # 若 IDE 要看原图，则在副本上拉正，只让识别使用矫正图。
                recognition_img = img if rectified_view_active else img.copy()
                recognition_img.rotation_corr(corners=quad_corners)
                recognition_points = rectified_grid_points
                display_rectified = rectified_view_active
            except Exception as exc:
                print("RECTIFIED_RECOGNITION_FAILED:", repr(exc))
                rectified_recognition_failed = True
                rectified_recognition_active = False
                recognition_img = img
                recognition_points = grid_points

        rectify_done_us = time.ticks_us()
        recognize_map(recognition_img, recognition_points, element_matrix, char_matrix)
        recognize_done_us = time.ticks_us()

        # 识别仍走原图、但 IDE 要看拉正图时，在识别完成后才改变 framebuffer。
        # 因此显示操作不会反过来污染本帧的取色结果。
        if (rectified_view_active and not rectified_view_failed and
                not display_rectified and quad_corners is not None):
            try:
                img.rotation_corr(corners=quad_corners)
                display_rectified = True
            except Exception as exc:
                print("RECTIFIED_VIEW_FAILED:", repr(exc))
                rectified_view_failed = True

        display_points = rectified_grid_points if display_rectified else grid_points

        if DEBUG_ENABLE and DEBUG_DRAW_ROI:
            draw_calibration_boundary(img, rectified=display_rectified)

        if DEBUG_ENABLE and DEBUG_DRAW_GRID_LINES:
            draw_grid_lines(img, rectified=display_rectified)

        if DEBUG_ENABLE and DEBUG_DRAW_POINTS:
            draw_recognition_points(img, element_matrix, display_points)
        display_done_us = time.ticks_us()

        if DEBUG_ENABLE and time.ticks_diff(now_ms, last_print_ms) >= DEBUG_PRINT_PERIOD_MS:
            loop_us = time.ticks_diff(time.ticks_us(), loop_start_us)
            loop_fps = 1000000.0 / loop_us if loop_us > 0 else 0.0
            print_map(
                char_matrix,
                clock.fps(),
                loop_fps,
                loop_us,
                time.ticks_diff(snapshot_done_us, loop_start_us),
                time.ticks_diff(rectify_done_us, snapshot_done_us),
                time.ticks_diff(recognize_done_us, rectify_done_us),
                time.ticks_diff(display_done_us, recognize_done_us),
                sensor.get_exposure_us())
            last_print_ms = now_ms


main()
