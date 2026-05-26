import sensor
import time

# ==================== USER_SWITCHES：现场最常改 ===================
# 调试开关：正式跑帧率时建议 DEBUG_ENABLE=False。
DEBUG_ENABLE = True          # True=打印地图并绘制调试图形；False=正式高速运行
DEBUG_DRAW_ROI = True        # True=画绿色地图边界/拉正边框
DEBUG_DRAW_GRID_LINES = True # True=画 16x12 网格线，用于检查格子是否对齐
DEBUG_DRAW_POINTS = True     # True=画识别结果圆点；False=画面更干净、显示更快
DEBUG_PRINT_PERIOD_MS = 500

# 显示/识别路径：
# 标定看图：SHOW_RECTIFIED_VIEW=True,  USE_RECTIFIED_RECOGNITION=True
# 正式高速：SHOW_RECTIFIED_VIEW=False, USE_RECTIFIED_RECOGNITION=False
# 只看拉正：SHOW_RECTIFIED_VIEW=True,  USE_RECTIFIED_RECOGNITION=False
SHOW_RECTIFIED_VIEW = True       # True=IDE 显示拉正图；False=IDE 显示原图
USE_RECTIFIED_RECOGNITION = True # True=在拉正图上识别；False=在原图投影点上识别

# 稳定输出地图：同一格连续多帧识别为新类型后才切换。
STABILIZE_OUTPUT = True # True=过滤单帧跳动；False=直接输出当前帧识别
STABLE_CHANGE_COUNT = 2

# 相机自动项锁定。固定后画面和 snapshot() 帧周期更稳定。
CAMERA_MANUAL_EXPOSURE = True # True=手动曝光；False=自动曝光
CAMERA_LOCK_GAIN = True       # True=关闭自动增益，亮度更稳定
CAMERA_LOCK_WHITEBAL = True   # True=关闭自动白平衡，颜色比例更稳定


# ==================== CAMERA_AND_FRAME：图像尺寸与曝光 ===================
# False: QVGA 320x240，运行速度更高，当前人工标定坐标按此模式填写。
# True : VGA  640x480，细节更多但帧率更低；启用后必须重新填写 MAP_CORNERS。
USE_VGA = False # True=VGA 640x480 更清晰但更慢；False=QVGA 320x240 更快
FRAME_SCALE = 2 if USE_VGA else 1
IMG_WIDTH = 320 * FRAME_SCALE
IMG_HEIGHT = 240 * FRAME_SCALE

# 仅在 CAMERA_MANUAL_EXPOSURE=True 时生效；短曝光可减少中心区域过曝。
CAMERA_EXPOSURE_US = 500


# ==================== GRID_GEOMETRY：网格与标定 ===================
GRID_COLS = 16
GRID_ROWS = 12

# MAP_CORNERS 表示屏幕地图外边界，用于四角透视标定。
# 四点按 左上、右上、右下、左下 填写，指向完整 16x12 地图外边界。
MAP_CORNERS = (
    (18, 11),
    (307, 18),
    (305, 223),
    (16, 227),
)

# 拉正图上的有效采样区域边距。四角已对齐但整张网格略偏时，只调这里。
GRID_LEFT_MARGIN = 0 * FRAME_SCALE
GRID_RIGHT_MARGIN = 0 * FRAME_SCALE
GRID_TOP_MARGIN = 0 * FRAME_SCALE
GRID_BOTTOM_MARGIN = 0 * FRAME_SCALE

# ==================== COLOR_THRESHOLDS：颜色亮度门槛 ===================
# color_sum 只用于排除极暗噪声，主分类仍看归一化颜色比例。
MIN_SPACE_SUM = 45
MIN_BOX_GOAL_SUM = 60
MIN_PLAYER_SUM = 60
MIN_BOMB_SUM = 60
DARK_PIXEL_THRESHOLD = 60


# ==================== SAMPLING_OFFSETS：多点取样偏移 ===================
PLAYER_SAMPLE_OFFSETS = (
    (0, 0),
    (-5 * FRAME_SCALE, 0),
    (5 * FRAME_SCALE, 0),
    (-7 * FRAME_SCALE, 0),
    (7 * FRAME_SCALE, 0),
    (0, -4 * FRAME_SCALE),
    (0, 4 * FRAME_SCALE),
)

SPACE_CONFIRM_OFFSETS = (
    (0, 0),
    (-5 * FRAME_SCALE, -5 * FRAME_SCALE),
    (5 * FRAME_SCALE, -5 * FRAME_SCALE),
    (-5 * FRAME_SCALE, 5 * FRAME_SCALE),
    (5 * FRAME_SCALE, 5 * FRAME_SCALE),
)

SPECIAL_SAMPLE_OFFSETS = (
    (0, 0),
    (-4 * FRAME_SCALE, 0),
    (4 * FRAME_SCALE, 0),
    (0, -4 * FRAME_SCALE),
    (0, 4 * FRAME_SCALE),
)


# ==================== DISPLAY_MAPS：输出字符与调试绘制颜色 ===================
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


def get_average_pixel(img, x, y, size=2):
    # 在格子中心附近做 size x size 均值采样；默认 2x2，避免混入太多边缘颜色。
    r_sum = 0
    g_sum = 0
    b_sum = 0
    count = 0

    if size < 1:
        size = 1
    start = -(size // 2)
    end = start + size
    width = img.width()
    height = img.height()
    for dy in range(start, end):
        for dx in range(start, end):
            px = x + dx
            py = y + dy
            if 0 <= px < width and 0 <= py < height:
                pixel = img.get_pixel(px, py)
                r_sum += pixel[0]
                g_sum += pixel[1]
                b_sum += pixel[2]
                count += 1

    if count == 0:
        return (0, 0, 0)
    return (r_sum // count, g_sum // count, b_sum // count)


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
    # 原图识别路径所用的采样点：点随四角透视落到各格中心。
    transform = build_quad_transform(MAP_CORNERS)
    if transform is None:
        print("MAP_CORNERS_INVALID")
        return None

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
    left = GRID_LEFT_MARGIN
    top = GRID_TOP_MARGIN
    right = img_width - GRID_RIGHT_MARGIN
    bottom = img_height - GRID_BOTTOM_MARGIN
    width = right - left
    height = bottom - top
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            x = int((left + (col + 0.5) * width / GRID_COLS) + 0.5)
            y = int((top + (row + 0.5) * height / GRID_ROWS) + 0.5)
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

    for idx in range(4):
        start = MAP_CORNERS[idx]
        end = MAP_CORNERS[(idx + 1) % 4]
        img.draw_line(
            (int(start[0]), int(start[1]), int(end[0]), int(end[1])),
            color=(0, 255, 0), thickness=2)


def draw_grid_lines(img, rectified=False):
    # 网格线跟随当前显示坐标系：拉正图画规则网格，原图画投影后的四边形网格。
    color = (255, 255, 255)

    if rectified:
        width = img.width()
        height = img.height()
        left = GRID_LEFT_MARGIN
        top = GRID_TOP_MARGIN
        right = width - GRID_RIGHT_MARGIN
        bottom = height - GRID_BOTTOM_MARGIN
        grid_width = right - left
        grid_height = bottom - top
        if grid_width <= 0 or grid_height <= 0:
            return
        for col in range(GRID_COLS + 1):
            x = int((left + col * grid_width / GRID_COLS) + 0.5)
            if x < 0:
                x = 0
            if x >= width:
                x = width - 1
            y0 = top
            y1 = bottom - 1
            if y0 < 0:
                y0 = 0
            if y1 >= height:
                y1 = height - 1
            img.draw_line((x, y0, x, y1), color=color, thickness=1)
        for row in range(GRID_ROWS + 1):
            y = int((top + row * grid_height / GRID_ROWS) + 0.5)
            if y < 0:
                y = 0
            if y >= height:
                y = height - 1
            x0 = left
            x1 = right - 1
            if x0 < 0:
                x0 = 0
            if x1 >= width:
                x1 = width - 1
            img.draw_line((x0, y, x1, y), color=color, thickness=1)
        return

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


def is_space_color(rn, gn, bn, color_sum):
    return (
        color_sum > MIN_SPACE_SUM and
        bn > 115 and
        bn > rn + 45 and
        bn > gn + 30
    )


def is_goal_color(rn, gn, bn, color_sum):
    return (
        color_sum > MIN_BOX_GOAL_SUM and
        rn > 75 and
        bn > 95 and
        gn < 55 and
        rn > gn + 45 and
        bn > gn + 60 and
        abs(rn - bn) < 85
    )


def is_bomb_color(rn, gn, bn, color_sum):
    return (
        color_sum > MIN_BOMB_SUM and
        rn > 125 and
        gn < 75 and
        bn < 95 and
        rn > gn + 55 and
        rn > bn + 45
    )


def is_box_color(rn, gn, bn, color_sum):
    return (
        color_sum > MIN_BOX_GOAL_SUM and
        rn > 90 and
        gn > 95 and
        bn < 45 and
        abs(rn - gn) < 40
    )


def is_box_candidate(rn, gn, bn, color_sum):
    return (
        color_sum > MIN_BOX_GOAL_SUM and
        rn > 75 and
        gn > 75 and
        bn < 75 and
        rn > bn + 35 and
        gn > bn + 40 and
        abs(rn - gn) < 70
    )


def is_goal_candidate(rn, gn, bn, color_sum):
    return (
        color_sum > MIN_BOX_GOAL_SUM and
        rn > 55 and
        bn > 85 and
        gn < 80 and
        rn > gn + 25 and
        bn > gn + 35
    )


def is_bomb_candidate(rn, gn, bn, color_sum):
    return (
        color_sum > MIN_BOMB_SUM and
        rn > 95 and
        gn < 100 and
        bn < 120 and
        rn > gn + 30 and
        rn > bn + 25
    )


def is_player_color(rn, gn, bn, color_sum):
    # 小车格子由亮绿色和青色两半组成。参考图中实测主色约为：
    # 绿色半边 RGB=(36,255,42)，青色半边 RGB=(36,255,255)。
    # 这里用归一化比例判断，避免曝光变化导致 RGB 绝对值整体变亮或变暗。
    if color_sum < MIN_PLAYER_SUM:
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


def is_player_candidate(rn, gn, bn, color_sum):
    # 只有中心点已经接近小车色时才补采周围点，避免每个蓝地/墙格都多读像素。
    if color_sum < MIN_PLAYER_SUM or rn > 85 or gn < 90:
        return False
    return gn > rn + 40 and (gn >= 125 or bn <= 130)


def sample_player_color(img, x, y, center_r, center_g, center_b):
    # 小车中心可能落在两色交界或模糊边缘；中心和周围点任一点命中即可。
    center_rn, center_gn, center_bn, center_sum = normalize_color(
        center_r, center_g, center_b)
    if is_player_color(center_rn, center_gn, center_bn, center_sum):
        return True
    if not is_player_candidate(center_rn, center_gn, center_bn, center_sum):
        return False

    for dx, dy in PLAYER_SAMPLE_OFFSETS:
        if dx == 0 and dy == 0:
            continue
        r, g, b = get_average_pixel(img, x + dx, y + dy)
        rn, gn, bn, color_sum = normalize_color(r, g, b)
        if is_player_color(rn, gn, bn, color_sum):
            return True
    return False


def sample_special_color(img, x, y, center_r, center_g, center_b, predicate):
    for dx, dy in SPECIAL_SAMPLE_OFFSETS:
        if dx == 0 and dy == 0:
            r, g, b = center_r, center_g, center_b
        else:
            r, g, b = get_average_pixel(img, x + dx, y + dy)
        rn, gn, bn, color_sum = normalize_color(r, g, b)
        if predicate(rn, gn, bn, color_sum):
            return True
    return False


def confirm_space_color(img, x, y, center_r, center_g, center_b):
    # 墙体纹理可能有局部蓝点；空地需要多个分散采样点都呈蓝色。
    center_rn, center_gn, center_bn, center_sum = normalize_color(
        center_r, center_g, center_b)
    if not is_space_color(center_rn, center_gn, center_bn, center_sum):
        return False

    blue_count = 1
    for dx, dy in SPACE_CONFIRM_OFFSETS:
        if dx == 0 and dy == 0:
            continue
        else:
            r, g, b = get_average_pixel(img, x + dx, y + dy)
        rn, gn, bn, color_sum = normalize_color(r, g, b)
        if is_space_color(rn, gn, bn, color_sum):
            blue_count += 1
    return blue_count >= 3


def classify_element(img, row_idx, col_idx, x, y):
    # 逐飞地图外圈固定为墙，强制处理可减少外圈纹理和边缘透视带来的误判。
    if (row_idx == 0 or row_idx == GRID_ROWS - 1 or
            col_idx == 0 or col_idx == GRID_COLS - 1):
        return "wall"

    r, g, b = get_average_pixel(img, x, y)

    # 若中心位置被黑色调试点或阴影覆盖，只尝试偏移重采样，不在这里定类。
    if (r < DARK_PIXEL_THRESHOLD and g < DARK_PIXEL_THRESHOLD and
            b < DARK_PIXEL_THRESHOLD):
        rr, gg, bb = get_average_pixel(img, x + 3, y + 3)
        if rr + gg + bb > r + g + b:
            r, g, b = rr, gg, bb

    if sample_player_color(img, x, y, r, g, b):
        return "player"

    rn, gn, bn, color_sum = normalize_color(r, g, b)

    if is_box_color(rn, gn, bn, color_sum):
        return "box"
    if (is_box_candidate(rn, gn, bn, color_sum) and
            sample_special_color(img, x, y, r, g, b, is_box_color)):
        return "box"

    # 目标点为品红色：G 明显低，暗角/边缘允许 B 比 R 偏高。
    if is_goal_color(rn, gn, bn, color_sum):
        return "goal"
    if (is_goal_candidate(rn, gn, bn, color_sum) and
            sample_special_color(img, x, y, r, g, b, is_goal_color)):
        return "goal"

    # 炸弹为红色：R 比例明显高，G/B 都低，避免把黄色箱子或品红目标误判为炸弹。
    if is_bomb_color(rn, gn, bn, color_sum):
        return "bomb"
    if (is_bomb_candidate(rn, gn, bn, color_sum) and
            sample_special_color(img, x, y, r, g, b, is_bomb_color)):
        return "bomb"

    # 蓝色空地要求 B 通道比例明显占优，并通过分散采样确认不是墙体局部蓝纹理。
    if is_space_color(rn, gn, bn, color_sum) and confirm_space_color(img, x, y, r, g, b):
        return "space"

    return "wall"


def recognize_map(img, grid_points, raw_element_matrix):
    for idx, (x, y) in enumerate(grid_points):
        row_idx = idx // GRID_COLS
        col_idx = idx % GRID_COLS
        raw_element_matrix[row_idx][col_idx] = classify_element(
            img, row_idx, col_idx, x, y)


def update_stable_map(raw_element_matrix, stable_element_matrix,
                      pending_element_matrix, pending_count_matrix, char_matrix):
    for row_idx in range(GRID_ROWS):
        for col_idx in range(GRID_COLS):
            raw_element = raw_element_matrix[row_idx][col_idx]
            stable_element = stable_element_matrix[row_idx][col_idx]

            if (not STABILIZE_OUTPUT or STABLE_CHANGE_COUNT <= 1 or
                    stable_element == ""):
                stable_element = raw_element
                pending_element_matrix[row_idx][col_idx] = ""
                pending_count_matrix[row_idx][col_idx] = 0
            elif raw_element == stable_element:
                pending_element_matrix[row_idx][col_idx] = ""
                pending_count_matrix[row_idx][col_idx] = 0
            else:
                pending_element = pending_element_matrix[row_idx][col_idx]
                if raw_element == pending_element:
                    pending_count_matrix[row_idx][col_idx] += 1
                else:
                    pending_element_matrix[row_idx][col_idx] = raw_element
                    pending_count_matrix[row_idx][col_idx] = 1

                if pending_count_matrix[row_idx][col_idx] >= STABLE_CHANGE_COUNT:
                    stable_element = raw_element
                    pending_element_matrix[row_idx][col_idx] = ""
                    pending_count_matrix[row_idx][col_idx] = 0

            stable_element_matrix[row_idx][col_idx] = stable_element
            char_matrix[row_idx][col_idx] = ELEMENT_CHAR[stable_element]


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
    if grid_points is None:
        return
    rectified_grid_points = build_rectified_grid_points(IMG_WIDTH, IMG_HEIGHT)
    quad_corners = build_quad_corner_list()
    raw_element_matrix = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
    element_matrix = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
    pending_element_matrix = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
    pending_count_matrix = [[0 for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
    char_matrix = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]

    clock = time.clock()
    last_print_ms = time.ticks_ms()

    print("OPENART_GRID_RECOGNIZER_READY")
    print("FRAME_MODE=%s" % ("VGA" if USE_VGA else "QVGA"))
    print("GRID_ROWS=%d GRID_COLS=%d" % (GRID_ROWS, GRID_COLS))
    print("MAP_CORNERS=%s" % str(MAP_CORNERS))
    print("SHOW_RECTIFIED_VIEW=%s USE_RECTIFIED_RECOGNITION=%s" %
          (str(SHOW_RECTIFIED_VIEW), str(USE_RECTIFIED_RECOGNITION)))
    print("GRID_MARGIN=(%d,%d,%d,%d) STABILIZE_OUTPUT=%s STABLE_CHANGE_COUNT=%d" %
          (GRID_LEFT_MARGIN, GRID_RIGHT_MARGIN, GRID_TOP_MARGIN, GRID_BOTTOM_MARGIN,
           str(STABILIZE_OUTPUT), STABLE_CHANGE_COUNT))
    print("DEBUG_DRAW_ROI=%s DEBUG_DRAW_GRID_LINES=%s DEBUG_DRAW_POINTS=%s" %
          (str(DEBUG_DRAW_ROI), str(DEBUG_DRAW_GRID_LINES), str(DEBUG_DRAW_POINTS)))
    print("CAMERA_MANUAL_EXPOSURE=%s CAMERA_EXPOSURE_US=%d" %
          (str(CAMERA_MANUAL_EXPOSURE), CAMERA_EXPOSURE_US))
    print("CAMERA_LOCK_GAIN=%s CAMERA_LOCK_WHITEBAL=%s" %
          (str(CAMERA_LOCK_GAIN), str(CAMERA_LOCK_WHITEBAL)))

    rectified_view_active = SHOW_RECTIFIED_VIEW
    rectified_recognition_active = USE_RECTIFIED_RECOGNITION
    rectified_view_failed = False
    rectified_recognition_failed = False

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

        if rectified_recognition_active and not rectified_recognition_failed:
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
        recognize_map(recognition_img, recognition_points, raw_element_matrix)
        update_stable_map(
            raw_element_matrix, element_matrix,
            pending_element_matrix, pending_count_matrix, char_matrix)
        recognize_done_us = time.ticks_us()

        # 识别仍走原图、但 IDE 要看拉正图时，在识别完成后才改变 framebuffer。
        # 因此显示操作不会反过来污染本帧的取色结果。
        if (rectified_view_active and not rectified_view_failed and
                not display_rectified):
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
