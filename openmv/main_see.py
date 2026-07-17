import sensor
import time
import math
from machine import UART

# ==================== UART 地图发送配置 ===================
UART_MAP_SEND_ENABLE = True  # True=开启 UART 地图发送；False=关闭
UART_MAP_SEND_INDEX = 12     # UART 编号，OpenART Plus 用户串口使用 UART(12)
UART_MAP_SEND_BAUD = 115200  # 波特率，与 RT1064 接收端一致
UART_MAP_SEND_PERIOD_MS = 200  # 发送周期，单位 ms
UART_MAP_RX_ENABLE = True    # True=解析 MCU 命令，同时丢弃无需处理的 MAP_OK 回包
CENTER_SAMPLE_COUNT = 5
OBSERVATION_SAMPLE_COUNT = 3
UART_RX_LINE_MAX = 48

# ==================== USER_SWITCHES：现场最常改 ===================
MODE_RUN = 0
MODE_DEBUG = 1
WORK_MODE = MODE_RUN

if WORK_MODE == MODE_RUN:
    DEBUG_ENABLE = False
    DEBUG_DRAW_ROI = False
    DEBUG_DRAW_GRID_LINES = False
    DEBUG_DRAW_POINTS = False
    DEBUG_PLAYER_CENTER_ENABLE = False
    DEBUG_PLAYER_HEADING_ENABLE = False
    DEBUG_OBSERVATION_ENABLE = False
    SHOW_RECTIFIED_VIEW = True
    USE_RECTIFIED_RECOGNITION = True
else:
    DEBUG_ENABLE = True
    DEBUG_DRAW_ROI = True
    DEBUG_DRAW_GRID_LINES = False
    DEBUG_DRAW_POINTS = False
    DEBUG_PLAYER_CENTER_ENABLE = False
    DEBUG_PLAYER_HEADING_ENABLE = True
    DEBUG_OBSERVATION_ENABLE = False
    SHOW_RECTIFIED_VIEW = True
    USE_RECTIFIED_RECOGNITION = True

DEBUG_PRINT_PERIOD_MS = 1000

# 稳定输出地图：同一格连续多帧识别为新类型后才切换。
STABILIZE_OUTPUT = True # True=过滤单帧跳动；False=直接输出当前帧识别
STABLE_CHANGE_COUNT = 3

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
    (13, 11),
    (310,18),
    (305,220),
    (17, 231),
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
MIN_PLAYER_SUM = 45
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

PLAYER_CENTER_SEARCH_RADIUS = 14 * FRAME_SCALE
PLAYER_RECENT_C_FALLBACK_FRAMES = 3
PLAYER_CENTER_LOST_FRAME_LIMIT = 3
PLAYER_BLOB_PIXELS_THRESHOLD = 6
PLAYER_BLOB_AREA_THRESHOLD = 6
PLAYER_BLOB_MARGIN = 2
PLAYER_PRECISE_MIN_COLOR_PIXELS = 12  # 绿色、青色各自至少命中的像素数
PLAYER_PRECISE_TRIM_PERCENT = 10      # 外框两侧各忽略10%离群颜色像素
PLAYER_HEADING_CORE_RADIUS = 3 * FRAME_SCALE  # 绿/青主体中心的亚像素求均值半径
PLAYER_HEADING_BOUNDARY_MAX_GAP = 2 * FRAME_SCALE  # 允许分界处有少量过渡像素
PLAYER_HEADING_BOUNDARY_NORMAL_BAND = 3 * FRAME_SCALE  # 分界须靠近两色中心中面
PLAYER_HEADING_BOUNDARY_MIN_POINTS = 6
PLAYER_HEADING_BOUNDARY_MIN_AXIS_RATIO = 4.0
PLAYER_HEADING_MIN_VECTOR_Q = 20      # 绿青质心至少相距0.20格
PLAYER_HEADING_MAX_VECTOR_Q = 80      # 超过0.80格视为颜色误配
PLAYER_HEADING_SAMPLE_COUNT = 5
PLAYER_HEADING_MIN_VALID_SAMPLES = 4
PLAYER_HEADING_OUTLIER_MAX_DEG = 5.0

# OpenMV find_blobs() 使用 LAB 阈值。这里先给一组偏宽的初值，
# 再用 blob 中心的 RGB 归一化颜色做二次确认，现场还可以继续微调。
PLAYER_GREEN_BLOB_THRESHOLD = (20, 100, -70, -6, -5, 90)
PLAYER_CYAN_BLOB_THRESHOLD = (20, 100, -70, -6, -128, 15)
BOX_CELL_MIN_HITS = 2
BOX_CELL_MAX_MEAN_OFFSET = 8
BOX_CENTER_MIN_COLOR_PIXELS = 12
BOX_YELLOW_BLOB_THRESHOLD = (40, 100, -50, 25, 20, 127)
BOX_BLOB_AREA_THRESHOLD = 12
BOX_BLOB_MARGIN = 2
BOX_REQUEST_MATCH_MAX_OFFSET_Q = 80
BOX_SAMPLE_OFFSETS = (
    (-8, -8), (-4, -8), (0, -8), (4, -8), (8, -8),
    (-8, -4), (-4, -4), (0, -4), (4, -4), (8, -4),
    (-8, 0),  (-4, 0),  (0, 0),  (4, 0),  (8, 0),
    (-8, 4),  (-4, 4),  (0, 4),  (4, 4),  (8, 4),
    (-8, 8),  (-4, 8),  (0, 8),  (4, 8),  (8, 8),
)

LAUNCH_PLAYER_WINDOW_ENABLE = True
LAUNCH_PLAYER_ROW_MIN = 5
LAUNCH_PLAYER_ROW_MAX = 6
LAUNCH_PLAYER_COL_MIN = 0
LAUNCH_PLAYER_COL_MAX = 1

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
        bn > 135 and            # 提高蓝色门槛，减少墙面误判
        bn > rn + 55 and        # 蓝色比红色差距加大
        bn > gn + 40            # 蓝色比绿色差距加大
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


def is_player_green_half(rn, gn, bn, color_sum):
    if color_sum < MIN_PLAYER_SUM:
        return False
    return (
        rn <= 70 and
        gn >= 145 and
        bn <= 110 and
        gn > rn + 70 and
        gn > bn + 45
    )


def is_player_cyan_half(rn, gn, bn, color_sum):
    if color_sum < MIN_PLAYER_SUM:
        return False
    return (
        rn <= 70 and
        80 <= gn <= 185 and
        80 <= bn <= 190 and
        gn > rn + 35 and
        bn > rn + 35 and
        abs(gn - bn) <= 70
    )


def is_player_mixed_half(rn, gn, bn, color_sum):
    if color_sum < MIN_PLAYER_SUM:
        return False
    # 真实摄像头可能把绿/青边界采成过渡色，保留一个窄的混合区间。
    return (
        rn <= 70 and
        gn >= 130 and
        60 <= bn <= 145 and
        gn > rn + 70 and
        gn > bn + 20
    )


def is_player_color(rn, gn, bn, color_sum):
    # 小车格子由亮绿色和青色两半组成。参考图中实测主色约为：
    # 绿色半边 RGB=(36,255,42)，青色半边 RGB=(36,255,255)。
    # 这里用归一化比例判断，避免曝光变化导致 RGB 绝对值整体变亮或变暗。
    return (is_player_green_half(rn, gn, bn, color_sum) or
            is_player_cyan_half(rn, gn, bn, color_sum) or
            is_player_mixed_half(rn, gn, bn, color_sum))


def is_player_candidate(rn, gn, bn, color_sum):
    # 只有中心点已经接近小车色时才补采周围点，避免每个蓝地/墙格都多读像素。
    if color_sum < MIN_PLAYER_SUM or rn > 95 or gn < 80:
        return False
    return gn > rn + 30 and (gn >= 110 or bn <= 145)


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


def sample_player_half_color(img, x, y, center_r, center_g, center_b, predicate):
    center_rn, center_gn, center_bn, center_sum = normalize_color(
        center_r, center_g, center_b)
    if predicate(center_rn, center_gn, center_bn, center_sum):
        return True
    for dx, dy in PLAYER_SAMPLE_OFFSETS:
        if dx == 0 and dy == 0:
            continue
        r, g, b = get_average_pixel(img, x + dx, y + dy)
        rn, gn, bn, color_sum = normalize_color(r, g, b)
        if predicate(rn, gn, bn, color_sum):
            return True
    return False


def find_player_coarse_center(recognition_points, preferred_matrix, fallback_matrix):
    total_x = 0
    total_y = 0
    count = 0

    for matrix in (preferred_matrix, fallback_matrix):
        if matrix is None:
            continue

        for row_idx in range(GRID_ROWS):
            for col_idx in range(GRID_COLS):
                if matrix[row_idx][col_idx] != "player":
                    continue

                idx = row_idx * GRID_COLS + col_idx
                x, y = recognition_points[idx]
                total_x += x
                total_y += y
                count += 1
        if count > 0:
            break

    if count == 0:
        return None
    return (total_x // count, total_y // count)


def select_recent_player_anchor(current_center, recent_center, missing_frames):
    if current_center is not None:
        return current_center
    if missing_frames >= PLAYER_RECENT_C_FALLBACK_FRAMES:
        return recent_center
    return None


def blob_center_is_player(blob, img, predicate):
    blob_center = get_average_pixel(img, blob.cx(), blob.cy(), size=2)
    rn, gn, bn, color_sum = normalize_color(
        blob_center[0], blob_center[1], blob_center[2])
    return predicate(rn, gn, bn, color_sum)


def detect_player_center(img, recognition_points, preferred_matrix, fallback_matrix,
                         previous_center=None):
    using_previous_center = False
    coarse_center = find_player_coarse_center(
        recognition_points, preferred_matrix, fallback_matrix)
    if coarse_center is None:
        coarse_center = previous_center
        using_previous_center = True
    if coarse_center is None:
        return None

    coarse_x, coarse_y = coarse_center
    cell_half_w = max(PLAYER_CENTER_SEARCH_RADIUS, img.width() // GRID_COLS)
    cell_half_h = max(PLAYER_CENTER_SEARCH_RADIUS, img.height() // GRID_ROWS)
    if using_previous_center:
        cell_half_w = max(cell_half_w, (img.width() * 3) // (GRID_COLS * 2))
        cell_half_h = max(cell_half_h, (img.height() * 3) // (GRID_ROWS * 2))
    left = coarse_x - cell_half_w
    top = coarse_y - cell_half_h
    right = coarse_x + cell_half_w
    bottom = coarse_y + cell_half_h

    if left < 0:
        left = 0
    if top < 0:
        top = 0
    width = img.width()
    height = img.height()
    if right > width:
        right = width
    if bottom > height:
        bottom = height
    if right <= left or bottom <= top:
        return coarse_center

    roi = (left, top, right - left, bottom - top)

    green_blobs = img.find_blobs(
        [PLAYER_GREEN_BLOB_THRESHOLD],
        roi=roi,
        pixels_threshold=PLAYER_BLOB_PIXELS_THRESHOLD,
        area_threshold=PLAYER_BLOB_AREA_THRESHOLD,
        merge=True,
        margin=PLAYER_BLOB_MARGIN)
    cyan_blobs = img.find_blobs(
        [PLAYER_CYAN_BLOB_THRESHOLD],
        roi=roi,
        pixels_threshold=PLAYER_BLOB_PIXELS_THRESHOLD,
        area_threshold=PLAYER_BLOB_AREA_THRESHOLD,
        merge=True,
        margin=PLAYER_BLOB_MARGIN)

    green_blob = None
    green_blob_pixels = 0
    for blob in green_blobs:
        if blob.pixels() > green_blob_pixels and blob_center_is_player(blob, img, is_player_green_half):
            green_blob = blob
            green_blob_pixels = blob.pixels()

    cyan_blob = None
    cyan_blob_pixels = 0
    for blob in cyan_blobs:
        if blob.pixels() > cyan_blob_pixels and blob_center_is_player(blob, img, is_player_cyan_half):
            cyan_blob = blob
            cyan_blob_pixels = blob.pixels()

    if green_blob is None or cyan_blob is None:
        return None

    green_center_x = green_blob.cx()
    green_center_y = green_blob.cy()
    cyan_center_x = cyan_blob.cx()
    cyan_center_y = cyan_blob.cy()

    return ((green_center_x + cyan_center_x) // 2,
            (green_center_y + cyan_center_y) // 2)


def trimmed_histogram_bounds(histogram, total_count, trim_percent):
    trim_count = total_count * trim_percent // 100
    accumulated = 0
    left_index = 0
    right_index = len(histogram) - 1

    for index, count in enumerate(histogram):
        accumulated += count
        if accumulated > trim_count:
            left_index = index
            break

    accumulated = 0
    for index in range(len(histogram) - 1, -1, -1):
        accumulated += histogram[index]
        if accumulated > trim_count:
            right_index = index
            break
    return (left_index, right_index)


def histogram_core_mean_coordinate(histogram, total_count, origin, radius):
    lower_rank = (total_count - 1) // 2
    upper_rank = total_count // 2
    accumulated = 0
    lower_index = 0
    upper_index = 0
    lower_found = False

    for index, count in enumerate(histogram):
        accumulated += count
        if accumulated > lower_rank and not lower_found:
            lower_index = index
            lower_found = True
        if accumulated > upper_rank:
            upper_index = index
            break

    center = (lower_index + upper_index) / 2.0
    weighted_sum = 0
    core_count = 0

    for index, count in enumerate(histogram):
        if abs(index - center) > radius:
            continue
        weighted_sum += (origin + index) * count
        core_count += count
    return weighted_sum / core_count


def player_heading_centers_from_boundary(labels, width, height,
                                         left, top,
                                         green_center, cyan_center):
    rough_dx = cyan_center[0] - green_center[0]
    rough_dy = cyan_center[1] - green_center[1]
    distance = math.sqrt(rough_dx * rough_dx + rough_dy * rough_dy)
    if distance < 0.001:
        return None
    center_x = (green_center[0] + cyan_center[0]) / 2.0
    center_y = (green_center[1] + cyan_center[1]) / 2.0
    rough_unit_x = rough_dx / distance
    rough_unit_y = rough_dy / distance
    point_count = 0
    x_sum = 0.0
    y_sum = 0.0
    xx_sum = 0.0
    yy_sum = 0.0
    xy_sum = 0.0

    for y in range(height):
        for x in range(width):
            label = labels[y * width + x]
            if label == 0:
                continue
            for step_x, step_y in ((1, 0), (0, 1)):
                for gap in range(1, PLAYER_HEADING_BOUNDARY_MAX_GAP + 1):
                    other_x = x + step_x * gap
                    other_y = y + step_y * gap
                    if other_x >= width or other_y >= height:
                        break
                    other = labels[other_y * width + other_x]
                    if other == 0:
                        continue
                    if other == label:
                        break

                    point_x = left + (x + other_x) / 2.0
                    point_y = top + (y + other_y) / 2.0
                    normal_offset = abs(
                        (point_x - center_x) * rough_unit_x +
                        (point_y - center_y) * rough_unit_y)
                    if normal_offset > PLAYER_HEADING_BOUNDARY_NORMAL_BAND:
                        break
                    point_count += 1
                    x_sum += point_x
                    y_sum += point_y
                    xx_sum += point_x * point_x
                    yy_sum += point_y * point_y
                    xy_sum += point_x * point_y
                    break

    if point_count < PLAYER_HEADING_BOUNDARY_MIN_POINTS:
        return None

    mean_x = x_sum / point_count
    mean_y = y_sum / point_count
    covariance_xx = xx_sum / point_count - mean_x * mean_x
    covariance_yy = yy_sum / point_count - mean_y * mean_y
    covariance_xy = xy_sum / point_count - mean_x * mean_y
    discriminant = math.sqrt(
        (covariance_xx - covariance_yy) *
        (covariance_xx - covariance_yy) +
        4.0 * covariance_xy * covariance_xy)
    major_variance = (covariance_xx + covariance_yy + discriminant) / 2.0
    minor_variance = (covariance_xx + covariance_yy - discriminant) / 2.0
    if minor_variance < 0.0:
        minor_variance = 0.0
    if (major_variance < 1.0 or
            major_variance < ((minor_variance + 0.01) *
                              PLAYER_HEADING_BOUNDARY_MIN_AXIS_RATIO)):
        return None

    boundary_axis = math.atan2(
        2.0 * covariance_xy,
        covariance_xx - covariance_yy) / 2.0
    normal_x = math.sin(boundary_axis)
    normal_y = -math.cos(boundary_axis)
    if normal_x * rough_dx + normal_y * rough_dy < 0.0:
        normal_x = -normal_x
        normal_y = -normal_y

    half_distance = distance / 2.0
    return ((center_x - normal_x * half_distance,
             center_y - normal_y * half_distance),
            (center_x + normal_x * half_distance,
             center_y + normal_y * half_distance))


def detect_player_pose_precise(img, anchor_center):
    if anchor_center is None:
        return None

    half_w = max(PLAYER_CENTER_SEARCH_RADIUS,
                 (img.width() * 9) // (GRID_COLS * 10))
    half_h = max(PLAYER_CENTER_SEARCH_RADIUS,
                 (img.height() * 9) // (GRID_ROWS * 10))
    left = max(0, anchor_center[0] - half_w)
    top = max(0, anchor_center[1] - half_h)
    right = min(img.width(), anchor_center[0] + half_w + 1)
    bottom = min(img.height(), anchor_center[1] + half_h + 1)
    if right <= left or bottom <= top:
        return None

    x_histogram = [0 for _ in range(right - left)]
    y_histogram = [0 for _ in range(bottom - top)]
    green_x_histogram = [0 for _ in range(right - left)]
    green_y_histogram = [0 for _ in range(bottom - top)]
    cyan_x_histogram = [0 for _ in range(right - left)]
    cyan_y_histogram = [0 for _ in range(bottom - top)]
    color_labels = bytearray((right - left) * (bottom - top))
    green_count = 0
    cyan_count = 0
    total_count = 0

    for y in range(top, bottom):
        for x in range(left, right):
            pixel = img.get_pixel(x, y)
            rn, gn, bn, color_sum = normalize_color(pixel[0], pixel[1], pixel[2])
            is_green = is_player_green_half(rn, gn, bn, color_sum)
            is_cyan = is_player_cyan_half(rn, gn, bn, color_sum)
            is_player_pixel = is_green or is_cyan
            if is_green and is_cyan:
                # 重叠区按蓝色占比归到唯一一侧，避免同一像素同时拉动两个色心。
                is_green = (gn - bn) > 58
                is_cyan = not is_green
            label_index = (y - top) * (right - left) + (x - left)
            if is_green:
                color_labels[label_index] = 1
                green_count += 1
                green_x_histogram[x - left] += 1
                green_y_histogram[y - top] += 1
            if is_cyan:
                color_labels[label_index] = 2
                cyan_count += 1
                cyan_x_histogram[x - left] += 1
                cyan_y_histogram[y - top] += 1
            if is_player_pixel:
                x_histogram[x - left] += 1
                y_histogram[y - top] += 1
                total_count += 1

    if (green_count < PLAYER_PRECISE_MIN_COLOR_PIXELS or
            cyan_count < PLAYER_PRECISE_MIN_COLOR_PIXELS):
        return None

    x_min, x_max = trimmed_histogram_bounds(
        x_histogram, total_count, PLAYER_PRECISE_TRIM_PERCENT)
    y_min, y_max = trimmed_histogram_bounds(
        y_histogram, total_count, PLAYER_PRECISE_TRIM_PERCENT)
    player_center = ((left + x_min + left + x_max) // 2,
                     (top + y_min + top + y_max) // 2)
    # yaw 的绿/青基线很短。先用中位数锁定各自主体，再对中心核心求均值：
    # 既排除蓝底高亮误点，又避免纯中位数的0.5像素角度量化。
    green_center = (
        histogram_core_mean_coordinate(
            green_x_histogram, green_count, left,
            PLAYER_HEADING_CORE_RADIUS),
        histogram_core_mean_coordinate(
            green_y_histogram, green_count, top,
            PLAYER_HEADING_CORE_RADIUS))
    cyan_center = (
        histogram_core_mean_coordinate(
            cyan_x_histogram, cyan_count, left,
            PLAYER_HEADING_CORE_RADIUS),
        histogram_core_mean_coordinate(
            cyan_y_histogram, cyan_count, top,
            PLAYER_HEADING_CORE_RADIUS))
    heading_centers = player_heading_centers_from_boundary(
        color_labels, right - left, bottom - top, left, top,
        green_center, cyan_center)
    if heading_centers is None:
        green_center = None
        cyan_center = None
    else:
        green_center, cyan_center = heading_centers
    return (player_center, green_center, cyan_center)


def detect_player_center_precise(img, anchor_center):
    pose = detect_player_pose_precise(img, anchor_center)
    return None if pose is None else pose[0]


def box_cell_has_coverage(img, center_x, center_y):
    hits = 0
    hit_dx_sum = 0
    hit_dy_sum = 0
    width = img.width()
    height = img.height()
    for dx, dy in BOX_SAMPLE_OFFSETS:
        x = center_x + dx * FRAME_SCALE
        y = center_y + dy * FRAME_SCALE
        if not (0 <= x < width and 0 <= y < height):
            continue
        r, g, b = img.get_pixel(x, y)
        rn, gn, bn, color_sum = normalize_color(r, g, b)
        if is_box_candidate(rn, gn, bn, color_sum):
            hits += 1
            hit_dx_sum += dx * FRAME_SCALE
            hit_dy_sum += dy * FRAME_SCALE
    max_sum = hits * BOX_CELL_MAX_MEAN_OFFSET * FRAME_SCALE
    return (hits >= BOX_CELL_MIN_HITS and
            abs(hit_dx_sum) < max_sum and abs(hit_dy_sum) < max_sum)


def split_box_blob_centers(blob, cell_w, cell_h):
    # 相邻同色箱子会成为一个大 blob，按单格尺寸拆回独立中心。
    split_cols = max(1, min(GRID_COLS,
                            (blob.w() + cell_w // 2) // cell_w))
    split_rows = max(1, min(GRID_ROWS,
                            (blob.h() + cell_h // 2) // cell_h))
    centers = []
    for split_row in range(split_rows):
        top = blob.y() + blob.h() * split_row // split_rows
        bottom = blob.y() + blob.h() * (split_row + 1) // split_rows
        for split_col in range(split_cols):
            left = blob.x() + blob.w() * split_col // split_cols
            right = blob.x() + blob.w() * (split_col + 1) // split_cols
            centers.append(((left + right) // 2, (top + bottom) // 2))
    return centers


def deduplicate_box_centers(centers, cell_w, cell_h):
    # 网格线可能把同一个跨格箱子切开，合并距离不足 0.6 格的中心。
    max_dx = max(2, cell_w * 3 // 5)
    max_dy = max(2, cell_h * 3 // 5)
    groups = []
    for center_x, center_y in centers:
        matched_group = None
        for group in groups:
            group_x = (group[0] + group[2] // 2) // group[2]
            group_y = (group[1] + group[2] // 2) // group[2]
            if (abs(center_x - group_x) <= max_dx and
                    abs(center_y - group_y) <= max_dy):
                matched_group = group
                break
        if matched_group is None:
            groups.append([center_x, center_y, 1])
        else:
            matched_group[0] += center_x
            matched_group[1] += center_y
            matched_group[2] += 1

    result = []
    for sum_x, sum_y, count in groups:
        result.append(((sum_x + count // 2) // count,
                       (sum_y + count // 2) // count))
    return result


def detect_box_centers(img, recognition_points, element_matrix):
    cell_w = max(6, img.width() // GRID_COLS)
    cell_h = max(6, img.height() // GRID_ROWS)
    coarse_centers = []
    for row_idx in range(GRID_ROWS):
        for col_idx in range(GRID_COLS):
            if element_matrix[row_idx][col_idx] != "box":
                continue
            coarse_centers.append(
                recognition_points[row_idx * GRID_COLS + col_idx])
    if not coarse_centers:
        return []

    left = max(0, min(center[0] for center in coarse_centers) - cell_w)
    top = max(0, min(center[1] for center in coarse_centers) - cell_h)
    right = min(img.width(),
                max(center[0] for center in coarse_centers) + cell_w + 1)
    bottom = min(img.height(),
                 max(center[1] for center in coarse_centers) + cell_h + 1)
    if right <= left or bottom <= top:
        return []

    blobs = img.find_blobs(
        [BOX_YELLOW_BLOB_THRESHOLD],
        roi=(left, top, right - left, bottom - top),
        pixels_threshold=BOX_CENTER_MIN_COLOR_PIXELS,
        area_threshold=BOX_BLOB_AREA_THRESHOLD,
        merge=True,
        margin=BOX_BLOB_MARGIN)
    detected_centers = []
    max_match_dx = max(4, cell_w * 4 // 5)
    max_match_dy = max(4, cell_h * 4 // 5)
    for blob in blobs:
        for detected_x, detected_y in split_box_blob_centers(
                blob, cell_w, cell_h):
            for coarse_x, coarse_y in coarse_centers:
                if (abs(detected_x - coarse_x) <= max_match_dx and
                        abs(detected_y - coarse_y) <= max_match_dy):
                    detected_centers.append((detected_x, detected_y))
                    break
    return deduplicate_box_centers(detected_centers, cell_w, cell_h)


def select_box_center(box_centers, recognition_points, row_idx, col_idx):
    if not (0 <= row_idx < GRID_ROWS and 0 <= col_idx < GRID_COLS):
        return None
    center_x, center_y = recognition_points[
        row_idx * GRID_COLS + col_idx]
    if col_idx + 1 < GRID_COLS:
        cell_w = abs(recognition_points[
            row_idx * GRID_COLS + col_idx + 1][0] - center_x)
    else:
        cell_w = abs(center_x - recognition_points[
            row_idx * GRID_COLS + col_idx - 1][0])
    if row_idx + 1 < GRID_ROWS:
        cell_h = abs(recognition_points[
            (row_idx + 1) * GRID_COLS + col_idx][1] - center_y)
    else:
        cell_h = abs(center_y - recognition_points[
            (row_idx - 1) * GRID_COLS + col_idx][1])
    cell_w = max(6, cell_w)
    cell_h = max(6, cell_h)
    max_dx = max(4, cell_w * 4 // 5)
    max_dy = max(4, cell_h * 4 // 5)
    best_center = None
    best_distance = None
    ambiguous = False
    for detected_x, detected_y in box_centers:
        if (abs(detected_x - center_x) > max_dx or
                abs(detected_y - center_y) > max_dy):
            continue
        distance = ((detected_x - center_x) * (detected_x - center_x) +
                    (detected_y - center_y) * (detected_y - center_y))
        if best_distance is None or distance < best_distance:
            best_center = (detected_x, detected_y)
            best_distance = distance
            ambiguous = False
        elif distance == best_distance:
            ambiguous = True
    return None if ambiguous else best_center


def detect_box_center(img, recognition_points, row_idx, col_idx):
    element_matrix = [["space" for _ in range(GRID_COLS)]
                      for _ in range(GRID_ROWS)]
    if not (0 <= row_idx < GRID_ROWS and 0 <= col_idx < GRID_COLS):
        return None
    element_matrix[row_idx][col_idx] = "box"
    box_centers = detect_box_centers(img, recognition_points, element_matrix)
    return select_box_center(
        box_centers, recognition_points, row_idx, col_idx)


def resolve_requested_box_center(img, recognition_points,
                                 element_matrix, row_idx, col_idx):
    if not (0 <= row_idx < GRID_ROWS and 0 <= col_idx < GRID_COLS):
        return None
    if element_matrix[row_idx][col_idx] != "box":
        return None
    box_centers = detect_box_centers(
        img, recognition_points, element_matrix)
    return select_box_center(
        box_centers, recognition_points, row_idx, col_idx)


def box_center_grid_matches_request(box_center_grid, row_idx, col_idx):
    if box_center_grid is None:
        return False
    col_q, row_q = box_center_grid
    col_offset_q = col_q - (col_idx * 100 + 50)
    row_offset_q = row_q - (row_idx * 100 + 50)
    return (-BOX_REQUEST_MATCH_MAX_OFFSET_Q <= col_offset_q <=
            BOX_REQUEST_MATCH_MAX_OFFSET_Q and
            -BOX_REQUEST_MATCH_MAX_OFFSET_Q <= row_offset_q <=
            BOX_REQUEST_MATCH_MAX_OFFSET_Q)


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
        rhs0 = x - c
        rhs1 = y - f
        determinant = m00 * m11 - m01 * m10
        if -0.000001 < determinant < 0.000001:
            return None
        u = (rhs0 * m11 - m01 * rhs1) / determinant
        v = (m00 * rhs1 - rhs0 * m10) / determinant
        col_q = int(u * GRID_COLS * 100 + 0.5)
        row_q = int(v * GRID_ROWS * 100 + 0.5)
        if col_q < 0 or col_q >= GRID_COLS * 100:
            return None
        if row_q < 0 or row_q >= GRID_ROWS * 100:
            return None
        return (col_q, row_q)

    left = GRID_LEFT_MARGIN
    top = GRID_TOP_MARGIN
    right = IMG_WIDTH - GRID_RIGHT_MARGIN
    bottom = IMG_HEIGHT - GRID_BOTTOM_MARGIN
    width = right - left
    height = bottom - top
    if width <= 0 or height <= 0:
        return None

    x, y = center
    col_q = int(((x - left) * GRID_COLS * 100 / width) + 0.5)
    row_q = int(((y - top) * GRID_ROWS * 100 / height) + 0.5)
    if col_q < 0 or col_q >= GRID_COLS * 100:
        return None
    if row_q < 0 or row_q >= GRID_ROWS * 100:
        return None
    return (col_q, row_q)


def shortest_heading_error_deg(target_deg, current_deg):
    error = target_deg - current_deg
    while error > 180.0:
        error -= 360.0
    while error < -180.0:
        error += 360.0
    return error


def player_heading_to_yaw_deg(green_center, cyan_center,
                              rectified=True, raw_transform=None):
    green_grid = image_center_to_grid_q(
        green_center, rectified, raw_transform)
    cyan_grid = image_center_to_grid_q(
        cyan_center, rectified, raw_transform)
    if green_grid is None or cyan_grid is None:
        return None

    dx_q = cyan_grid[0] - green_grid[0]
    dy_q = cyan_grid[1] - green_grid[1]
    distance_sq = dx_q * dx_q + dy_q * dy_q
    if (distance_sq < PLAYER_HEADING_MIN_VECTOR_Q *
            PLAYER_HEADING_MIN_VECTOR_Q or
            distance_sq > PLAYER_HEADING_MAX_VECTOR_Q *
            PLAYER_HEADING_MAX_VECTOR_Q):
        return None

    # 地图 yaw 约定：下=0°，右=90°，上=180°，左=270°。
    yaw_deg = math.atan2(dx_q, dy_q) * 180.0 / math.pi
    return yaw_deg + 360.0 if yaw_deg < 0.0 else yaw_deg


def filter_player_heading_samples(samples):
    if len(samples) < PLAYER_HEADING_SAMPLE_COUNT:
        return None

    reference = samples[0]
    best_score = None
    for candidate in samples:
        score = 0.0
        for sample in samples:
            score += abs(shortest_heading_error_deg(sample, candidate))
        if best_score is None or score < best_score:
            reference = candidate
            best_score = score

    sin_sum = 0.0
    cos_sum = 0.0
    accepted_count = 0
    for sample in samples:
        if abs(shortest_heading_error_deg(sample, reference)) > \
                PLAYER_HEADING_OUTLIER_MAX_DEG:
            continue
        angle_rad = sample * math.pi / 180.0
        sin_sum += math.sin(angle_rad)
        cos_sum += math.cos(angle_rad)
        accepted_count += 1

    if accepted_count < PLAYER_HEADING_MIN_VALID_SAMPLES:
        return None
    yaw_deg = math.atan2(sin_sum, cos_sum) * 180.0 / math.pi
    if yaw_deg < 0.0:
        yaw_deg += 360.0
    return (yaw_deg, accepted_count)


def resolve_player_center(img, recognition_points,
                          raw_element_matrix, stable_element_matrix,
                          previous_precise, previous_cell,
                          rectified, raw_transform):
    anchor = previous_precise
    if anchor is None and previous_cell is not None:
        row_idx, col_idx = previous_cell
        if (0 <= row_idx < GRID_ROWS and 0 <= col_idx < GRID_COLS):
            anchor = recognition_points[row_idx * GRID_COLS + col_idx]
    if anchor is None:
        anchor = find_player_coarse_center(
            recognition_points, raw_element_matrix, stable_element_matrix)

    blob_center = detect_player_center(
        img, recognition_points,
        raw_element_matrix, stable_element_matrix, anchor)
    precise_anchor = blob_center if blob_center is not None else anchor
    precise_pose = detect_player_pose_precise(img, precise_anchor)
    if precise_pose is None:
        return None
    precise_center, green_center, cyan_center = precise_pose
    grid_q = image_center_to_grid_q(
        precise_center, rectified, raw_transform)
    if grid_q is None:
        return None
    return (precise_center, grid_q,
            (grid_q[1] // 100, grid_q[0] // 100),
            green_center, cyan_center)


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
    r, g, b = get_average_pixel(img, x, y)

    # 若中心位置被黑色调试点或阴影覆盖，只尝试偏移重采样，不在这里定类。
    if (r < DARK_PIXEL_THRESHOLD and g < DARK_PIXEL_THRESHOLD and
            b < DARK_PIXEL_THRESHOLD):
        rr, gg, bb = get_average_pixel(img, x + 3, y + 3)
        if rr + gg + bb > r + g + b:
            r, g, b = rr, gg, bb

    in_launch_player_window = (
        LAUNCH_PLAYER_WINDOW_ENABLE and
        LAUNCH_PLAYER_ROW_MIN <= row_idx <= LAUNCH_PLAYER_ROW_MAX and
        LAUNCH_PLAYER_COL_MIN <= col_idx <= LAUNCH_PLAYER_COL_MAX)
    if in_launch_player_window:
        if sample_player_color(img, x, y, r, g, b):
            return "player"
    else:
        # 逐飞地图外圈固定为墙；左发车窗口按普通格识别。
        if (row_idx == 0 or row_idx == GRID_ROWS - 1 or
                col_idx == 0 or col_idx == GRID_COLS - 1):
            return "wall"
        if sample_player_color(img, x, y, r, g, b):
            return "player"

    rn, gn, bn, color_sum = normalize_color(r, g, b)

    if is_box_color(rn, gn, bn, color_sum):
        return "box"
    if (is_box_candidate(rn, gn, bn, color_sum) and
            sample_special_color(img, x, y, r, g, b, is_box_color)):
        return "box"
    if box_cell_has_coverage(img, x, y):
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


def count_player_cells(element_matrix):
    count = 0
    for row in element_matrix:
        for element in row:
            if element == "player":
                count += 1
    return count


def update_non_player_background(element_matrix, background_matrix,
                                 protected_cell=None):
    for row_idx in range(GRID_ROWS):
        for col_idx in range(GRID_COLS):
            if protected_cell == (row_idx, col_idx):
                continue
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
                if element_matrix[row_idx][col_idx] != "player":
                    continue
                if selected_row >= 0:
                    return None
                selected_row = row_idx
                selected_col = col_idx
        if selected_row < 0:
            return None

    canonical = []
    for _ in range(GRID_ROWS):
        row = []
        for _ in range(GRID_COLS):
            row.append("")
        canonical.append(row)
    for row_idx in range(GRID_ROWS):
        for col_idx in range(GRID_COLS):
            element = element_matrix[row_idx][col_idx]
            background = background_matrix[row_idx][col_idx]
            if (row_idx == selected_row and col_idx == selected_col):
                canonical[row_idx][col_idx] = "+" if background == "goal" else "C"
            elif element == "player":
                if background == "":
                    return None
                canonical[row_idx][col_idx] = ELEMENT_CHAR[background]
            else:
                canonical[row_idx][col_idx] = ELEMENT_CHAR[element]
    return canonical


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
              snapshot_us, rectify_us, recognize_us, display_us, exposure_us,
              player_center=None):
    print("=" * 40)
    print("当前识别字符地图：")
    for row in char_matrix:
        print("".join(row))
    print("CLK_FPS %.2f LOOP_FPS %.2f LOOP %dus EXP %dus" %
          (fps, loop_fps, loop_us, exposure_us))
    print("TIME CAP %dus RECT %dus REC %dus DISP %dus" %
          (snapshot_us, rectify_us, recognize_us, display_us))
    if player_center is not None:
        print("PLAYER_CENTER %d,%d" % (player_center[0], player_center[1]))


def draw_player_heading_debug(img, green_center, cyan_center, yaw_deg):
    if green_center is None or cyan_center is None:
        return
    green_x = int(green_center[0] + 0.5)
    green_y = int(green_center[1] + 0.5)
    cyan_x = int(cyan_center[0] + 0.5)
    cyan_y = int(cyan_center[1] + 0.5)
    dx = cyan_center[0] - green_center[0]
    dy = cyan_center[1] - green_center[1]
    length = math.sqrt(dx * dx + dy * dy)
    if length <= 0.001:
        return

    end_x = int(cyan_center[0] + dx * 0.8 + 0.5)
    end_y = int(cyan_center[1] + dy * 0.8 + 0.5)
    unit_x = dx / length
    unit_y = dy / length
    back_x = end_x - unit_x * 5 * FRAME_SCALE
    back_y = end_y - unit_y * 5 * FRAME_SCALE
    side_x = -unit_y * 3 * FRAME_SCALE
    side_y = unit_x * 3 * FRAME_SCALE

    img.draw_line((green_x, green_y, end_x, end_y),
                  color=(255, 0, 0), thickness=2)
    img.draw_line((end_x, end_y,
                   int(back_x + side_x), int(back_y + side_y)),
                  color=(255, 0, 0), thickness=2)
    img.draw_line((end_x, end_y,
                   int(back_x - side_x), int(back_y - side_y)),
                  color=(255, 0, 0), thickness=2)
    img.draw_cross(green_x, green_y, color=(0, 255, 0), thickness=2)
    img.draw_cross(cyan_x, cyan_y, color=(0, 255, 255), thickness=2)
    if yaw_deg is not None:
        label_x = max(0, min(img.width() - 52, cyan_x + 6 * FRAME_SCALE))
        label_y = max(0, cyan_y - 12 * FRAME_SCALE)
        img.draw_string(label_x, label_y, "Y%.1f" % yaw_deg,
                        color=(255, 255, 0), scale=1)


def init_uart_map():
    if not UART_MAP_SEND_ENABLE:
        print("UART_MAP_SEND_DISABLED")
        return None
    try:
        uart = UART(UART_MAP_SEND_INDEX, baudrate=UART_MAP_SEND_BAUD)
        uart.init(UART_MAP_SEND_BAUD, bits=8, parity=None, stop=1)
        print("UART_MAP_SEND_READY index=%d baud=%d" % (UART_MAP_SEND_INDEX, UART_MAP_SEND_BAUD))
        return uart
    except Exception as exc:
        print("UART_MAP_SEND_INIT_FAILED:", repr(exc))
        return None


center_request_active = False
center_request_sample_count = 0
center_request_generation = 0
observation_request_active = False
observation_request_row = 0
observation_request_col = 0
observation_request_sample_count = 0
observation_request_generation = 0
map_uart_rx_line = ""


def parse_map_uart_line(line):
    global center_request_active
    global center_request_sample_count
    global center_request_generation
    global observation_request_active
    global observation_request_row
    global observation_request_col
    global observation_request_sample_count
    global observation_request_generation

    if line == "CENTER_REQ":
        observation_request_active = False
        observation_request_sample_count = 0
        center_request_active = True
        center_request_sample_count = 0
        center_request_generation += 1
        return
    if line.startswith("OBSERVE_REQ "):
        fields = line[12:].split(",")
        if len(fields) != 2:
            return
        try:
            row_idx = int(fields[0])
            col_idx = int(fields[1])
        except Exception:
            return
        if not (0 <= row_idx < GRID_ROWS and 0 <= col_idx < GRID_COLS):
            observation_request_active = False
            return
        center_request_active = False
        center_request_sample_count = 0
        observation_request_row = row_idx
        observation_request_col = col_idx
        observation_request_sample_count = 0
        observation_request_generation += 1
        observation_request_active = True


def poll_map_uart_rx(uart):
    global map_uart_rx_line

    if not UART_MAP_RX_ENABLE or uart is None:
        return
    try:
        count = uart.any()
        if not count:
            return
        data = uart.read(count)
        if data is None:
            return
        for value in data:
            if value == 10:
                parse_map_uart_line(map_uart_rx_line)
                map_uart_rx_line = ""
            elif value == 13:
                continue
            elif 32 <= value <= 126:
                if len(map_uart_rx_line) < UART_RX_LINE_MAX:
                    map_uart_rx_line += chr(value)
                else:
                    map_uart_rx_line = ""
    except Exception:
        map_uart_rx_line = ""


def process_center_request(uart, player_center_grid,
                           canonical_char_matrix, player_yaw_deg):
    global center_request_active
    global center_request_sample_count

    if (not center_request_active or uart is None or
            canonical_char_matrix is None):
        return False
    if player_center_grid is None:
        return send_map_uart(uart, canonical_char_matrix, None)

    request_generation = center_request_generation
    sample_index = center_request_sample_count + 1
    if not send_map_uart(uart, canonical_char_matrix, player_center_grid):
        return False
    if (not center_request_active or
            center_request_generation != request_generation):
        return True
    yaw_q = 0
    yaw_valid = 0
    if player_yaw_deg is not None:
        yaw_q = int(player_yaw_deg * 100.0 + 0.5) % 36000
        yaw_valid = 1
    try:
        uart.write("CENTER_SAMPLE %d,%d,%d,%d,%d\n" %
                   (sample_index,
                    player_center_grid[0], player_center_grid[1],
                    yaw_q, yaw_valid))
    except Exception:
        return False

    center_request_sample_count = sample_index
    if center_request_sample_count >= CENTER_SAMPLE_COUNT:
        center_request_active = False
    return True


def process_observation_request(uart, canonical_char_matrix,
                                player_center_grid, box_center_grid):
    global observation_request_active
    global observation_request_sample_count

    if (not observation_request_active or uart is None or
            canonical_char_matrix is None or
            player_center_grid is None or box_center_grid is None):
        return False
    request_generation = observation_request_generation
    sample_index = observation_request_sample_count + 1
    if not send_map_uart(uart, canonical_char_matrix, player_center_grid):
        return False
    if (not observation_request_active or
            observation_request_generation != request_generation):
        return True
    try:
        uart.write("OBSERVE_SAMPLE %d,%d,%d,%d,%d\n" %
                   (sample_index,
                    player_center_grid[0], player_center_grid[1],
                    box_center_grid[0], box_center_grid[1]))
    except Exception:
        return False
    observation_request_sample_count = sample_index
    if observation_request_sample_count >= OBSERVATION_SAMPLE_COUNT:
        observation_request_active = False
    return True


def send_map_uart(uart, char_matrix, player_center_grid=None):
    if uart is None:
        return False
    try:
        poll_map_uart_rx(uart)
        uart.write("MAP_BEGIN\n")
        for row in char_matrix:
            line = "".join(row) + "\n"
            uart.write(line)
        if player_center_grid is not None:
            uart.write("PLAYER_CENTER_GRID %d,%d 1\n" %
                       (player_center_grid[0], player_center_grid[1]))
        else:
            uart.write("PLAYER_CENTER_GRID 0,0 0\n")
        uart.write("MAP_END\n")
        poll_map_uart_rx(uart)
        return True
    except Exception as exc:
        print("UART_MAP_SEND_ERROR:", repr(exc))
        return False


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
    map_uart = init_uart_map()
    # 两套点阵均只在启动时生成一次：原图投影点阵与拉正后的规则点阵。
    grid_points = build_grid_points()
    if grid_points is None:
        return
    rectified_grid_points = build_rectified_grid_points(IMG_WIDTH, IMG_HEIGHT)
    quad_corners = build_quad_corner_list()
    raw_grid_transform = build_quad_transform(MAP_CORNERS)
    if raw_grid_transform is None:
        return
    raw_element_matrix = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
    element_matrix = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
    pending_element_matrix = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
    pending_count_matrix = [[0 for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
    char_matrix = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
    player_background_matrix = [["" for _ in range(GRID_COLS)]
                                for _ in range(GRID_ROWS)]

    clock = time.clock()
    last_print_ms = time.ticks_ms()
    last_uart_send_ms = time.ticks_ms()

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
    player_center_anchor = None
    last_precise_player_center = None
    last_precise_player_grid = None
    player_center_lost_frames = 0
    last_player_cell = None
    player_heading_history = []
    player_heading_lost_frames = 0
    player_heading_filtered_deg = None
    player_heading_accepted_count = 0
    last_center_request_generation = center_request_generation
    last_observation_request_generation = observation_request_generation

    while True:
        loop_start_us = time.ticks_us()
        clock.tick()
        poll_map_uart_rx(map_uart)
        request_generation_changed = (
            center_request_generation != last_center_request_generation or
            observation_request_generation !=
            last_observation_request_generation)
        if request_generation_changed:
            player_center_anchor = None
            last_precise_player_center = None
            last_precise_player_grid = None
            player_center_lost_frames = 0
            last_player_cell = None
            player_heading_history = []
            player_heading_lost_frames = 0
            player_heading_filtered_deg = None
            player_heading_accepted_count = 0
            last_center_request_generation = center_request_generation
            last_observation_request_generation = observation_request_generation
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
                player_center_anchor = None
                last_precise_player_center = None
                last_precise_player_grid = None
                player_center_lost_frames = 0
                last_player_cell = None
                player_heading_history = []
                player_heading_lost_frames = 0
                player_heading_filtered_deg = None
                player_heading_accepted_count = 0

        rectify_done_us = time.ticks_us()
        recognize_map(recognition_img, recognition_points, raw_element_matrix)
        update_stable_map(
            raw_element_matrix, element_matrix,
            pending_element_matrix, pending_count_matrix, char_matrix)
        player_count = count_player_cells(element_matrix)
        unique_player_cell = None
        if player_count == 1:
            for player_row in range(GRID_ROWS):
                for player_col in range(GRID_COLS):
                    if element_matrix[player_row][player_col] == "player":
                        unique_player_cell = (player_row, player_col)
                        break
                if unique_player_cell is not None:
                    break

        need_precise_player_center = (
            center_request_active or observation_request_active or
            DEBUG_PLAYER_CENTER_ENABLE or DEBUG_PLAYER_HEADING_ENABLE or
            DEBUG_OBSERVATION_ENABLE or
            player_count != 1)
        precise_player_center = None
        precise_player_grid = None
        precise_player_cell = None
        player_green_center = None
        player_cyan_center = None
        player_heading_raw_deg = None
        if need_precise_player_center:
            player_center_result = resolve_player_center(
                recognition_img, recognition_points,
                raw_element_matrix, element_matrix,
                player_center_anchor, last_player_cell,
                rectified_recognition_active, raw_grid_transform)
            if player_center_result is not None:
                precise_player_center = player_center_result[0]
                precise_player_grid = player_center_result[1]
                precise_player_cell = player_center_result[2]
                player_green_center = player_center_result[3]
                player_cyan_center = player_center_result[4]
                player_center_anchor = precise_player_center
                last_precise_player_center = precise_player_center
                last_precise_player_grid = precise_player_grid
                player_center_lost_frames = 0
            else:
                player_center_lost_frames += 1
                if (player_center_lost_frames < PLAYER_CENTER_LOST_FRAME_LIMIT and
                        last_precise_player_center is not None and
                        last_precise_player_grid is not None):
                    precise_player_center = last_precise_player_center
                    precise_player_grid = last_precise_player_grid
                    precise_player_cell = (
                        precise_player_grid[1] // 100,
                        precise_player_grid[0] // 100)
                else:
                    player_center_anchor = None
                    last_precise_player_center = None
                    last_precise_player_grid = None
                    last_player_cell = None

        if center_request_active or DEBUG_PLAYER_HEADING_ENABLE:
            player_heading_raw_deg = player_heading_to_yaw_deg(
                player_green_center, player_cyan_center,
                rectified_recognition_active, raw_grid_transform)
            if player_heading_raw_deg is not None:
                player_heading_lost_frames = 0
                player_heading_history.append(player_heading_raw_deg)
                if len(player_heading_history) > PLAYER_HEADING_SAMPLE_COUNT:
                    player_heading_history.pop(0)
                heading_result = filter_player_heading_samples(
                    player_heading_history)
                if heading_result is None:
                    player_heading_filtered_deg = None
                    player_heading_accepted_count = 0
                else:
                    player_heading_filtered_deg = heading_result[0]
                    player_heading_accepted_count = heading_result[1]
            else:
                player_heading_lost_frames += 1
                if player_heading_lost_frames >= PLAYER_CENTER_LOST_FRAME_LIMIT:
                    player_heading_history = []
                    player_heading_filtered_deg = None
                    player_heading_accepted_count = 0

        precise_center_is_authoritative = (
            precise_player_grid is not None and
            (center_request_active or observation_request_active or
             player_count != 1))
        canonical_center_grid = (precise_player_grid
                                 if precise_center_is_authoritative else None)
        selected_player_cell = (precise_player_cell
                                if precise_center_is_authoritative
                                else unique_player_cell)
        update_non_player_background(
            element_matrix, player_background_matrix,
            selected_player_cell)
        canonical_char_matrix = build_canonical_player_map(
            element_matrix, player_background_matrix,
            canonical_center_grid)
        if canonical_char_matrix is not None:
            last_player_cell = selected_player_cell

        center_map_sent = False
        if (precise_player_center is not None or
                time.ticks_diff(now_ms, last_uart_send_ms) >=
                UART_MAP_SEND_PERIOD_MS):
            center_map_sent = process_center_request(
                map_uart, precise_player_grid,
                canonical_char_matrix, player_heading_raw_deg)
        if center_map_sent:
            last_uart_send_ms = now_ms
        box_centers = []
        if DEBUG_OBSERVATION_ENABLE:
            box_centers = detect_box_centers(
                recognition_img, recognition_points, element_matrix)
        observation_box_center = None
        if observation_request_active:
            observation_box_center = resolve_requested_box_center(
                recognition_img, recognition_points,
                element_matrix,
                observation_request_row, observation_request_col)
        observation_player_grid = precise_player_grid
        observation_box_grid = image_center_to_grid_q(
            observation_box_center,
            rectified_recognition_active, raw_grid_transform)
        if not box_center_grid_matches_request(
                observation_box_grid,
                observation_request_row, observation_request_col):
            observation_box_grid = None
        observation_map_sent = process_observation_request(
            map_uart, canonical_char_matrix,
            observation_player_grid, observation_box_grid)
        if observation_map_sent:
            last_uart_send_ms = now_ms
        debug_box_centers = []
        if DEBUG_OBSERVATION_ENABLE:
            debug_seen_centers = []
            for debug_row in range(GRID_ROWS):
                for debug_col in range(GRID_COLS):
                    if element_matrix[debug_row][debug_col] != "box":
                        continue
                    debug_center = select_box_center(
                        box_centers, recognition_points, debug_row, debug_col)
                    if (debug_center is None or
                            debug_center in debug_seen_centers):
                        continue
                    debug_seen_centers.append(debug_center)
                    debug_grid = image_center_to_grid_q(
                        debug_center, rectified_recognition_active,
                        raw_grid_transform)
                    debug_box_centers.append(
                        (debug_row, debug_col, debug_center, debug_grid))
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
        if (DEBUG_PLAYER_CENTER_ENABLE and precise_player_center is not None and
                (rectified_recognition_active == display_rectified)):
            img.draw_cross(precise_player_center[0], precise_player_center[1],
                           color=(255, 255, 0), thickness=2)
        if (DEBUG_PLAYER_HEADING_ENABLE and
                rectified_recognition_active == display_rectified):
            draw_player_heading_debug(
                img, player_green_center, player_cyan_center,
                player_heading_filtered_deg)
        if DEBUG_OBSERVATION_ENABLE:
            if rectified_recognition_active == display_rectified:
                for _, _, debug_center, _ in debug_box_centers:
                    img.draw_cross(debug_center[0], debug_center[1],
                                   color=(0, 255, 0), thickness=2)
        display_done_us = time.ticks_us()

        if ((DEBUG_ENABLE or DEBUG_PLAYER_CENTER_ENABLE or
                DEBUG_PLAYER_HEADING_ENABLE or DEBUG_OBSERVATION_ENABLE) and
                time.ticks_diff(now_ms, last_print_ms) >= DEBUG_PRINT_PERIOD_MS):
            loop_us = time.ticks_diff(time.ticks_us(), loop_start_us)
            loop_fps = 1000000.0 / loop_us if loop_us > 0 else 0.0
            debug_player_center_grid = image_center_to_grid_q(
                precise_player_center,
                rectified_recognition_active,
                raw_grid_transform)
            print_map(
                canonical_char_matrix if canonical_char_matrix is not None else char_matrix,
                clock.fps(),
                loop_fps,
                loop_us,
                time.ticks_diff(snapshot_done_us, loop_start_us),
                time.ticks_diff(rectify_done_us, snapshot_done_us),
                time.ticks_diff(recognize_done_us, rectify_done_us),
                time.ticks_diff(display_done_us, recognize_done_us),
                sensor.get_exposure_us(),
                precise_player_center)
            if debug_player_center_grid is not None:
                print("PLAYER_CENTER_GRID %d,%d 1" %
                      (debug_player_center_grid[0], debug_player_center_grid[1]))
            else:
                print("PLAYER_CENTER_GRID 0,0 0")
            if DEBUG_PLAYER_HEADING_ENABLE:
                if player_heading_raw_deg is None:
                    print("PLAYER_YAW invalid samples=%d/%d" %
                          (len(player_heading_history),
                           PLAYER_HEADING_SAMPLE_COUNT))
                elif player_heading_filtered_deg is None:
                    print("PLAYER_YAW raw=%.2f filtered=WAIT samples=%d/%d" %
                          (player_heading_raw_deg,
                           len(player_heading_history),
                           PLAYER_HEADING_SAMPLE_COUNT))
                else:
                    print("PLAYER_YAW raw=%.2f filtered=%.2f accepted=%d/%d" %
                          (player_heading_raw_deg,
                           player_heading_filtered_deg,
                           player_heading_accepted_count,
                           PLAYER_HEADING_SAMPLE_COUNT))
            if DEBUG_OBSERVATION_ENABLE:
                for debug_row, debug_col, _, debug_grid in debug_box_centers:
                    if debug_grid is not None:
                        print("BOX_CENTER_GRID row=%d col=%d center=%d,%d" %
                              (debug_row, debug_col,
                               debug_grid[0], debug_grid[1]))
            last_print_ms = now_ms

        if (UART_MAP_SEND_ENABLE and map_uart is not None and
                canonical_char_matrix is not None and
                time.ticks_diff(now_ms, last_uart_send_ms) >= UART_MAP_SEND_PERIOD_MS):
            periodic_center_grid = (precise_player_grid
                                    if player_count != 1 else None)
            send_map_uart(map_uart, canonical_char_matrix,
                          periodic_center_grid)
            last_uart_send_ms = now_ms


main()
