import sensor
import time

# ==================== 墙体识别 ===================
USE_VGA = False
FRAME_SCALE = 2 if USE_VGA else 1
IMG_WIDTH = 320 * FRAME_SCALE
IMG_HEIGHT = 240 * FRAME_SCALE
GRID_COLS = 16
GRID_ROWS = 12

# 地图ROI
MAP_ROI = (25 * FRAME_SCALE, 15 * FRAME_SCALE, 270 * FRAME_SCALE, 210 * FRAME_SCALE)

# 采样点配置
COL_STEP = 17 * FRAME_SCALE
ROW_STEP = 17 * FRAME_SCALE
DOT_START_X = 35 * FRAME_SCALE
DOT_START_Y = 25 * FRAME_SCALE
SAMPLE_OFFSET_X = -6 * FRAME_SCALE
SAMPLE_OFFSET_Y = -6 * FRAME_SCALE

# 人工四角标定：四点为完整16x12地图外边界，顺序为左上、右上、右下、左下。
# 初始值与当前固定步长网格近似等价；屏幕倾斜时只需改这里的四个点。
# 下列坐标按 QVGA 图像填写，切换 USE_VGA 后需要重新标定。
USE_QUAD_CALIBRATION = True
MAP_CORNERS = (
    (20.5, 10.5),
    (292.5, 10.5),
    (292.5, 214.5),
    (20.5, 214.5),
)

# 元素映射
ELEMENT_CODE = {
    "wall": 0,
    "space": 1,
    "goal": 2,
    "box": 3,
    "bomb": 4,
    "player": 5
}

CODE_ELEMENT = {
    0: "wall",
    1: "space",
    2: "goal",
    3: "box",
    4: "bomb",
    5: "player",
}

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

# ==================== 调试开关 ===================
DEBUG_ENABLE = True
DEBUG_DRAW_ROI = True
DEBUG_DRAW_POINTS = True
SHOW_RECTIFIED_VIEW = True
USE_RECTIFIED_RECOGNITION = True
DEBUG_PRINT_PERIOD_MS = 500
ENABLE_BOMB = False
CAMERA_MANUAL_EXPOSURE = False
CAMERA_EXPOSURE_US = 1000


def get_average_pixel(img, x, y, size=2):
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


def draw_recognition_points(img, map_matrix, grid_points):
    for idx, (x, y) in enumerate(grid_points):
        row_idx = idx // GRID_COLS
        col_idx = idx % GRID_COLS
        element = CODE_ELEMENT[map_matrix[row_idx][col_idx]]
        img.draw_circle(x, y, 3, color=DRAW_COLOR[element], fill=True)


def classify_element(img, row_idx, col_idx, x, y):
    if (row_idx == 0 or row_idx == GRID_ROWS - 1 or
            col_idx == 0 or col_idx == GRID_COLS - 1):
        return "wall"

    r, g, b = get_average_pixel(img, x, y)

    if r < 60 and g < 60 and b < 60:
        r, g, b = get_average_pixel(img, x + 3, y + 3)
        if r < 60 and g < 60 and b < 60:
            return "space"

    if (
        r > b + 15 and
        g > b + 15 and
        r > 45 and g > 45 and
        abs(r - g) < 65
    ):
        return "box"

    if g > r + 12 and g > b + 12:
        return "player"

    if (
        r > g + 30 and b > g + 30 and
        r > 110 and b > 110 and
        g < 70 and
        abs(r - b) < 30
    ):
        return "goal"

    if ENABLE_BOMB and r > g + 25 and r > b + 25:
        return "bomb"

    if b > r + 140 and b > g + 35:
        return "space"

    return "wall"


def recognize_map(img, grid_points):
    map_matrix = [[0 for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
    char_matrix = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]

    for idx, (x, y) in enumerate(grid_points):
        row_idx = idx // GRID_COLS
        col_idx = idx % GRID_COLS
        element = classify_element(img, row_idx, col_idx, x, y)
        map_matrix[row_idx][col_idx] = ELEMENT_CODE[element]
        char_matrix[row_idx][col_idx] = ELEMENT_CHAR[element]

    return map_matrix, char_matrix


def print_map(map_matrix, char_matrix, fps):
    print("=" * 40)
    print("当前识别地图：")
    for row in map_matrix:
        print(row)
    print("当前识别字符地图：")
    for row in char_matrix:
        print("".join(row))
    print("FPS %.2f" % fps)


def init_camera():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA if USE_VGA else sensor.QVGA)

    if CAMERA_MANUAL_EXPOSURE:
        try:
            sensor.set_auto_exposure(False, exposure_us=CAMERA_EXPOSURE_US)
        except Exception as exc:
            print("set_auto_exposure skipped:", repr(exc))

    sensor.skip_frames(time=2000)


def main():
    init_camera()
    grid_points = build_grid_points()
    rectified_grid_points = build_rectified_grid_points(IMG_WIDTH, IMG_HEIGHT)
    quad_corners = build_quad_corner_list() if USE_QUAD_CALIBRATION else None

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
    print("CAMERA_MANUAL_EXPOSURE=%s CAMERA_EXPOSURE_US=%d" %
          (str(CAMERA_MANUAL_EXPOSURE), CAMERA_EXPOSURE_US))

    rectified_view_active = SHOW_RECTIFIED_VIEW and USE_QUAD_CALIBRATION
    rectified_recognition_active = USE_RECTIFIED_RECOGNITION and USE_QUAD_CALIBRATION
    rectified_view_failed = False
    rectified_recognition_failed = False
    if (SHOW_RECTIFIED_VIEW or USE_RECTIFIED_RECOGNITION) and not USE_QUAD_CALIBRATION:
        print("RECTIFIED_MODE_NEEDS_QUAD_CALIBRATION")

    while True:
        clock.tick()
        img = sensor.snapshot()
        now_ms = time.ticks_ms()

        recognition_img = img
        recognition_points = grid_points
        rectified_img = None

        if rectified_recognition_active and not rectified_recognition_failed and quad_corners is not None:
            try:
                recognition_img = img.copy()
                recognition_img.rotation_corr(corners=quad_corners)
                recognition_points = rectified_grid_points
            except Exception as exc:
                print("RECTIFIED_RECOGNITION_FAILED:", repr(exc))
                rectified_recognition_failed = True
                rectified_recognition_active = False
                recognition_img = img
                recognition_points = grid_points

        map_matrix, char_matrix = recognize_map(recognition_img, recognition_points)

        if SHOW_RECTIFIED_VIEW and not rectified_view_failed and quad_corners is not None:
            try:
                if rectified_recognition_active and not rectified_recognition_failed and recognition_img is not img:
                    rectified_img = recognition_img
                else:
                    rectified_img = img.copy()
                    rectified_img.rotation_corr(corners=quad_corners)
            except Exception as exc:
                print("RECTIFIED_VIEW_FAILED:", repr(exc))
                rectified_view_failed = True
                rectified_img = None

        display_img = rectified_img if rectified_img is not None else img
        display_points = rectified_grid_points if rectified_img is not None else grid_points

        if DEBUG_ENABLE and DEBUG_DRAW_ROI:
            draw_calibration_boundary(display_img, rectified=(rectified_img is not None))

        if DEBUG_ENABLE and DEBUG_DRAW_POINTS:
            draw_recognition_points(display_img, map_matrix, display_points)

        if DEBUG_ENABLE and time.ticks_diff(now_ms, last_print_ms) >= DEBUG_PRINT_PERIOD_MS:
            print_map(map_matrix, char_matrix, clock.fps())
            last_print_ms = now_ms


main()
