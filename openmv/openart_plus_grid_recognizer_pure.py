import sensor
import time

# ==================== 墙体识别 ===================
USE_VGA = True
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

# 元素映射
ELEMENT_CODE = {
    "wall": 0,
    "space": 1,
    "goal": 2,
    "box": 3,
    "bomb": 4,
    "player": 5
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


def build_grid_points():
    grid_points = []
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            dot_x = DOT_START_X + col * COL_STEP
            dot_y = DOT_START_Y + row * ROW_STEP
            x = dot_x + SAMPLE_OFFSET_X
            y = dot_y + SAMPLE_OFFSET_Y
            grid_points.append((x, y))
    return grid_points


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
        if DEBUG_ENABLE and DEBUG_DRAW_POINTS:
            img.draw_circle(x, y, 3, color=DRAW_COLOR[element], fill=True)

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

    clock = time.clock()
    last_print_ms = time.ticks_ms()

    print("OPENART_GRID_RECOGNIZER_READY")
    print("FRAME_MODE=%s" % ("VGA" if USE_VGA else "QVGA"))
    print("GRID_ROWS=%d GRID_COLS=%d" % (GRID_ROWS, GRID_COLS))
    print("MAP_ROI=%s" % str(MAP_ROI))
    print("DOT_START_X=%d DOT_START_Y=%d COL_STEP=%d ROW_STEP=%d OFFSET=(%d,%d)" %
          (DOT_START_X, DOT_START_Y, COL_STEP, ROW_STEP, SAMPLE_OFFSET_X, SAMPLE_OFFSET_Y))
    print("CAMERA_MANUAL_EXPOSURE=%s CAMERA_EXPOSURE_US=%d" %
          (str(CAMERA_MANUAL_EXPOSURE), CAMERA_EXPOSURE_US))

    while True:
        clock.tick()
        img = sensor.snapshot()
        now_ms = time.ticks_ms()

        if DEBUG_ENABLE and DEBUG_DRAW_ROI:
            img.draw_rectangle(MAP_ROI, color=(0, 255, 0), thickness=2)

        map_matrix, char_matrix = recognize_map(img, grid_points)

        if DEBUG_ENABLE and time.ticks_diff(now_ms, last_print_ms) >= DEBUG_PRINT_PERIOD_MS:
            print_map(map_matrix, char_matrix, clock.fps())
            last_print_ms = now_ms


main()
