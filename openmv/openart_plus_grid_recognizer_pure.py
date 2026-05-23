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

# ==================== 自动定位开关 ===================
AUTO_LOCATE_ENABLE = True
AUTO_LOCATE_ON_START = True
AUTO_RELOCATE_ENABLE = False
AUTO_RELOCATE_PERIOD_MS = 1000
AUTO_LOCATE_RETRY_MS = 1000
AUTO_RELOCATE_ON_BAD_FRAME = True
ROI_STABLE_REQUIRED = 2
ROI_STABLE_MAX_SHIFT = 6 * FRAME_SCALE
ROI_STABLE_MAX_SIZE_DIFF = 10 * FRAME_SCALE
ROI_ASPECT_TOLERANCE = 0.25
WALL_SCAN_STEP = 8 * FRAME_SCALE
WALL_CHROMA_MAX = 130
WALL_MIN_BRIGHTNESS = 25
WALL_MAX_BRIGHTNESS = 250
WALL_ROW_HIT_RATIO = 0.30
WALL_COL_HIT_RATIO = 0.30
WALL_RUN_RATIO = 0.20
LOCATE_SCAN_STEP = max(4, FRAME_SCALE * 4)


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


def rgb_from_pixel(pixel):
    if isinstance(pixel, tuple):
        return (int(pixel[0]), int(pixel[1]), int(pixel[2]))

    value = int(pixel)
    red = ((value >> 11) & 0x1F) * 255 // 31
    green = ((value >> 5) & 0x3F) * 255 // 63
    blue = (value & 0x1F) * 255 // 31
    return (red, green, blue)


def wall_chroma(rgb):
    avg = (rgb[0] + rgb[1] + rgb[2]) // 3
    return (abs(rgb[0] - avg) +
            abs(rgb[1] - avg) +
            abs(rgb[2] - avg))


def is_wall_candidate(rgb):
    avg = (rgb[0] + rgb[1] + rgb[2]) // 3
    if avg < WALL_MIN_BRIGHTNESS or avg > WALL_MAX_BRIGHTNESS:
        return False
    return wall_chroma(rgb) <= WALL_CHROMA_MAX


def wall_sample_hit(img, x, y):
    rgb = rgb_from_pixel(img.get_pixel(x, y))
    return is_wall_candidate(rgb)


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


def build_grid_points_from_roi(roi):
    roi_x = roi[0]
    roi_y = roi[1]
    roi_w = roi[2]
    roi_h = roi[3]
    grid_points = []

    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            x = int(roi_x + (col + 0.5) * roi_w / GRID_COLS + 0.5)
            y = int(roi_y + (row + 0.5) * roi_h / GRID_ROWS + 0.5)
            grid_points.append((x, y))

    return grid_points


def clamp_roi_to_image(roi, img_width, img_height):
    roi_x = roi[0]
    roi_y = roi[1]
    roi_w = roi[2]
    roi_h = roi[3]

    if roi_x < 0:
        roi_w += roi_x
        roi_x = 0
    if roi_y < 0:
        roi_h += roi_y
        roi_y = 0
    if roi_x + roi_w > img_width:
        roi_w = img_width - roi_x
    if roi_y + roi_h > img_height:
        roi_h = img_height - roi_y

    if roi_w < 1:
        roi_w = 1
    if roi_h < 1:
        roi_h = 1

    return (roi_x, roi_y, roi_w, roi_h)


def expand_roi(roi, margin, img_width, img_height):
    return clamp_roi_to_image(
        (roi[0] - margin, roi[1] - margin, roi[2] + margin * 2, roi[3] + margin * 2),
        img_width,
        img_height)


def roi_is_similar(a, b):
    return (abs(a[0] - b[0]) <= ROI_STABLE_MAX_SHIFT and
            abs(a[1] - b[1]) <= ROI_STABLE_MAX_SHIFT and
            abs(a[2] - b[2]) <= ROI_STABLE_MAX_SIZE_DIFF and
            abs(a[3] - b[3]) <= ROI_STABLE_MAX_SIZE_DIFF)


def roi_has_valid_shape(roi):
    if roi[2] <= 0 or roi[3] <= 0:
        return False

    aspect = roi[2] / roi[3]
    expected = GRID_COLS / GRID_ROWS
    return abs(aspect - expected) <= ROI_ASPECT_TOLERANCE


def row_wall_score(img, y, x_start, x_end, step):
    hits = 0
    count = 0
    best_run = 0
    current_run = 0

    for x in range(x_start, x_end + 1, step):
        if wall_sample_hit(img, x, y):
            hits += 1
            current_run += 1
            if current_run > best_run:
                best_run = current_run
        else:
            current_run = 0
        count += 1

    return (hits, count, best_run)


def col_wall_score(img, x, y_start, y_end, step):
    hits = 0
    count = 0
    best_run = 0
    current_run = 0

    for y in range(y_start, y_end + 1, step):
        if wall_sample_hit(img, x, y):
            hits += 1
            current_run += 1
            if current_run > best_run:
                best_run = current_run
        else:
            current_run = 0
        count += 1

    return (hits, count, best_run)


def longest_segment(values, step):
    if not values:
        return None

    best_start = values[0]
    best_end = values[0]
    best_len = 1

    cur_start = values[0]
    cur_end = values[0]

    for value in values[1:]:
        if value <= (cur_end + step):
            cur_end = value
        else:
            cur_len = ((cur_end - cur_start) // step) + 1
            if cur_len > best_len:
                best_len = cur_len
                best_start = cur_start
                best_end = cur_end
            cur_start = value
            cur_end = value

    cur_len = ((cur_end - cur_start) // step) + 1
    if cur_len > best_len:
        best_start = cur_start
        best_end = cur_end

    return (best_start, best_end)


def find_horizontal_band(img, y_start, y_end, x_start, x_end, step):
    sample_cols = ((x_end - x_start) // step) + 1
    min_hits = int(sample_cols * WALL_ROW_HIT_RATIO)
    if min_hits < 3:
        min_hits = 3
    min_run = int(sample_cols * WALL_RUN_RATIO)
    if min_run < 3:
        min_run = 3

    candidates = []
    for y in range(y_start, y_end + 1, LOCATE_SCAN_STEP):
        hits, count, best_run = row_wall_score(img, y, x_start, x_end, step)
        if count > 0 and hits >= min_hits and best_run >= min_run:
            candidates.append(y)

    return longest_segment(candidates, LOCATE_SCAN_STEP)


def find_vertical_band(img, x_start, x_end, y_start, y_end, step):
    sample_rows = ((y_end - y_start) // step) + 1
    min_hits = int(sample_rows * WALL_COL_HIT_RATIO)
    if min_hits < 3:
        min_hits = 3
    min_run = int(sample_rows * WALL_RUN_RATIO)
    if min_run < 3:
        min_run = 3

    candidates = []
    for x in range(x_start, x_end + 1, LOCATE_SCAN_STEP):
        hits, count, best_run = col_wall_score(img, x, y_start, y_end, step)
        if count > 0 and hits >= min_hits and best_run >= min_run:
            candidates.append(x)

    return longest_segment(candidates, LOCATE_SCAN_STEP)


def locate_map_roi(img, hint_roi=None):
    img_width = img.width()
    img_height = img.height()

    if hint_roi is None:
        search_roi = (0, 0, img_width, img_height)
    else:
        search_roi = expand_roi(hint_roi, max(LOCATE_SCAN_STEP * 4, 12 * FRAME_SCALE), img_width, img_height)

    search_x = search_roi[0]
    search_y = search_roi[1]
    search_w = search_roi[2]
    search_h = search_roi[3]
    search_x2 = search_x + search_w - 1
    search_y2 = search_y + search_h - 1

    top_band = find_horizontal_band(
        img,
        search_y,
        search_y + search_h // 2,
        search_x,
        search_x2,
        WALL_SCAN_STEP)
    bottom_band = find_horizontal_band(
        img,
        search_y + search_h // 2,
        search_y2,
        search_x,
        search_x2,
        WALL_SCAN_STEP)
    left_band = find_vertical_band(
        img,
        search_x,
        search_x + search_w // 2,
        search_y,
        search_y2,
        WALL_SCAN_STEP)
    right_band = find_vertical_band(
        img,
        search_x + search_w // 2,
        search_x2,
        search_y,
        search_y2,
        WALL_SCAN_STEP)

    if top_band is None or bottom_band is None or left_band is None or right_band is None:
        return None

    roi_x = left_band[0]
    roi_y = top_band[0]
    roi_w = right_band[1] - left_band[0] + 1
    roi_h = bottom_band[1] - top_band[0] + 1

    if roi_w < GRID_COLS * 8 or roi_h < GRID_ROWS * 8:
        return None

    roi = clamp_roi_to_image((roi_x, roi_y, roi_w, roi_h), img_width, img_height)
    if not roi_has_valid_shape(roi):
        return None

    return roi


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


def frame_looks_suspicious(map_matrix):
    inner_wall = 0
    inner_space = 0
    inner_cells = 0

    for row in range(1, GRID_ROWS - 1):
        for col in range(1, GRID_COLS - 1):
            inner_cells += 1
            if map_matrix[row][col] == ELEMENT_CODE["wall"]:
                inner_wall += 1
            elif map_matrix[row][col] == ELEMENT_CODE["space"]:
                inner_space += 1

    if inner_cells <= 0:
        return True
    if inner_wall >= int(inner_cells * 0.75):
        return True
    if inner_space < 4:
        return True
    return False


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
    fixed_grid_points = build_grid_points()

    clock = time.clock()
    last_print_ms = time.ticks_ms()
    last_locate_ms = time.ticks_ms() - AUTO_LOCATE_RETRY_MS
    active_roi = None
    candidate_roi = None
    candidate_streak = 0
    pending_relocate = AUTO_LOCATE_ON_START
    bad_frame_count = 0

    print("OPENART_GRID_RECOGNIZER_READY")
    print("FRAME_MODE=%s" % ("VGA" if USE_VGA else "QVGA"))
    print("GRID_ROWS=%d GRID_COLS=%d" % (GRID_ROWS, GRID_COLS))
    print("MAP_ROI=%s" % str(MAP_ROI))
    print("DOT_START_X=%d DOT_START_Y=%d COL_STEP=%d ROW_STEP=%d OFFSET=(%d,%d)" %
          (DOT_START_X, DOT_START_Y, COL_STEP, ROW_STEP, SAMPLE_OFFSET_X, SAMPLE_OFFSET_Y))
    print("CAMERA_MANUAL_EXPOSURE=%s CAMERA_EXPOSURE_US=%d" %
          (str(CAMERA_MANUAL_EXPOSURE), CAMERA_EXPOSURE_US))
    print("AUTO_LOCATE_ENABLE=%s AUTO_LOCATE_ON_START=%s AUTO_RELOCATE_ENABLE=%s" %
          (str(AUTO_LOCATE_ENABLE), str(AUTO_LOCATE_ON_START), str(AUTO_RELOCATE_ENABLE)))
    print("AUTO_RELOCATE_PERIOD_MS=%d AUTO_LOCATE_RETRY_MS=%d ROI_STABLE_REQUIRED=%d" %
          (AUTO_RELOCATE_PERIOD_MS, AUTO_LOCATE_RETRY_MS, ROI_STABLE_REQUIRED))

    while True:
        clock.tick()
        img = sensor.snapshot()
        now_ms = time.ticks_ms()

        if AUTO_LOCATE_ENABLE:
            should_locate = False
            locate_hint = active_roi if active_roi is not None else MAP_ROI

            if active_roi is None:
                should_locate = AUTO_LOCATE_ON_START and \
                    (time.ticks_diff(now_ms, last_locate_ms) >= AUTO_LOCATE_RETRY_MS)
            elif pending_relocate:
                should_locate = time.ticks_diff(now_ms, last_locate_ms) >= AUTO_LOCATE_RETRY_MS
            elif AUTO_RELOCATE_ENABLE and time.ticks_diff(now_ms, last_locate_ms) >= AUTO_RELOCATE_PERIOD_MS:
                should_locate = True

            if should_locate:
                located_roi = locate_map_roi(img, locate_hint)
                last_locate_ms = time.ticks_ms()

                if located_roi is not None:
                    if candidate_roi is not None and roi_is_similar(candidate_roi, located_roi):
                        candidate_streak += 1
                    else:
                        candidate_roi = located_roi
                        candidate_streak = 1

                    if candidate_streak >= ROI_STABLE_REQUIRED:
                        active_roi = located_roi
                        candidate_roi = None
                        candidate_streak = 0
                        pending_relocate = False
                        bad_frame_count = 0
                else:
                    if active_roi is None and DEBUG_ENABLE:
                        print("AUTO_ROI_NOT_FOUND")
        else:
            active_roi = MAP_ROI

        if active_roi is not None:
            grid_points = build_grid_points_from_roi(active_roi) if AUTO_LOCATE_ENABLE else fixed_grid_points
            if DEBUG_ENABLE and DEBUG_DRAW_ROI:
                img.draw_rectangle(active_roi, color=(0, 255, 0), thickness=2)
            if candidate_roi is not None and DEBUG_DRAW_ROI:
                img.draw_rectangle(candidate_roi, color=(255, 255, 0), thickness=1)

            map_matrix, char_matrix = recognize_map(img, grid_points)

            if AUTO_LOCATE_ENABLE and AUTO_RELOCATE_ON_BAD_FRAME:
                if frame_looks_suspicious(map_matrix):
                    bad_frame_count += 1
                else:
                    bad_frame_count = 0
                if bad_frame_count >= ROI_STABLE_REQUIRED:
                    pending_relocate = True
        else:
            if DEBUG_ENABLE and DEBUG_DRAW_ROI:
                img.draw_rectangle(MAP_ROI, color=(0, 255, 0), thickness=2)
            if DEBUG_ENABLE and time.ticks_diff(now_ms, last_print_ms) >= DEBUG_PRINT_PERIOD_MS:
                print("WAIT_LOCATE FPS %.2f" % clock.fps())
                last_print_ms = now_ms
            continue

        if DEBUG_ENABLE and time.ticks_diff(now_ms, last_print_ms) >= DEBUG_PRINT_PERIOD_MS:
            if active_roi is None:
                print("FPS %.2f ACTIVE_ROI=None" % clock.fps())
            else:
                print("ACTIVE_ROI=%s pending=%s bad=%d candidate=%s" %
                      (str(active_roi), str(pending_relocate), bad_frame_count, str(candidate_roi)))
                print_map(map_matrix, char_matrix, clock.fps())
            last_print_ms = now_ms


main()
