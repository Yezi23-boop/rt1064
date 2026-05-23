import sensor
import time


# Run this file from OpenMV/OpenART IDE as a temporary script.
# It does not write E:/main.py and does not send UART frames.

GRID_COLS = 12
GRID_ROWS = 16
CAMERA_FRAMESIZE = sensor.VGA

# Initial ROI from the provided screenshot.
# Reference image size ~= 537x752, map area ~= x=0 y=32 w=536 h=720.
# The script scales this ROI to the actual sensor frame size at runtime.
# Adjust these reference values in OpenMV/OpenART IDE until sample points sit
# near the centers of the map cells.
REF_IMAGE_W = 537
REF_IMAGE_H = 752
ROI_X = 0
ROI_Y = 32
ROI_W = 536
ROI_H = 720

SAMPLE_RADIUS = 2
MAX_COLOR_DISTANCE = 120
PRINT_PERIOD_MS = 500
DRAW_DEBUG = False
DRAW_EVERY_N = 5
SHOW_ROI_RECT = True
SHOW_SAMPLE_POINTS = True
PRINT_UNKNOWN_DETAIL = False
PRINT_ALL_RGB = False
AUTO_ROI_ENABLE = True
AUTO_ROI_SCAN_STEP = 5
AUTO_ROI_MIN_BLUE_POINTS = 120
AUTO_ROI_BLUE_ROW_DIV = 4
AUTO_ROI_BLUE_COL_DIV = 4
AUTO_ROI_BAD_FRAME_LIMIT = 4
AUTO_ROI_MAX_UNKNOWN = 16
AUTO_ROI_MIN_GRID_WALLS = 16
AUTO_ROI_SMOOTH_NUM = 7
AUTO_ROI_SMOOTH_DEN = 10
AUTO_ROI_RETRY_MS = 1000
AUTO_ROI_GAP_TOLERANCE = 2
AUTO_QUAD_ENABLE = True
AUTO_QUAD_FIND_THRESHOLD = 5000
AUTO_QUAD_RETRY_MS = 1000
AUTO_QUAD_MIN_AREA = 12000
AUTO_QUAD_EDGE_MIN_RATIO_NUM = 2
AUTO_QUAD_EDGE_MIN_RATIO_DEN = 3
AUTO_QUAD_CENTER_MIN_RATIO_NUM = 1
AUTO_QUAD_CENTER_MIN_RATIO_DEN = 3

BLUE_MIN_VALUE = 120
BLUE_MIN_DOMINANCE = 55
WALL_MAX_CHROMA = 70
WALL_MIN_BRIGHTNESS = 35
WALL_MAX_BRIGHTNESS = 235
WALL_REF_DISTANCE = 120
WALL_EDGE_MIN_RATIO_NUM = 2
WALL_EDGE_MIN_RATIO_DEN = 5
WALL_BAND_MIN_RATIO_NUM = 1
WALL_BAND_MIN_RATIO_DEN = 3

# SeekFree article reference colors plus screenshot-calibrated references.
# Output characters follow the game/map_file convention:
#   # wall, - empty, . target, $ box, * bomb, C car, ? unknown
# Repeated chars are intentional: nearest-neighbor matching accepts any
# reference assigned to the same output character.
COLOR_REFS = (
    ("#", (57, 65, 82)),
    ("#", (160, 157, 176)),
    ("-", (33, 12, 255)),
    (".", (231, 0, 255)),
    (".", (255, 0, 255)),
    ("$", (148, 178, 0)),
    ("$", (255, 255, 0)),
    ("*", (255, 24, 74)),
    ("C", (16, 154, 0)),
    ("C", (36, 255, 42)),
    ("C", (0, 255, 255)),
)

DRAW_COLORS = {
    "#": (80, 160, 255),
    "-": (0, 255, 0),
    ".": (255, 0, 255),
    "$": (255, 255, 0),
    "*": (255, 0, 0),
    "C": (0, 255, 255),
    "?": (255, 0, 0),
}


def rgb_from_pixel(pixel):
    if isinstance(pixel, tuple):
        return (int(pixel[0]), int(pixel[1]), int(pixel[2]))

    value = int(pixel)
    red = ((value >> 11) & 0x1F) * 255 // 31
    green = ((value >> 5) & 0x3F) * 255 // 63
    blue = (value & 0x1F) * 255 // 31
    return (red, green, blue)


def color_distance(rgb_a, rgb_b):
    dr = rgb_a[0] - rgb_b[0]
    dg = rgb_a[1] - rgb_b[1]
    db = rgb_a[2] - rgb_b[2]
    return int((dr * dr + dg * dg + db * db) ** 0.5)


def classify_rgb(rgb):
    best_char = "?"
    best_dist = 100000

    for item in COLOR_REFS:
        char = item[0]
        ref = item[1]
        dist = color_distance(rgb, ref)
        if dist < best_dist:
            best_dist = dist
            best_char = char

    if best_dist > MAX_COLOR_DISTANCE:
        return ("?", best_dist)
    return (best_char, best_dist)


def pixel_looks_like_blue_field(rgb):
    return ((rgb[2] >= BLUE_MIN_VALUE) and
            ((rgb[2] - rgb[0]) >= BLUE_MIN_DOMINANCE) and
            ((rgb[2] - rgb[1]) >= BLUE_MIN_DOMINANCE))


def pixel_looks_like_wall(rgb):
    avg = (rgb[0] + rgb[1] + rgb[2]) // 3
    chroma = abs(rgb[0] - avg) + abs(rgb[1] - avg) + abs(rgb[2] - avg)

    if ((chroma <= WALL_MAX_CHROMA) and
            (avg >= WALL_MIN_BRIGHTNESS) and
            (avg <= WALL_MAX_BRIGHTNESS)):
        return True

    for item in COLOR_REFS:
        if item[0] == "#" and color_distance(rgb, item[1]) <= WALL_REF_DISTANCE:
            return True

    return False


def sample_average_rgb(img, cx, cy, radius):
    width = img.width()
    height = img.height()
    total_r = 0
    total_g = 0
    total_b = 0
    count = 0

    for y in range(cy - radius, cy + radius + 1):
        if y < 0 or y >= height:
            continue
        for x in range(cx - radius, cx + radius + 1):
            if x < 0 or x >= width:
                continue
            rgb = rgb_from_pixel(img.get_pixel(x, y))
            total_r += rgb[0]
            total_g += rgb[1]
            total_b += rgb[2]
            count += 1

    if count == 0:
        return (0, 0, 0)

    return (total_r // count, total_g // count, total_b // count)


def count_changed(grid, last_grid):
    if last_grid is None:
        return 0

    changed = 0
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            if grid[row][col] != last_grid[row][col]:
                changed += 1
    return changed


def draw_sample_marker(img, x, y, char):
    color = DRAW_COLORS.get(char, (255, 255, 255))
    img.draw_rectangle((x - 2, y - 2, 5, 5), color=color)


def quad_point(corners, u, v):
    tl = corners[0]
    tr = corners[1]
    br = corners[2]
    bl = corners[3]

    x = ((1.0 - u) * (1.0 - v) * tl[0] +
         u * (1.0 - v) * tr[0] +
         u * v * br[0] +
         (1.0 - u) * v * bl[0])
    y = ((1.0 - u) * (1.0 - v) * tl[1] +
         u * (1.0 - v) * tr[1] +
         u * v * br[1] +
         (1.0 - u) * v * bl[1])
    return (int(x + 0.5), int(y + 0.5))


def draw_quad(img, corners, color):
    img.draw_line(corners[0][0], corners[0][1], corners[1][0], corners[1][1], color=color)
    img.draw_line(corners[1][0], corners[1][1], corners[2][0], corners[2][1], color=color)
    img.draw_line(corners[2][0], corners[2][1], corners[3][0], corners[3][1], color=color)
    img.draw_line(corners[3][0], corners[3][1], corners[0][0], corners[0][1], color=color)


def quad_bounds(corners):
    min_x = corners[0][0]
    max_x = corners[0][0]
    min_y = corners[0][1]
    max_y = corners[0][1]

    for pt in corners[1:]:
        if pt[0] < min_x:
            min_x = pt[0]
        if pt[0] > max_x:
            max_x = pt[0]
        if pt[1] < min_y:
            min_y = pt[1]
        if pt[1] > max_y:
            max_y = pt[1]

    return (min_x, min_y, max_x - min_x + 1, max_y - min_y + 1)


def quad_edge_score(img, corners):
    hits = 0
    total = 0
    offsets = (0.05, 0.25, 0.50, 0.75, 0.95)

    for u in offsets:
        rgb = sample_average_rgb(img, *quad_point(corners, u, 0.04), SAMPLE_RADIUS)
        if pixel_looks_like_wall(rgb):
            hits += 1
        total += 1

        rgb = sample_average_rgb(img, *quad_point(corners, u, 0.96), SAMPLE_RADIUS)
        if pixel_looks_like_wall(rgb):
            hits += 1
        total += 1

    for v in offsets:
        rgb = sample_average_rgb(img, *quad_point(corners, 0.04, v), SAMPLE_RADIUS)
        if pixel_looks_like_wall(rgb):
            hits += 1
        total += 1

        rgb = sample_average_rgb(img, *quad_point(corners, 0.96, v), SAMPLE_RADIUS)
        if pixel_looks_like_wall(rgb):
            hits += 1
        total += 1

    return (hits, total)


def quad_center_score(img, corners):
    hits = 0
    total = 0
    points = ((0.25, 0.25), (0.50, 0.25), (0.75, 0.25),
              (0.25, 0.50), (0.50, 0.50), (0.75, 0.50),
              (0.25, 0.75), (0.50, 0.75), (0.75, 0.75))

    for item in points:
        rgb = sample_average_rgb(img, *quad_point(corners, item[0], item[1]), SAMPLE_RADIUS)
        if pixel_looks_like_blue_field(rgb):
            hits += 1
        total += 1

    return (hits, total)


def detect_map_quad(img):
    rects = img.find_rects(threshold=AUTO_QUAD_FIND_THRESHOLD)
    best_corners = None
    best_score = -1

    for rect in rects:
        corners = rect.corners()
        bounds = quad_bounds(corners)
        bw = bounds[2]
        bh = bounds[3]
        if bw < 40 or bh < 40:
            continue
        if (bw * bh) < AUTO_QUAD_MIN_AREA:
            continue

        edge_hits, edge_total = quad_edge_score(img, corners)
        if edge_total <= 0:
            continue

        center_hits, center_total = quad_center_score(img, corners)
        score = (edge_hits * 100) + (center_hits * 20) + ((bw * bh) // 100)

        if (edge_hits * AUTO_QUAD_EDGE_MIN_RATIO_DEN) < (edge_total * AUTO_QUAD_EDGE_MIN_RATIO_NUM):
            continue
        if (center_hits * AUTO_QUAD_CENTER_MIN_RATIO_DEN) < (center_total * AUTO_QUAD_CENTER_MIN_RATIO_NUM):
            continue

        if score > best_score:
            best_score = score
            best_corners = corners

    return best_corners


def get_active_roi(img):
    scale_x = img.width() / REF_IMAGE_W
    scale_y = img.height() / REF_IMAGE_H
    roi_x = int(ROI_X * scale_x)
    roi_y = int(ROI_Y * scale_y)
    roi_w = int(ROI_W * scale_x)
    roi_h = int(ROI_H * scale_y)

    if roi_x < 0:
        roi_x = 0
    if roi_y < 0:
        roi_y = 0
    if (roi_x + roi_w) > img.width():
        roi_w = img.width() - roi_x
    if (roi_y + roi_h) > img.height():
        roi_h = img.height() - roi_y

    return (roi_x, roi_y, roi_w, roi_h)


def clamp_roi_to_image(img, roi):
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
    if (roi_x + roi_w) > img.width():
        roi_w = img.width() - roi_x
    if (roi_y + roi_h) > img.height():
        roi_h = img.height() - roi_y

    return (roi_x, roi_y, roi_w, roi_h)


def expand_roi_to_grid_aspect(img, roi):
    roi_x = roi[0]
    roi_y = roi[1]
    roi_w = roi[2]
    roi_h = roi[3]

    # Desired aspect: cols / rows = 12 / 16.
    desired_num = GRID_COLS
    desired_den = GRID_ROWS

    if roi_w * desired_den > roi_h * desired_num:
        target_h = (roi_w * desired_den) // desired_num
        delta = target_h - roi_h
        roi_y -= delta // 2
        roi_h = target_h
    else:
        target_w = (roi_h * desired_num) // desired_den
        delta = target_w - roi_w
        roi_x -= delta // 2
        roi_w = target_w

    return clamp_roi_to_image(img, (roi_x, roi_y, roi_w, roi_h))


def find_broad_hit_range(hits, threshold, gap_tolerance):
    start = -1
    end = -1
    gap = 0
    seen = False

    for i in range(len(hits)):
        if hits[i] >= threshold:
            if start < 0:
                start = i
            end = i
            seen = True
            gap = 0
        elif seen:
            gap += 1
            if gap > gap_tolerance:
                break

    return (start, end)


def expand_inner_blue_to_outer_roi(img, inner_roi):
    inner_x = inner_roi[0]
    inner_y = inner_roi[1]
    inner_w = inner_roi[2]
    inner_h = inner_roi[3]

    if inner_w <= 0 or inner_h <= 0:
        return None

    cell_w = inner_w / (GRID_COLS - 2)
    cell_h = inner_h / (GRID_ROWS - 2)

    roi_x = int(inner_x - cell_w + 0.5)
    roi_y = int(inner_y - cell_h + 0.5)
    roi_w = int(cell_w * GRID_COLS + 0.5)
    roi_h = int(cell_h * GRID_ROWS + 0.5)
    roi = clamp_roi_to_image(img, (roi_x, roi_y, roi_w, roi_h))
    return expand_roi_to_grid_aspect(img, roi)


def find_wall_row(img, x0, x1, y0, y1, prefer_high):
    if y0 < 0:
        y0 = 0
    if y1 >= img.height():
        y1 = img.height() - 1
    if x0 < 0:
        x0 = 0
    if x1 >= img.width():
        x1 = img.width() - 1
    if (y1 < y0) or (x1 < x0):
        return None

    selected_y = None
    step = AUTO_ROI_SCAN_STEP
    if step < 1:
        step = 1

    for y in range(y0, y1 + 1, step):
        hits = 0
        count = 0
        for x in range(x0, x1 + 1, step):
            count += 1
            if pixel_looks_like_wall(rgb_from_pixel(img.get_pixel(x, y))):
                hits += 1
        if count <= 0:
            continue
        if (hits * WALL_BAND_MIN_RATIO_DEN) >= (count * WALL_BAND_MIN_RATIO_NUM):
            selected_y = y
            if not prefer_high:
                return selected_y

    return selected_y


def find_wall_col(img, x0, x1, y0, y1, prefer_high):
    if x0 < 0:
        x0 = 0
    if x1 >= img.width():
        x1 = img.width() - 1
    if y0 < 0:
        y0 = 0
    if y1 >= img.height():
        y1 = img.height() - 1
    if (x1 < x0) or (y1 < y0):
        return None

    selected_x = None
    step = AUTO_ROI_SCAN_STEP
    if step < 1:
        step = 1

    for x in range(x0, x1 + 1, step):
        hits = 0
        count = 0
        for y in range(y0, y1 + 1, step):
            count += 1
            if pixel_looks_like_wall(rgb_from_pixel(img.get_pixel(x, y))):
                hits += 1
        if count <= 0:
            continue
        if (hits * WALL_BAND_MIN_RATIO_DEN) >= (count * WALL_BAND_MIN_RATIO_NUM):
            selected_x = x
            if not prefer_high:
                return selected_x

    return selected_x


def refine_outer_roi_from_wall_bands(img, inner_roi):
    inner_x = inner_roi[0]
    inner_y = inner_roi[1]
    inner_w = inner_roi[2]
    inner_h = inner_roi[3]

    if inner_w <= 0 or inner_h <= 0:
        return None

    cell_w = inner_w // (GRID_COLS - 2)
    cell_h = inner_h // (GRID_ROWS - 2)
    if cell_w <= 0 or cell_h <= 0:
        return None

    top_y = find_wall_row(
        img,
        inner_x,
        inner_x + inner_w - 1,
        inner_y - cell_h * 2,
        inner_y,
        True)
    bottom_y = find_wall_row(
        img,
        inner_x,
        inner_x + inner_w - 1,
        inner_y + inner_h - 1,
        inner_y + inner_h + cell_h * 2,
        False)
    left_x = find_wall_col(
        img,
        inner_x - cell_w * 2,
        inner_x,
        inner_y,
        inner_y + inner_h - 1,
        True)
    right_x = find_wall_col(
        img,
        inner_x + inner_w - 1,
        inner_x + inner_w + cell_w * 2,
        inner_y,
        inner_y + inner_h - 1,
        False)

    if top_y is None or bottom_y is None or left_x is None or right_x is None:
        return None

    roi_x = left_x - cell_w // 2
    roi_y = top_y - cell_h // 2
    roi_w = right_x - left_x + cell_w
    roi_h = bottom_y - top_y + cell_h

    return expand_roi_to_grid_aspect(img, clamp_roi_to_image(img, (roi_x, roi_y, roi_w, roi_h)))


def wall_edge_score(img, roi):
    roi_x = roi[0]
    roi_y = roi[1]
    roi_w = roi[2]
    roi_h = roi[3]
    total = 0
    hits = 0

    for col in range(GRID_COLS):
        cx = int(roi_x + (col + 0.5) * roi_w / GRID_COLS)
        for row in (0, GRID_ROWS - 1):
            cy = int(roi_y + (row + 0.5) * roi_h / GRID_ROWS)
            if pixel_looks_like_wall(sample_average_rgb(img, cx, cy, SAMPLE_RADIUS)):
                hits += 1
            total += 1

    for row in range(1, GRID_ROWS - 1):
        cy = int(roi_y + (row + 0.5) * roi_h / GRID_ROWS)
        for col in (0, GRID_COLS - 1):
            cx = int(roi_x + (col + 0.5) * roi_w / GRID_COLS)
            if pixel_looks_like_wall(sample_average_rgb(img, cx, cy, SAMPLE_RADIUS)):
                hits += 1
            total += 1

    return (hits, total)


def wall_edge_valid(img, roi):
    hits, total = wall_edge_score(img, roi)
    if total <= 0:
        return False
    return (hits * WALL_EDGE_MIN_RATIO_DEN) >= (total * WALL_EDGE_MIN_RATIO_NUM)


def detect_map_roi(img):
    matched = 0
    step = AUTO_ROI_SCAN_STEP

    if step < 1:
        step = 1

    sample_cols = (img.width() + step - 1) // step
    sample_rows = (img.height() + step - 1) // step
    row_hits = [0] * sample_rows
    col_hits = [0] * sample_cols
    row_index = 0


    for y in range(0, img.height(), step):
        col_index = 0
        for x in range(0, img.width(), step):
            if pixel_looks_like_blue_field(rgb_from_pixel(img.get_pixel(x, y))):
                matched += 1
                row_hits[row_index] += 1
                col_hits[col_index] += 1
            col_index += 1
        row_index += 1

    if matched < AUTO_ROI_MIN_BLUE_POINTS:
        return (None, matched)

    row_threshold = sample_cols // AUTO_ROI_BLUE_ROW_DIV
    col_threshold = sample_rows // AUTO_ROI_BLUE_COL_DIV
    if row_threshold < 2:
        row_threshold = 2
    if col_threshold < 2:
        col_threshold = 2

    min_row, max_row = find_broad_hit_range(row_hits, row_threshold, AUTO_ROI_GAP_TOLERANCE)
    min_col, max_col = find_broad_hit_range(col_hits, col_threshold, AUTO_ROI_GAP_TOLERANCE)

    if (min_row < 0) or (min_col < 0):
        return (None, matched)

    min_x = min_col * step
    min_y = min_row * step
    max_x = (max_col + 1) * step - 1
    max_y = (max_row + 1) * step - 1

    min_x -= step
    min_y -= step
    max_x += step
    max_y += step

    inner_roi = clamp_roi_to_image(img, (min_x, min_y, max_x - min_x + 1, max_y - min_y + 1))
    roi = refine_outer_roi_from_wall_bands(img, inner_roi)
    if roi is None:
        roi = expand_inner_blue_to_outer_roi(img, inner_roi)
    if roi is None:
        return (None, matched)
    if not wall_edge_valid(img, roi):
        return (None, matched)
    return (roi, matched)


def smooth_roi(last_roi, new_roi):
    if last_roi is None:
        return new_roi

    out = []
    for i in range(4):
        value = ((last_roi[i] * AUTO_ROI_SMOOTH_NUM) +
                 (new_roi[i] * (AUTO_ROI_SMOOTH_DEN - AUTO_ROI_SMOOTH_NUM))) // AUTO_ROI_SMOOTH_DEN
        out.append(value)
    return (out[0], out[1], out[2], out[3])


def roi_in_image(img, roi):
    return ((roi[2] > 0) and (roi[3] > 0) and
            (roi[0] >= 0) and (roi[1] >= 0) and
            ((roi[0] + roi[2]) <= img.width()) and
            ((roi[1] + roi[3]) <= img.height()))


def grid_is_abnormal(counts, unknowns):
    if len(unknowns) > AUTO_ROI_MAX_UNKNOWN:
        return True
    if counts.get("#", 0) < AUTO_ROI_MIN_GRID_WALLS:
        return True
    return False


def recognize_grid(img, draw_debug, roi, corners=None):
    grid = []
    unknowns = []
    counts = {"#": 0, "-": 0, ".": 0, "$": 0, "*": 0, "C": 0, "?": 0}
    use_quad = corners is not None

    if use_quad:
        if SHOW_ROI_RECT or draw_debug:
            draw_quad(img, corners, (0, 255, 0))
    else:
        roi_x = roi[0]
        roi_y = roi[1]
        roi_w = roi[2]
        roi_h = roi[3]

        if SHOW_ROI_RECT or draw_debug:
            img.draw_rectangle((roi_x, roi_y, roi_w, roi_h), color=(0, 255, 0))

    for row in range(GRID_ROWS):
        chars = []
        for col in range(GRID_COLS):
            if use_quad:
                cx, cy = quad_point(corners, (col + 0.5) / GRID_COLS, (row + 0.5) / GRID_ROWS)
            else:
                cx = int(roi_x + (col + 0.5) * roi_w / GRID_COLS)
                cy = int(roi_y + (row + 0.5) * roi_h / GRID_ROWS)
            rgb = sample_average_rgb(img, cx, cy, SAMPLE_RADIUS)
            char, dist = classify_rgb(rgb)

            chars.append(char)
            counts[char] = counts.get(char, 0) + 1

            if char == "?":
                unknowns.append((row, col, rgb, dist))

            if SHOW_SAMPLE_POINTS or draw_debug:
                draw_sample_marker(img, cx, cy, char)

            if PRINT_ALL_RGB:
                print("RGB row=%02d col=%02d rgb=%s char=%s dist=%d" %
                      (row, col, str(rgb), char, dist))

        grid.append("".join(chars))

    return (grid, counts, unknowns)


def print_grid(frame_id, grid, counts, unknowns, changed):
    print("FRAME %d unknown=%d changed=%d #=%d -=%d .=%d $=%d *=%d C=%d" %
          (frame_id, len(unknowns), changed,
           counts.get("#", 0), counts.get("-", 0), counts.get(".", 0),
           counts.get("$", 0), counts.get("*", 0), counts.get("C", 0)))

    for row in range(GRID_ROWS):
        print("ROW%02d:%s" % (row, grid[row]))

    if PRINT_UNKNOWN_DETAIL:
        for item in unknowns:
            row = item[0]
            col = item[1]
            rgb = item[2]
            dist = item[3]
            print("UNKNOWN row=%02d col=%02d rgb=%s dist=%d" %
                  (row, col, str(rgb), dist))


sensor.reset()
sensor.set_pixformat(sensor.RGB565)

# OpenMV/OpenART requires a frame size after sensor.reset(); otherwise
# snapshot() can return a 0x0 image. ROI detection still adapts to this size.
sensor.set_framesize(CAMERA_FRAMESIZE)
sensor.skip_frames(time=1000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=500)

clock = time.clock()
last_print_ms = time.ticks_ms()
last_grid = None
last_auto_roi = None
last_auto_quad = None
bad_frame_count = 0
force_relocate = AUTO_ROI_ENABLE
last_roi_search_ms = 0
last_quad_search_ms = 0
frame_id = 0

print("OPENART_GRID_RECOGNIZER_READY")
print("GRID_COLS=%d GRID_ROWS=%d" % (GRID_COLS, GRID_ROWS))
print("REF_FALLBACK_IMAGE=%dx%d" % (REF_IMAGE_W, REF_IMAGE_H))
print("REF_FALLBACK_ROI x=%d y=%d w=%d h=%d" % (ROI_X, ROI_Y, ROI_W, ROI_H))
print("SAMPLE_RADIUS=%d MAX_COLOR_DISTANCE=%d" %
      (SAMPLE_RADIUS, MAX_COLOR_DISTANCE))
print("AUTO_ROI_ENABLE=%s SCAN_STEP=%d MIN_BLUE_POINTS=%d BAD_LIMIT=%d" %
      (str(AUTO_ROI_ENABLE), AUTO_ROI_SCAN_STEP, AUTO_ROI_MIN_BLUE_POINTS,
       AUTO_ROI_BAD_FRAME_LIMIT))
print("AUTO_QUAD_ENABLE=%s FIND_THRESHOLD=%d RETRY_MS=%d" %
      (str(AUTO_QUAD_ENABLE), AUTO_QUAD_FIND_THRESHOLD, AUTO_QUAD_RETRY_MS))

while True:
    clock.tick()
    img = sensor.snapshot()
    frame_id += 1
    roi_source = "ref"
    roi_points = 0
    active_roi = get_active_roi(img)
    active_quad = None
    now_ms = time.ticks_ms()

    if AUTO_QUAD_ENABLE:
        if last_auto_quad is not None:
            active_quad = last_auto_quad
            roi_source = "quad_cached"

        should_search_quad = False
        if force_relocate:
            should_search_quad = True
        elif last_auto_quad is None:
            if time.ticks_diff(now_ms, last_quad_search_ms) >= AUTO_QUAD_RETRY_MS:
                should_search_quad = True

        if should_search_quad:
            last_quad_search_ms = now_ms
            auto_quad = detect_map_quad(img)
            if auto_quad is not None:
                last_auto_quad = auto_quad
                active_quad = auto_quad
                active_roi = quad_bounds(auto_quad)
                roi_source = "quad_auto"
                force_relocate = False
                bad_frame_count = 0

    if AUTO_ROI_ENABLE:
        if last_auto_roi is not None and active_quad is None:
            active_roi = last_auto_roi
            roi_source = "cached"

        should_search = False
        if force_relocate:
            should_search = True
        elif last_auto_roi is None:
            if time.ticks_diff(now_ms, last_roi_search_ms) >= AUTO_ROI_RETRY_MS:
                should_search = True

        if should_search:
            last_roi_search_ms = now_ms
            auto_roi, roi_points = detect_map_roi(img)
            if auto_roi is not None:
                last_auto_roi = smooth_roi(last_auto_roi, auto_roi)
                active_roi = last_auto_roi
                roi_source = "auto"
                force_relocate = False
                bad_frame_count = 0
            elif last_auto_roi is not None:
                active_roi = last_auto_roi
                roi_source = "cached"
            else:
                roi_source = "none"

    if roi_source == "none":
        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_PERIOD_MS:
            print("AUTO_ROI_NOT_FOUND image=%dx%d blue_points=%d" %
                  (img.width(), img.height(), roi_points))
            last_print_ms = now_ms
        continue

    if active_quad is None and not roi_in_image(img, active_roi):
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_PERIOD_MS:
            print("ROI_OUT_OF_RANGE image=%dx%d roi=%s" %
                  (img.width(), img.height(), str(active_roi)))
            last_print_ms = now_ms
        continue

    draw_debug = DRAW_DEBUG and ((frame_id % DRAW_EVERY_N) == 0)
    grid, counts, unknowns = recognize_grid(img, draw_debug, active_roi, active_quad)
    changed = count_changed(grid, last_grid)
    abnormal = grid_is_abnormal(counts, unknowns)
    if active_quad is not None:
        wall_hits, wall_total = quad_edge_score(img, active_quad)
    else:
        wall_hits, wall_total = wall_edge_score(img, active_roi)
    wall_ok = (wall_total > 0) and ((wall_hits * WALL_EDGE_MIN_RATIO_DEN) >= (wall_total * WALL_EDGE_MIN_RATIO_NUM))
    if not wall_ok:
        abnormal = True

    if abnormal:
        bad_frame_count += 1
        if bad_frame_count >= AUTO_ROI_BAD_FRAME_LIMIT:
            last_auto_roi = None
            last_auto_quad = None
            force_relocate = AUTO_ROI_ENABLE
    else:
        bad_frame_count = 0

    if time.ticks_diff(now_ms, last_print_ms) >= PRINT_PERIOD_MS:
        print("IMAGE %dx%d ROI_SOURCE=%s ROI_POINTS=%d ACTIVE_ROI=%s BAD=%d WALL=%d/%d" %
              (img.width(), img.height(), roi_source, roi_points, str(active_roi),
               bad_frame_count, wall_hits, wall_total))
        print_grid(frame_id, grid, counts, unknowns, changed)
        print("FPS %.2f" % clock.fps())
        last_print_ms = now_ms

    last_grid = grid
