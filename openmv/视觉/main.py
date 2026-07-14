import sensor
import time
import tf
import gc
from machine import UART


UART_INDEX = 12
UART_BAUD = 115200
UART_RX_LINE_MAX = 64
ZOOM = 1.0

CARTOON_MODEL = "/sd/cartoon_V3.tflite"
NUMBER_MODEL = "/sd/number_V1.tflite"
CARTOON_LABELS = "/sd/cartoon_labels.txt"
NUMBER_LABELS = "/sd/number_labels.txt"
CLASS_CONFIDENCE_Q_MIN = 750
CLASS_STABLE_FRAMES = 5
DEBUG_CLASSIFY_PRINT = True  # True=IDE串口打印当前模式、类别和置信度

MODE_NONE = 0
MODE_BOX = 1
MODE_TARGET = 2

DEBUG_REALTIME_ENABLE = False  # True=PC实时调试；MCU联调必须为False
DEBUG_REALTIME_MODE = "BOX"    # "BOX"=卡通，"TARGET"=数字

vision_mode = MODE_NONE
pending_mode = MODE_NONE
active_net = None
active_labels = None
classification_candidate = -1
classification_stable_count = 0
request_active = False
request_id = 0
sample_id = 0
uart_rx_line = ""


def debug_realtime_mode():
    return MODE_TARGET if DEBUG_REALTIME_MODE == "TARGET" else MODE_BOX


def reset_request():
    global request_active, sample_id
    request_active = False
    sample_id = 0
    reset_classification_stability()


def reset_classification_stability():
    global classification_candidate, classification_stable_count

    classification_candidate = -1
    classification_stable_count = 0


def classification_stability_push(class_id, confidence_q):
    global classification_candidate, classification_stable_count

    if confidence_q < CLASS_CONFIDENCE_Q_MIN:
        reset_classification_stability()
        return False
    if classification_candidate != class_id:
        classification_candidate = class_id
        classification_stable_count = 1
    elif classification_stable_count < CLASS_STABLE_FRAMES:
        classification_stable_count += 1
    return classification_stable_count >= CLASS_STABLE_FRAMES


def parse_uart_line(line, uart):
    global pending_mode, request_active, request_id, sample_id

    line = line.strip()
    if line == "VISION_MODE BOX":
        reset_request()
        if vision_mode == MODE_BOX and active_net is not None:
            pending_mode = MODE_NONE
            uart.write("VISION_READY BOX\n")
        else:
            pending_mode = MODE_BOX
        return
    if line == "VISION_MODE TARGET":
        reset_request()
        if vision_mode == MODE_TARGET and active_net is not None:
            pending_mode = MODE_NONE
            uart.write("VISION_READY TARGET\n")
        else:
            pending_mode = MODE_TARGET
        return
    if line.startswith("VISION_REQ "):
        try:
            next_request_id = int(line[11:])
        except Exception:
            return
        if vision_mode == MODE_NONE or next_request_id <= 0 or next_request_id > 65535:
            return
        request_id = next_request_id
        sample_id = 0
        request_active = True
        return
    if line.startswith("VISION_ACK "):
        try:
            ack_id = int(line[11:])
        except Exception:
            return
        if request_active and ack_id == request_id:
            reset_request()
        return
    if line == "VISION_CANCEL":
        reset_request()


def poll_uart_rx(uart):
    global uart_rx_line

    try:
        count = uart.any()
        if count <= 0:
            return
        data = uart.read(count)
        if data is None:
            return
        for value in data:
            if value == 13:
                continue
            if value == 10:
                if uart_rx_line:
                    parse_uart_line(uart_rx_line, uart)
                uart_rx_line = ""
            elif len(uart_rx_line) < UART_RX_LINE_MAX:
                uart_rx_line += chr(value)
            else:
                uart_rx_line = ""
    except Exception as exc:
        print("VISION_UART_RX_ERROR:", repr(exc))
        uart_rx_line = ""


def process_classification(uart, class_id, confidence_q):
    global sample_id

    if not request_active or vision_mode == MODE_NONE:
        return False
    if class_id < 0 or class_id > 9:
        return False
    if confidence_q < 0:
        confidence_q = 0
    elif confidence_q > 1000:
        confidence_q = 1000
    if not classification_stability_push(class_id, confidence_q):
        return False
    sample_id += 1
    uart.write("VISION_SAMPLE %d %d %d %d\n" %
               (request_id, sample_id, class_id, confidence_q))
    return True


def release_active_model():
    global vision_mode, active_net, active_labels

    model_loaded = active_net is not None
    vision_mode = MODE_NONE
    active_net = None
    active_labels = None
    if model_loaded:
        tf.free_from_fb()
    gc.collect()


def activate_pending_mode(uart):
    global vision_mode, pending_mode, active_net, active_labels

    mode = pending_mode
    if mode == MODE_NONE:
        return False
    pending_mode = MODE_NONE

    model_path = CARTOON_MODEL if mode == MODE_BOX else NUMBER_MODEL
    labels_path = CARTOON_LABELS if mode == MODE_BOX else NUMBER_LABELS
    mode_text = "BOX" if mode == MODE_BOX else "TARGET"
    try:
        release_active_model()
        active_net = tf.load(model_path, load_to_fb=True)
        active_labels = [line.rstrip() for line in open(labels_path)]
        if len(active_labels) != 10:
            raise Exception("expected 10 labels")
    except Exception as exc:
        print("VISION_MODEL_SWITCH_ERROR mode=%s:" % mode_text, repr(exc))
        try:
            release_active_model()
        except Exception:
            pass
        uart.write("VISION_ERROR ASSET\n")
        return False

    vision_mode = mode
    uart.write("VISION_READY %s\n" % mode_text)
    print("VISION_MODEL_READY mode=%s" % mode_text)
    return True


def set_zoom(level):
    width = 320
    height = 240
    crop_w = int(width / level)
    crop_h = int(height / level)
    offset_x = (width - crop_w) // 2
    offset_y = (height - crop_h) // 2
    sensor.set_windowing((offset_x, offset_y, crop_w, crop_h))
    sensor.skip_frames(time=100)


def init_camera():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_brightness(400)
    sensor.set_contrast(2)
    sensor.set_vflip(True)
    set_zoom(ZOOM)
    sensor.set_hmirror(True)
    sensor.skip_frames(time=200)


def init_uart():
    uart = UART(UART_INDEX, baudrate=UART_BAUD)
    uart.init(UART_BAUD, bits=8, parity=None, stop=1)
    return uart


def main():
    global pending_mode

    init_camera()
    uart = init_uart()

    uart.write("VISION_READY BOOT\n")
    print("ART2_UART_READY")
    if DEBUG_REALTIME_ENABLE:
        pending_mode = debug_realtime_mode()

    while True:
        poll_uart_rx(uart)
        if pending_mode != MODE_NONE:
            activate_pending_mode(uart)
        img = sensor.snapshot()
        if DEBUG_REALTIME_ENABLE:
            active_mode = debug_realtime_mode()
            classify_active = True
        else:
            active_mode = vision_mode
            classify_active = request_active and vision_mode != MODE_NONE
        if classify_active:
            try:
                result = tf.classify(active_net, img)[0]
                outputs = result.output()
                class_id = outputs.index(max(outputs))
                confidence_q = int(outputs[class_id] * 1000 + 0.5)
                if DEBUG_REALTIME_ENABLE:
                    stable = classification_stability_push(class_id, confidence_q)
                    sent = False
                else:
                    sent = process_classification(uart, class_id, confidence_q)
                    stable = sent
                if DEBUG_CLASSIFY_PRINT:
                    mode_text = "BOX" if active_mode == MODE_BOX else "TARGET"
                    if confidence_q < CLASS_CONFIDENCE_Q_MIN:
                        print("VISION_REJECT mode=%s class=%d label=%s confidence=%.3f" %
                              (mode_text, class_id, active_labels[class_id],
                               confidence_q / 1000.0))
                    elif stable:
                        print("VISION_RESULT mode=%s class=%d label=%s mapped_digit=%d confidence=%.3f" %
                              (mode_text, class_id, active_labels[class_id], class_id,
                               confidence_q / 1000.0))
                    else:
                        print("VISION_PENDING mode=%s class=%d stable=%d/%d confidence=%.3f" %
                              (mode_text, class_id, classification_stable_count,
                               CLASS_STABLE_FRAMES, confidence_q / 1000.0))
            except Exception as exc:
                print("VISION_CLASSIFY_ERROR:", repr(exc))
        gc.collect()


main()
