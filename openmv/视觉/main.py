import sensor
import time
import tf
import gc
from machine import UART


UART_INDEX = 12
UART_BAUD = 115200
UART_RX_LINE_MAX = 64

CARTOON_MODEL = "/sd/cartoon_V2.tflite"
NUMBER_MODEL = "/sd/number_V1.tflite"

MODE_NONE = 0
MODE_BOX = 1
MODE_TARGET = 2

vision_mode = MODE_NONE
request_active = False
request_id = 0
sample_id = 0
uart_rx_line = ""


def reset_request():
    global request_active, sample_id
    request_active = False
    sample_id = 0


def parse_uart_line(line, uart):
    global vision_mode, request_active, request_id, sample_id

    line = line.strip()
    if line == "VISION_MODE BOX":
        reset_request()
        vision_mode = MODE_BOX
        uart.write("VISION_READY BOX\n")
        return
    if line == "VISION_MODE TARGET":
        reset_request()
        vision_mode = MODE_TARGET
        uart.write("VISION_READY TARGET\n")
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
        return
    if class_id < 0 or class_id > 9:
        return
    if confidence_q < 0:
        confidence_q = 0
    elif confidence_q > 1000:
        confidence_q = 1000
    sample_id += 1
    uart.write("VISION_SAMPLE %d %d %d %d\n" %
               (request_id, sample_id, class_id, confidence_q))


def init_camera():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_brightness(400)
    sensor.set_contrast(2)
    sensor.set_vflip(True)
    sensor.set_hmirror(True)
    sensor.skip_frames(time=500)


def init_uart():
    uart = UART(UART_INDEX, baudrate=UART_BAUD)
    uart.init(UART_BAUD, bits=8, parity=None, stop=1)
    return uart


def main():
    init_camera()
    uart = init_uart()

    try:
        cartoon_net = tf.load(CARTOON_MODEL, load_to_fb=True)
        number_net = tf.load(NUMBER_MODEL, load_to_fb=True)
    except Exception as exc:
        print("VISION_MODEL_LOAD_ERROR:", repr(exc))
        uart.write("VISION_ERROR MODEL_MEMORY\n")
        while True:
            poll_uart_rx(uart)
            sensor.snapshot()

    uart.write("VISION_READY BOOT\n")
    print("ART2_MODELS_READY")

    while True:
        poll_uart_rx(uart)
        img = sensor.snapshot()
        if request_active and vision_mode != MODE_NONE:
            net = cartoon_net if vision_mode == MODE_BOX else number_net
            try:
                outputs = tf.classify(net, img)[0].output()
                class_id = outputs.index(max(outputs))
                confidence_q = int(outputs[class_id] * 1000 + 0.5)
                process_classification(uart, class_id, confidence_q)
            except Exception as exc:
                print("VISION_CLASSIFY_ERROR:", repr(exc))
        gc.collect()


main()
