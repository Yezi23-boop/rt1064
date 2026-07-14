from pathlib import Path

import numpy as np
from PIL import Image
from ai_edge_litert.interpreter import Interpreter


ROOT = Path(__file__).resolve().parents[1]
VISION_DIR = ROOT / "openmv" / "视觉"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
SAMPLE_COUNT = 3


def readable_images(class_dir):
    valid = []
    invalid = []
    for path in sorted(class_dir.iterdir()):
        if path.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        try:
            with Image.open(path) as image:
                image.load()
            valid.append(path)
        except OSError as exc:
            invalid.append((path, str(exc)))
    return valid, invalid


def select_samples(paths):
    if len(paths) < SAMPLE_COUNT:
        raise RuntimeError("fewer than 3 readable images")
    return [paths[0], paths[len(paths) // 2], paths[-1]]


def preprocess(path):
    with Image.open(path) as image:
        image = image.convert("RGB").resize((128, 128), Image.Resampling.BILINEAR)
        pixels = np.asarray(image, dtype=np.int16)
    return np.clip(pixels - 128, -128, 127).astype(np.int8)[None, ...]


def run_dataset(name, dataset_dir, model_path):
    interpreter = Interpreter(model_content=model_path.read_bytes())
    interpreter.allocate_tensors()
    input_detail = interpreter.get_input_details()[0]
    output_detail = interpreter.get_output_details()[0]
    output_scale, output_zero = output_detail["quantization"]
    passed = 0
    total = 0

    print("\n=== %s ===" % name)
    for class_dir in sorted(path for path in dataset_dir.iterdir() if path.is_dir()):
        expected = int(class_dir.name[:2])
        readable, invalid = readable_images(class_dir)
        for path, error in invalid:
            print("SKIP invalid image: %s (%s)" % (path, error))

        results = []
        for path in select_samples(readable):
            interpreter.set_tensor(input_detail["index"], preprocess(path))
            interpreter.invoke()
            raw_output = interpreter.get_tensor(output_detail["index"])[0]
            probabilities = ((raw_output.astype(np.int16) - output_zero) *
                             output_scale)
            predicted = int(np.argmax(probabilities))
            confidence = float(probabilities[predicted])
            total += 1
            if predicted == expected:
                passed += 1
            results.append("%s -> %d (%.3f)" %
                           (path.name, predicted, confidence))
        print("class_id=%d: %s" % (expected, " | ".join(results)))

    print("%s RESULT: %d/%d (%.1f%%)" %
          (name, passed, total, passed * 100.0 / total))
    return passed == total


cartoon_ok = run_dataset(
    "cartoon",
    VISION_DIR / "1" / "train",
    VISION_DIR / "cartoon_V3.tflite",
)
number_ok = run_dataset(
    "number",
    VISION_DIR / "train",
    VISION_DIR / "number_V1.tflite",
)

if not cartoon_ok or not number_ok:
    raise SystemExit(1)

print("\nart2-model-sample PASS")
