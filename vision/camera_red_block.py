# CanMV / WonderMV K210: red region test, NOT a trained object detector.
import sensor
import image
import time
import lcd
import ujson
import gc

# LAB starting values only. Tune for your block and lighting in Threshold Editor.
RED_LAB = (20, 100, 20, 127, 10, 127)
MIN_PIXELS = 150
MIN_AREA = 150
OUTPUT_ENABLED = True  # Set False to isolate USB/terminal output issues.
OUTPUT_INTERVAL_MS = 1000
THRESHOLDS = [RED_LAB]

# Grid origin: lower-left; +X right; +Y toward upper grid row; units mm.
# Printed grid v2: X = 0, 100, 200; Y = 0, 95, 190 mm.
# Grid v2, 2026-11-04: fitted to 9 points at the fixed 320 x 240 observation pose.
GRID_WIDTH_MM = 200
GRID_HEIGHT_MM = 190
H = (
    (1.2130414816649202, 0.320394474809634, -90.18308692644314),
    (0.0008814946953217581, -1.1361661469104847, 255.46360818761565),
    (0.00006433416897809051, 0.00324863051176157, 1.0),
)


def pixel_to_mm(cx, cy):
    if H is None:
        return None
    denominator = H[2][0] * cx + H[2][1] * cy + H[2][2]
    if abs(denominator) < 0.000001:
        return None
    return (
        (H[0][0] * cx + H[0][1] * cy + H[0][2]) / denominator,
        (H[1][0] * cx + H[1][1] * cy + H[1][2]) / denominator
    )

lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320 x 240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=500)

clock = time.clock()
last_output = time.ticks_ms()
frame_id = 0
data = {
    "frame": 0, "found": False,
    "method": "color_lab", "class": "red_region",
    "width": 320, "height": 240, "unit": "pixel",
    "cx": None, "cy": None,
    "x_mm": None, "y_mm": None,
    "coordinate_frame": "grid_lower_left", "inside_grid": False,
    "calibration_ready": H is not None
}
gc.collect()

while True:
    clock.tick()
    img = sensor.snapshot()
    frame_id += 1
    blobs = img.find_blobs(
        THRESHOLDS, pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA, merge=False
    )
    # First test assumes a single red block on a plain background.
    target = None
    for blob in blobs:
        if target is None or blob.pixels() > target.pixels():
            target = blob
    data["frame"] = frame_id
    data["found"] = target is not None
    data["cx"] = None
    data["cy"] = None
    data["x_mm"] = None
    data["y_mm"] = None
    data["inside_grid"] = False
    if target is not None:
        cx, cy = target.cx(), target.cy()
        img.draw_rectangle(target.rect(), color=(0, 255, 0))
        img.draw_cross(cx, cy, color=(255, 255, 255))
        img.draw_string(0, 0, "RED x=%d y=%d" % (cx, cy), color=(0, 255, 0))
        data["cx"] = cx
        data["cy"] = cy
        xy = pixel_to_mm(cx, cy)
        if xy is not None:
            x_mm, y_mm = xy
            data["x_mm"] = round(x_mm, 1)
            data["y_mm"] = round(y_mm, 1)
            data["inside_grid"] = (
                0 <= x_mm <= GRID_WIDTH_MM and 0 <= y_mm <= GRID_HEIGHT_MM
            )
            img.draw_string(0, 14, "mm X=%.1f Y=%.1f" % (x_mm, y_mm), color=(0, 255, 0))
    else:
        img.draw_string(0, 0, "NO RED TARGET", color=(255, 255, 255))
    lcd.display(img)
    if time.ticks_diff(time.ticks_ms(), last_output) >= OUTPUT_INTERVAL_MS:
        if OUTPUT_ENABLED:
            print(ujson.dumps(data))
        gc.collect()
        last_output = time.ticks_ms()
    time.sleep_ms(50)
