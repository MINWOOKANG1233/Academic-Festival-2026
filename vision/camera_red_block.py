# CanMV / WonderMV K210: single-object detection on a fixed work surface.
import sensor
import image
import time
import lcd
import ujson
import gc

# Use "red" to restore the original LAB detector.
DETECTION_MODE = "foreground"
# Start the foreground detector with the tray empty.
BACKGROUND_DELAY_MS = 3000
GRID_ROI = (14, 35, 291, 192)
# Threshold the brightness of the RGB difference, regardless of its hue.
DIFFERENCE_LAB = (12, 100, -128, 127, -128, 127)
RED_LAB = (20, 100, 20, 127, 10, 127)
MIN_PIXELS = 150
MIN_AREA = 150
OUTPUT_ENABLED = True  # Enable JSON output.
USB_C_OUTPUT_ENABLED = True  # False keeps the JSON in the CanMV IDE terminal.
UART_OUTPUT_ENABLED = False  # Enable when sending JSON through WonderMV's external UART.
OUTPUT_INTERVAL_MS = 1000

usb = None
if OUTPUT_ENABLED and USB_C_OUTPUT_ENABLED:
    from machine import UART
    usb = UART.repl_uart()
    # Release the IDE debug link and stream JSON over the same USB-C cable.
    usb.init(115200, 8, None, 1, read_buf_len=2048, ide=False)

uart = None
if UART_OUTPUT_ENABLED:
    from hiwonder import hw_uart
    uart = hw_uart()

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


def foreground_target(img, background):
    img.difference(background)
    blobs = img.find_blobs(
        [DIFFERENCE_LAB], roi=GRID_ROI, pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA, merge=False,
    )
    if len(blobs) != 1:
        return None
    blob = blobs[0]
    x, y, w, h = blob.rect()
    left, top, width, height = GRID_ROI
    # Reject a clipped silhouette or a change that covers much of the scene.
    if (x <= left or y <= top or x + w >= left + width or y + h >= top + height
            or w * h > width * height // 4):
        return None
    return blob


lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320 x 240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
if DETECTION_MODE == "foreground":
    sensor.set_auto_exposure(False)
sensor.skip_frames(time=500)

background = None
if DETECTION_MODE == "foreground":
    print("Keep the tray empty. Capturing background in 3 seconds.")
    sensor.skip_frames(time=BACKGROUND_DELAY_MS)
    background = sensor.snapshot().copy()
    print("Background captured. Place one object on the tray.")

clock = time.clock()
last_output = time.ticks_ms()
frame_id = 0
data = {
    "frame": 0, "found": False,
    "method": "background_difference" if background is not None else "color_lab",
    "class": "foreground_object" if background is not None else "red_region",
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
    if background is not None:
        target = foreground_target(img, background)
    else:
        blobs = img.find_blobs(
            [RED_LAB], pixels_threshold=MIN_PIXELS,
            area_threshold=MIN_AREA, merge=False
        )
        target = max(blobs, key=lambda blob: blob.pixels()) if blobs else None
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
        img.draw_string(0, 0, "OBJECT x=%d y=%d" % (cx, cy), color=(0, 255, 0))
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
        img.draw_string(0, 0, "NO SINGLE TARGET", color=(255, 255, 255))
    lcd.display(img)
    if time.ticks_diff(time.ticks_ms(), last_output) >= OUTPUT_INTERVAL_MS:
        if OUTPUT_ENABLED or uart is not None:
            message = ujson.dumps(data)
            if OUTPUT_ENABLED:
                if usb is None:
                    print(message)
                else:
                    usb.write((message + "\n").encode())
            if uart is not None:
                uart.send_bytearray(list((message + "\n").encode()))
        gc.collect()
        last_output = time.ticks_ms()
    time.sleep_ms(50)
