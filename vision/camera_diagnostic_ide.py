# CanMV / WonderMV K210: single-object detection on a fixed work surface.
import sensor
import image
import time
import lcd
import ujson
import gc
import ubinascii

# Use "red" to restore the original LAB detector.
DETECTION_MODE = "foreground"
# Start the foreground detector with the tray empty.
BACKGROUND_DELAY_MS = 3000
GRID_ROI = (0, 0, 320, 240)
# Threshold the brightness of the RGB difference, regardless of its hue.
DIFFERENCE_LAB = (12, 100, -128, 127, -128, 127)
RED_LAB = (20, 100, 20, 127, 10, 127)
MIN_PIXELS = 150
MIN_AREA = 150
OUTPUT_ENABLED = True  # Enable JSON output.
USB_C_OUTPUT_ENABLED = False  # False keeps the JSON in the CanMV IDE terminal.
USB_JPEG_OUTPUT_ENABLED = False  # Raw photo for the stopped-pose check in the PC app.
UART_OUTPUT_ENABLED = False  # Enable when sending JSON through WonderMV's external UART.
OUTPUT_INTERVAL_MS = 1000

usb = None

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


def foreground_target(img, background, original=None):
    img.difference(background)
    blobs = img.find_blobs(
        [DIFFERENCE_LAB], roi=GRID_ROI, pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA, merge=False,
    )
    if not blobs:
        return None, "NO BLOB"
    blobs = sorted(blobs, key=lambda candidate: candidate.pixels(), reverse=True)
    if len(blobs) > 1 and sum(candidate.pixels() for candidate in blobs[1:]) * 4 >= blobs[0].pixels():
        return None, "MULTIPLE BLOBS (%s px)" % ", ".join(str(candidate.pixels()) for candidate in blobs[:4])
    blob = blobs[0]
    x, y, w, h = blob.rect()
    left, top, width, height = GRID_ROI
    # Reject a clipped silhouette or a change that covers much of the scene.
    if x <= left or y <= top or x + w >= left + width or y + h >= top + height:
        return None, "AT ROI EDGE"
    if w * h > width * height // 4:
        if original is not None:
            bright = original.find_blobs(
                [(72, 100, -20, 20, -25, 25)], roi=(x, y, w, h),
                pixels_threshold=450, area_threshold=800, merge=False,
            )
            if len(bright) == 1:
                candidate = bright[0]
                bx, by, bw, bh = candidate.rect()
                if (bx > x and by > y and bx+bw < x+w and by+bh < y+h
                        and 20 <= bw <= 110 and 30 <= bh <= 150
                        and candidate.pixels() * 2 >= bw * bh):
                    return candidate, "OK BRIGHT FOREGROUND"
        return None, "BLOB TOO LARGE · box=%dx%d at (%d,%d), changed=%d px, limit=%d px2" % (
            w, h, x, y, blob.pixels(), width * height // 4
        )
    return blob, "OK"


print("CAMERA START: initializing LCD and sensor")
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

# Finish sensor setup before giving up the IDE link. Initialization errors remain
# visible in the IDE instead of being hidden by the UART mode switch.
if OUTPUT_ENABLED and USB_C_OUTPUT_ENABLED:
    print("CAMERA INITIALIZED: switching to USB stream; IDE preview may stop")
    time.sleep_ms(500)
    from machine import UART
    usb = UART.repl_uart()
    usb.init(115200, 8, None, 1, read_buf_len=2048, ide=False)
    usb.write(b"LEARM CAMERA READY\n")

while True:
    clock.tick()
    img = sensor.snapshot()
    request = (usb.read() or b"") if usb is not None and usb.any() else b""
    if b"P" in request:
        usb.write(b"LEARM CAMERA READY\n")
    if b"B" in request:
        background = img.copy()
        data["method"] = "background_difference"
        data["class"] = "foreground_object"
        usb.write(b"BACKGROUND OK\n")
    frame_id += 1
    output_due = time.ticks_diff(time.ticks_ms(), last_output) >= OUTPUT_INTERVAL_MS
    raw_jpeg = None
    if output_due and usb is not None and USB_JPEG_OUTPUT_ENABLED:
        raw_jpeg = bytes(img.compressed(quality=45).to_bytes())
    if background is not None:
        target, status = foreground_target(img, background, img.copy() if output_due else None)
    else:
        blobs = img.find_blobs(
            [RED_LAB], pixels_threshold=MIN_PIXELS,
            area_threshold=MIN_AREA, merge=False
        )
        target = max(blobs, key=lambda blob: blob.pixels()) if blobs else None
        status = "OK" if target is not None else "NO RED TARGET"
    data["frame"] = frame_id
    data["found"] = target is not None
    data["status"] = status
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
        img.draw_string(0, 0, status, color=(255, 255, 255))
    lcd.display(img)
    if output_due:
        if OUTPUT_ENABLED or uart is not None:
            message = ujson.dumps(data)
            if OUTPUT_ENABLED:
                if usb is None:
                    print(message)
                else:
                    usb.write((message + "\n").encode())
                    if raw_jpeg is not None:
                        usb.write(b"JPEG " + ubinascii.b2a_base64(raw_jpeg).rstrip() + b"\n")
            if uart is not None:
                uart.send_bytearray(list((message + "\n").encode()))
        gc.collect()
        last_output = time.ticks_ms()
    time.sleep_ms(50)
