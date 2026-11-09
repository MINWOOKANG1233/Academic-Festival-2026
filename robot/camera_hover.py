"""Read one stable WonderMV target and approach it using the grid v2 calibration."""
from __future__ import annotations

import argparse
import json
import math
import time
from contextlib import ExitStack

from coordinate_pick import LeArmSerial, Point
from grid_hover import approach_from_home, grid_to_arm, require_hover_range, reset_observation_pose


def read_detection(line: bytes) -> tuple[int, float, float] | None:
    try:
        data = json.loads(line)
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict) or not (
        data.get("found") is True
        and data.get("calibration_ready") is True
        and data.get("inside_grid") is True
        and data.get("coordinate_frame") == "grid_lower_left"
        and (data.get("width"), data.get("height")) == (320, 240)
    ):
        return None
    frame = data.get("frame")
    x_mm, y_mm = data.get("x_mm"), data.get("y_mm")
    if type(frame) is not int or type(x_mm) not in (int, float) or type(y_mm) not in (int, float):
        return None
    if not (0 <= x_mm <= 200 and 0 <= y_mm <= 190):
        return None
    return frame, x_mm, y_mm


def read_stable_target(camera, timeout: float = 15) -> tuple[float, float]:
    deadline = time.monotonic() + timeout
    samples: list[tuple[float, float]] = []
    last_frame = -1
    while time.monotonic() < deadline:
        line = camera.read_until(size=1024)
        if not line:
            continue
        detection = read_detection(line) if line.endswith(b"\n") else None
        if detection is None:
            samples.clear()
            continue
        frame, x_mm, y_mm = detection
        if frame <= last_frame:
            samples.clear()
            continue
        if any(math.hypot(x_mm - x, y_mm - y) > 3 for x, y in samples):
            samples.clear()
        last_frame = frame
        samples.append((x_mm, y_mm))
        if len(samples) == 3:
            return sum(x for x, _ in samples) / 3, sum(y for _, y in samples) / 3
    raise TimeoutError("카메라에서 연속된 안정 좌표 3개를 받지 못했습니다.")


def main() -> None:
    parser = argparse.ArgumentParser(description="WonderMV에서 인식한 물체 좌표로 집게 접근 시험")
    parser.add_argument("--camera-port", required=True, help="WonderMV USB-C 또는 외부 UART 포트")
    parser.add_argument("--camera-baud", type=int, default=115200)
    parser.add_argument("--robot-port", help="로봇팔 보드의 별도 포트")
    parser.add_argument("--execute", action="store_true", help="인식 좌표를 확정한 후 실제 접근")
    parser.add_argument("--trim-forward-mm", type=int, default=0, help="실측 보정량. 양수는 팔에서 멀어지는 쪽")
    parser.add_argument("--trim-left-mm", type=int, default=0, help="실측 보정량. 양수는 격자 기준 왼쪽")
    args = parser.parse_args()
    if args.execute and not args.robot_port:
        parser.error("--execute에는 --robot-port가 필요합니다.")
    if args.robot_port and args.camera_port.casefold() == args.robot_port.casefold():
        parser.error("카메라와 로봇팔은 서로 다른 포트를 사용해야 합니다.")
    if max(abs(args.trim_forward_mm), abs(args.trim_left_mm)) > 10:
        parser.error("한 번의 시험 보정량은 축마다 ±10 mm 이내여야 합니다.")

    import serial

    try:
        with ExitStack() as stack:
            port = serial.Serial(baudrate=args.camera_baud, timeout=0.5, write_timeout=2)
            port.port = args.camera_port
            port.dtr = port.rts = False
            port.open()
            stack.callback(port.close)
            if args.execute:
                arm = LeArmSerial(args.robot_port, trace=True)
                stack.callback(arm.close)
                home = reset_observation_pose(arm)
            port.reset_input_buffer()
            print("보정 때와 같은 관찰 자세에서 물체 좌표를 기다립니다.", flush=True)
            x_mm, y_mm = read_stable_target(port)
            nominal = grid_to_arm(x_mm, y_mm)
            target = Point(nominal.x + args.trim_forward_mm, nominal.y + args.trim_left_mm, nominal.z)
            print(f"확정 격자 좌표 ({x_mm:.2f}, {y_mm:.2f}) mm → "
                  f"로봇 명령 ({target.x}, {target.y}, {target.z}) mm", flush=True)
            require_hover_range(target)
            if args.execute:
                approach_from_home(arm, target, home)
    except (ValueError, OSError) as error:
        raise SystemExit(f"중단: {error}") from None


if __name__ == "__main__":
    main()
