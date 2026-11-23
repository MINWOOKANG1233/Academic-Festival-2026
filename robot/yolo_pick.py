"""YOLO -> fixed observation-plane calibration -> existing supervised pick.

Default is camera-only. No robot connection without --execute.
One object per scene; no bin placement is claimed by this program.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import dataclass
from io import BytesIO
import math
from pathlib import Path

from PIL import Image

from camera_hover import CameraTarget
from demo_pick import (demo_approach, demo_lower, align_demo_depth,
                       lift_demo_object)
from grid_hover import grid_to_arm
from object_pick import OBJECTS
from side_pick import CameraStream, SidePick, Step, raw_photo
from sort_bins import bin_path, place_in_bin

ROOT = Path(__file__).resolve().parents[1]
LABELS = {"candy_tin": "eclipse", "cube_block": "cube", "tissue_core": "tube"}
# Same 2026-11-04 fixed-pose plane calibration as vision/camera_red_block.py.
H = ((1.2130414816649202, .320394474809634, -90.18308692644314),
     (.0008814946953217581, -1.1361661469104847, 255.46360818761565),
     (.00006433416897809051, .00324863051176157, 1.0))


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    cx: float
    cy: float
    x_mm: float
    y_mm: float


def pixel_to_mm(cx, cy):
    if not all(map(math.isfinite, (cx, cy))) or not (0 <= cx < 320 and 0 <= cy < 240):
        raise ValueError("중심 픽셀 좌표가 유효하지 않습니다.")
    den = H[2][0]*cx + H[2][1]*cy + H[2][2]
    if abs(den) < 1e-6:
        raise ValueError("좌표 변환 분모가 0에 가깝습니다.")
    x = (H[0][0]*cx + H[0][1]*cy + H[0][2])/den
    y = (H[1][0]*cx + H[1][1]*cy + H[1][2])/den
    if not (0 <= x <= 200 and 0 <= y <= 190):
        raise ValueError("물체 중심이 보정 작업 영역 밖입니다.")
    return x, y


def select_detection(rows, names, confidence):
    """Reject extra boxes, unknown classes and clipped objects; never pick largest."""
    candidates = [row for row in rows if float(row[4]) >= confidence]
    if len(candidates) != 1:
        raise ValueError(f"물체 상자가 {len(candidates)}개입니다. 물체 하나만 놓고 오검출을 확인하세요.")
    x1, y1, x2, y2, score, class_id = map(float, candidates[0])
    if not all(map(math.isfinite, (x1, y1, x2, y2, score, class_id))):
        raise ValueError("검출값이 유효하지 않습니다.")
    label = names.get(int(class_id))
    if label not in LABELS:
        raise ValueError(f"지원하지 않는 클래스: {label}")
    if not (1 < x1 < x2 < 319 and 1 < y1 < y2 < 239):
        raise ValueError("물체가 화면 가장자리에 닿거나 잘렸습니다.")
    cx, cy = (x1+x2)/2, (y1+y2)/2
    x, y = pixel_to_mm(cx, cy)
    return Detection(label, score, cx, cy, x, y)


def compatible(first, second, tolerance=3):
    return first.label == second.label and math.hypot(first.x_mm-second.x_mm, first.y_mm-second.y_mm) <= tolerance


def detect_photo(model, jpeg, confidence):
    with Image.open(BytesIO(jpeg)) as image:
        if image.size != (320, 240):
            raise ValueError("카메라는 320×240이어야 합니다.")
        result = model.predict(image.convert("RGB"), imgsz=320, conf=confidence,
                               device="cpu", verbose=False)[0]
    return select_detection(result.boxes.data.cpu().tolist(), model.names, confidence)


def stable_target(camera, model, confidence, output=None, *, report=None, check_stop=None, on_photo=None):
    samples = []
    for attempt in range(9):
        if check_stop:
            check_stop()
        jpeg = raw_photo(camera)
        if check_stop:
            check_stop()
        if on_photo:
            on_photo(jpeg)
        try:
            detection = detect_photo(model, jpeg, confidence)
        except ValueError as error:
            samples.clear()
            (report or print)(f"검출 대기: {error}")
            continue
        if samples and not all(compatible(old, detection) for old in samples):
            samples.clear()
        samples.append(detection)
        (report or print)(f"{detection.label} 확신도 {detection.confidence:.2f} · "
              f"픽셀 ({detection.cx:.1f}, {detection.cy:.1f}) · "
              f"격자 ({detection.x_mm:.1f}, {detection.y_mm:.1f}) mm")
        if len(samples) == 3:
            if output is not None:
                output.mkdir(parents=True, exist_ok=True)
                (output/'observation.jpg').write_bytes(jpeg)
            return detection
    raise ValueError("같은 종류·3 mm 이내의 안정 검출 3개를 얻지 못했습니다.")


class YoloSidePick(SidePick):
    def detect(self, forward, left):
        self.require_step(Step.DETECT)
        self.check_stop()
        if forward or left:
            raise ValueError("이 연결 프로그램은 추가 수동 보정을 사용하지 않습니다.")
        self.detection = self.yolo_target
        self.report(f"YOLO 확정 좌표 사용: {self.detection}")
        self.step = Step.APPROACH


def supervised_pick(cycle, ask=input, *, sort=False):
    cycle.fast = cycle.profile is OBJECTS['cube']
    cycle.detect(0, 0)
    demo_approach(cycle)
    if ask('접근 완료. 실제 위치·집게 방향을 확인하고 하강·집기를 진행하려면 GRAB 입력: ').strip() != 'GRAB':
        print('접근 자세에서 멈췄습니다. 자동 복귀하지 않습니다.')
        return
    align_demo_depth(cycle, demo_lower(cycle))
    cycle.insert(0)
    if sort:
        bin_path(cycle, cycle.lift_pose)
    lift_demo_object(cycle)
    if sort:
        if ask("물체가 들렸고 분류통까지 경로가 비었으면 SORT 입력: ").strip() == "SORT":
            place_in_bin(cycle)
        else:
            print("들어 올린 자세를 유지합니다.")
        return
    if ask('상승 완료. 실제로 들렸는지 확인하세요. 원래 자리에 내려놓고 복귀하려면 RELEASE 입력: ').strip() != 'RELEASE':
        print('현재 자세를 유지합니다. 자동으로 집게를 열거나 복귀하지 않습니다.')
        return
    cycle.release()
    cycle.home()


def main():
    parser = argparse.ArgumentParser(description="YOLO 인식 확인 / 기존 집기 코드 연결")
    parser.add_argument('--camera-port', required=True)
    parser.add_argument('--robot-port')
    parser.add_argument('--model', type=Path, default=ROOT/'vision/models/best_v1_3class.pt')
    parser.add_argument('--conf', type=float, default=.6)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--sort', action='store_true')
    args = parser.parse_args()
    if not 0 < args.conf <= 1:
        parser.error('확신도는 0 초과 1 이하여야 합니다.')
    if args.execute and not args.robot_port:
        parser.error('--execute에는 --robot-port가 필요합니다.')
    if args.robot_port and args.robot_port.casefold() == args.camera_port.casefold():
        parser.error('카메라와 로봇 포트는 달라야 합니다.')
    if not args.model.is_file():
        parser.error(f'모델 파일 없음: {args.model}')
    from ultralytics import YOLO
    import serial
    model = YOLO(str(args.model))
    if model.task != 'detect' or set(model.names.values()) != set(LABELS):
        raise ValueError('세 물체용 객체 검출 모델이 아닙니다.')
    with ExitStack() as stack:
        port = serial.Serial(port=None, baudrate=115200, timeout=.12, write_timeout=2)
        port.dtr = port.rts = False
        port.port = args.camera_port
        port.open()
        stack.callback(port.close)
        camera = CameraStream(port)
        camera.wait_ready(lambda: None)
        cycle = None
        if args.execute:
            if input('보정 배치·관찰 자세 복귀 경로를 확인했다면 INIT 입력: ').strip() != 'INIT':
                return
            from coordinate_pick import LeArmSerial
            arm = LeArmSerial(args.robot_port)
            stack.callback(arm.close)
            cycle = YoloSidePick(camera, arm, OBJECTS['cube'], report=print,
                                 on_photo=lambda label, jpeg: None, cancelled=lambda: False)
            cycle.home()
        else:
            print('인식 확인 모드: 로봇 연결 없음. 팔을 보정 당시 관찰 자세로 두세요.', flush=True)
        output = ROOT/'experiments/yolo_live'
        target = stable_target(camera, model, args.conf, output)
        print(f'종류 → {OBJECTS[LABELS[target.label]].name} / 사진 → {output / "observation.jpg"}')
        if cycle is None:
            return
        if input('사진·물체 방향·이동 경로 확인 후 PICK 입력 (다른 입력은 취소): ').strip() != 'PICK':
            return
        fresh = stable_target(camera, model, args.conf, output)
        if not compatible(target, fresh):
            raise ValueError('확인 중 물체 종류 또는 위치가 바뀌어 중단했습니다.')
        cycle.profile = OBJECTS[LABELS[fresh.label]]
        cycle.yolo_target = CameraTarget(fresh.x_mm, fresh.y_mm,
                                         grid_to_arm(fresh.x_mm, fresh.y_mm), (0, 0))
        # Existing photos still refine alignment after approach. No moving-pose H conversion.
        supervised_pick(cycle, sort=args.sort)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyboardInterrupt) as error:
        raise SystemExit(f'중단: {error}') from None
