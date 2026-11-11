"""Read one stable WonderMV target and approach it using the grid v2 calibration."""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from contextlib import ExitStack
from dataclasses import dataclass

from coordinate_pick import LeArmSerial, Point, execute_stages
from grid_hover import approach_from_home, grid_to_arm, require_hover_range, reset_observation_pose
from object_pick import (
    OBJECTS, ObjectProfile, PickSetup, execute_pick_in_one_pass, gripper_open_position,
    plan_object_pick, plan_release_and_clear,
)

CAMERA_LINE_LIMIT = 65536


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
        line = camera.read_until(size=CAMERA_LINE_LIMIT)
        if not line or line.startswith(b"JPEG "):
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


@dataclass(frozen=True)
class CameraTarget:
    x_mm: float
    y_mm: float
    point: Point
    trims: tuple[int, int]


def capture_target(port, trim_forward_mm: int, trim_left_mm: int, *, select_trims=None,
                   report=None, limit_hover_radius=True) -> CameraTarget:
    if report is None:
        report = lambda message: print(message, flush=True)
    port.reset_input_buffer()
    report("보정 때와 같은 관찰 자세에서 물체 좌표를 기다립니다.")
    x_mm, y_mm = read_stable_target(port)
    if select_trims is not None:
        saved = select_trims(x_mm, y_mm)
        if saved is not None:
            trim_forward_mm, trim_left_mm = saved
            report(f"이 위치의 저장 보정 적용 · 앞뒤 {trim_forward_mm:+d}, 좌우 {trim_left_mm:+d} mm")
    nominal = grid_to_arm(x_mm, y_mm)
    target = Point(nominal.x + trim_forward_mm, nominal.y + trim_left_mm, nominal.z)
    report(f"확정 격자 좌표 ({x_mm:.2f}, {y_mm:.2f}) mm → "
           f"로봇 명령 ({target.x}, {target.y}, {target.z}) mm")
    if limit_hover_radius:
        require_hover_range(target)
    return CameraTarget(x_mm, y_mm, target, (trim_forward_mm, trim_left_mm))


def wait_for_cycle_key(prompt: str) -> bool:
    import msvcrt

    # Discard keystrokes queued during motion, including a held Enter key.
    quiet_until = time.monotonic() + 0.4
    while time.monotonic() < quiet_until:
        if msvcrt.kbhit():
            msvcrt.getwch()
            quiet_until = time.monotonic() + 0.4
        else:
            time.sleep(0.02)
    print(prompt, flush=True)
    while True:
        key = msvcrt.getwch()
        if key in ("\x00", "\xe0"):
            msvcrt.getwch()
        elif key == "\r":
            return True
        elif key.lower() == "q":
            return False


def run_pick_cycle(port, arm: LeArmSerial, profile: ObjectProfile, setup: PickSetup,
                   trim_forward_mm: int, trim_left_mm: int, *,
                   wait_for_action=None, check_alignment=None, select_trims=None, report=print) -> None:
    if wait_for_action is None:
        def wait_for_action(state, prompt):
            return wait_for_cycle_key(prompt)

    home = None
    retry_trims = None
    while retry_trims is not None or wait_for_action("pick", "[대기] 손과 경로를 비운 뒤 Enter로 집기, Q로 종료"):
        if home is None:
            report("초기 관찰 자세로 복귀 중")
            home = reset_observation_pose(arm)
            report(f"초기 자세 관절값: {home}")
        trims = retry_trims if retry_trims is not None else (trim_forward_mm, trim_left_mm)
        lookup = select_trims if retry_trims is None else None
        retry_trims = None
        try:
            detection = capture_target(port, *trims, select_trims=lookup, report=report)
        except (TimeoutError, ValueError) as error:
            report(f"집기 대기: {error}")
            continue
        target = detection.point
        stages = plan_object_pick(target, profile, setup, one_pass=True)
        approach_from_home(arm, target, home, report=report)
        if check_alignment is not None:
            decision = check_alignment(detection)
            if decision is None:
                return
            if decision is not True:
                retry_trims = decision
                home = None
                continue
        execute_pick_in_one_pass(arm, stages, report=report)
        if not wait_for_action("release",
            "[상승 완료] 들어 올린 결과와 원래 자리 아래를 확인한 뒤 Enter로 내려놓기, Q로 종료"
        ):
            return
        execute_pick_in_one_pass(arm, plan_release_and_clear(target, setup), report=report)
        report("초기 관찰 자세로 복귀 중")
        home = reset_observation_pose(arm)
        report(f"초기 자세 관절값: {home}")
        report("내려놓고 초기 자세로 복귀했습니다. 물체를 옮긴 뒤 다음 시험을 시작하세요.")


def main() -> None:
    parser = argparse.ArgumentParser(description="WonderMV에서 인식한 물체 좌표로 집게 접근 시험")
    parser.add_argument("--camera-port", required=True, help="WonderMV USB-C 또는 외부 UART 포트")
    parser.add_argument("--camera-baud", type=int, default=115200)
    parser.add_argument("--robot-port", help="로봇팔 보드의 별도 포트")
    parser.add_argument("--execute", action="store_true", help="인식 좌표를 확정한 후 실제 접근")
    parser.add_argument("--trim-forward-mm", type=int, default=0, help="실측 보정량. 양수는 팔에서 멀어지는 쪽")
    parser.add_argument("--trim-left-mm", type=int, default=0, help="실측 보정량. 양수는 격자 기준 왼쪽")
    parser.add_argument("--object", choices=OBJECTS, help="고정 방향 집기 시험 물체")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--one-pass", action="store_true", help="시작 확인 한 번으로 접근부터 들어 올리기까지 실행")
    mode.add_argument("--cycle", action="store_true", help="Windows에서 Enter로 집기와 내려놓기·복귀를 번갈아 실행")
    height = parser.add_mutually_exclusive_group()
    height.add_argument("--hover-contact-height-mm", type=float, help="접근 자세에서 작업면부터 집게 접촉면 중앙까지 높이. 물체 중간까지 하강량 계산")
    height.add_argument("--descent-mm", type=int, help="직접 조정할 하강량")
    parser.add_argument("--wrist-position", type=int, help="물체 방향과 맞춘 손목 서보값")
    opening = parser.add_mutually_exclusive_group()
    opening.add_argument("--open-position", type=int, help="측정한 열림에 해당하는 집게 서보값")
    opening.add_argument("--open-angle-deg", type=int, help="Hiwonder claw_set 기준 열림 각도 0~90°")
    parser.add_argument("--close-position", type=int, help="물체에 맞춰 확인한 닫힘 서보값")
    parser.add_argument("--measured-opening-mm", type=float, help="열림 상태의 양쪽 집게 안쪽 실측 간격")
    args = parser.parse_args()
    if args.execute and not args.robot_port:
        parser.error("--execute에는 --robot-port가 필요합니다.")
    if args.cycle and (not args.execute or not args.object or sys.platform != "win32" or not sys.stdin.isatty()):
        parser.error("--cycle에는 Windows PowerShell, --execute 및 --object가 필요합니다.")
    if args.robot_port and args.camera_port.casefold() == args.robot_port.casefold():
        parser.error("카메라와 로봇팔은 서로 다른 포트를 사용해야 합니다.")
    if max(abs(args.trim_forward_mm), abs(args.trim_left_mm)) > 10:
        parser.error("한 번의 시험 보정량은 축마다 ±10 mm 이내여야 합니다.")

    profile = OBJECTS.get(args.object)
    if args.one_pass and profile is None:
        parser.error("--one-pass에는 --object를 지정하세요.")
    setup = None
    open_position = args.open_position
    if args.open_angle_deg is not None:
        try:
            open_position = gripper_open_position(args.open_angle_deg)
        except ValueError as error:
            parser.error(str(error))
    descent_mm = args.descent_mm
    if args.hover_contact_height_mm is not None:
        if profile is None:
            parser.error("--hover-contact-height-mm에는 --object가 필요합니다.")
        try:
            descent_mm = profile.midpoint_descent_mm(args.hover_contact_height_mm)
        except ValueError as error:
            parser.error(str(error))
    values = (descent_mm, args.wrist_position, open_position,
              args.close_position, args.measured_opening_mm)
    if profile is None and any(value is not None for value in values):
        parser.error("집기 설정을 사용하려면 --object를 지정하세요.")
    if profile is not None:
        print(f"{profile.name} · 바닥 {profile.footprint_mm[0]}×{profile.footprint_mm[1]} mm"
              f" · 높이 {profile.height_mm} mm · 파지 폭 {profile.grip_width_mm} mm")
        print(f"기본 파지 높이: 작업면 위 {profile.grasp_height_mm:g} mm")
        print(profile.alignment)
        if all(value is not None for value in values):
            setup = PickSetup(*values)
            try:
                setup.validate(profile)
            except ValueError as error:
                parser.error(str(error))
        elif args.execute:
            parser.error("집기 실행에는 접근 높이 또는 하강량, 손목·열기·닫기 서보값과 실측 열림 간격이 필요합니다.")
        else:
            print("집기 설정이 아직 비어 있어 접근 좌표만 미리 봅니다.")

    import serial

    try:
        with ExitStack() as stack:
            port = serial.Serial(baudrate=args.camera_baud, timeout=1.5, write_timeout=2)
            port.port = args.camera_port
            port.dtr = port.rts = False
            port.open()
            stack.callback(port.close)
            if args.cycle:
                arm = LeArmSerial(args.robot_port, trace=True)
                stack.callback(arm.close)
                run_pick_cycle(port, arm, profile, setup, args.trim_forward_mm, args.trim_left_mm)
                return
            if args.execute:
                prompt = ("이 위치의 집기 설정을 확인했고 손과 전체 이동 경로가 비어 있으면 '실행': "
                          if args.one_pass else
                          "물체 방향을 고정했고 초기화·접근 경로와 물체 위 공간이 비어 있으면 '실행': ")
                if setup is not None and input(prompt).strip() != "실행":
                    return
                arm = LeArmSerial(args.robot_port, trace=True)
                stack.callback(arm.close)
                home = reset_observation_pose(arm)
            target = capture_target(port, args.trim_forward_mm, args.trim_left_mm).point
            stages = plan_object_pick(target, profile, setup, one_pass=args.one_pass) if setup is not None else []
            if stages:
                for stage in stages:
                    print(f"  {stage.label}")
            if args.execute:
                approach_from_home(arm, target, home)
                if stages:
                    if args.one_pass:
                        execute_pick_in_one_pass(arm, stages)
                        print("집기·상승 명령 완료. 물체가 실제로 들렸는지 확인하세요.")
                    else:
                        execute_stages(arm, stages)
    except (ValueError, OSError) as error:
        raise SystemExit(f"중단: {error}") from None


if __name__ == "__main__":
    main()
