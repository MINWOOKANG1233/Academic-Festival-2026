"""Convert grid v2 coordinates in millimetres and preview or run a hover trial."""
from __future__ import annotations

import argparse
import math
import time

from coordinate_pick import LeArmSerial, Point, deltas, packet


HOME = Point(150, 0, 20)
HOME_SERVOS = (222, 497, 180, 127, 413, 494)


def grid_to_arm(x_mm: float, y_mm: float) -> Point:
    """Return the board command coordinates at the trial's hover height."""
    if not (0 <= x_mm <= 200 and 0 <= y_mm <= 190):
        raise ValueError("격자 v2 범위는 X 0~200 mm, Y 0~190 mm입니다.")
    # Match the trial's forward rounding: 47.5 mm maps to arm X = 213 mm.
    return Point(math.floor(y_mm + 165.5), round(100 - x_mm), HOME.z)


def main() -> None:
    parser = argparse.ArgumentParser(description="격자 v2 좌표(mm)로 집게 접근 시험")
    parser.add_argument("x_mm", type=float, help="격자 왼쪽 아래 원에서 오른쪽 거리")
    parser.add_argument("y_mm", type=float, help="격자 왼쪽 아래 원에서 팔 반대쪽 거리")
    parser.add_argument("--execute", action="store_true", help="초기화 후 실제 접근")
    parser.add_argument("--port", help="로봇팔 보드 포트 (예: COM4)")
    args = parser.parse_args()

    try:
        target = grid_to_arm(args.x_mm, args.y_mm)
    except ValueError as error:
        parser.error(str(error))

    print(f"격자 ({args.x_mm:g}, {args.y_mm:g}) mm → 로봇 명령 "
          f"({target.x}, {target.y}, {target.z}) mm")
    if not args.execute:
        return
    if not args.port:
        parser.error("--execute에는 --port가 필요합니다.")

    # Limit physical trials to the near part of the tray and the tested centerline.
    near_area = 50 <= args.x_mm <= 150 and 0 <= args.y_mm <= 47.5
    centerline = args.x_mm == 100 and 0 <= args.y_mm <= 95
    if not (near_area or centerline):
        parser.error("실제 이동은 X 50~150 / Y 0~47.5 mm 또는 중앙선 X 100 / Y 0~95 mm에서만 가능합니다.")

    if input("격자판 배치가 보정 당시와 같고 집게와 초기화 경로가 비어 있으면 '실행': ").strip() != "실행":
        return

    arm = LeArmSerial(args.port, trace=True)
    try:
        if arm.inspect() != 2:
            raise SystemExit("버스 서보 보드에서만 실행할 수 있습니다.")
        arm.write(packet(12))
        time.sleep(3)
        positions = arm.read_positions()
        if any(abs(positions[i] - value) > 10 for i, value in enumerate(HOME_SERVOS, 1)):
            raise SystemExit(f"초기 관절 자세가 예상 범위를 벗어났습니다: {positions}")
        for command in deltas(HOME, target, step_mm=127):
            arm.write(command)
        time.sleep(3)
        print("이동 후 관절값:", arm.read_positions())
        print("집게 끝이 목표 위에 왔는지 확인하세요.")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
