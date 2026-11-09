"""Convert grid v2 coordinates in millimetres and preview or run a hover trial."""
from __future__ import annotations

import argparse
import math
import time

from coordinate_pick import LeArmSerial, Point, deltas, packet


HOME = Point(150, 0, 20)
# Base ID 6 is nominally 500 at straight ahead. Observed resets settle on
# either side of that value depending on the direction of approach.
HOME_SERVOS = (222, 497, 180, 127, 413, 500)
# Initial horizontal envelope at HOME.z. 260 mm was observed on the centerline.
# The extra 5 mm permits a supervised boundary trial. This is not an IK model.
HOVER_RADIUS_MM = 265


def grid_to_arm(x_mm: float, y_mm: float) -> Point:
    """Return the board command coordinates at the trial's hover height."""
    if not (0 <= x_mm <= 200 and 0 <= y_mm <= 190):
        raise ValueError("격자 v2 범위는 X 0~200 mm, Y 0~190 mm입니다.")
    # Match the trial's forward rounding: 47.5 mm maps to arm X = 213 mm.
    return Point(math.floor(y_mm + 165.5), round(100 - x_mm), HOME.z)


def require_hover_range(target: Point) -> None:
    radius = math.hypot(target.x, target.y)
    if radius > HOVER_RADIUS_MM:
        raise ValueError(f"이동 범위 밖: 수평 반경 {radius:.1f} mm, 현재 시험 한계 {HOVER_RADIUS_MM} mm.")


def reset_observation_pose(arm: LeArmSerial) -> dict[int, int]:
    if arm.inspect() != 2:
        raise ValueError("버스 서보 보드에서만 실행할 수 있습니다.")
    arm.write(packet(12))
    time.sleep(3)
    positions = arm.read_positions()
    if any(abs(positions[i] - value) > 10 for i, value in enumerate(HOME_SERVOS, 1)):
        raise OSError(f"초기 관절 자세가 예상 범위를 벗어났습니다: {positions}")
    return positions


def approach_from_home(arm: LeArmSerial, target: Point, positions: dict[int, int]) -> None:
    require_hover_range(target)
    for command in deltas(HOME, target, step_mm=25):
        arm.write(command)
        time.sleep(1.5)
        current = arm.read_positions()
        if max(abs(current[i] - positions[i]) for i in (3, 4, 5, 6)) <= 2:
            raise OSError("이동 확인 불가: 관절 응답 변화가 작습니다. 남은 명령을 중단했습니다.")
        positions = current
    print("이동 후 관절값:", positions)
    print("집게 끝이 목표 위에 왔는지 확인하세요.")


def execute_hover(port: str, x_mm: float, y_mm: float) -> None:
    target = grid_to_arm(x_mm, y_mm)
    require_hover_range(target)
    if input("격자판 배치가 보정 당시와 같고 집게와 초기화·이동 경로가 비어 있으면 '실행': ").strip() != "실행":
        return

    arm = LeArmSerial(port, trace=True)
    try:
        approach_from_home(arm, target, reset_observation_pose(arm))
    finally:
        arm.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="격자 v2 좌표(mm)로 집게 접근 시험")
    parser.add_argument("x_mm", type=float, help="격자 왼쪽 아래 원에서 오른쪽 거리")
    parser.add_argument("y_mm", type=float, help="격자 왼쪽 아래 원에서 팔 반대쪽 거리")
    parser.add_argument("--execute", action="store_true", help="초기화 후 실제 접근")
    parser.add_argument("--port", help="로봇팔 보드 포트 (예: COM4)")
    args = parser.parse_args()

    try:
        target = grid_to_arm(args.x_mm, args.y_mm)
    except (ValueError, OSError) as error:
        parser.error(str(error))

    print(f"격자 ({args.x_mm:g}, {args.y_mm:g}) mm → 로봇 명령 "
          f"({target.x}, {target.y}, {target.z}) mm")
    if not args.execute:
        return
    if not args.port:
        parser.error("--execute에는 --port가 필요합니다.")

    try:
        execute_hover(args.port, args.x_mm, args.y_mm)
    except ValueError as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
