"""Supervised fixed-orientation pick trials after a camera-guided hover."""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

from coordinate_pick import Point, Stage, deltas, gripper, packet


@dataclass(frozen=True)
class ObjectProfile:
    name: str
    footprint_mm: tuple[int, int]
    height_mm: int
    grip_width_mm: int
    alignment: str


OBJECTS = {
    "tube": ObjectProfile("휴지심", (40, 40), 95, 40, "세워서 위쪽 테두리 아래의 옆면을 지름 방향으로 가볍게 잡기"),
    "eclipse": ObjectProfile("이클립스 통", (39, 20), 76, 20, "76 mm 방향으로 세우고 20 mm 두께를 가로질러 넓은 두 면 잡기"),
    "cube": ObjectProfile("빨간 큐브", (40, 40), 40, 40, "평평하게 두고 마주 보는 두 면 잡기"),
}


def gripper_open_position(angle_deg: int) -> int:
    """Convert the ESP32 claw_set opening angle to a bus-servo position."""
    if not 0 <= angle_deg <= 90:
        raise ValueError("집게 열림 각도는 0~90° 범위로 입력하세요.")
    return 200 + (90 - angle_deg) * 500 // 90


@dataclass(frozen=True)
class PickSetup:
    descent_mm: int
    wrist_position: int
    open_position: int
    close_position: int
    measured_opening_mm: float

    def validate(self, profile: ObjectProfile) -> None:
        if not 1 <= self.descent_mm <= 100:
            raise ValueError("실측한 하강량은 1~100 mm 범위로 입력하세요.")
        if not 125 <= self.wrist_position <= 875:
            raise ValueError("손목 서보 위치는 125~875 범위로 입력하세요.")
        if not 200 <= self.open_position < self.close_position <= 700:
            raise ValueError("집게 서보 위치는 200 ≤ 열기 < 닫기 ≤ 700이어야 합니다.")
        if not math.isfinite(self.measured_opening_mm) or self.measured_opening_mm < profile.grip_width_mm + 4:
            raise ValueError(f"열린 집게 안쪽 간격이 최소 {profile.grip_width_mm + 4} mm인지 실측하세요.")


def plan_object_pick(hover: Point, profile: ObjectProfile, setup: PickSetup) -> list[Stage]:
    setup.validate(profile)
    # ID 2 is wrist roll. IDs 3–6 handle the board's Cartesian movement.
    wrist = packet(3, struct.pack("<BHBH", 1, 1000, 2, setup.wrist_position))
    contact = Point(hover.x, hover.y, hover.z - setup.descent_mm)
    lift_mm = min(setup.descent_mm, 10)
    lifted = Point(contact.x, contact.y, contact.z + lift_mm)
    descend = deltas(hover, contact, step_mm=5)
    lift = deltas(contact, lifted, step_mm=5)
    return [
        Stage("물체 위 정렬과 여유 높이 확인 후 집게 열기", (gripper(setup.open_position),), 1.2),
        Stage(f"손목 방향 맞추기 · {profile.alignment}", (wrist,), 1.2),
        *[Stage(f"수직 하강 {index}/{len(descend)} · 바닥과 물체 사이 여유 확인", (command,), 1.2)
          for index, command in enumerate(descend, 1)],
        Stage("양쪽 접촉 위치를 확인한 뒤 집게 닫기", (gripper(setup.close_position),), 1.2),
        *[Stage(f"파지를 확인한 뒤 수직 상승 {index}/{len(lift)}", (command,), 1.2)
          for index, command in enumerate(lift, 1)],
    ]
