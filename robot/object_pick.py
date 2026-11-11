"""Supervised fixed-orientation pick trials after a camera-guided hover."""
from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass

from coordinate_pick import LeArmSerial, Point, Stage, deltas, gripper, packet


@dataclass(frozen=True)
class ObjectProfile:
    name: str
    footprint_mm: tuple[int, int]
    height_mm: int
    grip_width_mm: int
    alignment: str

    @property
    def grasp_height_mm(self) -> float:
        return self.height_mm / 2

    def midpoint_descent_mm(self, hover_contact_height_mm: float) -> int:
        if not math.isfinite(hover_contact_height_mm) or hover_contact_height_mm <= self.height_mm:
            raise ValueError("접근 자세에서 집게 접촉면이 물체 윗면보다 높아야 합니다.")
        return round(hover_contact_height_mm - self.grasp_height_mm)


OBJECTS = {
    "tube": ObjectProfile("휴지심", (40, 40), 95, 40, "세운 원통의 중간 높이에서 지름 방향으로 가볍게 잡기"),
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


def vertical_stages(label: str, start: Point, end: Point, *, one_pass: bool) -> list[Stage]:
    commands = deltas(start, end, step_mm=1 if one_pass else 5)
    group_size = 10 if one_pass else 1
    wait = 0.08 if one_pass else 1.2
    return [Stage(label, commands[index:index + group_size], wait)
            for index in range(0, len(commands), group_size)]


def plan_object_pick(hover: Point, profile: ObjectProfile, setup: PickSetup, *,
                     one_pass: bool = False) -> list[Stage]:
    setup.validate(profile)
    # ID 2 is wrist roll. IDs 3–6 handle the board's Cartesian movement.
    wrist = packet(3, struct.pack("<BHBH", 1, 1000, 2, setup.wrist_position))
    contact = Point(hover.x, hover.y, hover.z - setup.descent_mm)
    lift_mm = min(setup.descent_mm, 10)
    lifted = Point(contact.x, contact.y, contact.z + lift_mm)
    return [
        Stage("물체 위 정렬과 여유 높이 확인 후 집게 열기", (gripper(setup.open_position),), 1.2),
        Stage(f"손목 방향 맞추기 · {profile.alignment}", (wrist,), 1.2),
        *vertical_stages("수직 하강 · 바닥과 물체 사이 여유 확인", hover, contact, one_pass=one_pass),
        Stage("양쪽 접촉 위치를 확인한 뒤 집게 닫기", (gripper(setup.close_position),), 1.2),
        *vertical_stages("수직으로 조금 들어 올리기", contact, lifted, one_pass=one_pass),
    ]


def plan_release_and_clear(hover: Point, setup: PickSetup) -> list[Stage]:
    contact = Point(hover.x, hover.y, hover.z - setup.descent_mm)
    held = Point(contact.x, contact.y, contact.z + min(setup.descent_mm, 10))
    return [
        *vertical_stages("잡았던 높이까지 내려놓기", held, contact, one_pass=True),
        Stage("집게 열기", (gripper(setup.open_position, duration_ms=1500),), 1.8),
        *vertical_stages("물체 위까지 수직 상승", contact, hover, one_pass=True),
    ]


def execute_pick_in_one_pass(arm: LeArmSerial, stages: list[Stage], *, report=None) -> None:
    """Run the prechecked pick plan with joint feedback and no stage prompts."""
    previous = arm.read_positions()
    for stage in stages:
        if report is not None:
            report(stage.label)
        for command in stage.packets:
            arm.write(command)
            time.sleep(stage.wait)
        current = arm.read_positions()
        if report is not None:
            report(f"관절값: {current}")
        command = stage.packets[-1]
        if command[3] == 4:
            change = max(abs(current[i] - previous[i]) for i in (3, 4, 5))
            # A one- or two-millimetre remainder may fall within servo play.
            if change > 40 or (len(stage.packets) >= 3 and change <= 2):
                raise OSError(f"{stage.label}: 관절 움직임을 확인하지 못해 집기 순서를 멈췄습니다.")
        else:
            _, _, servo_id, target = struct.unpack("<BHBH", command[4:])
            if abs(current[servo_id] - target) > 12:
                raise OSError(f"{stage.label}: 서보가 확인한 위치에 도달하지 않아 멈췄습니다.")
        previous = current
