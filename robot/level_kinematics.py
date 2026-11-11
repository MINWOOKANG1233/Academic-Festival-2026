"""LeArm bus-servo geometry for trajectories with an explicit tool pitch."""

from dataclasses import dataclass
import math
import struct

from coordinate_pick import packet


# Link lengths from Hiwonder's LeArm kinematics source, converted from cm.
BASE_HEIGHT, UPPER_ARM, FOREARM, TOOL = 28.9, 104.3, 89.0, 177.0
DEGREES_PER_UNIT = 0.24


@dataclass(frozen=True)
class ToolPose:
    x: float
    y: float
    z: float
    pitch: float


def pose_from_joints(joints: dict[int, int]) -> ToolPose:
    yaw, shoulder, elbow, wrist = (
        math.radians(value * DEGREES_PER_UNIT)
        for value in (joints[6] - 500, 875 - joints[5], joints[4] - 500, joints[3] - 500)
    )
    pitch = shoulder + elbow + wrist
    radius = UPPER_ARM * math.cos(shoulder) + FOREARM * math.cos(shoulder + elbow) + TOOL * math.cos(pitch)
    z = BASE_HEIGHT + UPPER_ARM * math.sin(shoulder) + FOREARM * math.sin(shoulder + elbow) + TOOL * math.sin(pitch)
    return ToolPose(radius * math.cos(yaw), radius * math.sin(yaw), z, math.degrees(pitch))


def solve_joints(pose: ToolPose) -> dict[int, int]:
    if not all(map(math.isfinite, (pose.x, pose.y, pose.z, pose.pitch))) or pose.x <= 0:
        raise ValueError("팔 앞쪽의 유효한 좌표를 입력하세요.")
    pitch = math.radians(pose.pitch)
    r = math.hypot(pose.x, pose.y) - TOOL * math.cos(pitch)
    z = pose.z - BASE_HEIGHT - TOOL * math.sin(pitch)
    cosine = (r*r + z*z - UPPER_ARM**2 - FOREARM**2) / (2 * UPPER_ARM * FOREARM)
    if not -1 <= cosine <= 1:
        raise ValueError("이 위치와 각도에 도달할 수 없어요.")
    elbow = -math.acos(cosine)
    shoulder = math.atan2(z, r) - math.atan2(FOREARM * math.sin(elbow), UPPER_ARM + FOREARM * cosine)
    yaw, shoulder, elbow, wrist = map(math.degrees, (
        math.atan2(pose.y, pose.x), shoulder, elbow, pitch - shoulder - elbow,
    ))
    if not (abs(yaw) <= 90 and 0 <= shoulder <= 180 and -90 <= elbow <= 0 and abs(wrist) <= 90):
        raise ValueError("이 위치와 집게 각도의 조합은 관절 범위를 벗어나요.")
    return {6: round(500 + yaw / DEGREES_PER_UNIT), 5: round(875 - shoulder / DEGREES_PER_UNIT),
            4: round(500 + elbow / DEGREES_PER_UNIT), 3: round(500 + wrist / DEGREES_PER_UNIT)}


def tool_waypoints(start: ToolPose, end: ToolPose) -> list[ToolPose]:
    start_values = (start.x, start.y, start.z, start.pitch)
    change = [b - a for a, b in zip(start_values, (end.x, end.y, end.z, end.pitch))]
    steps = max(1, math.ceil(max(math.hypot(*change[:3]) / 6, abs(change[3]) / 6)))
    return [
        ToolPose(*(a + delta * step / steps for a, delta in zip(start_values, change)))
        for step in range(1, steps + 1)
    ]


def plan_tool_move(start: ToolPose, end: ToolPose) -> list[dict[int, int]]:
    return [solve_joints(pose) for pose in tool_waypoints(start, end)]


def joint_packet(joints: dict[int, int], duration_ms: int = 600) -> bytes:
    payload = struct.pack("<BH", len(joints), duration_ms)
    payload += b"".join(struct.pack("<BH", i, value) for i, value in sorted(joints.items()))
    return packet(3, payload)
