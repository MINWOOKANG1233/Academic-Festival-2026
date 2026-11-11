"""Operator-gated camera, diagonal approach and tilted pick sequence."""

import base64
import binascii
from dataclasses import replace
from enum import Enum, auto
import json
import math
import struct
import time
from io import BytesIO

from PIL import Image

from camera_hover import CAMERA_LINE_LIMIT, capture_target
from coordinate_pick import Point, deltas, gripper
from grid_hover import HOME, reset_observation_pose
from level_kinematics import ToolPose, joint_packet, plan_tool_move, pose_from_joints, solve_joints, tool_waypoints


class Step(Enum):
    HOME = auto()
    DETECT = auto()
    APPROACH = auto()
    LOWER = auto()
    ALIGN = auto()
    CLOSE = auto()
    LIFT = auto()
    RELEASE = auto()
    RETURN = auto()
    STOPPED = auto()


class CameraStream:
    """Read USB camera lines in chunks while preserving the serial reader interface."""

    def __init__(self, port):
        self.port = port
        self.buffer = bytearray()

    def reset_input_buffer(self):
        self.port.reset_input_buffer()
        self.buffer.clear()

    def capture_background(self, check_stop, timeout=12):
        self.reset_input_buffer()
        check_stop()
        self.port.write(b"B")
        deadline = time.monotonic() + timeout
        acknowledged = False
        empty = 0
        while time.monotonic() < deadline:
            check_stop()
            line = self.read_until(size=CAMERA_LINE_LIMIT)
            if line.strip() == b"BACKGROUND OK":
                acknowledged = True
                continue
            if not acknowledged or not line.startswith(b"{"):
                continue
            try:
                data = json.loads(line)
            except (ValueError, UnicodeError):
                continue
            empty = empty + 1 if data.get("status") == "NO BLOB" and data.get("found") is False else 0
            if empty >= 2:
                return
        if acknowledged:
            raise TimeoutError("배경은 촬영했지만 빈 화면이 안정되지 않았어요. 작업면을 비우고 다시 촬영하세요.")
        raise TimeoutError("카메라가 배경 촬영 요청에 응답하지 않았어요. 업데이트한 카메라 코드가 실행 중인지 확인하세요.")

    def wait_ready(self, check_stop, *, require_background=False, timeout=8):
        self.reset_input_buffer()
        deadline = time.monotonic() + timeout
        stream_seen = False
        while time.monotonic() < deadline:
            check_stop()
            if require_background:
                self.port.write(b"P")
            for _ in range(3):
                check_stop()
                line = self.read_until(size=CAMERA_LINE_LIMIT)
                if line.strip() == b"LEARM CAMERA READY":
                    return
                if b'"coordinate_frame"' in line and b'"grid_lower_left"' in line and line.startswith(b"{"):
                    stream_seen = True
                    if not require_background:
                        return
                if time.monotonic() >= deadline:
                    break
        if stream_seen:
            raise TimeoutError("카메라 영상은 들어오지만 배경 촬영 기능이 응답하지 않아요. 최신 카메라 코드를 실행하세요.")
        raise TimeoutError("카메라 데이터가 들어오지 않아요. 카메라 포트와 카메라 코드 실행 상태를 확인하세요.")

    def read_until(self, *, size):
        deadline = time.monotonic() + 1.2
        while True:
            end = self.buffer.find(b"\n", 0, size)
            if end >= 0:
                count = end + 1
                break
            if len(self.buffer) >= size:
                count = size
                break
            if time.monotonic() >= deadline:
                return b""  # Keep a partial line until the next read.
            self.buffer.extend(self.port.read(min(max(self.port.in_waiting, 1), 4096)))
        line = bytes(self.buffer[:count])
        del self.buffer[:count]
        return line


def raw_photo(camera, timeout=12):
    camera.reset_input_buffer()
    deadline = time.monotonic() + timeout
    seen = False
    while time.monotonic() < deadline:
        line = camera.read_until(size=CAMERA_LINE_LIMIT)
        if not (line.startswith(b"JPEG ") and line.endswith(b"\n")):
            continue
        try:
            jpeg = base64.b64decode(line[5:].strip(), validate=True)
        except binascii.Error:
            continue
        if not (jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9")):
            continue
        if seen:  # Discard a frame which may already have been in flight.
            return jpeg
        seen = True
    raise TimeoutError("원본 사진을 못 받았어요. JPEG 출력을 켠 CanMV 코드를 확인하세요.")


class SidePick:
    def __init__(self, camera, arm, profile, *, report, on_photo, cancelled, progress=None):
        self.camera = camera
        self.arm = arm
        self.profile = profile
        self.report = report
        self.on_photo = on_photo
        self.cancelled = cancelled
        self.progress = progress or (lambda _message: None)
        self.step = Step.HOME
        self.joints = None
        self.jog_total = (0, 0, 0)
        self.fast = False

    def check_stop(self):
        if self.cancelled():
            raise InterruptedError("이후 명령 전송을 중단했어요.")

    def require_step(self, *steps):
        if self.step not in steps:
            raise ValueError("현재 단계에서는 이 동작을 실행할 수 없어요.")

    def send(self, command, wait):
        self.check_stop()
        self.arm.write(command)
        time.sleep(wait)
        self.check_stop()
        self.joints = self.arm.read_positions()
        self.report(f"관절값 {self.joints}")
        return self.joints

    def snapshot(self, label):
        self.check_stop()
        self.progress("카메라 사진 받는 중")
        jpeg = raw_photo(self.camera)
        with Image.open(BytesIO(jpeg)) as frame:
            if frame.size != (320, 240):
                raise OSError("카메라 해상도가 보정값과 달라요. 320 × 240으로 맞추세요.")
            frame.load()
        self.on_photo(label, jpeg)
        self.check_stop()
        return jpeg

    def move_tool(self, end, plan=None, *, settle_yaw=False):
        if plan is None:
            plan = plan_tool_move(self.pose, end)
        actual = self.arm.read_positions()
        if any(abs(actual[i] - self.joints[i]) > 12 for i in range(2, 7)):
            raise OSError("직전 관절 자세와 달라졌어요. 현재 위치를 확인하세요.")
        self.report(f"목표 모델 자세 {end}")
        for targets in plan:
            duration, wait = (450, .57) if self.fast else (600, .72)
            actual = self.send(joint_packet(targets, duration), wait)
            if any(abs(actual[i] - goal) > 12 for i, goal in targets.items()):
                time.sleep(.6)
                self.check_stop()
                actual = self.arm.read_positions()
                self.joints = actual
                self.report(f"관절 정착 재확인 {actual}")
            if any(abs(actual[i] - goal) > 12 for i, goal in targets.items()):
                raise OSError("목표 관절값에 도달하지 못했어요. 남은 이동을 중단했어요.")
        if settle_yaw:
            for retry in range(3):
                if abs(actual[6] - targets[6]) <= 3:
                    break
                self.progress(f"밑판 보정 재시도 {retry + 1}/3")
                self.report(f"밑판 목표 {targets[6]} / 응답 {actual[6]} · 같은 목표로 재전송")
                actual = self.send(joint_packet(targets, 650), .85)
            yaw_error = abs(actual[6] - targets[6])
            if yaw_error > 9 or any(abs(actual[i] - goal) > 12 for i, goal in targets.items()):
                raise OSError(f"밑판 보정 목표 {targets[6]} / 실제 {actual[6]} · 관절 오차가 커서 중단했어요.")
            if yaw_error > 3:
                self.pose = pose_from_joints(actual)
                self.report(f"밑판 잔여 오차 {yaw_error} 서보 단위 · 실제 관절 자세로 갱신하고 사진 재확인")
                return
        self.pose = end

    def home(self):
        self.require_step(Step.HOME, Step.RETURN)
        self.progress("초기 자세로 복귀 · 집게 열기")
        self.check_stop()
        self.joints = reset_observation_pose(self.arm)
        self.report(f"관절값 {self.joints}")
        self.step = Step.DETECT

    def capture_background(self):
        self.require_step(Step.DETECT)
        self.progress("빈 배경 촬영 · 카메라 응답 확인")
        self.camera.capture_background(self.check_stop)
        self.snapshot("background")

    def detect(self, forward, left):
        self.require_step(Step.DETECT)
        self.progress("물체 찾기 · 안정된 좌표 확인")
        self.check_stop()
        if max(abs(forward), abs(left)) > 30:
            raise ValueError("초기 XY 보정은 축마다 ±30 mm 이내로 입력하세요.")
        self.detection = capture_target(self.camera, forward, left, report=self.report, limit_hover_radius=False)
        self.snapshot("observe")
        self.step = Step.APPROACH

    def approach(self, standoff, rise):
        self.require_step(Step.APPROACH)
        self.progress("물체 앞으로 대각선 접근")
        if not (15 <= standoff <= 80 and 0 <= rise <= 140):
            raise ValueError("남길 거리는 15~80 mm, 접근 상승량은 0~140 mm로 입력하세요.")
        target = self.detection.point
        radius = math.hypot(target.x, target.y)
        self.direction = (target.x / radius, target.y / radius)
        stop = Point(round(target.x - standoff * self.direction[0]),
                     round(target.y - standoff * self.direction[1]), HOME.z + rise)
        if radius <= standoff or stop.x <= 0:
            raise ValueError("물체 앞에 남길 거리가 부족해요.")
        commands = deltas(HOME, stop, step_mm=20)
        point = HOME
        for command in commands:
            dx, dy, dz = struct.unpack("bbb", command[4:])
            point = Point(point.x + dx, point.y + dy, point.z + dz)
            for pitch in range(-90, 1):
                try:
                    solve_joints(ToolPose(point.x, point.y, point.z, pitch))
                    break
                except ValueError:
                    continue
            else:
                raise ValueError(f"대각선 접근 지점 {point}에 도달할 관절 조합이 없어요.")
        positions = self.joints
        self.report(f"높은 접근 명령 {stop}")
        for command in commands:
            actual = self.send(command, .78 if self.fast else 1.0)
            if max(abs(actual[i] - positions[i]) for i in range(3, 7)) <= 2:
                raise OSError("접근 중 관절 변화가 작아 이동을 멈췄어요.")
            positions = actual
        self.pose = pose_from_joints(self.joints)
        self.jog_total = (0, 0, 0)
        self.report(f"현재 모델 자세 {self.pose}")
        jpeg = self.snapshot("approach")
        self.step = Step.LOWER
        return jpeg

    def lower(self, measured_height, height_trim, opening, pitch, roll):
        self.require_step(Step.LOWER)
        self.progress("파지 높이로 하강")
        if not (-70 <= pitch <= -30 and 125 <= roll <= 875):
            raise ValueError("파지 기울기는 −70~−30°, 손목 회전 서보는 125~875로 입력하세요.")
        grasp_height = self.profile.grasp_height_mm + height_trim
        if not (abs(height_trim) <= 20 and 5 <= grasp_height <= self.profile.height_mm - 3):
            raise ValueError("파지 높이 보정은 ±20 mm 안에서 물체 옆면을 기준으로 입력하세요.")
        descent = measured_height - grasp_height
        if not 0 <= descent <= 160 or measured_height <= self.profile.height_mm:
            raise ValueError("열린 집게 접촉면 높이를 실측하세요. 물체 윗면보다 높아야 해요.")
        if not self.profile.grip_width_mm + 4 <= opening <= 100:
            raise ValueError(f"열린 집게 안쪽 실측 간격이 {self.profile.grip_width_mm + 4} mm 이상인지 확인하세요.")
        high = self.pose
        goal = replace(high, z=high.z - descent, pitch=pitch)
        waypoints = tool_waypoints(high, goal)
        plan = [solve_joints(point) for point in waypoints]
        if self.fast:
            # Check every modeled point, but send fewer commands while high above the grasp.
            # Keep the original spacing for the final 25 mm and always send the last point.
            indices = [i for i, point in enumerate(waypoints)
                       if i % 2 == 1 or point.z - goal.z <= 25 or i == len(waypoints)-1]
            waypoints = [waypoints[i] for i in indices]
            plan = [plan[i] for i in indices]
            self.report(f"자동 하강 이동 명령 {len(plan)}개")
        try:
            solve_joints(high)
            return_pose = high
        except ValueError:
            # The board can settle just outside the PC model's limit at the approach pose.
            # Return to the first validated descent waypoint before the board reset.
            return_pose = waypoints[0]
        actual = self.send(joint_packet({2: roll}), .85)
        if abs(actual[2] - roll) > 12:
            raise OSError("손목 회전이 목표값에 도달하지 못했어요.")
        self.move_tool(goal, plan)
        self.surface_z = high.z - measured_height
        self.high_pose = return_pose
        self.alignment_history = []
        self.jog_total = (0, 0, 0)
        jpeg = self.snapshot("align")
        self.step = Step.ALIGN
        return jpeg

    def jog(self, x, y, z, *, settle_yaw=False, forward_limit=15, along_target=False):
        self.require_step(Step.LOWER, Step.ALIGN)
        if not 0 < max(abs(x), abs(y), abs(z)) <= 10 or sum(v != 0 for v in (x, y, z)) != 1:
            raise ValueError("한 번에 한 축을 1~10 mm 움직일 수 있어요.")
        total = tuple(a + b for a, b in zip(self.jog_total, (x, y, z)))
        if not -15 <= total[0] <= forward_limit or any(abs(value) > 15 for value in total[1:]):
            raise ValueError("정렬 누적 이동 한계를 넘었어요.")
        if self.step is Step.LOWER and z:
            raise ValueError("첫 사진에서는 앞뒤·좌우만 보정하세요.")
        if self.step is Step.ALIGN and not 5 <= self.pose.z + z - self.surface_z <= self.profile.height_mm - 3:
            raise ValueError("미세 이동 후 접촉면이 물체 옆면의 시험 높이를 벗어나요.")
        previous = self.pose
        dx, dy = (x * self.direction[0], x * self.direction[1]) if along_target else (x, 0)
        self.move_tool(replace(previous, x=previous.x + dx, y=previous.y + y + dy, z=previous.z + z),
                       settle_yaw=bool(y) and (self.fast or settle_yaw))
        if self.step is Step.ALIGN:
            self.alignment_history.append(previous)
        self.jog_total = total
        return self.snapshot("approach" if self.step is Step.LOWER else "align")

    def insert(self, distance):
        self.require_step(Step.ALIGN)
        self.progress("전진 경로 확인" if distance else "파지·후퇴 경로 확인")
        if not 0 <= distance <= 30:
            raise ValueError("사진에서 확인한 마지막 전진 거리를 0~30 mm로 입력하세요.")
        x, y = self.direction
        self.aligned_pose = self.pose
        goal = replace(self.pose, x=self.pose.x + distance * x, y=self.pose.y + distance * y)
        # Check the lift and exit before advancing near the object.
        lift = replace(goal, z=goal.z + 10)
        plan_tool_move(goal, lift)
        plan_tool_move(lift, goal)
        plan_tool_move(goal, self.aligned_pose)
        self.exit_plan()
        if distance:
            self.progress("열린 집게로 전진")
            self.move_tool(goal)
            jpeg = self.snapshot("insert")
        else:
            jpeg = None
        self.step = Step.CLOSE
        return jpeg

    def close(self, position):
        self.require_step(Step.CLOSE)
        self.progress("집게 닫기 · 관절 응답 확인")
        if not 201 <= position <= 700:
            raise ValueError("물체에 맞춰 확인한 닫힘 서보값을 201~700 범위로 입력하세요.")
        actual = self.send(gripper(position, 1500), 1.8)
        self.report(f"닫힘 명령 {position} · 실제 {actual[1]} · 양쪽 접촉을 눈으로 확인하세요.")
        self.step = Step.LIFT

    def lift(self):
        self.require_step(Step.LIFT)
        self.progress("물체 들어 올리기")
        self.contact_pose = self.pose
        self.move_tool(replace(self.pose, z=self.pose.z + 10))
        self.step = Step.RELEASE

    def release(self):
        self.require_step(Step.RELEASE)
        self.progress("물체 내려놓기 · 후퇴 경로 확인")
        retreat = self.aligned_pose
        drop_plan = plan_tool_move(self.pose, self.contact_pose)
        retreat_plan = plan_tool_move(self.contact_pose, retreat)
        exit_plan = self.exit_plan()
        self.move_tool(self.contact_pose, drop_plan)
        actual = self.send(gripper(200, 1500), 1.8)
        if abs(actual[1] - 200) > 15:
            raise OSError("집게 열림을 확인하지 못했어요. 후퇴를 중단했어요.")
        self.progress("열린 집게로 후퇴")
        self.move_tool(retreat, retreat_plan)
        for goal, plan in exit_plan:
            self.move_tool(goal, plan)
        self.step = Step.RETURN

    def exit_plan(self):
        start = self.aligned_pose
        path = []
        for goal in (*reversed(self.alignment_history), self.high_pose):
            path.append((goal, plan_tool_move(start, goal)))
            start = goal
        return path
