"""LeArm AI stock firmware: plan a pick in mm, execute with operator checks.

No motion at import or by default. The stock 0x04 command has no position ACK;
software coordinates below are commanded estimates, never measured XYZ.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import struct
import time


def packet(command: int, payload: bytes = b"") -> bytes:
    return b"\x55\x55" + bytes((len(payload) + 2, command)) + payload


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Stage:
    label: str
    packets: tuple[bytes, ...]
    wait: float


def grid_point(point: Point) -> None:
    for value in (point.x, point.y, point.z):
        if not math.isfinite(value) or value != round(value):
            raise ValueError("좌표는 유한한 정수 mm로 입력하세요.")


def deltas(start: Point, end: Point, step_mm: int = 5) -> tuple[bytes, ...]:
    grid_point(start)
    grid_point(end)
    if not 1 <= step_mm <= 127:
        raise ValueError("이동 간격은 1~127 mm여야 합니다.")
    distance = tuple(int(b - a) for a, b in zip(vars(start).values(), vars(end).values()))
    count = max(1, math.ceil(max(map(abs, distance)) / step_mm))
    previous = (0, 0, 0)
    result = []
    for index in range(1, count + 1):
        current = tuple(round(v * index / count) for v in distance)
        delta = tuple(a - b for a, b in zip(current, previous))
        if any(delta):
            result.append(packet(4, struct.pack("bbb", *delta)))
        previous = current
    return tuple(result)


def gripper(position: int, duration_ms: int = 1000) -> bytes:
    if not 200 <= position <= 700:
        raise ValueError("버스 서보 집게 위치는 공식 범위 200~700 안이어야 합니다.")
    return packet(3, bytes((1,)) + struct.pack("<HBH", duration_ms, 1, position))


def return_stages(target: Point, travel_z: int, lift_mm: int,
                  initial: dict[int, int]) -> list[Stage]:
    """Return via the elevated route, then restore the captured joint posture.

    Joint restoration also restores the gripper; use only for an EMPTY pick test.
    """
    if set(initial) != set(range(1, 7)) or any(not 0 <= p <= 1000 for p in initial.values()):
        raise ValueError("복귀용 관절 위치 기록이 올바르지 않습니다.")
    lifted = Point(target.x, target.y, target.z + lift_mm)
    above = Point(target.x, target.y, travel_z)
    home_above = Point(150, 0, travel_z)
    home = Point(150, 0, 20)
    payload = bytes((6,)) + struct.pack("<H", 2000)
    for servo_id in range(1, 7):
        payload += struct.pack("<BH", servo_id, initial[servo_id])
    return [
        Stage("빈 집게 시험 복귀: 목표 위로 상승", deltas(lifted, above), .25),
        Stage("빈 집게 시험 복귀: 기준점 위로 이동", deltas(above, home_above), .25),
        Stage("빈 집게 시험 복귀: 기준 자세 높이로 하강", deltas(home_above, home), .25),
        Stage("시험 직전의 관절/집게 자세 복원", (packet(3, payload),), 2.5),
    ]


def plan_pick(target: Point, close_position: int, travel_z: int = 100,
              lift_mm: int = 50, open_position: int = 200) -> list[Stage]:
    grid_point(target)
    # Conservative software envelope, NOT collision detection or an IK solver.
    if not (80 <= target.x <= 250 and -120 <= target.y <= 120 and 20 <= target.z <= 150):
        raise ValueError("초기 시험 범위: X 80~250, Y -120~120, Z 20~150 mm.")
    if not 10 <= lift_mm <= 80 or not max(80, target.z + lift_mm) <= travel_z <= 200:
        raise ValueError("이동 높이는 80~200 mm이며 목표 Z+들어올림 높이 이상이어야 합니다.")
    if not open_position < close_position:
        raise ValueError("닫는 위치는 여는 위치보다 커야 합니다. 실제 물체에 맞게 조정하세요.")
    gripper(close_position)
    reset = Point(150, 0, 20)  # DEFAULT_X/Y/Z from official stock firmware (cm -> mm)
    elevated = Point(150, 0, travel_z)
    approach = Point(target.x, target.y, travel_z)
    lifted = Point(target.x, target.y, target.z + lift_mm)
    return [
        Stage("기준 자세 초기화: (150, 0, 20) mm / 집게와 모든 관절이 움직입니다", (packet(12),), 1.5),
        Stage(f"기준점에서 먼저 상승: {elevated}", deltas(reset, elevated), .25),
        Stage(f"목표 위로 이동: {approach}", deltas(elevated, approach), .25),
        Stage(f"집게 열기: {open_position}", (gripper(open_position),), 1.2),
        Stage(f"목표 높이로 내려가기: {target}", deltas(approach, target), .25),
        Stage(f"집게 닫기: {close_position}", (gripper(close_position),), 1.2),
        Stage(f"들어 올리기: {lifted}", deltas(target, lifted), .25),
    ]


class LeArmSerial:
    def __init__(self, port: str, trace: bool = False):
        import serial
        self.trace = trace
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = 9600
        self.serial.timeout = .2
        self.serial.write_timeout = 1
        self.serial.dtr = False
        self.serial.rts = False
        self.serial.open()
        time.sleep(2)

    def close(self):
        self.serial.close()

    def write(self, data: bytes):
        if getattr(self, "trace", False):
            print(f"[{time.strftime('%H:%M:%S')}] TX PC → 로봇팔: {data.hex(' ')}", flush=True)
        if self.serial.write(data) != len(data):
            raise IOError("명령이 일부만 전송됐습니다. 실행 중단; 기준 위치 재확인 필요.")
        self.serial.flush()

    def query(self, command: int, payload_size: int) -> bytes:
        self.serial.reset_input_buffer()
        self.write(packet(command))
        deadline = time.monotonic() + 3
        buffer = bytearray()
        while time.monotonic() < deadline:
            buffer.extend(self.serial.read(64))
            while len(buffer) >= 4:
                offset = buffer.find(b"\x55\x55")
                if offset < 0:
                    buffer[:] = buffer[-1:]
                    break
                del buffer[:offset]
                if len(buffer) < 4:
                    break
                length = buffer[2]
                if not 2 <= length <= 64:
                    del buffer[0]
                    continue
                if len(buffer) < length + 2:
                    break
                frame = bytes(buffer[:length + 2])
                del buffer[:length + 2]
                if getattr(self, "trace", False):
                    print(f"[{time.strftime('%H:%M:%S')}] RX 로봇팔 → PC: {frame.hex(' ')}", flush=True)
                if frame[3] == command and length == payload_size + 2:
                    return frame[4:]
        raise TimeoutError(f"0x{command:02X} 정상 응답 없음. 포트/펌웨어/다른 제어 앱을 확인하세요.")

    def inspect(self) -> int:
        # Official parser emits command + 2 bytes; resulting length is 4.
        version = self.query(1, 2)
        kind = version[0]
        if kind not in (1, 2):
            raise ValueError(f"알 수 없는 서보 종류: {kind}")
        print(f"서보 종류: {kind} ({'버스' if kind == 2 else 'PWM'}), 펌웨어: {version[1]}")
        positions = self.read_positions()
        self.initial_positions = positions
        print("서보 위치:", positions)
        return kind

    def read_positions(self) -> dict[int, int]:
        raw = self.query(13, 18)
        positions = dict(struct.unpack_from("<BH", raw, offset) for offset in range(0, 18, 3))
        if len(positions) != 6 or set(positions) != set(range(1, 7)):
            raise ValueError("서보 1~6 위치 응답이 올바르지 않습니다.")
        return positions


def execute_stages(robot: LeArmSerial, stages: list[Stage]) -> None:
    print("좌표 명령은 성공 응답이 없습니다. 부저/미이동/예상 밖 움직임 시 다음 단계로 진행하지 마세요.")
    print("이 프로그램은 충돌 회피를 하지 않습니다. 중단은 이후 명령만 멈추며 이미 보낸 동작은 계속될 수 있습니다.")
    for stage in stages:
        print("\n다음:", stage.label)
        if input("경로와 이전 동작을 확인했다면 실행 입력 (그 외 중단): ").strip() != "실행":
            raise InterruptedError("사용자가 다음 단계 전송을 중단했습니다.")
        for data in stage.packets:
            robot.write(data)
            time.sleep(stage.wait)
        print("현재 관절 위치:", robot.read_positions(), flush=True)
    print("계획한 명령 전송 완료. 실제 물체 집기 성공 여부는 직접 확인하세요.")


def main():
    parser = argparse.ArgumentParser(description="LeArm AI 좌표 집기: 기본은 계산/출력만, 단위 mm")
    parser.add_argument("x", nargs="?", type=float)
    parser.add_argument("y", nargs="?", type=float)
    parser.add_argument("z", nargs="?", type=float)
    parser.add_argument("--port", help="로봇팔 보드 포트. 카메라 포트와 구분")
    parser.add_argument("--inspect", action="store_true", help="버전/서보 위치 조회만; 이동 명령 없음")
    parser.add_argument("--trace", action="store_true", help="보낸 명령(TX)과 받은 응답(RX)을 표시")
    parser.add_argument("--execute", action="store_true", help="단계별 확인 후 실제 이동")
    parser.add_argument("--return-start", action="store_true", help="빈 집게 시험 전용: 시작 관절 자세로 복귀")
    parser.add_argument("--close-position", type=int, help="물체에 맞춰 확인한 버스 집게 닫힘 위치")
    parser.add_argument("--open-position", type=int, default=200)
    parser.add_argument("--travel-z", type=int, default=100)
    parser.add_argument("--lift-mm", type=int, default=50)
    args = parser.parse_args()
    robot = None
    try:
        if args.inspect and args.execute:
            raise ValueError("조회와 실제 실행은 따로 사용하세요.")
        stages = None
        if not args.inspect:
            if None in (args.x, args.y, args.z, args.close_position):
                raise ValueError("X Y Z와 --close-position을 입력하세요. 연결 확인은 --inspect --port COM번호.")
            stages = plan_pick(Point(args.x, args.y, args.z), args.close_position,
                               args.travel_z, args.lift_mm, args.open_position)
            for index, stage in enumerate(stages, 1):
                print(f"{index}. {stage.label} / {len(stage.packets)}개 명령")
            if not args.execute:
                print("미리보기 완료. 포트를 열거나 팔을 움직이지 않았습니다.")
                return
        if not args.port:
            raise ValueError("로봇팔 보드의 --port를 지정하세요.")
        robot = LeArmSerial(args.port, trace=args.trace)
        kind = robot.inspect()
        if args.execute:
            if kind != 2:
                raise ValueError("이 버전은 버스 서보 전용입니다. PWM 버전 실행을 중단합니다.")
            if args.return_start:
                print("복귀를 위해 저장한 시작 자세:", robot.initial_positions, flush=True)
                stages += return_stages(Point(args.x, args.y, args.z), args.travel_z,
                                        args.lift_mm, robot.initial_positions)
            execute_stages(robot, stages)
    except (Exception, KeyboardInterrupt) as error:
        print(f"중단: {error}")
        raise SystemExit(1)
    finally:
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    main()
