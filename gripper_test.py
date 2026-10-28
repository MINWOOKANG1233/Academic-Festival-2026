
import serial
import time

# 로봇팔 설정
PORT = "COM3"
BAUDRATE = 9600
SERVO_ID = 1

# 서보 위치 읽기
def read_position(robot):
    robot.reset_input_buffer()

    # 전체 서보 위치 요청
    robot.write(bytes([0x55, 0x55, 0x02, 0x0D]))
    robot.flush()

    response = robot.read(22)

    # 응답 형식 확인
    if len(response) != 22:
        print("응답 길이 오류:", response.hex(" "))
        return None

    if response[:4] != bytes([0x55, 0x55, 0x14, 0x0D]):
        print("응답 형식 오류:", response.hex(" "))
        return None

    # ID:1 위치 추출
    for i in range(6):
        start = 4 + i * 3
        servo_id = response[start]
        position = int.from_bytes(
            response[start + 1:start + 3],
            "little"
        )

        if servo_id == SERVO_ID:
            return position

    return None


# 집게 이동 함수
def move_gripper(robot, position):
    duration = 1000

    command = bytes([
        0x55, 0x55,      # 패킷 시작
        0x08,            # 데이터 길이
        0x03,            # 서보 이동 명령
        0x01,            # 서보 개수
        duration & 0xFF,
        duration >> 8,
        SERVO_ID,        # ID:1 집게
        position & 0xFF,
        position >> 8
    ])

    robot.write(command)
    robot.flush()


# 메인 프로그램
with serial.Serial(PORT, BAUDRATE, timeout=2) as robot:
    time.sleep(2)

    # 1. 현재 위치 확인
    original = read_position(robot)
    print("현재 집게 위치:", original)

    if original is None:
        print("위치를 읽지 못해 이동을 중단합니다.")

    else:
        # 현재 위치에서 15만큼 이동
        target = original + 50

        if target > 950:
            print("목표 위치가 안전 테스트 범위를 벗어납니다.")

        else:
            # 2. 집게 이동
            print("목표 위치:", target)
            move_gripper(robot, target)
            time.sleep(2)

            # 3. 이동 후 위치 확인
            after = read_position(robot)
            print("이동 후 실제 위치:", after)

            # 4. 원래 위치로 복귀
            if after is not None:
                move_gripper(robot, original)
                time.sleep(2)

                final = read_position(robot)
                print("복귀 후 실제 위치:", final)

            else:
                print("응답이 없어 자동 복귀를 생략합니다.")

print("테스트 종료")
