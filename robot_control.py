
import serial
import time

# 로봇팔 연결
robot = serial.Serial("COM3", 9600, timeout=2)

# ESP32가 통신을 준비할 시간
time.sleep(2)

# 이전에 들어온 데이터 비우기
robot.reset_input_buffer()

# 현재 서보모터 위치 읽기 명령
command = bytes([0x55, 0x55, 0x02, 0x0D])

# 로봇팔에 명령 전송
robot.write(command)

# 응답 받기
response = robot.read(22)

print("응답 데이터:", response.hex(" "))

# 응답이 올바른지 확인
if (
    len(response) == 22
    and response[:4] == bytes([0x55, 0x55, 0x14, 0x0D])
):
    # 각 서보모터의 현재 위치 출력
    for i in range(6):
        start = 4 + i * 3
        servo_id = response[start]
        position = int.from_bytes(
            response[start + 1:start + 3],
            "little"
        )
        print(f"ID:{servo_id} 현재 위치: {position}")
else:
    print("정상적인 위치 응답을 받지 못했습니다.")

robot.close()
