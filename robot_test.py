import serial
import time

# 로봇팔과 연결된 COM 포트
robot = serial.Serial(
    port="COM3",
    baudrate=9600,
    timeout=1
)

# 연결 상태 출력
print("로봇팔 연결 성공!")
print("연결된 포트:", robot.port)

# ESP32에서 보내는 데이터 확인
time.sleep(2)

if robot.in_waiting > 0:
    data = robot.read(robot.in_waiting)
    print("받은 데이터:", data)
else:
    print("현재 수신 데이터 없음")

# 통신 종료
robot.close()
print("연결 종료")
