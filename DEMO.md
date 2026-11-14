# Windows LeArm 시연

WonderMV 카메라로 물체의 위치를 찾고 LeArm으로 접근, 정렬, 파지, 상승을 실행합니다. 제어 화면에서 큐브, 이클립스 통, 휴지심 중 하나를 선택합니다. 물체 종류 자동 판별과 분류함 배치는 프로젝트 목표입니다.

## 실행 방법

1. Windows에 Tkinter가 포함된 Python 3을 설치합니다. ZIP을 받았다면 압축을 모두 푼 뒤 `setup.cmd`를 한 번 실행합니다. 최초 설치에는 인터넷이 필요하며, 패키지는 폴더 안의 `.venv`에 설치됩니다.
2. 빈 작업면을 놓고 CanMV IDE에서 WonderMV에 `vision/camera_red_block.py`를 실행합니다. 카메라가 시작할 때 빈 배경을 캡처합니다.
3. `start_cube_control.cmd`를 실행하고 세부 설정에서 카메라와 로봇의 COM 포트를 맞춥니다. 기본값은 각각 COM3과 COM4입니다.
4. 물체를 선택하고 단계별 버튼으로 관측부터 파지까지 확인합니다. 전체 경로를 비운 상태에서 전체 자동 실행을 사용합니다.

## 파일별 역할

| 파일 | 용도 |
| --- | --- |
| `setup.cmd`, `start_cube_control.cmd`, `robot/cube_control.pyw` | Windows 최초 설치, 실행 파일과 제어 화면. COM 포트, 물체 선택, 사진 미리보기와 동작 버튼을 관리합니다. |
| `vision/camera_red_block.py` | WonderMV에서 전경을 검출하고 위치 JSON과 JPEG를 USB로 보냅니다. |
| `robot/demo_pick.py`, `robot/auto_cube_pick.py` | 물체별 사진 검출 결과로 위치를 보정하고 접근, 정렬, 하강, 파지, 상승을 계획합니다. |
| `robot/side_pick.py`, `robot/level_kinematics.py` | 카메라 수신, 집기 단계, 파지 후 이탈 경로를 관리하고 집게의 목표 위치·자세를 관절 명령으로 변환합니다. |
| `robot/camera_hover.py`, `robot/grid_hover.py` | 카메라 검출값을 읽고 작업면 좌표를 변환하며 관측 자세를 초기화합니다. |
| `robot/object_pick.py`, `robot/coordinate_pick.py` | 물체별 설정값과 LeArm 시리얼 통신, 이동 및 집게 명령을 제공합니다. |
| `robot/requirements.txt` | PC용 Python 의존성을 정의합니다. |
| `robot/tests/` | 장비 없이 동작 계획, 좌표 변환, 통신 명령을 검증합니다. |

테스트 실행 명령은 `.venv/Scripts/python.exe -m unittest discover -s robot/tests`입니다.
