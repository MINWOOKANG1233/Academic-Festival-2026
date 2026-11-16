# YOLO와 기존 집기 연결

기존 실행 파일은 그대로 보존했습니다. 새 진입점은 robot/yolo_pick.py입니다.

1. setup.cmd로 기존 환경 준비 후 setup_yolo.cmd로 YOLO 설치.
2. CanMV에서 이 폴더의 vision/camera_red_block.py 실행. 빈 배경으로 시작하고 IDE 연결 해제.
3. 팔을 기존 보정 당시 관찰 자세로 놓고 물체 하나를 놓습니다.
4. start_yolo_pick.cmd 실행 후 **카메라** 포트 입력. 기본은 인식만 확인하며 로봇 포트를 열지 않습니다.
5. 출력된 종류·격자 좌표와 experiments/yolo_live/observation.jpg 확인.

실제 집기는 start_yolo_grab.cmd를 실행합니다. 카메라 기본 포트는 COM5이며 로봇팔은 별도 COM 번호를 입력합니다. PowerShell에서는 다음 형태로도 실행할 수 있습니다. COM 번호는 실물에 맞게 변경합니다.

```powershell
.\.venv\Scripts\python.exe .\robot\yolo_pick.py --camera-port COM3 --robot-port COM4 --execute
```

INIT 확인 후 기존 초기 자세 복귀가 실행됩니다. YOLO가 같은 종류·3 mm 이내 위치를 세 사진에서 확인합니다. 사진을 확인한 뒤 PICK 입력하면 재검출해 종류·위치가 유지되는지 검사하고 접근합니다. 실제 위치·방향 확인 후 GRAB을 입력하면 하강·정렬·집기·상승합니다. 들어 올린 결과를 확인한 뒤 RELEASE를 입력하면 원래 자리에 내려놓고 복귀합니다. 다른 입력은 그 자세에서 중단하며 자동 복귀하지 않습니다.

- 클래스 자동 매핑: candy_tin → eclipse / cube_block → cube / tissue_core → tube.
- 기본 확신도 0.6. 물체 상자 여러 개, 화면 잘림, 보정 영역 밖 또는 기존 접근 반경 밖은 거절합니다.
- YOLO 입력은 깨끗한 JPEG 원본입니다. 이동 전 관찰 자세에서만 호모그래피를 적용하고 목표를 고정합니다.
- 접근 후 정렬은 기존 demo_pick의 색상·밝기 기반 사진 보정을 그대로 사용합니다.
- 종이 평면 호모그래피에 물체 상자 중심을 넣으므로 물체 높이에 따른 위치 차이가 남을 수 있습니다. 현재 카메라 각도·받침 위치 및 세워둔 물체 자세에서 먼저 확인해야 합니다.
- 기존 동작처럼 집기 후 **원래 자리에 내려놓고 복귀**합니다. 분류 통 좌표는 측정되지 않아 이 파일에 추가하지 않았습니다.
- 2026-11-16 실제 카메라 COM5에서 세 물체 검출을 확인했고, 사용자가 새 start_yolo_grab.cmd 실행으로 세 물체 모두 집기에 성공했다고 확인했습니다. 단계별 사람 확인이 포함된 통합 집기 성공이며, 반복 성공률 및 지정 통 배치는 별도 검증 대상입니다. 코드와 가상 장비 테스트 39개도 통과했습니다.
- IDE 화면이 USB 전환 메시지 후 멈출 수 있습니다. CanMV IDE가 포트를 계속 점유하면 IDE를 종료한 뒤 PC 프로그램을 실행하세요.
- 원인 구분용 camera_diagnostic_ide.py는 USB 스트리밍을 끈 진단용입니다. YOLO 실행에는 camera_red_block.py를 사용합니다.
