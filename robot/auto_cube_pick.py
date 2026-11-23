"""One-button cube trial for the measured workplane and camera setup."""

from dataclasses import replace
from io import BytesIO
from statistics import median

from PIL import Image

from level_kinematics import plan_tool_move
from object_pick import OBJECTS
from side_pick import Step


# Cube trial: 95 mm above the workplane at modeled Z 9.33 mm.
WORKPLANE_Z_MM = -85.67
ALIGN_X_PX = 146
PIXELS_PER_MM = 2.1  # Initial correction estimate. Each move is verified with another frame.


def cube_view(jpeg, *, allow_bottom_crop=False):
    with Image.open(BytesIO(jpeg)) as frame:
        rgb = frame.convert("RGB")
        pixels = [(x, y) for y in range(240) for x in range(320)
                  if (lambda r, g, b: (
                      r-g > 32 and (b-g > 12 or r-b > 20 or allow_bottom_crop)
                      or allow_bottom_crop and r >= 170 and r-g >= 14 and r-b >= 7
                  ))(*rgb.getpixel((x, y)))]
    points = [(x, y) for x, y in pixels if 65 <= x <= 255]
    bottom_crop = (allow_bottom_crop and len(points) >= 600
                   and sum(y >= 236 for _, y in points) >= 20
                   and len(points) >= .8 * len(pixels))
    if not bottom_crop and not (1800 <= len(points) <= 23000 and len(points) >= .85 * len(pixels)):
        raise ValueError("큐브가 사진에서 안정적으로 보이지 않아 자동 파지를 중단했어요.")
    if bottom_crop:
        left, right = min(x for x, _ in points), max(x for x, _ in points)
        if left == 65 or right == 255 or right - left < 44:
            raise ValueError("높은 자세에서 큐브 양옆을 확인하지 못했어요.")
        center = (left + right) / 2
    else:
        center = sum(x for x, _ in points) / len(points)
    def bright_top(offset):
        columns = range(round(center + offset), round(center + offset + 10))
        edges = []
        for x in columns:
            for y in range(240):
                r, g, b = rgb.getpixel((x, y))
                if r > 150 and r-g > 15 and b-g > 4:
                    edges.append(y)
                    break
        return edges

    left_edge = bright_top(-22)
    right_edge = bright_top(12)
    if allow_bottom_crop or len(left_edge) < 4 or len(right_edge) < 4:
        tops = {}
        mask = set(points)
        for x, y in points:
            if allow_bottom_crop and not all((x, y + dy) in mask for dy in range(8)):
                continue
            tops[x] = min(y, tops.get(x, y))

        def color_top(start):
            return [tops[x]
                    for x in range(round(center + start), round(center + start + 10))
                    if x in tops]

        left_edge, right_edge = color_top(-22), color_top(12)
        if allow_bottom_crop and tops:
            columns = sorted(tops)
            fifth = max(1, len(columns) // 5)
            left_edge = [tops[x] for x in columns[fifth:2*fifth]]
            right_edge = [tops[x] for x in columns[-2*fifth:-fifth]]
    if len(left_edge) < 4 or len(right_edge) < 4:
        raise ValueError("큐브 윗변을 충분히 확인하지 못했어요.")
    if abs(median(left_edge)-median(right_edge)) > (23 if allow_bottom_crop else 18):
        raise ValueError("큐브의 변이 집게 방향과 비스듬해 자동 파지를 중단했어요.")
    top = round(median(left_edge + right_edge)) if allow_bottom_crop else min(y for _, y in points)
    return center, top


def align_cube(cycle, jpeg, *, high):
    label = "approach" if high else "align"
    phase = "높은 자세" if high else "파지 높이"
    top_min, top_max = (125, 225) if high else (45, 115)
    tolerance = 12 if high else 7
    scale = 1.5 if high else PIXELS_PER_MM
    max_moves = 5 if high else 3
    total = 0
    previous_error = None
    previous_delta = None
    stalled = 0
    for attempt in range(max_moves + 1):
        for retry in range(3):
            try:
                center, top = cube_view(jpeg, allow_bottom_crop=high)
                break
            except ValueError as error:
                if retry == 2:
                    raise
                cycle.report(f"{error} 같은 자세에서 다시 촬영해요.")
                jpeg = cycle.snapshot(label)
        error = center - ALIGN_X_PX
        if previous_error is not None and abs(error) >= abs(previous_error)-1:
            # Give a slow joint one fresh frame to settle before judging the move.
            center, top = cube_view(cycle.snapshot(label), allow_bottom_crop=high)
            error = center - ALIGN_X_PX
        cycle.report(f"{phase} 사진 좌우 오차 {error:+.1f} px · 큐브 윗변 y={top} px")
        if not top_min <= top <= top_max or abs(error) > 52:
            raise ValueError(f"{phase}에서 큐브가 사진 시험 범위를 벗어났어요.")
        if abs(error) <= tolerance:
            return
        if previous_delta is not None:
            measured_scale = (error - previous_error) / previous_delta
            if 1 <= measured_scale <= 5:
                scale = measured_scale
        if previous_error is not None and abs(error) >= abs(previous_error)-1:
            stalled += 1
        else:
            stalled = 0
        if attempt == max_moves or stalled >= 2:
            raise ValueError(f"{phase}에서 재촬영해도 좌우 정렬이 확인되지 않았어요.")
        delta = max(-10, min(10, round(-error / scale)))
        if not high:
            delta = max(-15-total, min(15-total, delta))
        if not delta:
            raise ValueError(f"{phase}에서 자동 좌우 보정 한계에 도달했어요.")
        total += delta
        previous_error = error
        previous_delta = delta
        cycle.progress(f"{phase} 자동 좌우 보정 {attempt + 1}/{max_moves}")
        cycle.report(f"{phase} 좌우 보정 명령 {delta:+d} mm · 왼쪽이 +")
        jpeg = cycle.jog(0, delta, 0)


def auto_pick_cube(cycle):
    cycle.require_step(Step.DETECT)
    if cycle.profile is not OBJECTS["cube"]:
        raise ValueError("현재 자동 파지는 큐브용이에요.")
    cycle.fast = True
    cycle.detect(0, 0)
    align_cube(cycle, cycle.approach(30, 0), high=True)
    dx, dy = cycle.direction
    for pitch in range(-70, -29):
        low = replace(cycle.pose, z=WORKPLANE_Z_MM + 25, pitch=pitch)
        inserted = replace(low, x=low.x+25*dx, y=low.y+25*dy)
        try:
            plan_tool_move(cycle.pose, low)
            plan_tool_move(low, inserted)
            plan_tool_move(inserted, replace(inserted, z=inserted.z+10))
        except ValueError:
            continue
        break
    else:
        raise ValueError("이 위치에서 하강·전진·상승을 이어갈 파지 기울기를 찾지 못했어요.")
    cycle.report(f"현재 위치에서 계산된 파지 기울기 {pitch}°")
    align_cube(cycle, cycle.lower(cycle.pose.z-WORKPLANE_Z_MM, 5, 60, pitch, 497), high=False)
    center, top = cube_view(cycle.insert(25))
    cycle.report(f"전진 뒤 좌우 오차 {center-151:+.1f} px · 큐브 윗변 y={top} px")
    if abs(center-151) > 13 or not 92 <= top <= 154:
        raise ValueError("전진 뒤 사진이 닫힘 범위를 벗어났어요. 집게를 연 채 멈췄어요.")
    cycle.close(425)
    if not 395 <= cycle.joints[1] <= 422:
        raise ValueError("집게 닫힘 응답이 접촉 시험 범위를 벗어났어요. 상승 전에 멈췄어요.")
    cycle.lift()
