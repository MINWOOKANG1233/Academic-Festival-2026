"""Bench presets verified with the three demonstration objects."""

from dataclasses import replace
from io import BytesIO
import time

from PIL import Image

from auto_cube_pick import WORKPLANE_Z_MM, align_cube, cube_view
from level_kinematics import plan_tool_move
from object_pick import OBJECTS


GRASP_PRESETS = {
    OBJECTS["cube"]: (30, 0, 5, -70, 425),
    OBJECTS["eclipse"]: (40, 20, 15, -65, 605),
    OBJECTS["tube"]: (50, 35, 20, -63, 420),
}

PHOTO_ALIGNMENT = {
    OBJECTS["cube"]: (151, 136, 13, 10),
    OBJECTS["eclipse"]: (161, 139, 28, 10),
    OBJECTS["tube"]: (142, 133, 12, 8),
}
# Observed top-edge change over successful insertions of 25, 18 and 15 mm.
DEPTH_PIXELS_PER_MM = {
    OBJECTS["cube"]: 2.6,
    OBJECTS["eclipse"]: 2.6,
    OBJECTS["tube"]: 3.0,
}
MAX_SHORT_OF_DEPTH_PX = 5
FINAL_MARGIN_PX = 3
# At the 53 mm alignment pose, the user confirmed clearance in a frame at y=105.
ECLIPSE_SIDE_CLEARANCE_Y = 112


def join_eclipse_fragments(components):
    if len(components) < 2:
        return
    first, second = components[:2]
    left, right, top, bottom = component_bounds(first)
    other_left, other_right, other_top, other_bottom = component_bounds(second)
    overlap = min(right, other_right) - max(left, other_left) + 1
    gap = max(top - other_bottom, other_top - bottom)
    narrower = min(right - left + 1, other_right - other_left + 1)
    if overlap >= .7 * narrower and 0 < gap <= 20:
        first.extend(components.pop(1))


def component_bounds(points):
    xs, ys = zip(*points)
    return min(xs), max(xs), min(ys), max(ys)


def photo_position(profile, jpeg):
    if profile is OBJECTS["cube"]:
        return cube_view(jpeg)
    eclipse = profile is OBJECTS["eclipse"]
    with Image.open(BytesIO(jpeg)) as image:
        if image.size != (320, 240):
            raise ValueError("보정 사진의 크기가 320 × 240이 아니에요.")
        pixels = image.convert("RGB").load()
        mask = set()
        bottom = 195 if profile is OBJECTS["tube"] else 216
        for y in range(40, bottom):
            for x in range(55, 265):
                red, green, blue = pixels[x, y]
                if eclipse:
                    matches = blue-green > 16 and blue-red >= -4 and red-green >= 10
                else:
                    matches = y >= 65 and max(red, green, blue) < 112
                if matches:
                    mask.add((x, y))
    components = []
    while mask:
        part = [mask.pop()]
        for x, y in part:
            for neighbor in ((x-1, y), (x+1, y), (x, y-1), (x, y+1)):
                if neighbor in mask:
                    mask.remove(neighbor)
                    part.append(neighbor)
        if len(part) >= (450 if eclipse else 800):
            components.append(part)
    components.sort(key=len, reverse=True)
    if eclipse:
        join_eclipse_fragments(components)
    minimum_area = 650 if eclipse else 1600
    ambiguous = len(components) > 1 and (not eclipse or len(components[1]) * 3 >= len(components[0]))
    if not components or ambiguous or not minimum_area <= len(components[0]) <= 15500:
        areas = ", ".join(str(len(part)) for part in components[:3]) or "없음"
        raise ValueError(f"{profile.name} 윤곽을 확인하지 못했어요. 검출 면적 {areas} px².")
    object_pixels = components[0]
    left = min(x for x, _ in object_pixels)
    right = max(x for x, _ in object_pixels)
    if left <= 55 or right >= 264 or not (19 if eclipse else 25) <= right-left <= 175:
        raise ValueError("사진에서 물체 양옆이 잘려 있어요. 화면에서 직접 정렬을 확인하세요.")
    top = min(y for _, y in object_pixels)
    if eclipse:
        top_band = {x for x, y in object_pixels if y <= top + 8}
        if top == 40 or len(top_band) < 12:
            raise ValueError("이클립스 통 윗부분을 충분히 확인하지 못했어요.")
    return sum(x for x, _ in object_pixels) / len(object_pixels), top


def photo_center(profile, jpeg):
    return photo_position(profile, jpeg)[0]


def align_demo_object(cycle, jpeg, *, require_clearance=False):
    target, _, tolerance, _ = PHOTO_ALIGNMENT[cycle.profile]
    margin = 0 if cycle.profile is OBJECTS["eclipse"] else FINAL_MARGIN_PX
    previous_error = None
    for attempt in range(6):
        cycle.check_stop()
        error = photo_center(cycle.profile, jpeg) - target
        cycle.report(f"{cycle.profile.name} 사진 좌우 오차 {error:+.1f} px")
        if abs(error) <= tolerance:
            return jpeg
        if abs(error) > 48 or attempt == 5:
            break
        if previous_error is not None and abs(error) > abs(previous_error) + 3:
            break
        remaining = cycle.jog_total[1]
        delta = max(-8, min(8, round(-error / 2.5)))
        if previous_error is not None and error * previous_error > 0 and abs(error - previous_error) < 2:
            # Use a larger correction only when the new image shows little motion.
            boosted = min(10, abs(delta) + 2)
            delta = -boosted if error > 0 else boosted
            cycle.report("직전 좌우 보정 반응이 작아 다음 이동을 최대 2 mm 늘려요.")
        delta = max(-15-remaining, min(15-remaining, delta))
        if not delta:
            break
        if require_clearance and photo_position(cycle.profile, jpeg)[1] > ECLIPSE_SIDE_CLEARANCE_Y:
            raise ValueError("이클립스 통이 집게에 가까워 좌우 이동을 중단했어요.")
        cycle.progress(f"사진으로 자동 좌우 보정 {attempt + 1}/5")
        cycle.report(f"좌우 보정 {delta:+d} mm · 왼쪽이 +")
        jpeg = cycle.jog(0, delta, 0, settle_yaw=True)
        previous_error = error
    if abs(error) <= tolerance + margin:
        cycle.report(f"좌우 잔여 오차 {error:+.1f} px · 최종 허용 범위")
        return jpeg
    raise ValueError("자동 보정 뒤 오차가 남았어요. 사진과 실제 집게 위치를 확인하세요.")


def align_demo_depth(cycle, jpeg):
    target_x, target_y, x_tolerance, y_tolerance = PHOTO_ALIGNMENT[cycle.profile]
    eclipse = cycle.profile is OBJECTS["eclipse"]
    final_x_tolerance = x_tolerance + (0 if eclipse else FINAL_MARGIN_PX)
    forward_limit = 38 if eclipse else 30
    max_moves = 6 if eclipse else 4
    previous_y_error = None
    for attempt in range(max_moves + 1):
        cycle.check_stop()
        for retry in range(3):
            try:
                x, y = photo_position(cycle.profile, jpeg)
                break
            except ValueError as error:
                if retry == 2:
                    raise
                cycle.report(f"{error} 같은 자세에서 다시 촬영해요.")
                jpeg = cycle.snapshot("align")
        error_y = target_y - y
        cycle.report(f"파지 사진 좌우 {x-target_x:+.1f} px · 앞뒤 {error_y:+.1f} px")
        if eclipse and abs(x-target_x) > final_x_tolerance:
            for _ in range(2):
                if y <= ECLIPSE_SIDE_CLEARANCE_Y:
                    break
                retreat = min(8, 15 + cycle.jog_total[0])
                if retreat <= 0:
                    break
                cycle.progress("이클립스 통과 간격 확보 · 열린 집게 후퇴")
                jpeg = cycle.jog(-retreat, 0, 0, forward_limit=forward_limit, along_target=True)
                new_x, new_y = photo_position(cycle.profile, jpeg)
                if new_y >= y - 1:
                    raise ValueError("후퇴 후에도 이클립스 통과 간격이 늘었는지 확인하지 못했어요.")
                x, y = new_x, new_y
                previous_y_error = None
            error_y = target_y - y
        if abs(x-target_x) > x_tolerance:
            jpeg = align_demo_object(cycle, jpeg, require_clearance=eclipse)
            x, y = photo_position(cycle.profile, jpeg)
            error_y = target_y - y
        if abs(x-target_x) <= final_x_tolerance and -y_tolerance <= error_y <= MAX_SHORT_OF_DEPTH_PX:
            return
        if (attempt == max_moves or abs(error_y) > (95 if eclipse else 65)
                or previous_y_error is not None and abs(error_y) >= abs(previous_y_error)-1):
            break
        remaining = cycle.jog_total[0]
        delta = max(-8, min(8, round(error_y / DEPTH_PIXELS_PER_MM[cycle.profile])))
        delta = max(-15-remaining, min(forward_limit-remaining, delta))
        if not delta:
            break
        cycle.progress(f"사진으로 앞뒤 보정 {attempt + 1}/{max_moves}")
        cycle.report(f"앞뒤 보정 {delta:+d} mm · 앞으로가 +")
        jpeg = cycle.jog(delta, 0, 0, forward_limit=forward_limit, along_target=True)
        previous_y_error = error_y
    if abs(x-target_x) <= final_x_tolerance and -y_tolerance - FINAL_MARGIN_PX <= error_y <= MAX_SHORT_OF_DEPTH_PX:
        cycle.report(f"앞뒤 잔여 오차 {error_y:+.1f} px · 최종 허용 범위")
        return
    raise ValueError("사진에서 앞뒤 정렬을 확인하지 못해 집게를 연 채 멈췄어요.")


def demo_approach(cycle):
    standoff, rise, _, _, _ = GRASP_PRESETS[cycle.profile]
    jpeg = cycle.approach(standoff, rise)
    if cycle.profile is OBJECTS["cube"]:
        align_cube(cycle, jpeg, high=True)
    return jpeg


def prepare_demo_pick(cycle):
    demo_approach(cycle)
    return demo_lower(cycle)


def demo_lower(cycle):
    _, _, trim, preferred_pitch, _ = GRASP_PRESETS[cycle.profile]
    height = cycle.profile.grasp_height_mm + trim
    high = cycle.pose
    for pitch in sorted(range(-70, -29), key=lambda angle: abs(angle-preferred_pitch)):
        low = replace(high, z=WORKPLANE_Z_MM+height, pitch=pitch)
        try:
            plan_tool_move(high, low)
            plan_tool_move(low, replace(low, z=low.z+10))
        except ValueError:
            continue
        break
    else:
        raise ValueError("이 위치에서 집고 들어 올릴 자세를 계산하지 못했어요.")
    cycle.report(f"시연 목표 높이 {height:g} mm · 기울기 {pitch}°")
    return cycle.lower(high.z-WORKPLANE_Z_MM, trim, 60, pitch, 497)


def close_demo_object(cycle):
    _, _, _, _, close_position = GRASP_PRESETS[cycle.profile]
    cycle.close(close_position)
    bounds = {OBJECTS["cube"]: (395, 422), OBJECTS["eclipse"]: (520, 590), OBJECTS["tube"]: (375, 418)}
    low, high = bounds[cycle.profile]
    if not low <= cycle.joints[1] <= high:
        raise ValueError("집게 닫힘 응답이 이전 성공 범위와 달라 상승 전에 멈췄어요.")


def lift_demo_object(cycle):
    close_demo_object(cycle)
    cycle.lift()


def run_demo(cycle):
    cycle.fast = cycle.profile is OBJECTS["cube"]
    cycle.detect(0, 0)
    demo_approach(cycle)
    align_demo_depth(cycle, demo_lower(cycle))
    cycle.insert(0)
    lift_demo_object(cycle)
    cycle.progress("들어 올린 자세 유지")
    time.sleep(1)
    cycle.release()
    cycle.home()
