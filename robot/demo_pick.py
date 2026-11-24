"""Bench presets verified with the three demonstration objects."""

from dataclasses import replace
from io import BytesIO
import time

from PIL import Image

from auto_cube_pick import WORKPLANE_Z_MM, align_cube, cube_view
from level_kinematics import plan_tool_move
from object_pick import OBJECTS
from side_pick import AlignmentPathError, Step


GRASP_PRESETS = {
    OBJECTS["cube"]: (40, 0, 5, -70, 425),
    OBJECTS["eclipse"]: (40, 20, 15, -65, 605),
    OBJECTS["tube"]: (50, 35, 30, -63, 420),
}

PHOTO_ALIGNMENT = {
    OBJECTS["cube"]: (151, 136, 13, 10),
    OBJECTS["eclipse"]: (161, 139, 16, 10),
    OBJECTS["tube"]: (149, 142, 12, 8),
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
    vertical_overlap = min(bottom, other_bottom) - max(top, other_top) + 1
    shorter = min(bottom - top + 1, other_bottom - other_top + 1)
    side_gap = max(left - other_right, other_left - right)
    split_by_highlight = (side_gap <= 9 and vertical_overlap >= .8 * shorter
                          and abs(bottom - other_bottom) <= 8
                          and max(right, other_right) - min(left, other_left) <= 95)
    if (overlap >= .7 * narrower and 0 < gap <= 20) or split_by_highlight:
        first.extend(components.pop(1))


def component_bounds(points):
    xs, ys = zip(*points)
    return min(xs), max(xs), min(ys), max(ys)


def photo_position(profile, jpeg, *, high=False, bright_tube=False):
    if profile is OBJECTS["cube"]:
        return cube_view(jpeg, allow_bottom_crop=True)
    if profile is OBJECTS["tube"] and not high and not bright_tube:
        return photo_position(profile, jpeg, bright_tube=True)
    eclipse = profile is OBJECTS["eclipse"]
    with Image.open(BytesIO(jpeg)) as image:
        if image.size != (320, 240):
            raise ValueError("보정 사진의 크기가 320 × 240이 아니에요.")
        pixels = image.convert("RGB").load()
        mask = set()
        bottom = 195 if profile is OBJECTS["tube"] else 216
        if high or bright_tube:
            bottom = 240
        left_limit, right_limit = (5, 315) if not eclipse and (high or bright_tube) else (55, 265)
        for y in range(40, bottom):
            for x in range(left_limit, right_limit):
                red, green, blue = pixels[x, y]
                if eclipse:
                    matches = (blue-green > 16 and blue-red >= -4 and red-green >= 10
                               or min(red, green, blue) >= 165 and blue-green >= 8
                               and red-green >= 6 and blue-red >= -12)
                else:
                    matches = (min(red, green, blue) >= (130 if high else 175) if high or bright_tube
                               else y >= 65 and max(red, green, blue) < 112)
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
    if (high or bright_tube) and not eclipse:
        components = [part for part in components if
                      component_bounds(part)[0] > left_limit and component_bounds(part)[1] < right_limit-1
                      and 25 <= component_bounds(part)[1]-component_bounds(part)[0] <= 175
                      and component_bounds(part)[2] > 40]
    components.sort(key=len, reverse=True)
    if eclipse:
        join_eclipse_fragments(components)
    minimum_area = 650 if eclipse or bright_tube else 1600
    ambiguous = len(components) > 1 and (
        len(components[1]) * 2 >= len(components[0]) if bright_tube
        else not eclipse or len(components[1]) * 3 >= len(components[0])
    )
    if not components or ambiguous or not minimum_area <= len(components[0]) <= 15500:
        if not eclipse and not high and not bright_tube:
            return photo_position(profile, jpeg, bright_tube=True)
        areas = ", ".join(str(len(part)) for part in components[:3]) or "없음"
        raise ValueError(f"{profile.name} 윤곽을 확인하지 못했어요. 검출 면적 {areas} px².")
    object_pixels = components[0]
    left = min(x for x, _ in object_pixels)
    right = max(x for x, _ in object_pixels)
    max_width = 210 if bright_tube else 175
    if left <= left_limit or right >= right_limit-1 or not (19 if eclipse else 25) <= right-left <= max_width:
        if not eclipse and not high and not bright_tube:
            return photo_position(profile, jpeg, bright_tube=True)
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
        for retry in range(3):
            try:
                if cycle.step is Step.LOWER and cycle.profile is not OBJECTS["cube"]:
                    center = photo_position(cycle.profile, jpeg, high=True)[0]
                else:
                    center = photo_center(cycle.profile, jpeg)
                break
            except ValueError:
                if retry == 2:
                    raise
                cycle.report("좌우 보정 사진에서 윤곽이 불분명해 같은 자세에서 다시 촬영해요.")
                jpeg = cycle.snapshot("approach" if cycle.step is Step.LOWER else "align")
        error = center - target
        cycle.report(f"{cycle.profile.name} 사진 좌우 오차 {error:+.1f} px")
        if abs(error) <= tolerance:
            return jpeg
        if attempt == 5:
            cycle.report("좌우 보정 종료 · 재시도 소진")
            break
        if previous_error is not None and abs(error) > abs(previous_error) + 3:
            cycle.report("좌우 보정 종료 · 재촬영에서 오차 증가")
            break
        delta = max(-8, min(8, round(-error / 2.5)))
        if previous_error is not None and error * previous_error > 0 and abs(error - previous_error) < 2:
            # Use a larger correction only when the new image shows little motion.
            boosted = min(10, abs(delta) + 2)
            delta = -boosted if error > 0 else boosted
            cycle.report("직전 좌우 보정 반응이 작아 다음 이동을 최대 2 mm 늘려요.")
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
    final_x_tolerance = x_tolerance + FINAL_MARGIN_PX
    cube = cycle.profile is OBJECTS["cube"]
    forward_limit = 56 if cube else 38 if eclipse else 30
    max_moves = 9 if cube else 6 if eclipse else 4
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
        cycle.report(f"파지 사진 좌우 {x-target_x:+.1f} px · 윗변 Y {y:g} / 기준 {target_y:g} px · 앞뒤 {error_y:+.1f} px")
        if abs(x-target_x) <= final_x_tolerance and -y_tolerance <= error_y <= MAX_SHORT_OF_DEPTH_PX:
            return
        if eclipse and abs(x-target_x) > final_x_tolerance:
            for _ in range(2):
                if y <= ECLIPSE_SIDE_CLEARANCE_Y:
                    break
                retreat = 8
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
        if abs(x-target_x) > (final_x_tolerance if eclipse and y > ECLIPSE_SIDE_CLEARANCE_Y else x_tolerance):
            try:
                jpeg = align_demo_object(cycle, jpeg, require_clearance=eclipse)
                previous_y_error = None
                cycle.report("좌우 보정 뒤 새 사진을 앞뒤 보정 기준으로 사용")
            except AlignmentPathError:
                if not cube:
                    raise
                jpeg = cycle.snapshot("align")
                x, y = photo_position(cycle.profile, jpeg)
                if abs(x-target_x) > 24 or target_y-y < 35:
                    raise
                cycle.report("큐브 좌우 경로 제한 · 열린 집게로 짧게 전진한 뒤 재정렬")
                for distance in (5, 8, 10):
                    try:
                        jpeg = cycle.jog(distance, 0, 0, forward_limit=forward_limit, along_target=True)
                    except AlignmentPathError:
                        continue
                    break
                else:
                    raise AlignmentPathError("큐브 복구 전진 5·8·10 mm 경로를 찾지 못했어요.")
                continue
            x, y = photo_position(cycle.profile, jpeg)
            error_y = target_y - y
        if abs(x-target_x) <= final_x_tolerance and -y_tolerance <= error_y <= MAX_SHORT_OF_DEPTH_PX:
            return
        if previous_y_error is not None and abs(error_y) >= abs(previous_y_error)-1:
            reason = f"앞뒤 오차 개선 부족 · 직전 {previous_y_error:+.1f} px"
            break
        if attempt == max_moves:
            reason = f"앞뒤 보정 {max_moves}회 소진"
            break
        remaining = cycle.jog_total[0]
        delta = max(-8, min(8, round(error_y / DEPTH_PIXELS_PER_MM[cycle.profile])))
        if not delta:
            reason = f"추가 이동량 0 mm · 앞뒤 누적 {remaining:+g} mm"
            break
        cycle.progress(f"사진으로 앞뒤 보정 {attempt + 1}/{max_moves}")
        cycle.report(f"앞뒤 보정 {delta:+d} mm · 앞으로가 +")
        previous_pitch = cycle.pose.pitch
        jpeg = cycle.jog(delta, 0, 0, forward_limit=forward_limit, along_target=True)
        previous_y_error = error_y if abs(cycle.pose.pitch - previous_pitch) < 1 else None
        if previous_y_error is None:
            cycle.report("집게 각도 변경 · 새 사진에서 앞뒤 비교 기준 재설정")
    if abs(x-target_x) <= final_x_tolerance and -y_tolerance - FINAL_MARGIN_PX <= error_y <= MAX_SHORT_OF_DEPTH_PX:
        cycle.report(f"앞뒤 잔여 오차 {error_y:+.1f} px · 최종 허용 범위")
        return
    raise ValueError(f"앞뒤 정렬 중단 · {reason} · 윗변 Y {y:g} / 기준 {target_y:g} px · "
                     f"최종 앞뒤 {error_y:+.1f} px · 좌우 {x-target_x:+.1f} px")


def demo_approach(cycle):
    standoff, rise, _, _, _ = GRASP_PRESETS[cycle.profile]
    jpeg = cycle.approach(standoff, rise)
    for attempt in range(3):
        try:
            if cycle.profile is OBJECTS["cube"]:
                cube_view(jpeg, allow_bottom_crop=True)
            else:
                photo_position(cycle.profile, jpeg, high=True)
            break
        except ValueError:
            if attempt == 2:
                raise
            cycle.progress("높은 접근에서 윤곽 불분명 · 뒤로 8 mm 이동 후 재촬영")
            jpeg = cycle.jog(-8, 0, 0, along_target=True, back_limit=16)
    if cycle.profile is OBJECTS["cube"]:
        align_cube(cycle, jpeg, high=True)
    else:
        cycle.progress(f"높은 자세에서 {cycle.profile.name} 좌우 정렬")
        align_demo_object(cycle, jpeg)
    return jpeg


def prepare_demo_pick(cycle):
    demo_approach(cycle)
    return demo_lower(cycle)


def demo_lower(cycle):
    _, _, trim, preferred_pitch, _ = GRASP_PRESETS[cycle.profile]
    height = cycle.profile.grasp_height_mm + trim
    high = cycle.pose
    dx, dy = cycle.direction
    angles = sorted(range(-70, -29), key=lambda angle: abs(angle-preferred_pitch))
    approaches = [(0, False), (0, True), (8, True), (16, True), (24, True)]
    for pull, turn_first in approaches:
        for pitch in angles:
            start = replace(high, x=high.x-pull*dx, y=high.y-pull*dy, pitch=pitch) if turn_first else high
            low = replace(start, z=WORKPLANE_Z_MM+height, pitch=pitch)
            try:
                approach_plan = plan_tool_move(high, start) if start != high else None
                plan_tool_move(start, low)
            except ValueError:
                continue
            cycle.report(f"시연 목표 높이 {height:g} mm · 기울기 {pitch}° · 높은 자세 앞뒤 이동 {-pull:+d} mm")
            if approach_plan is not None:
                cycle.progress("높은 자세에서 집게 각도 조정")
                cycle.move_tool(start, approach_plan)
            args = (start.z-WORKPLANE_Z_MM, trim, 60, pitch, 497)
            return cycle.lower(*args)
    if high.z - WORKPLANE_Z_MM >= cycle.profile.height_mm + 20:
        for advance in (5, 8, 12, 16):
            for turn in (0, -3, -6, -10, 3, 6):
                start = replace(high, x=high.x+advance*dx, y=high.y+advance*dy,
                                pitch=high.pitch+turn)
                try:
                    approach_plan = plan_tool_move(high, start)
                except ValueError:
                    continue
                for pitch in angles:
                    low = replace(start, z=WORKPLANE_Z_MM+height, pitch=pitch)
                    try:
                        plan_tool_move(start, low)
                    except ValueError:
                        continue
                    cycle.report(f"높은 자세에서 {advance} mm 전진 · 하강 기울기 {pitch}°")
                    cycle.move_tool(start, approach_plan)
                    return cycle.lower(start.z-WORKPLANE_Z_MM, trim, 60, pitch, 497)
    raise ValueError("파지 높이까지 이동할 관절 경로를 찾지 못했어요.")


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
