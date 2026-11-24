"""Plan the carry and above-rim release for the three-bin layout."""

from dataclasses import replace

from coordinate_pick import gripper
from level_kinematics import ToolPose, plan_tool_move, pose_from_joints, solve_joints
from object_pick import OBJECTS
from side_pick import Step


# x, y, grasp height, travel pitch, bin pitch (mm and degrees).
BINS = {
    OBJECTS["cube"]: (110, -200, 25, -55, -55),
    OBJECTS["eclipse"]: (200, -200, 53, -50, -35),
    OBJECTS["tube"]: (130, 150, 77.5, -45, -45),
}
RELEASE_OFFSETS_MM = {OBJECTS["cube"]: (-20, 0)}


def bin_path(cycle, lifted):
    x, y, grasp, travel_pitch, bin_pitch = BINS[cycle.profile]
    offset_x, offset_y = RELEASE_OFFSETS_MM.get(cycle.profile, (0, 0))
    x, y = x + offset_x, y + offset_y
    for extra_height in (75, 85, 95):
        z = cycle.surface_z + grasp + extra_height
        high = replace(lifted, z=z, pitch=travel_pitch)
        above = ToolPose(x, y, z, bin_pitch)
        drop = replace(above, z=cycle.surface_z + grasp + 55)
        goals = (high, above, drop)
        try:
            path = []
            for start, goal in zip((lifted, *goals), goals):
                plan_tool_move(start, goal)
                first, last = solve_joints(start), solve_joints(goal)
                for step in range(1, 41):
                    pose = pose_from_joints({
                        servo: first[servo] + (last[servo]-first[servo])*step/40
                        for servo in first
                    })
                    if pose.z < min(start.z, goal.z)-3:
                        raise ValueError("한 번에 이동할 때 중간 높이가 낮아져요.")
                path.append((goal, [last]))
            return path
        except ValueError:
            continue
    raise ValueError("현재 파지 위치에서 지정 분류통까지 운반 경로를 찾지 못했어요.")


def place_in_bin(cycle):
    cycle.require_step(Step.RELEASE)
    path = bin_path(cycle, cycle.pose)
    for index, (goal, plan) in enumerate(path):
        cycle.progress(f"분류통 이동 {index + 1}/3")
        cycle.move_tool(goal, plan, transport=True)
    actual = cycle.send(gripper(200, 1500), 1.8)
    if abs(actual[1] - 200) > 15:
        raise OSError("분류통 위에서 집게 열림을 확인하지 못했어요.")
    cycle.step = Step.RETURN
    cycle.home()
