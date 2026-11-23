"""Plan the carry and above-rim release for the three-bin layout."""

from dataclasses import replace

from coordinate_pick import gripper
from level_kinematics import ToolPose, plan_tool_move
from object_pick import OBJECTS
from side_pick import Step


# x, y, grasp height, travel pitch, bin pitch (mm and degrees).
BINS = {
    OBJECTS["cube"]: (150, -165, 25, -55, -55),
    OBJECTS["eclipse"]: (250, -165, 53, -50, -35),
    OBJECTS["tube"]: (190, 145, 67.5, -45, -45),
}


def bin_path(cycle, lifted):
    x, y, grasp, travel_pitch, bin_pitch = BINS[cycle.profile]
    for extra_height in (75, 85, 95):
        z = cycle.surface_z + grasp + extra_height
        high = replace(lifted, z=z, pitch=travel_pitch)
        center = ToolPose(225, 0, z, travel_pitch)
        above = ToolPose(x, y, z, bin_pitch)
        drop = replace(above, z=cycle.surface_z + grasp + 55)
        goals = (high, center, above, drop)
        try:
            return [(goal, plan_tool_move(start, goal))
                    for start, goal in zip((lifted, *goals), goals)]
        except ValueError:
            continue
    raise ValueError("현재 파지 위치에서 지정 분류통까지 운반 경로를 찾지 못했어요.")


def place_in_bin(cycle):
    cycle.require_step(Step.RELEASE)
    path = bin_path(cycle, cycle.pose)
    for index, (goal, plan) in enumerate(path):
        cycle.move_tool(goal, plan, smooth=index in (1, 2))
    actual = cycle.send(gripper(200, 1500), 1.8)
    if abs(actual[1] - 200) > 15:
        raise OSError("분류통 위에서 집게 열림을 확인하지 못했어요.")
    cycle.step = Step.RETURN
    cycle.home()
