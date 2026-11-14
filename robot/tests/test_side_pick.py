import base64
from io import BytesIO
import json
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import Mock, patch

from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from coordinate_pick import Point
from auto_cube_pick import align_cube, auto_pick_cube, cube_view
from demo_pick import GRASP_PRESETS, align_demo_depth, align_demo_object, lift_demo_object, photo_center, prepare_demo_pick, run_demo
from grid_hover import HOME
from level_kinematics import ToolPose, pose_from_joints, solve_joints
from object_pick import OBJECTS
from side_pick import CameraStream, SidePick, Step


class Camera:
    y_mm = 80
    def __init__(self):
        with BytesIO() as output:
            Image.new("RGB", (320, 240)).save(output, "JPEG")
            self.jpeg = b"JPEG " + base64.b64encode(output.getvalue()) + b"\n"
        self.frame = 0

    def reset_input_buffer(self):
        pass

    def read_until(self, **_):
        self.frame += 1
        if self.frame % 2 == 0:
            return self.jpeg
        return (json.dumps(dict(frame=self.frame, found=True, calibration_ready=True,
                                inside_grid=True, coordinate_frame="grid_lower_left",
                                width=320, height=240, x_mm=100, y_mm=self.y_mm)) + "\n").encode()


class Arm:
    cube_contact = False
    def __init__(self):
        self.writes = []
        self.reset()

    def inspect(self):
        return 2

    def read_positions(self):
        return self.joints.copy()

    def reset(self):
        self.stock = HOME
        self.joints = {1: 200, 2: 497, 3: 182, 4: 127, 5: 413, 6: 500}

    def write(self, command):
        self.writes.append(command)
        if command[3] == 12:
            self.reset()
        elif command[3] == 3:
            self.joints.update(dict(struct.iter_unpack("<BH", command[7:])))
            if self.cube_contact and self.joints[1] == 425:
                self.joints[1] = 414
        elif command[3] == 4:
            dx, dy, dz = struct.unpack("bbb", command[4:])
            self.stock = Point(self.stock.x + dx, self.stock.y + dy, self.stock.z + dz)
            for pitch in sorted(range(-90, 1), key=lambda value: abs(value + 30)):
                try:
                    self.joints.update(solve_joints(ToolPose(*vars(self.stock).values(), pitch)))
                    return
                except ValueError:
                    continue
            raise AssertionError(self.stock)


class SidePickTests(unittest.TestCase):
    def test_eclipse_aligns_then_advances_at_observed_clearance(self):
        machine = Mock(profile=OBJECTS["eclipse"], jog_total=(0, 0, 0))
        with patch("demo_pick.photo_position", side_effect=[
            (130, 105), (154, 105), (154, 124), (154, 137),
        ]), patch("demo_pick.align_demo_object", return_value=b"aligned") as lateral:
            align_demo_depth(machine, b"observed")
        lateral.assert_called_once_with(machine, b"observed", require_clearance=True)
        self.assertEqual([call.args[0] for call in machine.jog.call_args_list], [8, 6])

    def test_eclipse_joins_vertically_split_outline(self):
        from PIL import ImageDraw

        frame = Image.new("RGB", (320, 240), "white")
        draw = ImageDraw.Draw(frame)
        draw.rectangle((137, 110, 185, 149), fill=(145, 90, 200))
        draw.rectangle((137, 160, 185, 177), fill=(145, 90, 200))
        with BytesIO() as output:
            frame.save(output, "JPEG")
            self.assertAlmostEqual(photo_center(OBJECTS["eclipse"], output.getvalue()), 161, delta=2)

    def test_eclipse_retreats_before_lateral_alignment_when_too_close(self):
        machine = Mock(profile=OBJECTS["eclipse"], jog_total=(0, 0, 0))
        with patch("demo_pick.photo_position", side_effect=[
            (132, 120), (132, 98), (161, 98), (161, 117), (161, 136),
        ]), patch("demo_pick.align_demo_object", return_value=b"aligned") as lateral:
            align_demo_depth(machine, b"close")
        self.assertEqual([call.args[0] for call in machine.jog.call_args_list], [-8, 8, 8])
        lateral.assert_called_once_with(machine, machine.jog.return_value, require_clearance=True)

    def test_eclipse_accepts_visible_top_under_previous_area_threshold(self):
        from PIL import ImageDraw

        frame = Image.new("RGB", (320, 240), "white")
        ImageDraw.Draw(frame).rectangle((145, 130, 177, 159), fill=(145, 90, 200))
        with BytesIO() as output:
            frame.save(output, "JPEG")
            self.assertAlmostEqual(photo_center(OBJECTS["eclipse"], output.getvalue()), 161, delta=2)

    def test_demo_does_not_close_at_old_early_depth(self):
        for key, x, before, after in (
            ("cube", 151, 126, 136),
            ("eclipse", 161, 130, 139),
            ("tube", 142, 125, 133),
        ):
            with self.subTest(object=key):
                machine = Mock(profile=OBJECTS[key], jog_total=(0, 0, 0))
                with patch("demo_pick.photo_position", side_effect=[(x, before), (x, after)]):
                    align_demo_depth(machine, b"frame")
                self.assertGreater(machine.jog.call_args.args[0], 0)

    def test_high_view_accepts_bottom_crop_but_not_side_crop(self):
        from PIL import ImageDraw

        for left, right, accepted in ((123, 177, True), (58, 112, False)):
            frame = Image.new("RGB", (320, 240))
            ImageDraw.Draw(frame).rectangle((left, 216, right, 239), fill=(235, 130, 190))
            with BytesIO() as output:
                frame.save(output, "JPEG")
                jpeg = output.getvalue()
            with self.assertRaises(ValueError):
                cube_view(jpeg)
            if accepted:
                center, _ = cube_view(jpeg, allow_bottom_crop=True)
                self.assertAlmostEqual(center, 150, delta=2)
                machine = Mock()
                align_cube(machine, jpeg, high=True)
                machine.jog.assert_not_called()
            else:
                with self.assertRaises(ValueError):
                    cube_view(jpeg, allow_bottom_crop=True)

    def test_lateral_alignment_boosts_only_after_little_image_motion(self):
        machine = Mock(profile=OBJECTS["eclipse"], jog_total=(0, 0, 0))
        with patch("demo_pick.photo_center", side_effect=[195, 194, 164]):
            align_demo_object(machine, b"frame")
        self.assertEqual([call.args for call in machine.jog.call_args_list], [(0, -8, 0), (0, -10, 0)])

    def test_small_yaw_error_retries_same_target_then_uses_measured_pose(self):
        arm = Arm()
        cycle = SidePick(Camera(), arm, OBJECTS["cube"], report=Mock(),
                         on_photo=Mock(), cancelled=lambda: False)
        actual = arm.read_positions()
        cycle.joints = actual
        targets = {i: actual[i] for i in range(2, 7)}
        targets[6] += 8
        cycle.send = Mock(return_value=actual)
        cycle.move_tool(ToolPose(180, 0, 0, -60), [targets], settle_yaw=True)
        self.assertEqual(cycle.send.call_count, 4)
        self.assertEqual(cycle.pose, pose_from_joints(actual))
        targets[6] += 2
        with self.assertRaisesRegex(OSError, "밑판 보정 목표"):
            cycle.move_tool(ToolPose(180, 0, 0, -60), [targets], settle_yaw=True)

    def test_camera_connection_confirms_background_protocol(self):
        port = Mock()
        camera = CameraStream(port)
        with patch.object(camera, "read_until", return_value=b"LEARM CAMERA READY\n"):
            camera.wait_ready(Mock(), require_background=True)
        port.write.assert_called_once_with(b"P")

    def test_background_requires_camera_ack_and_consecutive_empty_frames(self):
        port = Mock()
        camera = CameraStream(port)
        empty = b'{"status":"NO BLOB","found":false}\n'
        occupied = b'{"status":"OK","found":true}\n'
        with patch.object(camera, "read_until", side_effect=[
            empty, b"BACKGROUND OK\n", empty, occupied, empty, empty,
        ]) as read:
            camera.capture_background(Mock())
        port.write.assert_called_once_with(b"B")
        self.assertEqual(read.call_count, 6)

    def test_depth_corrects_from_new_photos_and_skips_unneeded_travel(self):
        machine = Mock(profile=OBJECTS["tube"], jog_total=(0, 0, 0))
        with patch("demo_pick.photo_position", side_effect=[(142, 98), (142, 109), (142, 132)]):
            align_demo_depth(machine, b"frame")
        self.assertEqual([call.args for call in machine.jog.call_args_list], [(8, 0, 0), (8, 0, 0)])
        machine.jog.reset_mock()
        with patch("demo_pick.photo_position", return_value=(142, 132)):
            align_demo_depth(machine, b"frame")
        machine.jog.assert_not_called()

    def test_depth_stops_when_the_image_does_not_improve(self):
        machine = Mock(profile=OBJECTS["tube"], jog_total=(0, 0, 0))
        with patch("demo_pick.photo_position", return_value=(142, 101)), self.assertRaises(ValueError):
            align_demo_depth(machine, b"frame")
        machine.jog.assert_called_once()

    @patch("time.sleep")
    def test_full_demo_returns_home_with_gripper_open(self, _sleep):
        arm = Arm()
        arm.cube_contact = True
        cycle = SidePick(Camera(), arm, OBJECTS["cube"], report=lambda _: None,
                         on_photo=lambda *_: None, cancelled=lambda: False)
        cycle.home()
        with patch("demo_pick.cube_view"), patch("demo_pick.align_cube"), patch("demo_pick.photo_position", return_value=(151, 136)):
            run_demo(cycle)
        self.assertIs(cycle.step, Step.DETECT)
        self.assertEqual(arm.joints[1], 200)

    def test_demo_photo_alignment_recalculates_after_each_move(self):
        machine = Mock(profile=OBJECTS["eclipse"], jog_total=(0, 0, 0))
        with patch("demo_pick.photo_center", side_effect=[201, 192, 163]):
            align_demo_object(machine, b"frame",)
        self.assertEqual([call.args for call in machine.jog.call_args_list], [(0, -8, 0), (0, -8, 0)])
        self.assertTrue(all(call.kwargs["settle_yaw"] for call in machine.jog.call_args_list))

    def test_demo_photo_rejects_two_objects(self):
        from PIL import ImageDraw

        frame = Image.new("RGB", (320, 240), "white")
        draw = ImageDraw.Draw(frame)
        draw.rectangle((80, 90, 118, 185), fill=(145, 90, 200))
        draw.rectangle((171, 92, 215, 183), fill=(145, 90, 200))
        with BytesIO() as output:
            frame.save(output, "JPEG")
            with self.assertRaises(ValueError):
                photo_center(OBJECTS["eclipse"], output.getvalue())

    def test_eclipse_photo_accepts_dominant_outline_with_small_reflection(self):
        from PIL import ImageDraw

        frame = Image.new("RGB", (320, 240), "white")
        draw = ImageDraw.Draw(frame)
        draw.rectangle((140, 115, 182, 191), fill=(145, 90, 200))
        draw.rectangle((204, 150, 231, 181), fill=(145, 90, 200))
        with BytesIO() as output:
            frame.save(output, "JPEG")
            self.assertAlmostEqual(photo_center(OBJECTS["eclipse"], output.getvalue()), 161, delta=2)

    @patch("time.sleep")
    def test_demo_presets_prepare_and_grip_without_extra_forward_motion(self, _sleep):
        for key, height in (("eclipse", 53), ("tube", 67.5)):
            with self.subTest(object=key):
                camera = Camera()
                camera.y_mm = 72
                arm = Arm()
                profile = OBJECTS[key]
                cycle = SidePick(camera, arm, profile, report=lambda _: None,
                                 on_photo=lambda *_: None, cancelled=lambda: False)
                cycle.home()
                cycle.detect(0, 0)
                with patch("demo_pick.photo_center", return_value=142), patch(
                    "demo_pick.photo_position", return_value=(161 if key == "eclipse" else 142, 172)
                ):
                    prepare_demo_pick(cycle)
                self.assertIs(cycle.step, Step.ALIGN)
                self.assertAlmostEqual(cycle.pose.z - cycle.surface_z, height)
                writes = len(arm.writes)
                cycle.insert(0)
                self.assertEqual(len(arm.writes), writes)
                closed = 550 if key == "eclipse" else 400
                send = arm.write

                def grip_feedback(command):
                    send(command)
                    if arm.joints[1] == GRASP_PRESETS[profile][-1]:
                        arm.joints[1] = closed

                with patch.object(arm, "write", side_effect=grip_feedback):
                    lift_demo_object(cycle)
                self.assertIs(cycle.step, Step.RELEASE)

    @patch("time.sleep")
    def test_automatic_cube_pick_corrects_then_lifts(self, _sleep):
        arm = Arm()
        arm.cube_contact = True
        photos = []
        camera = Camera()
        camera.y_mm = 65
        messages = []
        machine = SidePick(camera, arm, OBJECTS["cube"], report=messages.append,
                           on_photo=lambda label, _: photos.append(label), cancelled=lambda: False)
        machine.home()
        with patch("auto_cube_pick.cube_view", side_effect=[(150, 160), (166, 80), (147, 80), (151, 126)]):
            auto_pick_cube(machine)
        self.assertIs(machine.step, Step.RELEASE)
        self.assertEqual(machine.jog_total, (0, -10, 0))
        self.assertEqual(photos, ["observe", "approach", "align", "align", "insert"])
        self.assertTrue(machine.fast)
        self.assertIn("자동 하강 이동 명령 9개", messages)

    def test_large_photo_error_is_corrected_and_checked_again(self):
        machine = Mock()
        with patch("auto_cube_pick.cube_view", side_effect=[(195.2, 58), (165.9, 65), (151.5, 72)]):
            align_cube(machine, b"first frame", high=False)
        self.assertEqual([call.args for call in machine.jog.call_args_list], [(0, -10, 0), (0, -5, 0)])

    def test_high_photo_correction_stops_before_lowering(self):
        machine = Mock()
        with patch("auto_cube_pick.cube_view", side_effect=[(178.5, 158), (160, 160), (153, 160)]):
            align_cube(machine, b"first frame", high=True)
        self.assertEqual([call.args for call in machine.jog.call_args_list], [(0, -10, 0), (0, -8, 0)])
        machine.lower.assert_not_called()

    def test_rotated_cube_blocks_automatic_alignment(self):
        from PIL import ImageDraw

        frame = Image.new("RGB", (320, 240))
        ImageDraw.Draw(frame).polygon([(92, 110), (195, 50), (230, 158), (126, 219)], fill=(235, 130, 190))
        with BytesIO() as output:
            frame.save(output, "JPEG")
            with self.assertRaisesRegex(ValueError, "비스듬"):
                cube_view(output.getvalue())

    def test_slightly_slanted_cube_with_dim_top_is_accepted(self):
        from PIL import ImageDraw

        frame = Image.new("RGB", (320, 240))
        ImageDraw.Draw(frame).polygon([(92, 90), (195, 62), (230, 170), (126, 198)], fill=(142, 70, 115))
        with BytesIO() as output:
            frame.save(output, "JPEG")
            center, _ = cube_view(output.getvalue())
        self.assertTrue(150 <= center <= 175)

    def test_camera_stream_keeps_multiple_lines(self):
        class Port:
            data = b"metadata\nJPEG photo\n"

            @property
            def in_waiting(self):
                return len(self.data)

            def read(self, count):
                result, self.data = self.data[:count], self.data[count:]
                return result

        port = Port()
        camera = CameraStream(port)
        self.assertEqual(camera.read_until(size=65536), b"metadata\n")
        self.assertEqual(camera.read_until(size=65536), b"JPEG photo\n")

    def test_camera_stream_keeps_partial_line_on_timeout(self):
        port = Mock(in_waiting=1)
        port.read.side_effect = [b"{", b'"frame":1}\n']
        camera = CameraStream(port)
        with patch("side_pick.time.monotonic", side_effect=[0, 0, 2, 2, 2]):
            self.assertEqual(camera.read_until(size=65536), b"")
            self.assertEqual(camera.read_until(size=65536), b'{"frame":1}\n')

    @patch("time.sleep")
    def test_all_profiles_hold_pitch_and_wait_for_confirmation(self, _sleep):
        for name in ("cube", "eclipse", "tube"):
            with self.subTest(name=name):
                arm = Arm()
                photos = []
                machine = SidePick(Camera(), arm, OBJECTS[name], report=lambda _: None,
                                   on_photo=lambda label, _: photos.append(label), cancelled=lambda: False)
                machine.home()
                with self.assertRaises(ValueError):
                    machine.close(425)
                machine.detect(0, 0)
                machine.approach(30, 60)
                machine.jog(0, 1, 0)
                measured_height = machine.pose.z + 65
                before = len(arm.writes)
                with self.assertRaises(ValueError):
                    machine.lower(measured_height, 0, 60, 0, 497)
                self.assertEqual(len(arm.writes), before)
                pitch = -60 if name == "cube" else -55
                machine.lower(measured_height, 0, 60, pitch, 497)
                machine.jog(0, 1, 0)
                machine.insert(30)
                self.assertAlmostEqual(pose_from_joints(arm.joints).pitch, pitch, delta=.5)
                machine.close(425)
                machine.lift()
                machine.release()
                machine.home()
                self.assertIs(machine.step, Step.DETECT)
                self.assertEqual(photos, ["observe", "approach", "approach", "align", "align", "insert"])

    def test_cancellation_sends_no_motion(self):
        arm = Arm()
        machine = SidePick(Camera(), arm, OBJECTS["cube"], report=lambda _: None,
                           on_photo=lambda *_: None, cancelled=lambda: True)
        with self.assertRaises(InterruptedError):
            machine.home()
        self.assertEqual(arm.writes, [])
