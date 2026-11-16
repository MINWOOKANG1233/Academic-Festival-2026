from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yolo_pick import (Detection, YoloSidePick, compatible, pixel_to_mm,
                       select_detection, stable_target, supervised_pick)
from side_pick import Step
from object_pick import OBJECTS
from camera_hover import CameraTarget
from grid_hover import grid_to_arm


class YoloPickTests(unittest.TestCase):
    def test_no_lowering_when_approach_not_confirmed(self):
        cycle = Mock(profile=OBJECTS['cube'])
        with patch('yolo_pick.demo_approach'), patch('yolo_pick.demo_lower') as lower:
            supervised_pick(cycle, ask=lambda _: 'cancel')
        lower.assert_not_called()
        cycle.release.assert_not_called()

    def test_holds_after_lift_until_release_confirmation(self):
        cycle = Mock(profile=OBJECTS['cube'])
        with patch('yolo_pick.demo_approach'), patch('yolo_pick.demo_lower'), \
             patch('yolo_pick.align_demo_depth'), patch('yolo_pick.lift_demo_object') as lift:
            replies = iter(['GRAB', 'hold'])
            supervised_pick(cycle, ask=lambda _:next(replies))
        lift.assert_called_once_with(cycle)
        cycle.release.assert_not_called()
        cycle.home.assert_not_called()

    def test_releases_and_returns_only_after_confirmation(self):
        cycle = Mock(profile=OBJECTS['cube'])
        with patch('yolo_pick.demo_approach'), patch('yolo_pick.demo_lower'), \
             patch('yolo_pick.align_demo_depth'), patch('yolo_pick.lift_demo_object'):
            replies = iter(['GRAB', 'RELEASE'])
            supervised_pick(cycle, ask=lambda _:next(replies))
        cycle.release.assert_called_once()
        cycle.home.assert_called_once()

    def test_class_mapping_and_actual_plane_calibration(self):
        item = select_detection([[140, 140, 176, 180, .92, 0]], {0:'candy_tin'}, .6)
        self.assertEqual(item.label, 'candy_tin')
        self.assertAlmostEqual(item.x_mm, 100, delta=3)
        self.assertTrue(0 < item.y_mm < 95)

    def test_multiple_unknown_clipped_or_outside_detections_rejected(self):
        good = [140, 140, 176, 180, .92, 0]
        for rows, names in (([], {0:'cube_block'}),
                            ([good, good], {0:'cube_block'}),
                            ([good], {0:'person'}),
                            ([[0,140,176,180,.9,0]], {0:'cube_block'}),
                            ([[5,5,15,15,.9,0]], {0:'cube_block'})):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                select_detection(rows, names, .6)

    def test_class_and_location_must_stay_stable(self):
        first = Detection('cube_block', .9, 150, 150, 100, 50)
        self.assertFalse(compatible(first, Detection('candy_tin', .9,150,150,100,50)))
        self.assertFalse(compatible(first, Detection('cube_block', .9,150,150,104,50)))

    def test_yolo_target_bypasses_old_foreground_target_without_motion(self):
        arm = Mock()
        cycle = YoloSidePick(Mock(), arm, OBJECTS['tube'], report=lambda _:None,
                             on_photo=lambda *_:None, cancelled=lambda:False)
        target = CameraTarget(100,50,grid_to_arm(100,50),(0,0))
        cycle.yolo_target = target
        cycle.step = Step.DETECT
        cycle.detect(0,0)
        self.assertIs(cycle.detection, target)
        self.assertEqual(cycle.step, Step.APPROACH)
        arm.write.assert_not_called()

    def test_invalid_frame_resets_stability_samples(self):
        item = Detection('cube_block', .9,150,150,100,50)
        output = MagicMock()
        with patch('yolo_pick.raw_photo', return_value=b'jpeg') as read, \
             patch('yolo_pick.detect_photo', side_effect=[item,item,ValueError('bad'),item,item,item]):
            self.assertEqual(stable_target(Mock(),Mock(),.6,output),item)
        self.assertEqual(read.call_count,6)


if __name__ == '__main__':
    unittest.main()
