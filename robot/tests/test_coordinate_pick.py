import pathlib
import struct
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from coordinate_pick import Point, LeArmSerial, deltas, packet, plan_pick, return_stages


class CoordinatePickTests(unittest.TestCase):
    def test_negative_and_long_moves_preserve_endpoint(self):
        commands = deltas(Point(150, 0, 100), Point(230, -117, 41))
        moves = [struct.unpack('bbb', item[4:]) for item in commands]
        self.assertEqual(tuple(map(sum, zip(*moves))), (80, -117, -59))
        self.assertTrue(all(max(map(abs, move)) <= 5 for move in moves))
        self.assertTrue(all(item[:4] == bytes.fromhex('55 55 05 04') for item in commands))

    def test_plan_rejects_nonfinite_and_bad_clearance(self):
        for point in [Point(float('nan'), 0, 30), Point(150, 0, -1), Point(150.5, 0, 30)]:
            with self.assertRaises(ValueError):
                plan_pick(point, 500)
        with self.assertRaises(ValueError):
            plan_pick(Point(150, 0, 100), 500, travel_z=100)

    def test_plan_lifts_before_horizontal_and_closes_after_descent(self):
        stages = plan_pick(Point(180, 20, 30), 500)
        self.assertEqual(stages[0].packets, (bytes.fromhex('55 55 02 0c'),))
        self.assertTrue(all(struct.unpack('bbb', p[4:])[:2] == (0, 0) for p in stages[1].packets))
        self.assertEqual(stages[5].packets[0], bytes.fromhex('55 55 08 03 01 e8 03 01 f4 01'))

    def test_fragmented_response_resynchronizes(self):
        class Port:
            def __init__(self):
                self.parts = iter([b'noise\x55', b'\x55\x04', b'\x01\x02\x01'])
            def reset_input_buffer(self): pass
            def write(self, data): return len(data)
            def flush(self): pass
            def read(self, count): return next(self.parts, b'')
        robot = LeArmSerial.__new__(LeArmSerial)
        robot.serial = Port()
        self.assertEqual(robot.query(1, 2), b'\x02\x01')

    def test_return_restores_saved_six_servo_positions(self):
        initial = {1: 219, 2: 497, 3: 181, 4: 124, 5: 408, 6: 506}
        stages = return_stages(Point(150, 0, 50), 120, 50, initial)
        restore = stages[-1].packets[0]
        self.assertEqual(restore[:7], bytes.fromhex('55 55 17 03 06 d0 07'))
        decoded = dict(struct.unpack_from('<BH', restore, p) for p in range(7, 25, 3))
        self.assertEqual(decoded, initial)


if __name__ == '__main__':
    unittest.main()
