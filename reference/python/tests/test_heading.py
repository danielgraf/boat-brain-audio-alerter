import math
import unittest

from autopilot.heading import heading_error, normalize_180, normalize_360, yaw_rate


class TestHeading(unittest.TestCase):
    def test_normalize_360(self):
        self.assertAlmostEqual(normalize_360(370), 10)
        self.assertAlmostEqual(normalize_360(-10), 350)
        self.assertAlmostEqual(normalize_360(0), 0)

    def test_normalize_180_range(self):
        self.assertAlmostEqual(normalize_180(190), -170)
        self.assertAlmostEqual(normalize_180(-190), 170)
        # +/-180 boundary maps to +180, never -180.
        self.assertAlmostEqual(normalize_180(180), 180)
        self.assertAlmostEqual(normalize_180(-180), 180)

    def test_heading_error_shortest_arc(self):
        # 10 vs 350: shortest way is +20 (turn starboard), not -340.
        self.assertAlmostEqual(heading_error(10, 350), 20)
        self.assertAlmostEqual(heading_error(350, 10), -20)
        self.assertAlmostEqual(heading_error(90, 90), 0)
        # Across the 0 line the other way.
        self.assertAlmostEqual(heading_error(1, 359), 2)
        self.assertAlmostEqual(heading_error(359, 1), -2)

    def test_heading_error_never_exceeds_180(self):
        for d in range(0, 360, 7):
            for m in range(0, 360, 11):
                e = heading_error(d, m)
                self.assertLessEqual(abs(e), 180.0 + 1e-9)

    def test_yaw_rate_wrap_safe(self):
        # Crossing 359 -> 1 in 0.5 s is +2 deg over 0.5 s = +4 deg/s.
        self.assertAlmostEqual(yaw_rate(359, 1, 0.5), 4.0)
        self.assertAlmostEqual(yaw_rate(1, 359, 0.5), -4.0)

    def test_yaw_rate_guards_zero_dt(self):
        self.assertEqual(yaw_rate(0, 10, 0.0), 0.0)
        self.assertEqual(yaw_rate(0, 10, -1.0), 0.0)


if __name__ == "__main__":
    unittest.main()
