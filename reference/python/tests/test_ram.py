import unittest

from autopilot.ram import MountSide, RamCalibration, SimRam


class TestRamCalibration(unittest.TestCase):
    def test_center_is_midpoint_of_endpoints(self):
        cal = RamCalibration.from_endpoints(port_stop=100, starboard_stop=300)
        self.assertAlmostEqual(cal.centre, 200)
        # Rudder amidships -> centre stroke.
        self.assertAlmostEqual(cal.rudder_to_stroke(0.0), 200)

    def test_symmetric_travel_equal_each_side(self):
        # Physically asymmetric stops: centre..port = 150, centre..stbd = 50.
        cal = RamCalibration(
            centre=200, port_stop=50, starboard_stop=250,
            max_rudder_deg=30, symmetric_travel=True)
        cal2 = RamCalibration(
            centre=100, port_stop=50, starboard_stop=250,
            max_rudder_deg=30, symmetric_travel=True)
        # half_travel is the *smaller* half so movement is equal both ways.
        self.assertAlmostEqual(cal2.half_travel, 50)
        # Equal magnitude of stroke offset for +/- full rudder.
        s_stbd = cal2.rudder_to_stroke(30) - cal2.centre
        s_port = cal2.rudder_to_stroke(-30) - cal2.centre
        self.assertAlmostEqual(s_stbd, -s_port)

    def test_mount_side_flips_drive_sign(self):
        port = RamCalibration.from_endpoints(-1, 1, mount_side=MountSide.PORT)
        stbd = RamCalibration.from_endpoints(-1, 1, mount_side=MountSide.STARBOARD)
        self.assertEqual(port.drive_sign, -stbd.drive_sign)

    def test_rudder_stroke_roundtrip(self):
        cal = RamCalibration.from_endpoints(-2.0, 2.0, max_rudder_deg=35)
        for deg in (-35, -20, 0, 15, 35):
            stroke = cal.rudder_to_stroke(deg)
            self.assertAlmostEqual(cal.stroke_to_rudder(stroke), deg, places=6)

    def test_never_commands_past_physical_stop(self):
        cal = RamCalibration.from_endpoints(-1, 1, max_rudder_deg=35)
        s = cal.rudder_to_stroke(1000)  # absurd command
        self.assertLessEqual(s, 1.0 + 1e-9)
        self.assertGreaterEqual(s, -1.0 - 1e-9)


class TestSimRam(unittest.TestCase):
    def test_speed_limit(self):
        cal = RamCalibration.from_endpoints(-1, 1, max_rudder_deg=35)
        ram = SimRam(cal, speed_stroke_per_s=0.5)  # 0.5 stroke/s
        ram.command(35, 0.1)  # target stroke +1, but only 0.05 of movement allowed
        self.assertAlmostEqual(ram.stroke, 0.05, places=6)

    def test_reaches_target_over_time(self):
        cal = RamCalibration.from_endpoints(-1, 1, max_rudder_deg=35)
        ram = SimRam(cal, speed_stroke_per_s=2.0)
        for _ in range(200):
            ram.command(20, 0.1)
        self.assertAlmostEqual(ram.rudder_angle, 20, places=4)

    def test_motor_deadband_ignores_tiny_commands(self):
        cal = RamCalibration.from_endpoints(-1, 1, max_rudder_deg=35)
        cal.deadband_stroke = 0.1
        ram = SimRam(cal, speed_stroke_per_s=2.0, initial_stroke=0.0)
        ram.command(1.0, 0.1)  # ~0.03 stroke, inside 0.1 deadband -> no move
        self.assertEqual(ram.stroke, 0.0)


if __name__ == "__main__":
    unittest.main()
