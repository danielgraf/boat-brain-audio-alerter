import unittest

from autopilot.pid import PID, PIDConfig, PIDGains, PIDLimits


def make_pid(**overrides):
    cfg = PIDConfig(
        gains=PIDGains(kp=2.0, ki=0.0, kd=0.0),
        limits=PIDLimits(output_min=-30, output_max=30, slew_rate=1000, integral_limit=20),
        deadband=0.0,
        rate_filter_tau=0.0,
    )
    for k, v in overrides.items():
        setattr(cfg, k, v)
    return PID(cfg)


class TestPID(unittest.TestCase):
    def test_proportional_sign_and_magnitude(self):
        pid = make_pid()
        # +5 deg error -> +10 deg rudder (turn starboard).
        self.assertAlmostEqual(pid.update(5.0, 0.0, 0.1), 10.0)
        pid.reset()
        self.assertAlmostEqual(pid.update(-5.0, 0.0, 0.1), -10.0)

    def test_output_clamped(self):
        pid = make_pid()
        self.assertAlmostEqual(pid.update(100.0, 0.0, 0.1), 30.0)  # clamp to output_max

    def test_slew_rate_limits_change(self):
        pid = make_pid()
        pid.config.limits.slew_rate = 10.0  # deg/s
        out = pid.update(100.0, 0.0, 0.1)   # wants 30, but max step = 1 deg
        self.assertAlmostEqual(out, 1.0)

    def test_deadband_suppresses_small_error(self):
        pid = make_pid(deadband=3.0)
        self.assertEqual(pid.update(2.0, 0.0, 0.1), 0.0)      # within band -> no output
        pid.reset()
        # Soft deadband: 5 deg error acts like (5-3)=2 -> 4 deg rudder.
        self.assertAlmostEqual(pid.update(5.0, 0.0, 0.1), 4.0)

    def test_counter_rudder_opposes_turn(self):
        pid = make_pid()
        pid.config.gains = PIDGains(kp=0.0, ki=0.0, kd=5.0)
        # Turning to starboard (+rate) with zero error -> port (negative) rudder.
        self.assertLess(pid.update(0.0, 2.0, 0.1), 0.0)
        pid.reset()
        self.assertGreater(pid.update(0.0, -2.0, 0.1), 0.0)

    def test_integral_accumulates_and_clamps(self):
        pid = make_pid()
        pid.config.gains = PIDGains(kp=0.0, ki=1.0, kd=0.0)
        pid.config.limits.integral_limit = 5.0
        out = 0.0
        for _ in range(1000):
            out = pid.update(10.0, 0.0, 0.1)
        # Integral contribution capped at integral_limit despite persistent error.
        self.assertAlmostEqual(out, 5.0, places=6)

    def test_integral_active_band_freezes_on_big_error(self):
        pid = make_pid()
        pid.config.gains = PIDGains(kp=0.0, ki=1.0, kd=0.0)
        pid.config.integral_active_band = 5.0
        # Error 10 > band 5: integral must not grow (PD-only during course change).
        for _ in range(50):
            pid.update(10.0, 0.0, 0.1)
        self.assertAlmostEqual(pid.integral, 0.0)
        # Error 3 < band: integral now accumulates.
        pid.update(3.0, 0.0, 0.1)
        self.assertGreater(pid.integral, 0.0)

    def test_anti_windup_does_not_overshoot_on_saturation(self):
        # With output saturated, the integral should not keep piling up.
        pid = make_pid()
        pid.config.gains = PIDGains(kp=5.0, ki=2.0, kd=0.0)
        pid.config.limits.integral_limit = 100.0
        for _ in range(200):
            pid.update(20.0, 0.0, 0.1)  # huge error, output pinned at 30
        integ_saturated = pid.integral
        # Now the error clears; the output should come off the rail promptly,
        # i.e. the integral didn't wind up to a huge value.
        self.assertLess(integ_saturated, 40.0)

    def test_rate_filter_smooths(self):
        pid = make_pid(rate_filter_tau=1.0)
        pid.config.gains = PIDGains(kp=0.0, ki=0.0, kd=1.0)
        pid.update(0.0, 0.0, 0.1)   # prime the filter at rate 0
        pid.update(0.0, 10.0, 0.1)  # step the rate to 10
        # Filtered rate lags the step, so |output| < |kd*raw_rate| = 10.
        self.assertLess(abs(pid.output), 10.0)
        self.assertGreater(abs(pid.output), 0.0)


if __name__ == "__main__":
    unittest.main()
