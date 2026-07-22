import unittest

from autopilot.simulator import (
    BoatModel, RunLog, _count_reversals, evaluate,
)


class TestBoatModel(unittest.TestCase):
    def test_positive_rudder_turns_starboard(self):
        boat = BoatModel(K=0.15, T=3.0, heading=0.0)
        for _ in range(50):
            boat.step(10.0, 0.1)
        self.assertGreater(boat.heading, 0.0)      # turned toward starboard
        self.assertGreater(boat.yaw_rate_dps, 0.0)

    def test_steady_state_yaw_rate_matches_K(self):
        boat = BoatModel(K=0.15, T=3.0)
        for _ in range(2000):
            boat.step(10.0, 0.1)
        # r_ss = K * delta
        self.assertAlmostEqual(boat.yaw_rate_dps, 0.15 * 10.0, places=2)


class TestReversals(unittest.TestCase):
    def test_monotonic_has_no_reversals(self):
        self.assertEqual(_count_reversals([0, 1, 2, 3, 4], hysteresis=0.25), 0)

    def test_counts_big_zigzag_ignores_small(self):
        # One big down-up after a rise = ... rises, falls (1), rises (2).
        series = [0, 5, 0, 5]
        self.assertEqual(_count_reversals(series, hysteresis=0.25), 2)
        # Tiny dither below hysteresis doesn't count.
        series2 = [0, 0.1, 0.0, 0.1, 0.0]
        self.assertEqual(_count_reversals(series2, hysteresis=0.25), 0)


class TestEvaluate(unittest.TestCase):
    def _make_log(self, headings, desired, rudder):
        log = RunLog()
        from autopilot.heading import heading_error
        for i, h in enumerate(headings):
            log.t.append(i * 0.1)
            log.heading.append(h)
            log.desired.append(desired)
            log.error.append(heading_error(desired, h))
            log.rudder_cmd.append(rudder[i])
            log.rudder_actual.append(rudder[i])
            log.yaw_rate.append(0.0)
        return log

    def test_settled_series_reports_zero_overshoot(self):
        n = 200
        log = self._make_log([30.0] * n, 30.0, [0.0] * n)
        m = evaluate(log)
        self.assertEqual(m.max_overshoot, 0.0)
        self.assertEqual(m.rudder_reversals, 0)
        self.assertAlmostEqual(m.steady_state_error, 0.0)

    def test_empty_log_raises(self):
        with self.assertRaises(ValueError):
            evaluate(RunLog())


if __name__ == "__main__":
    unittest.main()
