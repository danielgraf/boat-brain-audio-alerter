import unittest

from autopilot.tuning import (
    NomotoParams, estimate_nomoto_from_step, gain_schedule, nomoto_pid_gains,
)


class TestTuning(unittest.TestCase):
    def test_pole_placement_matches_formula(self):
        p = NomotoParams(K=0.15, T=3.0)
        zeta, wn = 0.9, 0.5
        g = nomoto_pid_gains(p, damping_ratio=zeta, natural_frequency=wn,
                             integral_fraction=0.0)
        self.assertAlmostEqual(g.kp, wn * wn * p.T / p.K, places=6)
        self.assertAlmostEqual(g.kd, (2 * zeta * wn * p.T - 1) / p.K, places=6)
        self.assertEqual(g.ki, 0.0)

    def test_settling_time_gives_reasonable_bandwidth(self):
        p = NomotoParams(K=0.15, T=3.0)
        g_fast = nomoto_pid_gains(p, settling_time=4.0)
        g_slow = nomoto_pid_gains(p, settling_time=16.0)
        # Faster settling demands more proportional gain.
        self.assertGreater(g_fast.kp, g_slow.kp)

    def test_counter_rudder_non_negative(self):
        # An already-fast plant shouldn't yield negative counter rudder.
        p = NomotoParams(K=1.0, T=0.2)
        g = nomoto_pid_gains(p, settling_time=20.0)
        self.assertGreaterEqual(g.kd, 0.0)

    def test_speed_scaling_directions(self):
        p = NomotoParams(K=0.15, T=3.0)
        faster = p.scaled_to_speed(12.0, 6.0)
        # More speed -> more rudder authority (K up) and quicker response (T down).
        self.assertGreater(faster.K, p.K)
        self.assertLess(faster.T, p.T)

    def test_estimate_nomoto_from_step(self):
        # Synthesize a first-order step response r(t)=K*delta*(1-exp(-t/T)).
        import math
        K, T, delta = 0.15, 3.0, 10.0
        times = [i * 0.1 for i in range(300)]
        rates = [K * delta * (1 - math.exp(-t / T)) for t in times]
        est = estimate_nomoto_from_step(times, rates, delta)
        self.assertAlmostEqual(est.K, K, places=2)
        self.assertAlmostEqual(est.T, T, delta=0.3)

    def test_gain_schedule_covers_speeds(self):
        p = NomotoParams(K=0.15, T=3.0)
        sched = gain_schedule(p, ref_speed=6.0, speeds=[2, 4, 6, 8],
                              settling_time=8.0)
        self.assertEqual(len(sched), 4)
        speeds = [s for s, _ in sched]
        self.assertEqual(speeds, [2, 4, 6, 8])


if __name__ == "__main__":
    unittest.main()
