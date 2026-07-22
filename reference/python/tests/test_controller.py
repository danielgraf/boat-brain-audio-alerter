import unittest

from autopilot.config import AutopilotConfig
from autopilot.controller import CourseKeeper
from autopilot.ram import MountSide, RamCalibration, SimRam
from autopilot.simulator import BoatModel, Scenario, evaluate, run_closed_loop
from autopilot.tuning import NomotoParams


def build(config_overrides=None):
    cfg = AutopilotConfig()
    cfg.nomoto = NomotoParams(K=0.15, T=3.0)
    cfg.autotune()
    cfg.pid.deadband = 1.5
    cfg.pid.integral_active_band = 8.0
    cfg.heading_filter_tau = 1.0
    cfg.ram = RamCalibration.from_endpoints(-1, 1, mount_side=MountSide.PORT, max_rudder_deg=35)
    if config_overrides:
        config_overrides(cfg)
    ram = SimRam(cfg.ram, speed_stroke_per_s=cfg.ram_speed_stroke_per_s)
    return cfg, ram, CourseKeeper(cfg, ram)


class TestCourseKeeper(unittest.TestCase):
    def test_engage_locks_current_heading(self):
        cfg, ram, keeper = build()
        keeper.engage(current_heading=45.0)
        self.assertTrue(keeper.engaged)
        self.assertAlmostEqual(keeper.desired_heading, 45.0)

    def test_standby_stops(self):
        cfg, ram, keeper = build()
        keeper.engage(90.0, current_heading=90.0)
        keeper.standby()
        self.assertFalse(keeper.engaged)
        tick = keeper.update(90.0, 0.1)
        self.assertFalse(tick.engaged)

    def test_adjust_heading_wraps(self):
        cfg, ram, keeper = build()
        keeper.engage(350.0, current_heading=350.0)
        keeper.adjust_heading(20.0)
        self.assertAlmostEqual(keeper.desired_heading, 10.0)

    def test_low_steerage_holds_rudder(self):
        cfg, ram, keeper = build()
        keeper.engage(0.0, current_heading=30.0)
        # Big error but no steerage: should not command a correction.
        tick = keeper.update(30.0, 0.1, speed_knots=0.2)
        self.assertEqual(tick.reason, "low-steerage")

    def test_closed_loop_converges_and_is_damped(self):
        cfg, ram, keeper = build()
        boat = BoatModel(K=cfg.nomoto.K, T=cfg.nomoto.T, speed_knots=6.0)
        scen = Scenario(name="t", initial_heading=0.0, desired_heading=25.0, duration_s=90.0)
        log = run_closed_loop(keeper, boat, ram, scen, control_hz=cfg.control_hz)
        m = evaluate(log)
        self.assertLess(m.max_overshoot, 6.0)           # well damped
        self.assertIsNotNone(m.settling_time)           # actually settles
        self.assertLess(abs(m.steady_state_error), 3.0) # holds course

    def test_wind_bias_trimmed_by_integral(self):
        from autopilot.simulator import Disturbances
        cfg, ram, keeper = build(lambda c: setattr(c.pid.limits, "integral_limit", 20.0))
        boat = BoatModel(K=cfg.nomoto.K, T=cfg.nomoto.T, speed_knots=6.0)
        scen = Scenario(name="wind", initial_heading=90.0, desired_heading=90.0,
                        duration_s=150.0, disturbances=Disturbances(steady_yaw_dps2=0.4))
        log = run_closed_loop(keeper, boat, ram, scen, control_hz=cfg.control_hz)
        m = evaluate(log)
        # Integral should pull the steady offset back within the deadband-ish band.
        self.assertLess(abs(m.steady_state_error), 3.0)


if __name__ == "__main__":
    unittest.main()
