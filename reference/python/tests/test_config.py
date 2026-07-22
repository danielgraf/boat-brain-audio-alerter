import os
import tempfile
import unittest

from autopilot.config import AutopilotConfig
from autopilot.ram import MountSide, RamCalibration
from autopilot.tuning import NomotoParams


class TestConfig(unittest.TestCase):
    def test_roundtrip_preserves_values(self):
        cfg = AutopilotConfig()
        cfg.nomoto = NomotoParams(K=0.2, T=2.5)
        cfg.ram = RamCalibration.from_endpoints(
            -1.5, 1.5, mount_side=MountSide.STARBOARD, max_rudder_deg=32)
        cfg.autotune()
        cfg.target_damping = 0.8
        cfg.heading_filter_tau = 1.2

        d = cfg.to_dict()
        back = AutopilotConfig.from_dict(d)

        self.assertAlmostEqual(back.nomoto.K, 0.2)
        self.assertAlmostEqual(back.nomoto.T, 2.5)
        self.assertEqual(back.ram.mount_side, MountSide.STARBOARD)
        self.assertAlmostEqual(back.ram.max_rudder_deg, 32)
        self.assertAlmostEqual(back.target_damping, 0.8)
        self.assertAlmostEqual(back.heading_filter_tau, 1.2)
        self.assertAlmostEqual(back.pid.gains.kp, cfg.pid.gains.kp)

    def test_save_load_file(self):
        cfg = AutopilotConfig()
        cfg.autotune()
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "autopilot.json")
            cfg.save(path)
            self.assertTrue(os.path.exists(path))
            back = AutopilotConfig.load(path)
            self.assertAlmostEqual(back.pid.gains.kp, cfg.pid.gains.kp)
            self.assertEqual(back.ram.mount_side, cfg.ram.mount_side)

    def test_autotune_sets_gains_from_nomoto(self):
        cfg = AutopilotConfig()
        cfg.nomoto = NomotoParams(K=0.15, T=3.0)
        cfg.target_natural_freq = 0.5
        cfg.target_damping = 0.9
        g = cfg.autotune()
        self.assertAlmostEqual(g.kp, 0.5 * 0.5 * 3.0 / 0.15, places=6)


if __name__ == "__main__":
    unittest.main()
