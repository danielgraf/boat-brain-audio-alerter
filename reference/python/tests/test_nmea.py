import unittest

from autopilot.nmea import NmeaHeadingSource, parse_sentence


def with_checksum(body: str) -> str:
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    return f"${body}*{calc:02X}"


class TestNmea(unittest.TestCase):
    def test_hdt_true_heading(self):
        fix = parse_sentence(with_checksum("GPHDT,123.4,T"))
        self.assertIsNotNone(fix)
        self.assertAlmostEqual(fix.heading_true, 123.4)

    def test_hdg_magnetic(self):
        fix = parse_sentence(with_checksum("HCHDG,88.0,,,2.0,W"))
        self.assertAlmostEqual(fix.heading_magnetic, 88.0)

    def test_rot_deg_per_min(self):
        fix = parse_sentence(with_checksum("TIROT,-12.0,A"))
        self.assertAlmostEqual(fix.rot_deg_per_min, -12.0)

    def test_rot_invalid_status_dropped(self):
        fix = parse_sentence(with_checksum("TIROT,-12.0,V"))
        self.assertIsNone(fix.rot_deg_per_min)

    def test_rmc_cog_and_sog(self):
        body = "GPRMC,123519,A,4807.038,N,01131.000,E,22.4,84.4,230394,003.1,W"
        fix = parse_sentence(with_checksum(body))
        self.assertAlmostEqual(fix.heading_true, 84.4)
        self.assertAlmostEqual(fix.speed_knots, 22.4)

    def test_bad_checksum_rejected(self):
        self.assertIsNone(parse_sentence("$GPHDT,123.4,T*00"))

    def test_missing_checksum_accepted(self):
        fix = parse_sentence("$GPHDT,123.4,T")
        self.assertIsNotNone(fix)
        self.assertAlmostEqual(fix.heading_true, 123.4)

    def test_non_sentence_returns_none(self):
        self.assertIsNone(parse_sentence("hello"))
        self.assertIsNone(parse_sentence(""))

    def test_source_prefers_true_then_magnetic(self):
        src = NmeaHeadingSource(prefer_true=True)
        src.update(with_checksum("HCHDG,88.0,,,,"))
        self.assertAlmostEqual(src.heading, 88.0)   # only magnetic so far
        src.update(with_checksum("GPHDT,90.0,T"))
        self.assertAlmostEqual(src.heading, 90.0)   # true now available, preferred

    def test_source_rot_to_deg_per_sec(self):
        src = NmeaHeadingSource()
        src.update(with_checksum("TIROT,60.0,A"))
        self.assertAlmostEqual(src.rot_deg_per_sec, 1.0)  # 60 deg/min = 1 deg/s


if __name__ == "__main__":
    unittest.main()
