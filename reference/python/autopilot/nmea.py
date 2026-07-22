"""Minimal NMEA 0183 parser for the sentences an autopilot needs.

We only care about a handful of fields:

* heading  - HDG / HDT / HDM  (and, as a fallback, the heading in VTG/RMC COG)
* speed    - VHW (speed through water) / RMC / VTG (speed over ground)
* rate of turn - ROT

A dependency (``pynmea2``) exists and is excellent, but heading-hold needs so
little that a ~100-line, allocation-free, checksum-validating parser keeps the
boat brain free of pip installs. Swap in ``pynmea2`` later if you want the full
sentence set - :class:`NmeaHeadingSource` only depends on the parsed dataclass.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class NmeaFix:
    """Whatever navigation state the most recent sentence carried.

    Every field is optional: a single ROT sentence sets only ``rot`` and leaves
    heading/speed untouched, so consumers should merge successive fixes rather
    than expect one sentence to be complete.
    """

    heading_true: Optional[float] = None
    heading_magnetic: Optional[float] = None
    speed_knots: Optional[float] = None
    rot_deg_per_min: Optional[float] = None
    sentence_type: Optional[str] = None


def _checksum_ok(sentence: str) -> bool:
    """Validate the ``*HH`` XOR checksum if one is present.

    Sentences without a checksum are accepted (some talkers omit it); a present
    but wrong checksum is rejected.
    """
    if "*" not in sentence:
        return True
    body, _, chk = sentence.partition("*")
    chk = chk.strip()
    if len(chk) < 2:
        return False
    calc = 0
    for ch in body.lstrip("$!"):
        calc ^= ord(ch)
    try:
        return calc == int(chk[:2], 16)
    except ValueError:
        return False


def _to_float(field: str) -> Optional[float]:
    if not field:
        return None
    try:
        return float(field)
    except ValueError:
        return None


def parse_sentence(raw: str) -> Optional[NmeaFix]:
    """Parse one NMEA 0183 line into a :class:`NmeaFix`.

    Returns ``None`` for empty input, a bad checksum, or a sentence type we
    don't handle, so a caller can simply ignore ``None``.
    """
    if raw is None:
        return None
    sentence = raw.strip()
    if not sentence.startswith(("$", "!")):
        return None
    if not _checksum_ok(sentence):
        return None

    body = sentence.split("*", 1)[0].lstrip("$!")
    parts = body.split(",")
    if not parts or len(parts[0]) < 5:
        return None

    # parts[0] is the 5-char address: 2-char talker id + 3-char sentence id.
    stype = parts[0][2:]
    fields = parts[1:]

    if stype == "HDT":  # true heading
        return NmeaFix(heading_true=_to_float(fields[0]) if fields else None,
                       sentence_type=stype)
    if stype in ("HDM",):  # magnetic heading
        return NmeaFix(heading_magnetic=_to_float(fields[0]) if fields else None,
                       sentence_type=stype)
    if stype == "HDG":  # heading, deviation & variation
        mag = _to_float(fields[0]) if fields else None
        return NmeaFix(heading_magnetic=mag, sentence_type=stype)
    if stype == "ROT":  # rate of turn, deg/min, + = bow to starboard
        val = _to_float(fields[0]) if fields else None
        status = fields[1] if len(fields) > 1 else "A"
        if status == "V":  # data invalid
            val = None
        return NmeaFix(rot_deg_per_min=val, sentence_type=stype)
    if stype == "VHW":  # water speed & heading
        # fields: headT,T,headM,M,spdN,N,spdK,K
        headt = _to_float(fields[0]) if len(fields) > 0 else None
        headm = _to_float(fields[2]) if len(fields) > 2 else None
        spd = _to_float(fields[4]) if len(fields) > 4 else None
        return NmeaFix(heading_true=headt, heading_magnetic=headm,
                       speed_knots=spd, sentence_type=stype)
    if stype == "RMC":  # recommended minimum: COG(true) + SOG
        # fields: time,status,lat,N,lon,E,SOG,COG,date,magvar,E/W
        spd = _to_float(fields[6]) if len(fields) > 6 else None
        cog = _to_float(fields[7]) if len(fields) > 7 else None
        return NmeaFix(heading_true=cog, speed_knots=spd, sentence_type=stype)
    if stype == "VTG":  # course & speed over ground
        cog = _to_float(fields[0]) if len(fields) > 0 else None
        spd = _to_float(fields[4]) if len(fields) > 4 else None  # knots
        return NmeaFix(heading_true=cog, speed_knots=spd, sentence_type=stype)

    return None


class NmeaHeadingSource:
    """Stateful accumulator: feed it raw lines, read back the latest state.

    It prefers *true* heading (course made good over the ground the boat is
    actually holding), falling back to magnetic when true is unavailable. Rate
    of turn is reported straight from a ROT sentence if the talker provides one
    (a real rate gyro is far cleaner than differentiating a compass); otherwise
    the controller derives it from successive headings.
    """

    def __init__(self, prefer_true: bool = True):
        self.prefer_true = prefer_true
        self.heading_true: Optional[float] = None
        self.heading_magnetic: Optional[float] = None
        self.speed_knots: Optional[float] = None
        self.rot_deg_per_min: Optional[float] = None

    def update(self, raw: str) -> Optional[NmeaFix]:
        fix = parse_sentence(raw)
        if fix is None:
            return None
        if fix.heading_true is not None:
            self.heading_true = fix.heading_true
        if fix.heading_magnetic is not None:
            self.heading_magnetic = fix.heading_magnetic
        if fix.speed_knots is not None:
            self.speed_knots = fix.speed_knots
        if fix.rot_deg_per_min is not None:
            self.rot_deg_per_min = fix.rot_deg_per_min
        return fix

    @property
    def heading(self) -> Optional[float]:
        """Best available heading, honouring ``prefer_true``."""
        primary = self.heading_true if self.prefer_true else self.heading_magnetic
        secondary = self.heading_magnetic if self.prefer_true else self.heading_true
        return primary if primary is not None else secondary

    @property
    def rot_deg_per_sec(self) -> Optional[float]:
        """Sensor rate of turn in deg/s (ROT sentences are per minute)."""
        if self.rot_deg_per_min is None:
            return None
        return self.rot_deg_per_min / 60.0
