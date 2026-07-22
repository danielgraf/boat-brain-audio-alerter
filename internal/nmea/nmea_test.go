package nmea

import (
	"fmt"
	"math"
	"testing"
)

func withChecksum(body string) string {
	var c byte
	for i := 0; i < len(body); i++ {
		c ^= body[i]
	}
	return fmt.Sprintf("$%s*%02X", body, c)
}

func TestParseHDT(t *testing.T) {
	f := Parse(withChecksum("GPHDT,123.4,T"))
	if f == nil || f.HeadingTrue == nil || math.Abs(*f.HeadingTrue-123.4) > 1e-9 {
		t.Fatalf("HDT parse failed: %+v", f)
	}
}

func TestParseRMC(t *testing.T) {
	f := Parse(withChecksum("GPRMC,123519,A,4807.038,N,01131.000,E,22.4,84.4,230394,003.1,W"))
	if f == nil || f.COGTrue == nil || math.Abs(*f.COGTrue-84.4) > 1e-9 {
		t.Fatalf("RMC COG failed: %+v", f)
	}
	if f.SpeedKnots == nil || math.Abs(*f.SpeedKnots-22.4) > 1e-9 {
		t.Fatalf("RMC SOG failed: %+v", f)
	}
}

func TestParseROTInvalid(t *testing.T) {
	f := Parse(withChecksum("TIROT,-12.0,V"))
	if f == nil || f.ROTDegPerMin != nil {
		t.Fatalf("invalid ROT should be nil: %+v", f)
	}
}

func TestBadChecksum(t *testing.T) {
	if Parse("$GPHDT,123.4,T*00") != nil {
		t.Error("bad checksum should be rejected")
	}
}

func TestMissingChecksumAccepted(t *testing.T) {
	if Parse("$GPHDT,123.4,T") == nil {
		t.Error("missing checksum should be accepted")
	}
}

func TestSourcePrefersTrue(t *testing.T) {
	s := NewSource()
	s.Update(withChecksum("HCHDG,88.0,,,,"))
	if h := s.Heading(); h == nil || *h != 88 {
		t.Fatalf("magnetic fallback failed: %v", h)
	}
	s.Update(withChecksum("GPHDT,90.0,T"))
	if h := s.Heading(); h == nil || *h != 90 {
		t.Fatalf("should prefer true: %v", h)
	}
}

func TestSourceROTPerSec(t *testing.T) {
	s := NewSource()
	s.Update(withChecksum("TIROT,60.0,A"))
	if r := s.ROTDegPerSec(); r == nil || math.Abs(*r-1) > 1e-9 {
		t.Fatalf("60 deg/min should be 1 deg/s: %v", r)
	}
}
