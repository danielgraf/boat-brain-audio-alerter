package heading

import (
	"math"
	"testing"
)

func almost(t *testing.T, got, want, tol float64, msg string) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s: got %v want %v", msg, got, want)
	}
}

func TestNormalize360(t *testing.T) {
	almost(t, Normalize360(370), 10, 1e-9, "370")
	almost(t, Normalize360(-10), 350, 1e-9, "-10")
}

func TestNormalize180(t *testing.T) {
	almost(t, Normalize180(190), -170, 1e-9, "190")
	almost(t, Normalize180(-190), 170, 1e-9, "-190")
	almost(t, Normalize180(180), 180, 1e-9, "180")
	almost(t, Normalize180(-180), 180, 1e-9, "-180 maps to +180")
}

func TestErrorShortestArc(t *testing.T) {
	almost(t, Error(10, 350), 20, 1e-9, "10 vs 350")
	almost(t, Error(350, 10), -20, 1e-9, "350 vs 10")
	almost(t, Error(1, 359), 2, 1e-9, "1 vs 359")
}

func TestErrorNeverExceeds180(t *testing.T) {
	for d := 0; d < 360; d += 7 {
		for m := 0; m < 360; m += 11 {
			if e := Error(float64(d), float64(m)); math.Abs(e) > 180+1e-9 {
				t.Fatalf("Error(%d,%d)=%v exceeds 180", d, m, e)
			}
		}
	}
}

func TestYawRateWrapSafe(t *testing.T) {
	almost(t, YawRate(359, 1, 0.5), 4, 1e-9, "359->1")
	almost(t, YawRate(1, 359, 0.5), -4, 1e-9, "1->359")
	if YawRate(0, 10, 0) != 0 {
		t.Error("zero dt must give 0")
	}
}

func TestFromVector(t *testing.T) {
	almost(t, FromVector(0, 1), 0, 1e-9, "north")
	almost(t, FromVector(1, 0), 90, 1e-9, "east")
	almost(t, FromVector(0, -1), 180, 1e-9, "south")
	almost(t, FromVector(-1, 0), 270, 1e-9, "west")
}
