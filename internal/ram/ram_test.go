package ram

import (
	"math"
	"testing"
)

func TestCentreMidpoint(t *testing.T) {
	c, err := FromEndpoints(100, 300, Port, 35, true)
	if err != nil {
		t.Fatal(err)
	}
	if c.Centre != 200 {
		t.Errorf("centre got %v want 200", c.Centre)
	}
	if got := c.RudderToStroke(0); math.Abs(got-200) > 1e-9 {
		t.Errorf("amidships stroke got %v want 200", got)
	}
}

func TestSymmetricTravelEqualEachSide(t *testing.T) {
	c := Calibration{Centre: 100, PortStop: 50, StarboardStop: 250, MaxRudderDeg: 30, SymmetricTravel: true}
	if c.HalfTravel() != 50 {
		t.Errorf("half travel got %v want 50 (smaller half)", c.HalfTravel())
	}
	stbd := c.RudderToStroke(30) - c.Centre
	port := c.RudderToStroke(-30) - c.Centre
	if math.Abs(stbd+port) > 1e-9 {
		t.Errorf("travel not symmetric: +%v -%v", stbd, port)
	}
}

func TestMountSideFlipsDriveSign(t *testing.T) {
	p, _ := FromEndpoints(-1, 1, Port, 35, true)
	s, _ := FromEndpoints(-1, 1, Starboard, 35, true)
	if p.DriveSign() != -s.DriveSign() {
		t.Errorf("mount side should flip drive sign: %v %v", p.DriveSign(), s.DriveSign())
	}
}

func TestRudderStrokeRoundtrip(t *testing.T) {
	c, _ := FromEndpoints(-2, 2, Port, 35, true)
	for _, d := range []float64{-35, -20, 0, 15, 35} {
		if got := c.StrokeToRudder(c.RudderToStroke(d)); math.Abs(got-d) > 1e-6 {
			t.Errorf("roundtrip %v got %v", d, got)
		}
	}
}

func TestNeverPastStop(t *testing.T) {
	c, _ := FromEndpoints(-1, 1, Port, 35, true)
	if s := c.RudderToStroke(1000); s > 1+1e-9 || s < -1-1e-9 {
		t.Errorf("commanded past stop: %v", s)
	}
}

func TestSimRamSpeedLimit(t *testing.T) {
	c, _ := FromEndpoints(-1, 1, Port, 35, true)
	r := NewSimRam(c, 0.5)
	r.Command(35, 0.1) // target +1 stroke, only 0.05 allowed
	if math.Abs(r.Stroke()-0.05) > 1e-9 {
		t.Errorf("speed limit: stroke got %v want 0.05", r.Stroke())
	}
}

func TestSimRamReachesTarget(t *testing.T) {
	c, _ := FromEndpoints(-1, 1, Port, 35, true)
	r := NewSimRam(c, 2)
	for i := 0; i < 200; i++ {
		r.Command(20, 0.1)
	}
	if math.Abs(r.RudderAngle()-20) > 1e-3 {
		t.Errorf("did not reach target: %v", r.RudderAngle())
	}
}

func TestSimRamMotorDeadband(t *testing.T) {
	c, _ := FromEndpoints(-1, 1, Port, 35, true)
	c.DeadbandStroke = 0.1
	r := NewSimRam(c, 2)
	r.Command(1, 0.1) // ~0.03 stroke, inside deadband
	if r.Stroke() != 0 {
		t.Errorf("motor deadband should suppress tiny move, got %v", r.Stroke())
	}
}
