package pid

import (
	"math"
	"testing"
)

func testPID() *PID {
	return New(Config{
		Gains:              Gains{Kp: 2},
		Limits:             Limits{OutputMin: -30, OutputMax: 30, SlewRate: 1000, IntegralLimit: 20},
		IntegralActiveBand: math.Inf(1),
	})
}

func TestProportional(t *testing.T) {
	p := testPID()
	if got := p.Update(5, 0, 0.1); math.Abs(got-10) > 1e-9 {
		t.Errorf("got %v want 10", got)
	}
}

func TestOutputClamp(t *testing.T) {
	p := testPID()
	if got := p.Update(100, 0, 0.1); got != 30 {
		t.Errorf("got %v want 30", got)
	}
}

func TestSlewRate(t *testing.T) {
	p := testPID()
	p.Config.Limits.SlewRate = 10
	if got := p.Update(100, 0, 0.1); math.Abs(got-1) > 1e-9 {
		t.Errorf("got %v want 1 (10 deg/s * 0.1s)", got)
	}
}

func TestDeadband(t *testing.T) {
	p := testPID()
	p.Config.Deadband = 3
	if got := p.Update(2, 0, 0.1); got != 0 {
		t.Errorf("within band want 0 got %v", got)
	}
	p.Reset()
	if got := p.Update(5, 0, 0.1); math.Abs(got-4) > 1e-9 { // (5-3)*2
		t.Errorf("soft deadband got %v want 4", got)
	}
}

func TestCounterRudderOpposesTurn(t *testing.T) {
	p := testPID()
	p.Config.Gains = Gains{Kd: 5}
	if p.Update(0, 2, 0.1) >= 0 {
		t.Error("starboard turn should give port (negative) rudder")
	}
	p.Reset()
	if p.Update(0, -2, 0.1) <= 0 {
		t.Error("port turn should give starboard (positive) rudder")
	}
}

func TestIntegralClamp(t *testing.T) {
	p := testPID()
	p.Config.Gains = Gains{Ki: 1}
	p.Config.Limits.IntegralLimit = 5
	var out float64
	for i := 0; i < 1000; i++ {
		out = p.Update(10, 0, 0.1)
	}
	if math.Abs(out-5) > 1e-6 {
		t.Errorf("integral should clamp at 5, got %v", out)
	}
}

func TestAdaptiveDeadbandWidensInSeaway(t *testing.T) {
	p := testPID()
	p.Config.Gains = Gains{Kp: 2}
	p.Config.Deadband = 1.5
	p.Config.AdaptiveDeadband = true
	p.Config.DeadbandMax = 6
	p.Config.SeastateGain = 1.2
	p.Config.SeastateTau = 4
	// Feed an oscillating error (wave-induced wiggle) around zero mean.
	for i := 0; i < 4000; i++ {
		e := 3 * math.Sin(float64(i)*0.05) // ±3° wiggle, zero mean
		p.Update(e, 0, 0.05)
	}
	if p.Debug.Deadband <= 1.5 {
		t.Errorf("adaptive deadband should widen in a seaway, got %v", p.Debug.Deadband)
	}
	if p.Debug.Deadband > 6.0001 {
		t.Errorf("adaptive deadband should not exceed max, got %v", p.Debug.Deadband)
	}
}

func TestAdaptiveDeadbandStaysTightOnFlatWater(t *testing.T) {
	p := testPID()
	p.Config.Deadband = 1.5
	p.Config.AdaptiveDeadband = true
	p.Config.DeadbandMax = 6
	p.Config.SeastateGain = 1.2
	p.Config.SeastateTau = 4
	for i := 0; i < 2000; i++ {
		p.Update(0.2, 0, 0.05) // steady, no wiggle
	}
	if p.Debug.Deadband > 1.8 {
		t.Errorf("deadband should stay near base on flat water, got %v", p.Debug.Deadband)
	}
}

func TestIntegralActiveBand(t *testing.T) {
	p := testPID()
	p.Config.Gains = Gains{Ki: 1}
	p.Config.IntegralActiveBand = 5
	for i := 0; i < 50; i++ {
		p.Update(10, 0, 0.1) // error 10 > band 5, frozen
	}
	if p.Integral() != 0 {
		t.Errorf("integral should be frozen on big error, got %v", p.Integral())
	}
	p.Update(3, 0, 0.1) // within band now
	if p.Integral() <= 0 {
		t.Error("integral should accumulate within band")
	}
}
