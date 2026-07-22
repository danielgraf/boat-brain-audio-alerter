package controller

import (
	"math"
	"testing"

	"github.com/danielgraf/boat-brain-audio-alerter/internal/boatsim"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/config"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/heading"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/ram"
)

func build() (config.Autopilot, *ram.SimRam, *CourseKeeper) {
	cfg := config.Default()
	rw := ram.NewSimRam(cfg.Ram, cfg.RamSpeedStroke)
	return cfg, rw, New(cfg, rw)
}

// simulate runs the closed loop against the Nomoto boat and returns it.
func simulate(k *CourseKeeper, rw *ram.SimRam, cfg config.Autopilot, env boatsim.Environment, seconds float64) *boatsim.Boat {
	boat := boatsim.New(cfg.Nomoto.K, cfg.Nomoto.T, cfg.RefSpeedKnots)
	dt := 0.05
	steps := int(seconds / dt)
	for i := 0; i < steps; i++ {
		cog := boat.COG()
		sog := boat.SOGKnots()
		stw := boat.STWKnots
		k.Update(boat.Heading, dt, nil, &stw, &cog, &sog)
		boat.Step(rw.RudderAngle(), env, dt)
	}
	return boat
}

func TestEngageLocksHeading(t *testing.T) {
	_, _, k := build()
	k.Engage(45)
	if !k.Engaged() || math.Abs(k.DesiredHeading()-45) > 1e-9 {
		t.Errorf("engage failed: engaged=%v desired=%v", k.Engaged(), k.DesiredHeading())
	}
}

func TestStandbyStops(t *testing.T) {
	_, _, k := build()
	k.Engage(90)
	k.Standby()
	tick := k.Update(90, 0.1, nil, nil, nil, nil)
	if tick.Engaged {
		t.Error("should be disengaged")
	}
}

func TestAdjustHeadingWraps(t *testing.T) {
	_, _, k := build()
	k.Engage(350)
	k.AdjustHeading(20)
	if math.Abs(k.DesiredHeading()-10) > 1e-9 {
		t.Errorf("wrap failed: %v", k.DesiredHeading())
	}
}

func TestHeadingHoldConverges(t *testing.T) {
	cfg, rw, k := build()
	k.Engage(0)
	k.SetHeading(30)
	boat := simulate(k, rw, cfg, boatsim.Environment{}, 90)
	if e := math.Abs(heading.Error(30, boat.Heading)); e > 3 {
		t.Errorf("heading hold did not converge, error %v", e)
	}
}

func TestCOGTrackCrabsIntoCurrent(t *testing.T) {
	cfg, rw, k := build()
	k.Engage(0)
	k.SetMode(TrackCOG)
	k.SetCOG(90)
	// 3 kn current setting toward south pushes the boat off an easterly course.
	env := boatsim.Environment{CurrentSpeedKn: 3, CurrentDirDeg: 180}
	boat := simulate(k, rw, cfg, env, 150)

	cogErr := math.Abs(heading.Error(90, boat.COG()))
	if cogErr > 4 {
		t.Errorf("COG track did not hold ground course, COG error %v", cogErr)
	}
	// The boat must be crabbing: heading distinctly north of the 90° ground track.
	crab := heading.Error(boat.Heading, boat.COG())
	if crab > -15 {
		t.Errorf("expected the bow crabbed into the current (crab ~ -30), got %v", crab)
	}
}

func TestHeadingHoldDriftsUnderCurrent(t *testing.T) {
	// Contrast: plain heading hold does NOT hold the ground course under current.
	cfg, rw, k := build()
	k.Engage(0)
	k.SetMode(HeadingHold)
	k.SetHeading(90)
	env := boatsim.Environment{CurrentSpeedKn: 3, CurrentDirDeg: 180}
	boat := simulate(k, rw, cfg, env, 120)
	if e := math.Abs(heading.Error(90, boat.Heading)); e > 3 {
		t.Errorf("heading itself should hold 90, error %v", e)
	}
	if cogErr := math.Abs(heading.Error(90, boat.COG())); cogErr < 10 {
		t.Errorf("COG should have drifted well off 90 under current, error only %v", cogErr)
	}
}

func TestOffCourseAlarm(t *testing.T) {
	cfg := config.Default()
	cfg.OffCourseDeg = 20
	cfg.OffCourseSecs = 20
	rw := ram.NewSimRam(cfg.Ram, cfg.RamSpeedStroke)
	k := New(cfg, rw)
	k.Engage(0)
	k.SetHeading(0)
	// Hold the boat 40° off course (simulate a stuck heading) for >20 s.
	var last Tick
	for i := 0; i < int(25/0.1); i++ {
		stw := 6.0
		last = k.Update(40, 0.1, nil, &stw, nil, nil)
	}
	if !last.OffCourse {
		t.Errorf("off-course alarm should fire after >20s at 40° off, got %+v", last.OffCourse)
	}
	// Back on course clears it (allow the heading input filter to catch up).
	for i := 0; i < 100; i++ {
		stw := 6.0
		last = k.Update(0, 0.1, nil, &stw, nil, nil)
	}
	if last.OffCourse {
		t.Error("alarm should clear once back on course")
	}
}

func TestOffCourseNotBeforeDelay(t *testing.T) {
	cfg := config.Default()
	rw := ram.NewSimRam(cfg.Ram, cfg.RamSpeedStroke)
	k := New(cfg, rw)
	k.Engage(0)
	k.SetHeading(0)
	var last Tick
	for i := 0; i < int(10/0.1); i++ { // only 10 s < 20 s threshold
		stw := 6.0
		last = k.Update(40, 0.1, nil, &stw, nil, nil)
	}
	if last.OffCourse {
		t.Error("alarm should not fire before the persistence delay")
	}
}

func TestLowSteerageHoldsRudder(t *testing.T) {
	_, _, k := build()
	k.Engage(30)
	k.SetHeading(0)
	slow := 0.2
	tick := k.Update(30, 0.1, nil, &slow, nil, nil)
	if tick.Reason != "low-steerage" {
		t.Errorf("expected low-steerage guard, got %q", tick.Reason)
	}
}
