package boatsim

import (
	"math"
	"testing"

	"github.com/danielgraf/boat-brain-audio-alerter/internal/heading"
)

func TestSteadyStateYawRate(t *testing.T) {
	b := New(0.15, 3, 6)
	for i := 0; i < 3000; i++ {
		b.Step(10, Environment{}, 0.05)
	}
	// r_ss = K * delta
	if math.Abs(b.YawRate-0.15*10) > 0.02 {
		t.Errorf("steady yaw rate got %v want %v", b.YawRate, 0.15*10)
	}
}

func TestPositiveRudderTurnsStarboard(t *testing.T) {
	b := New(0.15, 3, 6)
	for i := 0; i < 50; i++ {
		b.Step(10, Environment{}, 0.1)
	}
	if b.Heading <= 0 || b.Heading > 180 {
		t.Errorf("should turn to starboard, heading %v", b.Heading)
	}
}

func TestCurrentMakesCOGDifferFromHeading(t *testing.T) {
	b := New(0.15, 3, 6)
	b.Heading = 90 // pointing east
	// Current setting toward south (180).
	env := Environment{CurrentSpeedKn: 3, CurrentDirDeg: 180}
	b.Step(0, env, 0.1)
	// COG should be south of east (between 90 and 180).
	cog := b.COG()
	if cog <= 90 || cog >= 180 {
		t.Errorf("COG should be south of east, got %v", cog)
	}
	// And distinctly different from heading.
	if math.Abs(heading.Error(cog, b.Heading)) < 5 {
		t.Errorf("current should separate COG from heading, crab=%v", heading.Error(cog, b.Heading))
	}
}

func TestPositionIntegrates(t *testing.T) {
	b := New(0.15, 3, 6)
	b.Heading = 0 // north
	for i := 0; i < 100; i++ {
		b.Step(0, Environment{}, 0.1)
	}
	// Moving north: Y (north) should grow, X (east) ~ 0.
	if b.Y <= 0 || math.Abs(b.X) > 1e-6 {
		t.Errorf("expected northward motion, got X=%v Y=%v", b.X, b.Y)
	}
	if b.SOGKnots() < 5.5 || b.SOGKnots() > 6.5 {
		t.Errorf("SOG should be ~STW with no current, got %v", b.SOGKnots())
	}
}
