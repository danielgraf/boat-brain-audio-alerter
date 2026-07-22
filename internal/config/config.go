// Package config holds the autopilot configuration and its JSON persistence.
//
// One struct tree holds everything the boat remembers between power cycles: PID
// tuning, deadband/filter settings, ram calibration (mount side, centre, end
// stops), the Nomoto parameters used for auto-tuning, and the COG-track outer
// loop settings. It serialises to a human-readable JSON file you can eyeball or
// hand-edit on the boat.
package config

import (
	"encoding/json"
	"os"

	"github.com/danielgraf/boat-brain-audio-alerter/internal/pid"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/ram"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/tuning"
)

// OuterLoop configures COG (course-over-ground) track steering.
//
// COG steering is a cascade: the fast heading-hold PID is the inner loop, and
// this slow outer loop nudges the target heading so the actual COG matches the
// desired ground course. The offset it settles on is the crab angle the boat
// needs to hold against current/leeway.
type OuterLoop struct {
	KpCrab        float64 `json:"kp_crab"`         // deg heading per deg COG error (proportional)
	KiCrab        float64 `json:"ki_crab"`         // integrator gain (per second) - learns the crab angle
	MaxCrab       float64 `json:"max_crab"`        // clamp on the crab correction (deg)
	ActiveBand    float64 `json:"active_band"`     // only integrate when |COG error| <= this (deg)
	COGFilterTau  float64 `json:"cog_filter_tau"`  // low-pass on the noisy COG signal (s)
	MinTrackSpeed float64 `json:"min_track_speed"` // SOG below which COG is unusable -> hold heading
}

// DefaultOuterLoop is deliberately slow, for time-scale separation from the
// inner heading loop (inner ~8 s, outer ~40 s), which keeps the cascade stable.
func DefaultOuterLoop() OuterLoop {
	return OuterLoop{
		KpCrab:        0.4,
		KiCrab:        0.05,
		MaxCrab:       35,
		ActiveBand:    25,
		COGFilterTau:  2.0,
		MinTrackSpeed: 1.0,
	}
}

// Autopilot is everything persisted for the heading/COG-hold autopilot.
type Autopilot struct {
	PID     pid.Config      `json:"pid"`
	Ram     ram.Calibration `json:"ram"`
	Nomoto  tuning.Nomoto   `json:"nomoto"`
	Targets tuning.Targets  `json:"targets"`
	Outer   OuterLoop       `json:"outer"`

	ControlHz        float64 `json:"control_hz"`
	RamSpeedStroke   float64 `json:"ram_speed_stroke_per_s"`
	RefSpeedKnots    float64 `json:"ref_speed_knots"`
	SpeedScheduling  bool    `json:"speed_scheduling"`
	MinSpeedKnots    float64 `json:"min_speed_knots"`
	HeadingFilterTau float64 `json:"heading_filter_tau"`
}

// Default returns a well-damped default configuration.
func Default() Autopilot {
	p := pid.DefaultConfig()
	p.Deadband = 1.5
	p.IntegralActiveBand = 8
	p.RateFilterTau = 1.0
	p.Limits.OutputMin, p.Limits.OutputMax = -30, 30
	p.Limits.SlewRate = 15
	p.Limits.IntegralLimit = 20

	cal := ram.DefaultCalibration()
	cal.DeadbandStroke = 0.03

	c := Autopilot{
		PID:              p,
		Ram:              cal,
		Nomoto:           tuning.Nomoto{K: 0.15, T: 3.0},
		Targets:          tuning.DefaultTargets(),
		Outer:            DefaultOuterLoop(),
		ControlHz:        10,
		RamSpeedStroke:   2.0,
		RefSpeedKnots:    6.0,
		SpeedScheduling:  true,
		MinSpeedKnots:    1.0,
		HeadingFilterTau: 1.5,
	}
	_ = c.Autotune()
	return c
}

// GainsFor returns pole-placement gains for a plant using the stored feel
// targets.
func (c *Autopilot) GainsFor(n tuning.Nomoto) (pid.Gains, error) {
	return tuning.Gains(n, c.Targets)
}

// Autotune sets PID.Gains from the Nomoto params at the reference speed.
func (c *Autopilot) Autotune() error {
	g, err := c.GainsFor(c.Nomoto)
	if err != nil {
		return err
	}
	c.PID.Gains = g
	return nil
}

// Save writes the config as indented JSON.
func (c *Autopilot) Save(path string) error {
	b, err := json.MarshalIndent(c, "", "  ")
	if err != nil {
		return err
	}
	b = append(b, '\n')
	return os.WriteFile(path, b, 0o644)
}

// Load reads a config from a JSON file.
func Load(path string) (Autopilot, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		return Autopilot{}, err
	}
	var c Autopilot
	if err := json.Unmarshal(b, &c); err != nil {
		return Autopilot{}, err
	}
	return c, nil
}
