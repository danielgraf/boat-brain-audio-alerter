// Package pid implements a heading PID with the damping features a marine
// autopilot actually needs to avoid "hunting" (weaving back and forth across
// the course while sawing the helm).
//
// The extra machinery beyond a textbook PID mirrors what commercial pilots add:
//
//   - Counter rudder = derivative on measurement. The D term acts on the boat's
//     rate of turn, not on the error, feeding in opposite rudder as the bow
//     swings toward target to kill overshoot. Acting on measurement (not error)
//     avoids a "derivative kick" on a new heading.
//   - Deadband: a few degrees of "don't care" so small wander doesn't chatter
//     the ram. Implemented as a soft deadband (subtract the band) for continuity.
//   - Low-pass filtering of the rate signal so compass noise isn't amplified by
//     the derivative term into a buzzing actuator.
//   - Anti-windup: the integral stops accumulating at the rudder stops so it
//     can't wind up and overshoot.
//   - PD during a course change, PID for keeping: the integral is frozen while
//     the error is large (IntegralActiveBand) so a big turn doesn't wind up.
//   - Output slew-rate limit: caps how fast the commanded rudder can change,
//     guaranteeing smooth inputs.
package pid

import "math"

// Gains are the PID tuning constants. Seed them from package tuning.
type Gains struct {
	Kp float64 `json:"kp"` // rudder deg per deg of heading error
	Ki float64 `json:"ki"` // rudder deg per deg-second of accumulated error
	Kd float64 `json:"kd"` // counter rudder: rudder deg per (deg/s) of turn
}

// Limits are the physical / comfort limits applied to the controller output.
type Limits struct {
	OutputMin     float64 `json:"output_min"`     // max rudder to port (deg)
	OutputMax     float64 `json:"output_max"`     // max rudder to starboard (deg)
	SlewRate      float64 `json:"slew_rate"`      // max change in commanded rudder (deg/s)
	IntegralLimit float64 `json:"integral_limit"` // clamp on the integral's rudder contribution (deg)
}

// Config bundles the gains, limits and the damping/comfort settings.
type Config struct {
	Gains              Gains   `json:"gains"`
	Limits             Limits  `json:"limits"`
	Deadband           float64 `json:"deadband"`             // heading error ignored within ± this (deg)
	RateFilterTau      float64 `json:"rate_filter_tau"`      // low-pass time constant for the rate term (s)
	IntegralActiveBand float64 `json:"integral_active_band"` // only integrate when |error| <= this (deg)
}

// DefaultConfig returns sensible limits and a wide-open integral band.
func DefaultConfig() Config {
	return Config{
		Limits: Limits{
			OutputMin:     -35,
			OutputMax:     35,
			SlewRate:      20,
			IntegralLimit: 10,
		},
		Deadband:           2,
		RateFilterTau:      0.5,
		IntegralActiveBand: math.Inf(1),
	}
}

// Debug is a per-step breakdown, handy for tuning plots and the visualizer.
type Debug struct {
	P            float64 `json:"p"`
	I            float64 `json:"i"`
	D            float64 `json:"d"`
	Raw          float64 `json:"raw"` // p+i+d before slew limiting
	Output       float64 `json:"output"`
	FilteredRate float64 `json:"filtered_rate"`
	Saturated    bool    `json:"saturated"`
}

// PID is a discrete PID heading controller. Call Update once per control tick.
type PID struct {
	Config Config

	integral     float64
	filteredRate float64
	output       float64
	haveRate     bool
	Debug        Debug
}

// New creates a PID with the given config.
func New(cfg Config) *PID { return &PID{Config: cfg} }

// Reset clears accumulated state (e.g. when re-engaging the pilot).
func (p *PID) Reset() {
	p.integral = 0
	p.filteredRate = 0
	p.output = 0
	p.haveRate = false
	p.Debug = Debug{}
}

// Output returns the last commanded rudder angle (deg).
func (p *PID) Output() float64 { return p.output }

// Integral returns the current integral accumulator (rudder deg).
func (p *PID) Integral() float64 { return p.integral }

func (p *PID) filterRate(rate, dt float64) float64 {
	tau := p.Config.RateFilterTau
	if tau <= 0 || !p.haveRate {
		p.filteredRate = rate
		p.haveRate = true
		return rate
	}
	alpha := dt / (tau + dt)
	p.filteredRate += alpha * (rate - p.filteredRate)
	return p.filteredRate
}

// Update advances the controller one step and returns the commanded rudder
// angle (deg, starboard positive).
//
// error is the signed heading error desired-measured (deg), already wrapped to
// (-180, 180]. rateOfTurn is the boat yaw rate (deg/s, starboard positive). dt
// is the time since the last update (s).
func (p *PID) Update(errDeg, rateOfTurn, dt float64) float64 {
	if dt <= 0 {
		return p.output
	}
	g := p.Config.Gains
	lim := p.Config.Limits

	// Deadband first: within the band the boat is "on course", so we neither
	// push proportionally nor wind up the integral.
	dbError := softDeadband(errDeg, p.Config.Deadband)
	filteredRate := p.filterRate(rateOfTurn, dt)

	// Proportional.
	pTerm := g.Kp * dbError
	// Derivative (counter rudder): oppose the turn.
	dTerm := -g.Kd * filteredRate

	// Integral, only accumulated near course (PD during a course change).
	accumulation := 0.0
	if math.Abs(errDeg) <= p.Config.IntegralActiveBand {
		accumulation = g.Ki * dbError * dt
	}
	provisional := clamp(p.integral+accumulation, -lim.IntegralLimit, lim.IntegralLimit)
	raw := pTerm + provisional + dTerm
	clamped := clamp(raw, lim.OutputMin, lim.OutputMax)
	saturated := clamped != raw

	// Anti-windup: only keep the new integral if it didn't push further into a
	// rail we're already pinned against.
	if saturated && provisional*(raw-clamped) > 0 {
		// keep old integral
	} else {
		p.integral = provisional
		raw = pTerm + p.integral + dTerm
		clamped = clamp(raw, lim.OutputMin, lim.OutputMax)
		saturated = clamped != raw
	}

	// Output slew-rate limit.
	maxStep := lim.SlewRate * dt
	p.output += clamp(clamped-p.output, -maxStep, maxStep)

	p.Debug = Debug{
		P: pTerm, I: p.integral, D: dTerm, Raw: raw, Output: p.output,
		FilteredRate: filteredRate, Saturated: saturated,
	}
	return p.output
}

// softDeadband shrinks error toward zero by band without a jump at the edge.
func softDeadband(err, band float64) float64 {
	if band <= 0 {
		return err
	}
	switch {
	case err > band:
		return err - band
	case err < -band:
		return err + band
	default:
		return 0
	}
}

func clamp(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}
