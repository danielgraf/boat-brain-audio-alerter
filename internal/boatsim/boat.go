// Package boatsim is a boat course-dynamics simulator for testing the pilot.
//
// The plant is the first-order Nomoto steering model (T·ṙ + r = K·δ), the
// standard tool for autopilot design. On top of the bare yaw model it adds a
// position integrator and the environmental effects that make course over
// ground (COG) differ from heading — current and leeway — so COG-track steering
// can actually be exercised:
//
//	ground velocity = boat-through-water + current + leeway
//	COG = bearing(ground velocity),  SOG = |ground velocity|
//
// Waves add an oscillatory yaw disturbance; wind adds weather helm (a steady
// yaw the integral term must trim) and leeway drift. The package is pure and
// deterministic — sensor noise is added by the caller — so it is easy to test.
package boatsim

import (
	"math"

	"github.com/danielgraf/boat-brain-audio-alerter/internal/heading"
)

// knotsToMS converts knots to metres per second.
const knotsToMS = 0.514444

// Environment holds the disturbances acting on the boat.
type Environment struct {
	WindSpeedKn    float64 `json:"wind_speed_kn"`    // true wind speed (knots)
	WindDirDeg     float64 `json:"wind_dir_deg"`     // direction wind blows FROM (compass)
	WaveAmpDps2    float64 `json:"wave_amp_dps2"`    // wave yaw-accel amplitude (deg/s²)
	WavePeriodS    float64 `json:"wave_period_s"`    // wave encounter period (s)
	CurrentSpeedKn float64 `json:"current_speed_kn"` // current speed (knots)
	CurrentDirDeg  float64 `json:"current_dir_deg"`  // direction current flows TOWARD (compass)
	LeewayCoef     float64 `json:"leeway_coef"`      // leeway drift as a fraction of wind speed
	WeatherHelm    float64 `json:"weather_helm"`     // yaw accel per knot of wind at the beam
}

// DefaultEnvironment is calm water.
func DefaultEnvironment() Environment {
	return Environment{
		WavePeriodS: 6,
		LeewayCoef:  0.03,
		WeatherHelm: 0.01,
	}
}

// Boat is the simulated vessel state.
type Boat struct {
	K, T     float64 // Nomoto parameters
	Heading  float64 // deg, compass
	YawRate  float64 // deg/s, starboard positive
	X, Y     float64 // position: east, north (metres)
	STWKnots float64 // speed through water (knots)

	waveClock float64
	groundVx  float64 // last ground velocity east (m/s)
	groundVy  float64 // last ground velocity north (m/s)
}

// New returns a boat with the given Nomoto parameters and speed through water.
func New(k, t, stwKnots float64) *Boat {
	return &Boat{K: k, T: t, STWKnots: stwKnots}
}

// Step integrates one time step under a rudder angle and environment.
func (b *Boat) Step(rudderDeg float64, env Environment, dt float64) {
	if dt <= 0 {
		return
	}
	b.waveClock += dt

	// Yaw disturbances: oscillatory waves + steady weather helm (rounds up into
	// the wind, proportional to wind strength and the beam-wind component).
	wave := 0.0
	if env.WaveAmpDps2 != 0 && env.WavePeriodS > 0 {
		wave = env.WaveAmpDps2 * math.Sin(2*math.Pi*b.waveClock/env.WavePeriodS)
	}
	relWind := heading.Error(env.WindDirDeg, b.Heading) // + = wind from starboard bow
	weather := env.WeatherHelm * env.WindSpeedKn * math.Sin(relWind*math.Pi/180)

	// First-order Nomoto yaw dynamics (semi-implicit Euler).
	rDot := (b.K*rudderDeg-b.YawRate)/b.T + wave + weather
	b.YawRate += rDot * dt
	b.Heading = heading.Normalize360(b.Heading + b.YawRate*dt)

	// Velocity through the water, along the heading.
	hr := b.Heading * math.Pi / 180
	vw := b.STWKnots * knotsToMS
	vx := vw * math.Sin(hr)
	vy := vw * math.Cos(hr)

	// Current, flowing toward CurrentDirDeg.
	cr := env.CurrentDirDeg * math.Pi / 180
	vc := env.CurrentSpeedKn * knotsToMS
	cx := vc * math.Sin(cr)
	cy := vc * math.Cos(cr)

	// Leeway: the boat slides downwind (wind blows FROM WindDirDeg, so drift is
	// TOWARD WindDirDeg+180).
	lr := (env.WindDirDeg + 180) * math.Pi / 180
	lw := env.LeewayCoef * env.WindSpeedKn * knotsToMS
	lx := lw * math.Sin(lr)
	ly := lw * math.Cos(lr)

	b.groundVx = vx + cx + lx
	b.groundVy = vy + cy + ly
	b.X += b.groundVx * dt
	b.Y += b.groundVy * dt
}

// COG returns the course over ground (deg, compass). Undefined for a stationary
// boat; returns the current heading as a sensible fallback.
func (b *Boat) COG() float64 {
	if b.groundVx == 0 && b.groundVy == 0 {
		return b.Heading
	}
	return heading.FromVector(b.groundVx, b.groundVy)
}

// SOGKnots returns the speed over ground in knots.
func (b *Boat) SOGKnots() float64 {
	return math.Hypot(b.groundVx, b.groundVy) / knotsToMS
}
