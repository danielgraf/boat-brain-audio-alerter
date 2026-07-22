// Package heading provides compass-heading arithmetic.
//
// All angles are in degrees, measured clockwise from north (0 = north,
// 90 = east). The one thing that trips up every heading controller is angle
// wrap-around: 359° and 1° are 2° apart, not 358°. Every comparison in the
// autopilot goes through Error so the sign and magnitude of a correction are
// always the shortest way round.
package heading

import "math"

// Normalize360 wraps an angle into the compass range [0, 360).
func Normalize360(angle float64) float64 {
	a := math.Mod(angle, 360)
	if a < 0 {
		a += 360
	}
	return a
}

// Normalize180 wraps an angle into (-180, 180].
//
// Used for signed errors: positive means "turn to starboard / clockwise",
// negative means "turn to port / anticlockwise".
func Normalize180(angle float64) float64 {
	a := math.Mod(angle+180, 360)
	if a < 0 {
		a += 360
	}
	a -= 180
	if a <= -180 {
		a += 360
	}
	return a
}

// Error is the signed shortest-arc error, desired - measured, in (-180, 180].
//
// A positive result means the boat must turn to starboard (clockwise) to reach
// the desired heading; negative means turn to port.
func Error(desired, measured float64) float64 {
	return Normalize180(desired - measured)
}

// YawRate is the rate of turn (deg/s) between two heading samples, wrap-safe.
//
// Positive = turning to starboard. Returns 0 for a non-positive dt so a stalled
// or duplicated sample can never produce an infinite derivative.
func YawRate(previous, current, dt float64) float64 {
	if dt <= 0 {
		return 0
	}
	return Normalize180(current-previous) / dt
}

// FromVector returns the compass bearing (deg, clockwise from north) of a
// velocity vector given its east and north components. Returns 0 for a zero
// vector. This is how course over ground is derived from a ground-velocity
// vector.
func FromVector(east, north float64) float64 {
	if east == 0 && north == 0 {
		return 0
	}
	return Normalize360(math.Atan2(east, north) * 180 / math.Pi)
}
