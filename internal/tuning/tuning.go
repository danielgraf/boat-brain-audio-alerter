// Package tuning derives well-damped PID gains instead of guessing them.
//
// The boat's yaw behaves (to first order) like the Nomoto model
//
//	T·ṙ + r = K·δ            (r = yaw rate, δ = rudder angle)
//
// With proportional + counter-rudder (PD on heading, derivative on yaw rate)
// the closed loop is a standard second-order system, so we can place its poles
// and get gains that are damped by construction:
//
//	Kp = ωₙ²·T / K
//	Kd = (2·ζ·ωₙ·T − 1) / K
//
// You choose two intuitive numbers: ζ (damping ratio; 0.9–1.0 gives no
// overshoot and no hunting) and ωₙ (bandwidth, or equivalently a settling time).
package tuning

import (
	"errors"
	"math"

	"github.com/danielgraf/boat-brain-audio-alerter/internal/pid"
)

// Nomoto holds first-order steering parameters.
//
// K is the yaw-rate gain (per second): steady yaw rate = K·rudder. T is the
// time constant (seconds): larger = more sluggish / more inertia.
type Nomoto struct {
	K float64 `json:"k"`
	T float64 `json:"t"`
}

// ScaledToSpeed scales K, T from refSpeed to speed.
//
// Rudder force ~ speed² while yaw rate ~ speed, so to first order K ~ speed and
// T ~ 1/speed. Used for speed-scheduled gains.
func (n Nomoto) ScaledToSpeed(speed, refSpeed float64) Nomoto {
	if refSpeed <= 0 || speed <= 0 {
		return n
	}
	ratio := speed / refSpeed
	return Nomoto{K: n.K * ratio, T: n.T / ratio}
}

// Targets are the "how it should feel" dials the gains derive from.
type Targets struct {
	Damping          float64 `json:"damping"`           // ζ; ~0.9 = brisk, no hunting
	SettlingTime     float64 `json:"settling_time"`     // s; used if NaturalFreq is 0
	NaturalFreq      float64 `json:"natural_freq"`      // rad/s; overrides SettlingTime if > 0
	IntegralFraction float64 `json:"integral_fraction"` // integral aggressiveness (0 disables)
}

// DefaultTargets returns a well-damped default (ζ=0.9, ~8 s settling).
func DefaultTargets() Targets {
	return Targets{Damping: 0.9, SettlingTime: 8, IntegralFraction: 0.1}
}

// Gains returns pole-placement PID gains for a first-order Nomoto plant.
func Gains(n Nomoto, t Targets) (pid.Gains, error) {
	if n.K == 0 || n.T <= 0 {
		return pid.Gains{}, errors.New("tuning: Nomoto K must be non-zero and T positive")
	}
	zeta := t.Damping
	if zeta <= 0 {
		zeta = 0.9
	}

	var wn float64
	switch {
	case t.NaturalFreq > 0:
		wn = t.NaturalFreq
	case t.SettlingTime > 0:
		// 2% settling time of a 2nd-order system ≈ 4/(ζ·ωₙ).
		wn = 4 / (zeta * t.SettlingTime)
	default:
		wn = 1 / n.T
	}

	absK := math.Abs(n.K)
	kp := wn * wn * n.T / n.K
	kd := (2*zeta*wn*n.T - 1) / absK
	if kd < 0 {
		kd = 0
	}
	ki := 0.0
	if t.IntegralFraction > 0 {
		ki = t.IntegralFraction * (zeta * wn) * kp
	}
	// If the plant K is negative (rudder sense reversed) keep the control-law
	// sign consistent with a positive effective K; the ram's mount-side sign
	// handles the wiring separately.
	if n.K < 0 {
		kp = -kp
		ki = -ki
	}
	return pid.Gains{Kp: kp, Ki: ki, Kd: kd}, nil
}

// EstimateNomotoFromStep recovers K and T from a rudder-step yaw-rate response.
//
// Hold the boat steady, put the rudder to rudderDeg and hold it, and log time
// vs yaw rate. The response is r(t) = K·δ·(1 − e^(−t/T)): K from the steady
// state, T as the time to reach 63.2% of it.
func EstimateNomotoFromStep(times, yawRates []float64, rudderDeg float64) (Nomoto, error) {
	if len(times) != len(yawRates) || len(times) < 3 {
		return Nomoto{}, errors.New("tuning: need matching, non-trivial time/rate series")
	}
	if rudderDeg == 0 {
		return Nomoto{}, errors.New("tuning: rudderDeg must be non-zero")
	}
	tail := len(yawRates) / 5
	if tail < 1 {
		tail = 1
	}
	var sum float64
	for _, r := range yawRates[len(yawRates)-tail:] {
		sum += r
	}
	rFinal := sum / float64(tail)
	k := rFinal / rudderDeg

	target := 0.632 * rFinal
	t0 := times[0]
	var tc float64
	found := false
	for i, r := range yawRates {
		if (rFinal >= 0 && r >= target) || (rFinal < 0 && r <= target) {
			tc = times[i] - t0
			found = true
			break
		}
	}
	if !found || tc <= 0 {
		tc = (times[len(times)-1] - times[0]) / 3
	}
	return Nomoto{K: k, T: tc}, nil
}
