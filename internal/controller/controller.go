// Package controller is the top-level "hold a course" loop.
//
// It supports two modes:
//
//   - HeadingHold: hold a compass heading. error = desired - measured, fed to
//     the damped PID which drives the ram.
//
//   - TrackCOG: hold a course over ground. This is a cascade. The heading-hold
//     PID stays as the fast inner loop; a slow outer loop nudges the target
//     heading so the *actual* COG matches the desired ground course. The offset
//     the outer loop settles on is exactly the crab angle needed to counter
//     current and leeway. COG is noisy and meaningless at low speed, so below
//     MinTrackSpeed the controller falls back to holding heading. Time-scale
//     separation (inner ~8 s, outer ~40 s) keeps the cascade stable.
package controller

import (
	"math"

	"github.com/danielgraf/boat-brain-audio-alerter/internal/config"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/heading"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/pid"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/ram"
)

// Mode is the steering reference.
type Mode string

const (
	HeadingHold Mode = "heading"
	TrackCOG    Mode = "cog"
)

// Tick is the diagnostics returned from each Update call.
type Tick struct {
	Engaged         bool    `json:"engaged"`
	Mode            Mode    `json:"mode"`
	DesiredHeading  float64 `json:"desired_heading"`
	DesiredCOG      float64 `json:"desired_cog"`
	HeadingSetpoint float64 `json:"heading_setpoint"`
	MeasuredHeading float64 `json:"measured_heading"`
	MeasuredCOG     float64 `json:"measured_cog"`
	Error           float64 `json:"error"`
	YawRate         float64 `json:"yaw_rate"`
	Crab            float64 `json:"crab"`
	RudderCommand   float64 `json:"rudder_command"`
	RudderAchieved  float64 `json:"rudder_achieved"`
	SpeedKnots      float64 `json:"speed_knots"`
	Reason          string  `json:"reason"`
	OffCourse       bool    `json:"off_course"` // heading off the locked course beyond alarm limits
}

// CourseKeeper is the closed-loop heading/COG controller.
type CourseKeeper struct {
	cfg config.Autopilot
	ram ram.Actuator
	pid *pid.PID

	engaged        bool
	mode           Mode
	desiredHeading float64
	desiredCOG     float64

	headingSetpoint float64
	haveSetpoint    bool

	prevHeading     float64
	havePrev        bool
	filteredHeading float64
	haveFiltered    bool
	filteredCOG     float64
	haveFilteredCOG bool

	crabIntegral float64
	trackInit    bool
	baseGains    pid.Gains

	offCourseFor float64 // seconds the heading has been beyond the alarm limit
	offCourse    bool
}

// New builds a CourseKeeper around a config and an actuator.
func New(cfg config.Autopilot, actuator ram.Actuator) *CourseKeeper {
	return &CourseKeeper{
		cfg:       cfg,
		ram:       actuator,
		pid:       pid.New(cfg.PID),
		mode:      HeadingHold,
		baseGains: cfg.PID.Gains,
	}
}

// PID exposes the inner controller (for the visualizer's tuning readouts).
func (k *CourseKeeper) PID() *pid.PID { return k.pid }

// SetBaseGains updates the un-scheduled gains (e.g. after a live re-tune). The
// speed scheduler rescales from these each tick.
func (k *CourseKeeper) SetBaseGains(g pid.Gains) {
	k.baseGains = g
	k.pid.Config.Gains = g
}

// Engaged reports whether the pilot is driving the helm.
func (k *CourseKeeper) Engaged() bool { return k.engaged }

// Mode returns the current steering reference.
func (k *CourseKeeper) Mode() Mode { return k.mode }

// DesiredHeading / DesiredCOG expose the current setpoints.
func (k *CourseKeeper) DesiredHeading() float64 { return k.desiredHeading }
func (k *CourseKeeper) DesiredCOG() float64     { return k.desiredCOG }

// Engage engages the pilot. With no explicit setpoint it locks onto the current
// heading ("press AUTO and hold what I've got").
func (k *CourseKeeper) Engage(currentHeading float64) {
	k.desiredHeading = heading.Normalize360(currentHeading)
	k.headingSetpoint = k.desiredHeading
	k.haveSetpoint = true
	k.engaged = true
	k.trackInit = false
	k.crabIntegral = 0
	k.offCourseFor = 0
	k.offCourse = false
	k.pid.Reset()
	k.prevHeading = heading.Normalize360(currentHeading)
	k.havePrev = true
	k.filteredHeading = heading.Normalize360(currentHeading)
	k.haveFiltered = true
}

// Standby disengages: stop driving the ram and let the helm go free.
func (k *CourseKeeper) Standby() {
	k.engaged = false
	k.ram.Stop()
}

// SetMode switches steering reference. Switching to COG-track re-learns the crab
// angle bumplessly on the next usable fix.
func (k *CourseKeeper) SetMode(m Mode) {
	if m != k.mode {
		k.mode = m
		k.trackInit = false
	}
}

// SetHeading dials in a new heading to hold (deg).
func (k *CourseKeeper) SetHeading(h float64) { k.desiredHeading = heading.Normalize360(h) }

// SetCOG dials in a new course over ground to hold (deg).
func (k *CourseKeeper) SetCOG(c float64) { k.desiredCOG = heading.Normalize360(c) }

// AdjustHeading nudges the heading setpoint (e.g. ±1 / ±10 buttons).
func (k *CourseKeeper) AdjustHeading(delta float64) {
	k.desiredHeading = heading.Normalize360(k.desiredHeading + delta)
}

// AdjustCOG nudges the COG setpoint.
func (k *CourseKeeper) AdjustCOG(delta float64) {
	k.desiredCOG = heading.Normalize360(k.desiredCOG + delta)
}

// lowpassAngle moves a filtered angle toward a target along the shortest arc.
func lowpassAngle(filtered, target, tau, dt float64) float64 {
	if tau <= 0 {
		return target
	}
	alpha := dt / (tau + dt)
	return heading.Normalize360(filtered + alpha*heading.Error(target, filtered))
}

func (k *CourseKeeper) applySpeedScheduling(speed float64) {
	if !k.cfg.SpeedScheduling {
		k.pid.Config.Gains = k.baseGains
		return
	}
	v := math.Max(speed, k.cfg.MinSpeedKnots)
	scaled := k.cfg.Nomoto.ScaledToSpeed(v, k.cfg.RefSpeedKnots)
	if g, err := k.cfg.GainsFor(scaled); err == nil {
		k.pid.Config.Gains = g
	} else {
		k.pid.Config.Gains = k.baseGains
	}
}

// outerLoop updates the heading setpoint from the COG error (cascade).
func (k *CourseKeeper) outerLoop(cog, measuredHeading, dt float64) float64 {
	o := k.cfg.Outer
	if k.haveFilteredCOG {
		k.filteredCOG = lowpassAngle(k.filteredCOG, cog, o.COGFilterTau, dt)
	} else {
		k.filteredCOG = heading.Normalize360(cog)
		k.haveFilteredCOG = true
	}
	e := heading.Error(k.desiredCOG, k.filteredCOG)
	nearCourse := math.Abs(e) <= o.ActiveBand

	switch {
	case !k.trackInit:
		// Seed the crab integral with the crab the boat is currently flying
		// (heading - desiredCOG) so acquisition steers from a sensible offset
		// rather than lurching, and the integral already holds ~the value it
		// needs to learn.
		k.crabIntegral = clamp(heading.Error(measuredHeading, k.desiredCOG), -o.MaxCrab, o.MaxCrab)
		k.trackInit = true
	case nearCourse:
		// Integrate only near course, with anti-windup (don't push a saturated
		// crab further into its limit). Far from course the error is large and
		// one-signed for many seconds, which would wind the crab up the wrong way.
		cand := clamp(k.crabIntegral+o.KiCrab*e*dt, -o.MaxCrab, o.MaxCrab)
		if (cand < o.MaxCrab || e < 0) && (cand > -o.MaxCrab || e > 0) {
			k.crabIntegral = cand
		}
	}

	// Near course, add proportional fine-tuning; during acquisition steer on the
	// learned crab alone so the big COG error doesn't fling the setpoint past
	// target.
	crab := k.crabIntegral
	if nearCourse {
		crab = o.KpCrab*e + k.crabIntegral
	}
	crab = clamp(crab, -o.MaxCrab, o.MaxCrab)
	return heading.Normalize360(k.desiredCOG + crab)
}

// Update runs one control tick.
//
// measuredHeading is the compass heading (deg). dt is seconds since the last
// tick. sensorYawRate, speedKnots, cog and sog are optional (pass nil when the
// value isn't available); cog/sog drive TrackCOG mode.
func (k *CourseKeeper) Update(measuredHeading, dt float64, sensorYawRate, speedKnots, cog, sog *float64) Tick {
	raw := heading.Normalize360(measuredHeading)
	speed := k.cfg.RefSpeedKnots
	if speedKnots != nil {
		speed = *speedKnots
	}

	// Low-pass the compass input (wrap-aware) so the derivative term isn't fed
	// noise. Real fluxgate/GPS compasses are physically damped; this models it.
	if k.cfg.HeadingFilterTau > 0 && k.haveFiltered {
		k.filteredHeading = lowpassAngle(k.filteredHeading, raw, k.cfg.HeadingFilterTau, dt)
	} else {
		k.filteredHeading = raw
		k.haveFiltered = true
	}
	measured := k.filteredHeading

	var rate float64
	switch {
	case sensorYawRate != nil:
		rate = *sensorYawRate
	case k.havePrev:
		rate = heading.YawRate(k.prevHeading, measured, dt)
	}
	k.prevHeading = measured
	k.havePrev = true

	measuredCOG := 0.0
	if cog != nil {
		measuredCOG = heading.Normalize360(*cog)
	}

	if !k.engaged {
		k.offCourseFor, k.offCourse = 0, false
		return Tick{
			Mode: k.mode, DesiredHeading: k.desiredHeading, DesiredCOG: k.desiredCOG,
			MeasuredHeading: measured, MeasuredCOG: measuredCOG, YawRate: rate,
			RudderCommand: k.ram.RudderAngle(), RudderAchieved: k.ram.RudderAngle(),
			SpeedKnots: speed, Reason: "standby",
		}
	}

	// Decide the heading setpoint.
	reason := "hold"
	cogUsable := k.mode == TrackCOG && cog != nil && sog != nil && *sog >= k.cfg.Outer.MinTrackSpeed
	switch {
	case cogUsable:
		k.headingSetpoint = k.outerLoop(*cog, measured, dt)
		reason = "track-cog"
	case k.mode == TrackCOG:
		// COG unusable (too slow / no fix): hold the last heading setpoint.
		k.trackInit = false
		reason = "cog-lowspeed-hold"
	default:
		k.headingSetpoint = k.desiredHeading
	}
	if !k.haveSetpoint {
		k.headingSetpoint = k.desiredHeading
		k.haveSetpoint = true
	}

	errDeg := heading.Error(k.headingSetpoint, measured)

	// Off-course alarm: heading off the locked course beyond the limit for too
	// long (ST4000 default 20° for 20 s).
	if k.cfg.OffCourseDeg > 0 && math.Abs(errDeg) > k.cfg.OffCourseDeg {
		k.offCourseFor += dt
	} else {
		k.offCourseFor = 0
	}
	k.offCourse = k.cfg.OffCourseSecs > 0 && k.offCourseFor >= k.cfg.OffCourseSecs

	// Displayed crab: the actual offset of the bow from the ground track.
	crab := heading.Error(measured, measuredCOG)
	if k.mode != TrackCOG {
		crab = 0
	}

	// Low-steerage guard: no water flow over the rudder means no authority, so
	// hold station rather than winding up and slamming the helm over.
	if speedKnots != nil && *speedKnots < k.cfg.MinSpeedKnots {
		achieved := k.ram.Command(k.ram.RudderAngle(), dt)
		return Tick{
			Engaged: true, Mode: k.mode, DesiredHeading: k.desiredHeading, DesiredCOG: k.desiredCOG,
			HeadingSetpoint: k.headingSetpoint, MeasuredHeading: measured, MeasuredCOG: measuredCOG,
			Error: errDeg, YawRate: rate, Crab: crab, RudderCommand: k.ram.RudderAngle(),
			RudderAchieved: achieved, SpeedKnots: speed, Reason: "low-steerage", OffCourse: k.offCourse,
		}
	}

	k.applySpeedScheduling(speed)
	rudderCmd := k.pid.Update(errDeg, rate, dt)
	achieved := k.ram.Command(rudderCmd, dt)

	return Tick{
		Engaged: true, Mode: k.mode, DesiredHeading: k.desiredHeading, DesiredCOG: k.desiredCOG,
		HeadingSetpoint: k.headingSetpoint, MeasuredHeading: measured, MeasuredCOG: measuredCOG,
		Error: errDeg, YawRate: rate, Crab: crab, RudderCommand: rudderCmd,
		RudderAchieved: achieved, SpeedKnots: speed, Reason: reason, OffCourse: k.offCourse,
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
