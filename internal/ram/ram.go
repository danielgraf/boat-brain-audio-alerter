// Package ram models the auto-helm ram: calibration and the rudder-angle ->
// drive mapping, plus a simulated actuator and the Actuator interface the real
// hardware driver satisfies.
//
// The controller thinks in rudder angle (deg, starboard positive). The physical
// world is a linear ram shoving the tiller. This package owns the boat-specific
// setup:
//
//   - MountSide: which side the ram is mounted. Extending the ram turns the boat
//     one way or the other depending on the side, captured as DriveSign so a
//     reversed install is a config change, not rewiring.
//   - Centre calibration: the feedback reading at which the rudder is amidships.
//     Calibrate by driving to each mechanical stop and taking the midpoint
//     (FromEndpoints).
//   - Equal travel each side: SymmetricTravel clamps usable stroke to the
//     smaller half so the helm has identical authority to port and starboard.
package ram

import "errors"

// MountSide is which side of the boat the ram is mounted; it sets drive sense.
type MountSide string

const (
	Port      MountSide = "port"
	Starboard MountSide = "starboard"
)

// Calibration maps rudder angle to ram stroke. Stroke units are whatever the
// feedback device reports (mm, ADC counts, ...) as long as they are linear with
// rudder angle; only ratios matter.
type Calibration struct {
	MountSide       MountSide `json:"mount_side"`
	Centre          float64   `json:"centre"`           // stroke at rudder amidships
	PortStop        float64   `json:"port_stop"`        // stroke at full port rudder
	StarboardStop   float64   `json:"starboard_stop"`   // stroke at full starboard rudder
	MaxRudderDeg    float64   `json:"max_rudder_deg"`   // rudder angle at the end stops
	SymmetricTravel bool      `json:"symmetric_travel"` // force equal usable travel about centre
	DeadbandStroke  float64   `json:"deadband_stroke"`  // ignore commands within this of current
}

// DefaultCalibration is a symmetric, port-mounted ram with normalized stroke.
func DefaultCalibration() Calibration {
	return Calibration{
		MountSide: Port, Centre: 0, PortStop: -1, StarboardStop: 1,
		MaxRudderDeg: 35, SymmetricTravel: true,
	}
}

// FromEndpoints builds a calibration from a dockside end-stop sweep: centre is
// the midpoint so the rudder sits amidships with equal stroke available each way.
func FromEndpoints(portStop, starboardStop float64, side MountSide, maxRudderDeg float64, symmetric bool) (Calibration, error) {
	if portStop == starboardStop {
		return Calibration{}, errors.New("ram: port and starboard stops must differ")
	}
	if maxRudderDeg <= 0 {
		return Calibration{}, errors.New("ram: maxRudderDeg must be positive")
	}
	return Calibration{
		MountSide:       side,
		Centre:          0.5 * (portStop + starboardStop),
		PortStop:        portStop,
		StarboardStop:   starboardStop,
		MaxRudderDeg:    maxRudderDeg,
		SymmetricTravel: symmetric,
	}, nil
}

// DriveSign is +1 or -1: the sign relating positive (starboard) rudder to
// increasing stroke, corrected for mount side.
func (c Calibration) DriveSign() float64 {
	base := 1.0
	if c.StarboardStop < c.PortStop {
		base = -1
	}
	if c.MountSide == Starboard {
		return -base
	}
	return base
}

// HalfTravel is the usable stroke from centre to the (possibly
// symmetric-limited) stop.
func (c Calibration) HalfTravel() float64 {
	toPort := abs(c.Centre - c.PortStop)
	toStbd := abs(c.StarboardStop - c.Centre)
	if c.SymmetricTravel {
		return min(toPort, toStbd)
	}
	return max(toPort, toStbd)
}

// RudderToStroke converts a commanded rudder angle to a target stroke reading.
func (c Calibration) RudderToStroke(rudderDeg float64) float64 {
	rudderDeg = clamp(rudderDeg, -c.MaxRudderDeg, c.MaxRudderDeg)
	frac := rudderDeg / c.MaxRudderDeg // -1..1, starboard positive
	var target float64
	if c.SymmetricTravel {
		target = c.Centre + c.DriveSign()*frac*c.HalfTravel()
	} else {
		var span float64
		if frac >= 0 {
			span = abs(c.StarboardStop - c.Centre)
		} else {
			span = abs(c.Centre - c.PortStop)
		}
		target = c.Centre + frac*span*c.DriveSign()
	}
	lo, hi := c.PortStop, c.StarboardStop
	if lo > hi {
		lo, hi = hi, lo
	}
	return clamp(target, lo, hi)
}

// StrokeToRudder is the inverse of RudderToStroke, for reading feedback back out.
func (c Calibration) StrokeToRudder(stroke float64) float64 {
	half := c.HalfTravel()
	if half == 0 {
		return 0
	}
	frac := (stroke - c.Centre) / (c.DriveSign() * half)
	return clamp(frac, -1, 1) * c.MaxRudderDeg
}

// Actuator accepts a rudder-angle command and reports the achieved angle. Both
// SimRam and the real Pi driver implement it.
type Actuator interface {
	// Command drives toward rudderDeg for dt seconds; returns achieved deg.
	Command(rudderDeg, dt float64) float64
	// Stop cuts drive to the ram (clutch out / motor off).
	Stop()
	// RudderAngle is the best estimate of the current rudder angle (deg).
	RudderAngle() float64
}

// SimRam is a software ram with finite speed and hard end stops. It models the
// two things that matter for control quality: the ram can only move so fast, and
// it stops dead at the mechanical limits.
type SimRam struct {
	Cal    Calibration
	Speed  float64 // stroke units per second
	stroke float64
}

// NewSimRam creates a simulated ram starting at the calibrated centre.
func NewSimRam(cal Calibration, speedStrokePerSec float64) *SimRam {
	return &SimRam{Cal: cal, Speed: abs(speedStrokePerSec), stroke: cal.Centre}
}

// Stroke returns the current raw stroke reading (for the visualizer).
func (r *SimRam) Stroke() float64 { return r.stroke }

// Command implements Actuator.
func (r *SimRam) Command(rudderDeg, dt float64) float64 {
	if dt <= 0 {
		return r.RudderAngle()
	}
	target := r.Cal.RudderToStroke(rudderDeg)
	if r.Cal.DeadbandStroke > 0 && abs(target-r.stroke) < r.Cal.DeadbandStroke {
		return r.RudderAngle()
	}
	maxStep := r.Speed * dt
	r.stroke += clamp(target-r.stroke, -maxStep, maxStep)
	lo, hi := r.Cal.PortStop, r.Cal.StarboardStop
	if lo > hi {
		lo, hi = hi, lo
	}
	r.stroke = clamp(r.stroke, lo, hi)
	return r.RudderAngle()
}

// Stop implements Actuator (a sim ram simply holds position).
func (r *SimRam) Stop() {}

// RudderAngle implements Actuator.
func (r *SimRam) RudderAngle() float64 { return r.Cal.StrokeToRudder(r.stroke) }

func clamp(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
