//go:build !linux

package ram

import "errors"

// PiRam is unavailable off Linux; this stub lets the rest of the code build on a
// dev machine (macOS/Windows) while the real driver is Linux-only.
type PiRam struct{}

// NewPiRam returns an error on non-Linux platforms.
func NewPiRam(cal Calibration, cfg PiConfig) (*PiRam, error) {
	return nil, errors.New("ram: GPIO ram driver is only available on Linux (Raspberry Pi)")
}

// Command is a no-op stub.
func (r *PiRam) Command(rudderDeg, dt float64) float64 { return 0 }

// Stop is a no-op stub.
func (r *PiRam) Stop() {}

// RudderAngle is a no-op stub.
func (r *PiRam) RudderAngle() float64 { return 0 }

// Close is a no-op stub.
func (r *PiRam) Close() {}
