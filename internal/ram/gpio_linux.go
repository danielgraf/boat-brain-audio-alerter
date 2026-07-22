//go:build linux

package ram

import (
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"time"
)

// sysfsGPIO is a minimal Linux sysfs GPIO output line.
//
// sysfs (/sys/class/gpio) is the simplest zero-dependency way to toggle a pin
// and works on every Raspberry Pi. It is deprecated in favour of the gpiochip
// character device; for a slow auto-helm ram the difference is immaterial, and
// this keeps the binary dependency-free. Swap in a gpiochar/periph driver later
// if you want edge interrupts or glitch-free handover.
type sysfsGPIO struct {
	pin   int
	value *os.File
}

func exportGPIO(pin int) (*sysfsGPIO, error) {
	base := fmt.Sprintf("/sys/class/gpio/gpio%d", pin)
	if _, err := os.Stat(base); os.IsNotExist(err) {
		if err := os.WriteFile("/sys/class/gpio/export", []byte(strconv.Itoa(pin)), 0o644); err != nil {
			return nil, fmt.Errorf("export gpio %d: %w", pin, err)
		}
		// Give udev a moment to create the node and set permissions.
		time.Sleep(150 * time.Millisecond)
	}
	if err := os.WriteFile(filepath.Join(base, "direction"), []byte("out"), 0o644); err != nil {
		return nil, fmt.Errorf("set direction gpio %d: %w", pin, err)
	}
	f, err := os.OpenFile(filepath.Join(base, "value"), os.O_WRONLY, 0o644)
	if err != nil {
		return nil, fmt.Errorf("open value gpio %d: %w", pin, err)
	}
	return &sysfsGPIO{pin: pin, value: f}, nil
}

func (g *sysfsGPIO) set(on bool) {
	b := []byte{'0'}
	if on {
		b[0] = '1'
	}
	_, _ = g.value.WriteAt(b, 0)
}

func (g *sysfsGPIO) close() {
	if g.value != nil {
		_ = g.value.Close()
	}
	_ = os.WriteFile("/sys/class/gpio/unexport", []byte(strconv.Itoa(g.pin)), 0o644)
}

// PiRam drives the auto-helm ram from Raspberry Pi GPIO. It satisfies Actuator.
type PiRam struct {
	cal    Calibration
	cfg    PiConfig
	stbd   *sysfsGPIO
	port   *sysfsGPIO
	stroke float64 // dead-reckoned estimate when no feedback is present
}

// NewPiRam opens the GPIO lines and returns a ready driver. The caller must Stop
// (or Close) it to release the pins.
func NewPiRam(cal Calibration, cfg PiConfig) (*PiRam, error) {
	stbd, err := exportGPIO(cfg.StarboardPin)
	if err != nil {
		return nil, err
	}
	port, err := exportGPIO(cfg.PortPin)
	if err != nil {
		stbd.close()
		return nil, err
	}
	r := &PiRam{cal: cal, cfg: cfg, stbd: stbd, port: port, stroke: cal.Centre}
	r.Stop()
	return r, nil
}

func (r *PiRam) currentStroke() float64 {
	if r.cfg.Feedback != nil {
		if v, ok := r.cfg.Feedback(); ok {
			r.stroke = v
			return v
		}
	}
	return r.stroke
}

// Command drives the ram toward rudderDeg for dt seconds and returns the
// achieved rudder angle. It is bang-bang with the ram's motor deadband: within
// DeadbandStroke of target the motor is stopped, which is the classic
// "dead-range" that stops the helm chattering.
func (r *PiRam) Command(rudderDeg, dt float64) float64 {
	target := r.cal.RudderToStroke(rudderDeg)
	cur := r.currentStroke()
	err := target - cur

	if r.cal.DeadbandStroke > 0 && abs(err) < r.cal.DeadbandStroke {
		r.Stop()
		return r.cal.StrokeToRudder(cur)
	}

	// Choose the physical direction, folding in the mount-side drive sign so a
	// reversed install is corrected in config, not wiring. +stroke error with a
	// +DriveSign means drive the starboard line.
	driveStarboard := (err * r.cal.DriveSign()) > 0
	if driveStarboard {
		r.port.set(false)
		r.stbd.set(true)
	} else {
		r.stbd.set(false)
		r.port.set(true)
	}

	// Dead-reckon the stroke when there is no feedback pot.
	if r.cfg.Feedback == nil {
		step := r.cfg.SpeedStroke * dt
		if step > abs(err) {
			step = abs(err)
		}
		if err > 0 {
			r.stroke += step
		} else {
			r.stroke -= step
		}
	}
	return r.cal.StrokeToRudder(r.currentStroke())
}

// Stop cuts drive to both motor lines (clutch out / motor off).
func (r *PiRam) Stop() {
	if r.stbd != nil {
		r.stbd.set(false)
	}
	if r.port != nil {
		r.port.set(false)
	}
}

// RudderAngle returns the best estimate of the current rudder angle.
func (r *PiRam) RudderAngle() float64 { return r.cal.StrokeToRudder(r.currentStroke()) }

// Close stops the ram and releases the GPIO lines.
func (r *PiRam) Close() {
	r.Stop()
	if r.stbd != nil {
		r.stbd.close()
	}
	if r.port != nil {
		r.port.close()
	}
}
