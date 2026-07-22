//go:build linux

package ram

import (
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"sync"
	"time"
)

// sysfsGPIO is a minimal Linux sysfs GPIO output line.
//
// sysfs (/sys/class/gpio) is the simplest zero-dependency way to toggle a pin
// and works on every Raspberry Pi. It is deprecated in favour of the gpiochip
// character device; for an auto-helm drive the difference is immaterial and
// this keeps the binary dependency-free.
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
		time.Sleep(150 * time.Millisecond) // let udev create the node
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
	if g == nil {
		return
	}
	if g.value != nil {
		_ = g.value.Close()
	}
	_ = os.WriteFile("/sys/class/gpio/unexport", []byte(strconv.Itoa(g.pin)), 0o644)
}

// PiRam drives a BTS7960 + drive-unit clutch from Raspberry Pi GPIO. It
// satisfies Actuator. A background goroutine performs software PWM so Command
// only needs to set the desired direction and duty.
type PiRam struct {
	cal  Calibration
	cfg  PiConfig
	rpwm *sysfsGPIO
	lpwm *sysfsGPIO
	en   *sysfsGPIO // optional
	clu  *sysfsGPIO // optional

	mu     sync.Mutex
	dir    int     // -1 port, 0 stop, +1 starboard
	duty   float64 // 0..1
	stroke float64 // dead-reckoned estimate when no feedback is present

	done chan struct{}
	wg   sync.WaitGroup
}

// NewPiRam opens the GPIO lines and starts the PWM loop. Call Close to release.
func NewPiRam(cal Calibration, cfg PiConfig) (*PiRam, error) {
	r := &PiRam{cal: cal, cfg: cfg, stroke: cal.Centre, done: make(chan struct{})}

	var err error
	if r.rpwm, err = exportGPIO(cfg.RPWMPin); err != nil {
		return nil, err
	}
	if r.lpwm, err = exportGPIO(cfg.LPWMPin); err != nil {
		r.rpwm.close()
		return nil, err
	}
	if cfg.EnablePin >= 0 {
		if r.en, err = exportGPIO(cfg.EnablePin); err != nil {
			r.rpwm.close()
			r.lpwm.close()
			return nil, err
		}
	}
	if cfg.ClutchPin >= 0 {
		if r.clu, err = exportGPIO(cfg.ClutchPin); err != nil {
			r.rpwm.close()
			r.lpwm.close()
			r.en.close()
			return nil, err
		}
	}
	r.Stop() // safe state: motor off, clutch released

	r.wg.Add(1)
	go r.pwmLoop()
	return r, nil
}

// pwmLoop toggles the active direction pin according to dir/duty. With PWMHz <= 0
// or duty >= 1 it holds the pin high (bang-bang).
func (r *PiRam) pwmLoop() {
	defer r.wg.Done()
	for {
		select {
		case <-r.done:
			return
		default:
		}
		r.mu.Lock()
		dir, duty := r.dir, r.duty
		hz := r.cfg.PWMHz
		r.mu.Unlock()

		if dir == 0 || duty <= 0 {
			r.rpwm.set(false)
			r.lpwm.set(false)
			time.Sleep(2 * time.Millisecond)
			continue
		}
		active, idle := r.rpwm, r.lpwm
		if dir < 0 {
			active, idle = r.lpwm, r.rpwm
		}
		idle.set(false)

		if hz <= 0 || duty >= 1 {
			active.set(true)
			time.Sleep(2 * time.Millisecond)
			continue
		}
		period := time.Second / time.Duration(hz)
		on := time.Duration(float64(period) * duty)
		active.set(true)
		time.Sleep(on)
		active.set(false)
		time.Sleep(period - on)
	}
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
// achieved rudder angle. Within the ram's motor deadband it stops (the classic
// "dead-range" that stops the helm chattering); otherwise it engages the clutch,
// enables the bridge, and sets direction + speed for the PWM loop.
func (r *PiRam) Command(rudderDeg, dt float64) float64 {
	// Command is called every tick while the pilot is engaged, so keep the
	// clutch engaged and the bridge enabled the whole time — matching the
	// ST4000+ interface where C+ is +12V for as long as the pilot is in Auto,
	// independent of whether the motor is currently moving. (Stop releases both.)
	if r.clu != nil {
		r.clu.set(true)
	}
	if r.en != nil {
		r.en.set(true)
	}

	target := r.cal.RudderToStroke(rudderDeg)
	cur := r.currentStroke()
	errStroke := target - cur

	if r.cal.DeadbandStroke > 0 && abs(errStroke) < r.cal.DeadbandStroke {
		r.hold() // motor off within the dead-range; clutch stays engaged
		return r.cal.StrokeToRudder(cur)
	}

	// Direction, with the mount-side drive sign folded in.
	dir := 1
	if errStroke*r.cal.DriveSign() < 0 {
		dir = -1
	}
	duty := r.dutyFor(abs(errStroke))

	r.mu.Lock()
	r.dir = dir
	r.duty = duty
	r.mu.Unlock()

	// Dead-reckon stroke when there is no feedback pot.
	if r.cfg.Feedback == nil {
		step := r.cfg.SpeedStroke * duty * dt
		if step > abs(errStroke) {
			step = abs(errStroke)
		}
		if errStroke > 0 {
			r.stroke += step
		} else {
			r.stroke -= step
		}
	}
	return r.cal.StrokeToRudder(r.currentStroke())
}

// dutyFor returns the PWM duty for a distance-to-target, tapering to MinDuty
// near the target for a soft landing. Full speed when PWM is disabled.
func (r *PiRam) dutyFor(dist float64) float64 {
	if r.cfg.PWMHz <= 0 {
		return 1
	}
	min := r.cfg.MinDuty
	if min <= 0 {
		min = 0.35
	}
	if r.cfg.RampBandStroke <= 0 || dist >= r.cfg.RampBandStroke {
		return 1
	}
	return min + (1-min)*(dist/r.cfg.RampBandStroke)
}

// hold keeps the clutch engaged but stops the motor (used inside the deadband so
// the drive still holds the helm without chattering).
func (r *PiRam) hold() {
	r.mu.Lock()
	r.dir, r.duty = 0, 0
	r.mu.Unlock()
}

// Stop cuts drive to the motor, disables the bridge and releases the clutch so
// the helm is free for hand steering (standby).
func (r *PiRam) Stop() {
	r.mu.Lock()
	r.dir, r.duty = 0, 0
	r.mu.Unlock()
	if r.rpwm != nil {
		r.rpwm.set(false)
	}
	if r.lpwm != nil {
		r.lpwm.set(false)
	}
	if r.en != nil {
		r.en.set(false)
	}
	if r.clu != nil {
		r.clu.set(false)
	}
}

// RudderAngle returns the best estimate of the current rudder angle.
func (r *PiRam) RudderAngle() float64 { return r.cal.StrokeToRudder(r.currentStroke()) }

// Close stops the ram, ends the PWM loop and releases the GPIO lines.
func (r *PiRam) Close() {
	r.Stop()
	close(r.done)
	r.wg.Wait()
	r.rpwm.close()
	r.lpwm.close()
	r.en.close()
	r.clu.close()
}
