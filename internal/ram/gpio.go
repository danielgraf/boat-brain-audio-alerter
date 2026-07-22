package ram

// PiConfig configures the Raspberry Pi ram driver for a BTS7960 (IBT-2 / HW-039)
// dual half-bridge driving a Raymarine/Autohelm ST-series drive.
//
// BTS7960 interface (the 8-pin logic header): RPWM, LPWM, R_EN, L_EN, R_IS,
// L_IS, VCC, GND. To drive one way you raise R_EN+L_EN and PWM (or hold high)
// RPWM with LPWM low; the other way swaps RPWM/LPWM. Tie R_EN and L_EN together
// to a single EnablePin so software can also cut the bridge. Power VCC from the
// Pi's 3V3 (BTS7960 logic is happy at 3.3 V).
//
// The mount-side sign from the Calibration is folded in automatically: RPWM is
// "drive toward starboard rudder", LPWM "toward port". If the boat turns the
// wrong way, flip MountSide in the calibration rather than swapping wires.
//
// ClutchPin drives the drive unit's clutch. On the ST4000+ the clutch line
// (C+/C-) is "+12V if the Autopilot is engaged; otherwise 0V", so it is
// energised for the whole time the pilot is in Auto (not just while the motor
// moves) and released on Stop/standby so the helm is free for hand steering.
// The clutch is an inductive 12V load: drive it via a relay or logic-level
// MOSFET with a flyback diode, not straight from the GPIO. Set ClutchPin < 0
// for a drive with no clutch.
//
// Maps to the ST4000+ drive interface: RPWM/LPWM -> the two motor leads
// (MD1/MD2, via the BTS7960's motor output), EnablePin -> R_EN+L_EN,
// ClutchPin -> C+ (C- to 0V). An optional 0-5V rudder reference (P9/P10) can be
// read back through Feedback via an SPI ADC.
//
// Rudder-reference feedback is optional. With Feedback nil the driver
// dead-reckons stroke from commanded motion; with a rudder pot, wire it through
// an SPI ADC (e.g. MCP3008) and supply a Feedback func returning raw stroke.
type PiConfig struct {
	RPWMPin   int // BCM pin -> BTS7960 RPWM (drive toward starboard rudder)
	LPWMPin   int // BCM pin -> BTS7960 LPWM (drive toward port rudder)
	EnablePin int // BCM pin -> R_EN + L_EN tied together; <0 if hardwired high
	ClutchPin int // BCM pin -> clutch driver; <0 for no clutch

	// PWMHz enables proportional ram speed via best-effort software PWM on the
	// active direction pin. 0 = bang-bang (full speed, most reliable). A few
	// hundred Hz is realistic from userspace; the motor's inertia smooths it.
	PWMHz float64
	// MinDuty is the lowest PWM duty that still moves the motor (0..1).
	MinDuty float64
	// RampBandStroke tapers speed to MinDuty within this distance of target for
	// a soft landing (0 = always full speed). Only used when PWMHz > 0.
	RampBandStroke float64

	SpeedStroke float64                // ram speed (stroke/s) for dead-reckoning
	Feedback    func() (float64, bool) // optional stroke reader; ok=false when unavailable
}
