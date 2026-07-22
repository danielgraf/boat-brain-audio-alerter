package ram

// PiConfig configures the Raspberry Pi ram driver.
//
// The ram is driven by a reversible motor through an H-bridge / relay board:
// one GPIO line drives the ram toward starboard rudder, the other toward port.
// The mount-side sign from the Calibration is folded in automatically, so these
// pins are simply "the two motor directions" — if the boat turns the wrong way,
// flip MountSide in the calibration rather than re-wiring.
//
// Rudder-reference feedback is optional. Many tiller pilots have none; with
// Feedback nil the driver dead-reckons stroke from commanded motion. If you have
// a rudder-angle potentiometer, wire it through an SPI ADC (e.g. MCP3008) and
// supply a Feedback func returning the raw stroke reading.
type PiConfig struct {
	StarboardPin int                    // GPIO (BCM) line that drives toward starboard rudder
	PortPin      int                    // GPIO (BCM) line that drives toward port rudder
	Feedback     func() (float64, bool) // optional stroke reader; ok=false when unavailable
	SpeedStroke  float64                // ram speed (stroke units/s) for dead-reckoning
}
