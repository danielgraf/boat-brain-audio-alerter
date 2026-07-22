//go:build !linux

package nmea

import (
	"errors"
	"os"
)

// OpenSerial is Linux-only; this stub lets the code build on a dev machine.
func OpenSerial(device string, baud int) (*os.File, error) {
	return nil, errors.New("nmea: serial input is only available on Linux")
}
