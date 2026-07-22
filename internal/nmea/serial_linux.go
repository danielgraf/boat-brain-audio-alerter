//go:build linux

package nmea

import (
	"fmt"
	"os"

	"golang.org/x/sys/unix"
)

// baudConstant maps a baud rate to its termios speed constant. NMEA 0183 talkers
// are almost always 4800 (standard) or 38400 (high-speed / AIS-multiplexed).
func baudConstant(baud int) (uint32, error) {
	switch baud {
	case 4800:
		return unix.B4800, nil
	case 9600:
		return unix.B9600, nil
	case 19200:
		return unix.B19200, nil
	case 38400:
		return unix.B38400, nil
	case 57600:
		return unix.B57600, nil
	case 115200:
		return unix.B115200, nil
	default:
		return 0, fmt.Errorf("nmea: unsupported baud %d", baud)
	}
}

// OpenSerial opens a serial device (e.g. /dev/ttyAMA0, /dev/ttyUSB0) at the
// given baud in raw 8N1 mode and returns it as an io.ReadCloser of NMEA lines.
func OpenSerial(device string, baud int) (*os.File, error) {
	speed, err := baudConstant(baud)
	if err != nil {
		return nil, err
	}
	f, err := os.OpenFile(device, os.O_RDWR|unix.O_NOCTTY|unix.O_NONBLOCK, 0)
	if err != nil {
		return nil, fmt.Errorf("nmea: open %s: %w", device, err)
	}
	fd := int(f.Fd())

	t := unix.Termios{
		Cflag: unix.CLOCAL | unix.CREAD | unix.CS8, // 8 data bits, ignore modem lines
		Iflag: unix.IGNPAR,                         // ignore framing/parity errors
	}
	t.Cc[unix.VMIN] = 1  // block for at least 1 byte
	t.Cc[unix.VTIME] = 0 // no inter-byte timer
	// Set both input and output speed.
	if err := unix.IoctlSetTermios(fd, unix.TCSETS, termiosWithSpeed(&t, speed)); err != nil {
		f.Close()
		return nil, fmt.Errorf("nmea: configure %s: %w", device, err)
	}
	// Back to blocking reads now that the port is configured.
	if err := unix.SetNonblock(fd, false); err != nil {
		f.Close()
		return nil, fmt.Errorf("nmea: set blocking %s: %w", device, err)
	}
	return f, nil
}

// termiosWithSpeed encodes the baud into the termios struct. On Linux the speed
// is carried in the low bits of Cflag (and mirrored in Ispeed/Ospeed).
func termiosWithSpeed(t *unix.Termios, speed uint32) *unix.Termios {
	t.Cflag |= speed
	t.Ispeed = speed
	t.Ospeed = speed
	return t
}
