// Command autopilot is the on-boat binary: it reads NMEA from a serial port,
// runs the heading/COG-hold controller, and drives the auto-helm ram over
// Raspberry Pi GPIO.
//
//	sudo ./autopilot -device /dev/ttyAMA0 -baud 4800 \
//	     -config /etc/boatbrain/autopilot.json \
//	     -stbd-pin 23 -port-pin 24 -heading 90
//
// Use -cog instead of -heading to hold a course over ground. With neither, it
// engages on the first heading it sees ("hold what I've got").
package main

import (
	"bufio"
	"flag"
	"log"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/danielgraf/boat-brain-audio-alerter/internal/config"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/controller"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/nmea"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/ram"
)

func main() {
	var (
		device   = flag.String("device", "/dev/ttyAMA0", "NMEA serial device")
		baud     = flag.Int("baud", 4800, "serial baud rate")
		cfgPath  = flag.String("config", "", "autopilot config JSON (optional)")
		stbdPin  = flag.Int("stbd-pin", 23, "BCM GPIO pin driving toward starboard rudder")
		portPin  = flag.Int("port-pin", 24, "BCM GPIO pin driving toward port rudder")
		holdHead = flag.Float64("heading", -1, "heading to hold (deg); <0 = hold current")
		holdCOG  = flag.Float64("cog", -1, "course over ground to hold (deg); enables COG track")
		hz       = flag.Float64("hz", 10, "control loop rate (Hz)")
	)
	flag.Parse()

	cfg := config.Default()
	if *cfgPath != "" {
		loaded, err := config.Load(*cfgPath)
		if err != nil {
			log.Fatalf("load config: %v", err)
		}
		cfg = loaded
	}

	// Open the ram driver (Raspberry Pi GPIO).
	piRam, err := ram.NewPiRam(cfg.Ram, ram.PiConfig{
		StarboardPin: *stbdPin,
		PortPin:      *portPin,
		SpeedStroke:  cfg.RamSpeedStroke,
	})
	if err != nil {
		log.Fatalf("ram: %v", err)
	}
	defer piRam.Close()

	// Open the NMEA serial feed.
	port, err := nmea.OpenSerial(*device, *baud)
	if err != nil {
		log.Fatalf("serial: %v", err)
	}
	defer port.Close()

	src := nmea.NewSource()
	var mu sync.Mutex

	// Reader goroutine: parse NMEA lines into the shared source.
	go func() {
		sc := bufio.NewScanner(port)
		for sc.Scan() {
			mu.Lock()
			src.Update(sc.Text())
			mu.Unlock()
		}
		if err := sc.Err(); err != nil {
			log.Printf("serial read: %v", err)
		}
	}()

	keeper := controller.New(cfg, piRam)

	// Graceful shutdown: stop driving the ram.
	sigc := make(chan os.Signal, 1)
	signal.Notify(sigc, syscall.SIGINT, syscall.SIGTERM)
	go func() {
		<-sigc
		keeper.Standby()
		piRam.Close()
		log.Println("autopilot stopped, helm released")
		os.Exit(0)
	}()

	ticker := time.NewTicker(time.Duration(float64(time.Second) / *hz))
	defer ticker.Stop()
	last := time.Now()
	engaged := false

	for now := range ticker.C {
		dt := now.Sub(last).Seconds()
		last = now

		mu.Lock()
		hdgPtr := src.Heading()
		rot := src.ROTDegPerSec()
		spd := src.SpeedKnots
		cog := src.COGTrue
		mu.Unlock()

		if hdgPtr == nil {
			continue // wait for a heading fix
		}
		heading := *hdgPtr

		if !engaged {
			switch {
			case *holdCOG >= 0:
				keeper.Engage(heading)
				keeper.SetMode(controller.TrackCOG)
				keeper.SetCOG(*holdCOG)
			case *holdHead >= 0:
				keeper.Engage(*holdHead)
			default:
				keeper.Engage(heading)
			}
			engaged = true
			log.Printf("engaged: mode=%s setpoint=%.0f", keeper.Mode(), setpointOf(keeper))
		}

		// SOG doubles as the steerage-speed signal when present.
		tick := keeper.Update(heading, dt, rot, spd, cog, spd)
		_ = tick
	}
}

func setpointOf(k *controller.CourseKeeper) float64 {
	if k.Mode() == controller.TrackCOG {
		return k.DesiredCOG()
	}
	return k.DesiredHeading()
}
