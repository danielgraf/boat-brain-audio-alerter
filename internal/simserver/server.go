// Package simserver runs the boat simulation and the autopilot in real time and
// serves an interactive web visualizer.
//
// Architecture: a single goroutine ticks the physics + control loop at a fixed
// rate under a mutex. The browser reads state via Server-Sent Events (GET
// /api/stream) and pushes control inputs via POST /api/command and
// /api/environment. This uses only the standard library (SSE + JSON), so the
// binary is self-contained and cross-compiles cleanly to a Raspberry Pi.
package simserver

import (
	"embed"
	"encoding/json"
	"fmt"
	"io/fs"
	"math"
	"math/rand"
	"net/http"
	"sync"
	"time"

	"github.com/danielgraf/boat-brain-audio-alerter/internal/boatsim"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/config"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/controller"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/ram"
)

//go:embed static
var staticFS embed.FS

// Server owns the simulation and HTTP handlers.
type Server struct {
	mu      sync.Mutex
	cfg     config.Autopilot
	boat    *boatsim.Boat
	ram     *ram.SimRam
	keeper  *controller.CourseKeeper
	env     boatsim.Environment
	rng     *rand.Rand
	tick    controller.Tick
	trail   []point
	elapsed float64

	// Sensor model.
	headingNoise float64
	sensorHz     float64
	lastSensor   float64
	measHeading  float64
	measCOG      float64
	measSOG      float64
}

type point struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	T float64 `json:"-"` // record time, not serialized
}

// New builds a server with a default calm-water scenario.
func New() *Server {
	cfg := config.Default()
	boat := boatsim.New(cfg.Nomoto.K, cfg.Nomoto.T, cfg.RefSpeedKnots)
	boat.Heading = 0
	rw := ram.NewSimRam(cfg.Ram, cfg.RamSpeedStroke)
	keeper := controller.New(cfg, rw)
	s := &Server{
		cfg:          cfg,
		boat:         boat,
		ram:          rw,
		keeper:       keeper,
		env:          boatsim.DefaultEnvironment(),
		rng:          rand.New(rand.NewSource(1)),
		headingNoise: 0.3,
		sensorHz:     10,
	}
	s.measHeading = boat.Heading
	return s
}

// Run starts the simulation loop and blocks serving HTTP on addr.
func (s *Server) Run(addr string) error {
	go s.loop()

	mux := http.NewServeMux()
	sub, _ := fs.Sub(staticFS, "static")
	mux.Handle("/", http.FileServer(http.FS(sub)))
	mux.HandleFunc("/api/stream", s.handleStream)
	mux.HandleFunc("/api/command", s.handleCommand)
	mux.HandleFunc("/api/environment", s.handleEnvironment)

	fmt.Printf("boat-brain simulator on http://%s\n", addr)
	return http.ListenAndServe(addr, mux)
}

// loop advances physics + control in real time.
func (s *Server) loop() {
	const hz = 50.0
	dt := 1.0 / hz
	ticker := time.NewTicker(time.Duration(dt * float64(time.Second)))
	defer ticker.Stop()
	for range ticker.C {
		s.step(dt)
	}
}

func (s *Server) step(dt float64) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.elapsed += dt

	// Sensor sample-and-hold with compass noise, at the sensor rate.
	if s.elapsed-s.lastSensor >= 1/s.sensorHz {
		s.lastSensor = s.elapsed
		s.measHeading = s.boat.Heading + s.rng.NormFloat64()*s.headingNoise
		s.measCOG = s.boat.COG()
		s.measSOG = s.boat.SOGKnots()
	}

	cog := s.measCOG
	sog := s.measSOG
	tick := s.keeper.Update(s.measHeading, dt, nil, &s.boat.STWKnots, &cog, &sog)
	s.tick = tick

	s.boat.Step(s.ram.RudderAngle(), s.env, dt)

	// Record a ground-track breadcrumb every ~0.5 s.
	if len(s.trail) == 0 || s.elapsed-s.trail[len(s.trail)-1].T >= 0.5 {
		s.trail = append(s.trail, point{X: s.boat.X, Y: s.boat.Y, T: s.elapsed})
		if len(s.trail) > 600 {
			s.trail = s.trail[len(s.trail)-600:]
		}
	}
}

// stateJSON is the snapshot streamed to the browser.
type stateJSON struct {
	Time      float64             `json:"time"`
	Engaged   bool                `json:"engaged"`
	Mode      string              `json:"mode"`
	Tick      controller.Tick     `json:"tick"`
	Boat      boatStateJSON       `json:"boat"`
	Env       boatsim.Environment `json:"env"`
	PID       pidJSON             `json:"pid"`
	Trail     []point             `json:"trail"`
	MountSide string              `json:"mount_side"`
}

type boatStateJSON struct {
	Heading    float64 `json:"heading"`
	COG        float64 `json:"cog"`
	SOG        float64 `json:"sog"`
	STW        float64 `json:"stw"`
	YawRate    float64 `json:"yaw_rate"`
	X          float64 `json:"x"`
	Y          float64 `json:"y"`
	RudderDeg  float64 `json:"rudder_deg"`
	RamStroke  float64 `json:"ram_stroke"`
	RamPercent float64 `json:"ram_percent"`
}

type pidJSON struct {
	Kp float64 `json:"kp"`
	Ki float64 `json:"ki"`
	Kd float64 `json:"kd"`
	P  float64 `json:"p"`
	I  float64 `json:"i"`
	D  float64 `json:"d"`
}

func (s *Server) snapshot() stateJSON {
	s.mu.Lock()
	defer s.mu.Unlock()

	half := s.cfg.Ram.HalfTravel()
	pct := 0.0
	if half != 0 {
		pct = (s.ram.Stroke() - s.cfg.Ram.Centre) / half * 100
	}
	g := s.keeper.PID().Config.Gains
	d := s.keeper.PID().Debug

	return stateJSON{
		Time:    s.elapsed,
		Engaged: s.keeper.Engaged(),
		Mode:    string(s.keeper.Mode()),
		Tick:    s.tick,
		Boat: boatStateJSON{
			Heading: s.boat.Heading, COG: s.boat.COG(), SOG: s.boat.SOGKnots(),
			STW: s.boat.STWKnots, YawRate: s.boat.YawRate, X: s.boat.X, Y: s.boat.Y,
			RudderDeg: s.ram.RudderAngle(), RamStroke: s.ram.Stroke(), RamPercent: pct,
		},
		Env:       s.env,
		PID:       pidJSON{Kp: g.Kp, Ki: g.Ki, Kd: g.Kd, P: d.P, I: d.I, D: d.D},
		Trail:     append([]point(nil), s.trail...),
		MountSide: string(s.cfg.Ram.MountSide),
	}
}

func (s *Server) handleStream(w http.ResponseWriter, r *http.Request) {
	flusher, ok := w.(http.Flusher)
	if !ok {
		http.Error(w, "streaming unsupported", http.StatusInternalServerError)
		return
	}
	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Connection", "keep-alive")

	ticker := time.NewTicker(50 * time.Millisecond) // ~20 fps
	defer ticker.Stop()
	enc := json.NewEncoder(w)
	for {
		select {
		case <-r.Context().Done():
			return
		case <-ticker.C:
			fmt.Fprint(w, "data: ")
			if err := enc.Encode(s.snapshot()); err != nil {
				return
			}
			fmt.Fprint(w, "\n")
			flusher.Flush()
		}
	}
}

// commandReq is the POST body for /api/command.
type commandReq struct {
	Action string  `json:"action"`         // engage|standby|mode|set|adjust|reset|stw|tune
	Mode   string  `json:"mode,omitempty"` // heading|cog
	Value  float64 `json:"value,omitempty"`
}

func (s *Server) handleCommand(w http.ResponseWriter, r *http.Request) {
	var req commandReq
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	switch req.Action {
	case "engage":
		s.keeper.Engage(s.boat.Heading)
	case "standby":
		s.keeper.Standby()
	case "mode":
		s.keeper.SetMode(controller.Mode(req.Mode))
	case "set":
		if s.keeper.Mode() == controller.TrackCOG {
			s.keeper.SetCOG(req.Value)
		} else {
			s.keeper.SetHeading(req.Value)
		}
	case "adjust":
		if s.keeper.Mode() == controller.TrackCOG {
			s.keeper.AdjustCOG(req.Value)
		} else {
			s.keeper.AdjustHeading(req.Value)
		}
	case "stw":
		s.boat.STWKnots = math.Max(0, req.Value)
	case "reset":
		s.boat.X, s.boat.Y = 0, 0
		s.trail = nil
	case "tune":
		// Value carries the damping ratio; re-autotune live.
		s.cfg.Targets.Damping = req.Value
		if err := s.cfg.Autotune(); err == nil {
			s.keeper.SetBaseGains(s.cfg.PID.Gains)
		}
	}
	w.WriteHeader(http.StatusNoContent)
}

func (s *Server) handleEnvironment(w http.ResponseWriter, r *http.Request) {
	var env boatsim.Environment
	if err := json.NewDecoder(r.Body).Decode(&env); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	s.mu.Lock()
	s.env = env
	s.mu.Unlock()
	w.WriteHeader(http.StatusNoContent)
}
