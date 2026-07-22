package config

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/danielgraf/boat-brain-audio-alerter/internal/ram"
	"github.com/danielgraf/boat-brain-audio-alerter/internal/tuning"
)

func TestDefaultAutotunes(t *testing.T) {
	c := Default()
	if c.PID.Gains.Kp == 0 || c.PID.Gains.Kd == 0 {
		t.Errorf("Default should autotune gains, got %+v", c.PID.Gains)
	}
}

func TestSaveLoadRoundtrip(t *testing.T) {
	c := Default()
	c.Nomoto = tuning.Nomoto{K: 0.2, T: 2.5}
	c.Ram.MountSide = ram.Starboard
	c.Ram.MaxRudderDeg = 32
	if err := c.Autotune(); err != nil {
		t.Fatal(err)
	}

	dir := t.TempDir()
	path := filepath.Join(dir, "autopilot.json")
	if err := c.Save(path); err != nil {
		t.Fatal(err)
	}
	if _, err := os.Stat(path); err != nil {
		t.Fatal(err)
	}
	back, err := Load(path)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(back.Nomoto.K-0.2) > 1e-9 || back.Ram.MountSide != ram.Starboard {
		t.Errorf("roundtrip mismatch: %+v", back)
	}
	if math.Abs(back.PID.Gains.Kp-c.PID.Gains.Kp) > 1e-9 {
		t.Errorf("gains not preserved: %v vs %v", back.PID.Gains.Kp, c.PID.Gains.Kp)
	}
}
