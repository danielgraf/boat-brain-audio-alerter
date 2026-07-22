package tuning

import (
	"math"
	"testing"
)

func TestPolePlacementFormula(t *testing.T) {
	n := Nomoto{K: 0.15, T: 3}
	zeta, wn := 0.9, 0.5
	g, err := Gains(n, Targets{Damping: zeta, NaturalFreq: wn})
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(g.Kp-wn*wn*n.T/n.K) > 1e-9 {
		t.Errorf("Kp got %v", g.Kp)
	}
	if math.Abs(g.Kd-(2*zeta*wn*n.T-1)/n.K) > 1e-9 {
		t.Errorf("Kd got %v", g.Kd)
	}
}

func TestSettlingTimeBandwidth(t *testing.T) {
	n := Nomoto{K: 0.15, T: 3}
	fast, _ := Gains(n, Targets{Damping: 0.9, SettlingTime: 4})
	slow, _ := Gains(n, Targets{Damping: 0.9, SettlingTime: 16})
	if fast.Kp <= slow.Kp {
		t.Error("faster settling should demand more Kp")
	}
}

func TestCounterRudderNonNegative(t *testing.T) {
	n := Nomoto{K: 1, T: 0.2}
	g, _ := Gains(n, Targets{Damping: 0.9, SettlingTime: 20})
	if g.Kd < 0 {
		t.Errorf("counter rudder should not be negative: %v", g.Kd)
	}
}

func TestScaledToSpeed(t *testing.T) {
	n := Nomoto{K: 0.15, T: 3}
	f := n.ScaledToSpeed(12, 6)
	if f.K <= n.K || f.T >= n.T {
		t.Errorf("more speed -> more K, less T; got K=%v T=%v", f.K, f.T)
	}
}

func TestEstimateNomotoFromStep(t *testing.T) {
	K, T, delta := 0.15, 3.0, 10.0
	var times, rates []float64
	for i := 0; i < 300; i++ {
		tt := float64(i) * 0.1
		times = append(times, tt)
		rates = append(rates, K*delta*(1-math.Exp(-tt/T)))
	}
	est, err := EstimateNomotoFromStep(times, rates, delta)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(est.K-K) > 0.01 {
		t.Errorf("K est %v want %v", est.K, K)
	}
	if math.Abs(est.T-T) > 0.3 {
		t.Errorf("T est %v want %v", est.T, T)
	}
}
