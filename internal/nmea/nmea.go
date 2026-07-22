// Package nmea is a minimal NMEA 0183 parser for the sentences an autopilot
// needs: heading (HDT/HDM/HDG), speed (VHW/RMC/VTG), course over ground
// (RMC/VTG), and rate of turn (ROT).
//
// It is intentionally tiny and allocation-light so the boat brain stays free of
// heavy dependencies. Source accumulates successive sentences into a current
// navigation state.
package nmea

import (
	"strconv"
	"strings"
)

// Fix is whatever navigation state the most recent sentence carried. Fields are
// pointers so "absent" is distinct from "zero": a lone ROT sentence sets only
// ROT and leaves heading/speed untouched.
type Fix struct {
	HeadingTrue     *float64
	HeadingMagnetic *float64
	COGTrue         *float64 // course over ground (true)
	SpeedKnots      *float64 // speed (through water or over ground per sentence)
	ROTDegPerMin    *float64
	SentenceType    string
}

func checksumOK(sentence string) bool {
	star := strings.IndexByte(sentence, '*')
	if star < 0 {
		return true // some talkers omit the checksum
	}
	body := sentence[:star]
	chk := strings.TrimSpace(sentence[star+1:])
	if len(chk) < 2 {
		return false
	}
	var calc byte
	for i := 0; i < len(body); i++ {
		c := body[i]
		if c == '$' || c == '!' {
			continue
		}
		calc ^= c
	}
	v, err := strconv.ParseUint(chk[:2], 16, 8)
	if err != nil {
		return false
	}
	return calc == byte(v)
}

func toFloat(s string) *float64 {
	if s == "" {
		return nil
	}
	v, err := strconv.ParseFloat(s, 64)
	if err != nil {
		return nil
	}
	return &v
}

// Parse parses one NMEA 0183 line into a Fix. It returns nil for empty input, a
// bad checksum, or an unhandled sentence type.
func Parse(raw string) *Fix {
	sentence := strings.TrimSpace(raw)
	if sentence == "" || (sentence[0] != '$' && sentence[0] != '!') {
		return nil
	}
	if !checksumOK(sentence) {
		return nil
	}
	body := sentence
	if star := strings.IndexByte(body, '*'); star >= 0 {
		body = body[:star]
	}
	body = strings.TrimLeft(body, "$!")
	parts := strings.Split(body, ",")
	if len(parts) == 0 || len(parts[0]) < 5 {
		return nil
	}
	stype := parts[0][2:] // drop the 2-char talker id
	f := parts[1:]

	field := func(i int) string {
		if i < len(f) {
			return f[i]
		}
		return ""
	}

	switch stype {
	case "HDT":
		return &Fix{HeadingTrue: toFloat(field(0)), SentenceType: stype}
	case "HDM":
		return &Fix{HeadingMagnetic: toFloat(field(0)), SentenceType: stype}
	case "HDG":
		return &Fix{HeadingMagnetic: toFloat(field(0)), SentenceType: stype}
	case "ROT":
		val := toFloat(field(0))
		if field(1) == "V" { // data invalid
			val = nil
		}
		return &Fix{ROTDegPerMin: val, SentenceType: stype}
	case "VHW": // headT,T,headM,M,spdN,N,spdK,K
		return &Fix{
			HeadingTrue:     toFloat(field(0)),
			HeadingMagnetic: toFloat(field(2)),
			SpeedKnots:      toFloat(field(4)),
			SentenceType:    stype,
		}
	case "RMC": // time,status,lat,N,lon,E,SOG,COG,date,magvar,E/W
		return &Fix{
			SpeedKnots:   toFloat(field(6)),
			COGTrue:      toFloat(field(7)),
			SentenceType: stype,
		}
	case "VTG": // COGt,T,COGm,M,knots,N,kph,K
		return &Fix{
			COGTrue:      toFloat(field(0)),
			SpeedKnots:   toFloat(field(4)),
			SentenceType: stype,
		}
	}
	return nil
}

// Source is a stateful accumulator: feed it raw lines, read back the latest
// state. It prefers true heading, falling back to magnetic. Rate of turn comes
// straight from a ROT sentence when the talker provides one (a real rate gyro is
// far cleaner than differentiating a compass).
type Source struct {
	PreferTrue      bool
	HeadingTrue     *float64
	HeadingMagnetic *float64
	COGTrue         *float64
	SpeedKnots      *float64
	ROTDegPerMin    *float64
}

// NewSource returns a Source preferring true heading.
func NewSource() *Source { return &Source{PreferTrue: true} }

// Update parses one line and merges it into the accumulated state.
func (s *Source) Update(raw string) *Fix {
	fix := Parse(raw)
	if fix == nil {
		return nil
	}
	if fix.HeadingTrue != nil {
		s.HeadingTrue = fix.HeadingTrue
	}
	if fix.HeadingMagnetic != nil {
		s.HeadingMagnetic = fix.HeadingMagnetic
	}
	if fix.COGTrue != nil {
		s.COGTrue = fix.COGTrue
	}
	if fix.SpeedKnots != nil {
		s.SpeedKnots = fix.SpeedKnots
	}
	if fix.ROTDegPerMin != nil {
		s.ROTDegPerMin = fix.ROTDegPerMin
	}
	return fix
}

// Heading returns the best available heading, honouring PreferTrue, or nil.
func (s *Source) Heading() *float64 {
	primary, secondary := s.HeadingTrue, s.HeadingMagnetic
	if !s.PreferTrue {
		primary, secondary = s.HeadingMagnetic, s.HeadingTrue
	}
	if primary != nil {
		return primary
	}
	return secondary
}

// ROTDegPerSec returns the sensor rate of turn in deg/s, or nil.
func (s *Source) ROTDegPerSec() *float64 {
	if s.ROTDegPerMin == nil {
		return nil
	}
	v := *s.ROTDegPerMin / 60
	return &v
}
