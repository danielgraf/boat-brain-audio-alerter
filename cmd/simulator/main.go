// Command simulator runs the interactive boat-brain autopilot simulator and
// serves the web visualizer.
//
//	go run ./cmd/simulator            # then open http://localhost:8080
//	go run ./cmd/simulator -addr :9000
package main

import (
	"flag"
	"log"

	"github.com/danielgraf/boat-brain-audio-alerter/internal/simserver"
)

func main() {
	addr := flag.String("addr", "localhost:8080", "listen address")
	flag.Parse()

	srv := simserver.New()
	log.Fatal(srv.Run(*addr))
}
