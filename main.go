// Copyright 2026 The ADC Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Command adc reads 10-channel frames from a USB serial ADC and prints
// the Pearson correlation matrix of the raw counts.
package main

import (
	"bufio"
	"errors"
	"flag"
	"fmt"
	"io"
	"math"
	"os"
	"strconv"
	"strings"
	"time"

	"golang.org/x/sys/unix"
)

const channels = 10

func main() {
	device := flag.String("device", "/dev/ttyUSB0", "serial device")
	baud := flag.Int("baud", 115200, "baud rate")
	frames := flag.Int("frames", 40, "number of frames to capture")
	timeout := flag.Duration("timeout", 0, "capture deadline (default: one second per frame, plus three seconds)")
	flag.Parse()

	if *frames < 2 {
		fmt.Fprintf(os.Stderr, "frames must be at least 2\n")
		os.Exit(1)
	}
	limit := *timeout
	if limit <= 0 {
		limit = time.Duration(*frames)*time.Second + 3*time.Second
	}

	port, err := openSerial(*device, *baud)
	if err != nil {
		fmt.Fprintf(os.Stderr, "open %s: %v\n", *device, err)
		os.Exit(1)
	}
	defer port.Close()

	fmt.Fprintf(os.Stderr, "capturing %d frames from %s at %d baud\n", *frames, *device, *baud)
	deadline := time.Now().Add(limit)
	if err := port.SetReadDeadline(deadline); err != nil {
		fmt.Fprintf(os.Stderr, "set deadline: %v\n", err)
		os.Exit(1)
	}
	samples, err := readFrames(port, *frames)
	if len(samples) < 2 {
		if err != nil {
			fmt.Fprintf(os.Stderr, "capture: %v\n", err)
		} else {
			fmt.Fprintf(os.Stderr, "capture: need at least 2 frames, got %d\n", len(samples))
		}
		os.Exit(1)
	}
	if err != nil && !errors.Is(err, os.ErrDeadlineExceeded) {
		fmt.Fprintf(os.Stderr, "capture stopped early: %v\n", err)
	}
	if errors.Is(err, os.ErrDeadlineExceeded) {
		fmt.Fprintf(os.Stderr, "stopped at deadline with %d frames\n", len(samples))
	}

	printCorrelation(os.Stdout, len(samples), correlation(samples))
}

// Linux termios baud codes. golang.org/x/sys/unix does not export the low rates.
const (
	b9600  = 0xd
	b19200 = 0xe
	b38400 = 0xf
)

func baudCode(rate int) (uint32, error) {
	switch rate {
	case 9600:
		return b9600, nil
	case 19200:
		return b19200, nil
	case 38400:
		return b38400, nil
	case 57600:
		return unix.B57600, nil
	case 115200:
		return unix.B115200, nil
	case 230400:
		return unix.B230400, nil
	default:
		return 0, fmt.Errorf("unsupported baud rate %d", rate)
	}
}

func openSerial(path string, baud int) (*os.File, error) {
	speed, err := baudCode(baud)
	if err != nil {
		return nil, err
	}
	f, err := os.OpenFile(path, unix.O_RDWR|unix.O_NOCTTY|unix.O_NONBLOCK|unix.O_CLOEXEC, 0)
	if err != nil {
		return nil, err
	}
	fd := int(f.Fd())
	term, err := unix.IoctlGetTermios(fd, unix.TCGETS)
	if err != nil {
		f.Close()
		return nil, err
	}
	term.Iflag = 0
	term.Oflag = 0
	term.Lflag = 0
	term.Cflag &^= unix.CSIZE | unix.PARENB | unix.CSTOPB | unix.CBAUD
	term.Cflag |= unix.CS8 | unix.CREAD | unix.CLOCAL | speed
	term.Ispeed = speed
	term.Ospeed = speed
	term.Cc[unix.VMIN] = 1
	term.Cc[unix.VTIME] = 0
	if err := unix.IoctlSetTermios(fd, unix.TCSETS, term); err != nil {
		f.Close()
		return nil, err
	}
	if err := unix.IoctlSetInt(fd, unix.TCFLSH, unix.TCIFLUSH); err != nil {
		f.Close()
		return nil, err
	}
	if err := unix.SetNonblock(fd, false); err != nil {
		f.Close()
		return nil, err
	}
	return f, nil
}

// parseLine reads one "CH<n>:<raw>\\t<volts>V" line. The raw count is kept.
func parseLine(line string) (ch int, raw float64, ok bool) {
	if !strings.HasPrefix(line, "CH") {
		return 0, 0, false
	}
	colon := strings.IndexByte(line, ':')
	if colon < 3 {
		return 0, 0, false
	}
	ch, err := strconv.Atoi(line[2:colon])
	if err != nil || ch < 0 || ch >= channels {
		return 0, 0, false
	}
	rest := line[colon+1:]
	field := rest
	if i := strings.IndexAny(rest, "\t "); i >= 0 {
		field = rest[:i]
	}
	raw, err = strconv.ParseFloat(field, 64)
	if err != nil {
		return 0, 0, false
	}
	return ch, raw, true
}

// readFrames collects n complete CH0..CH9 frames. A blank line ends a frame.
// A short read returns the frames gathered so far and the read error.
func readFrames(r io.Reader, n int) ([][channels]float64, error) {
	br := bufio.NewReader(r)
	frames := make([][channels]float64, 0, n)
	var cur [channels]float64
	var seen uint
	for len(frames) < n {
		line, err := br.ReadBytes('\n')
		if len(line) > 0 && (err == nil || line[len(line)-1] == '\n') {
			text := strings.TrimSpace(string(line))
			if text == "" {
				if seen == (1<<channels)-1 {
					frames = append(frames, cur)
				}
				cur = [channels]float64{}
				seen = 0
			} else if ch, raw, ok := parseLine(text); ok {
				if seen&(1<<uint(ch)) != 0 {
					cur = [channels]float64{}
					seen = 0
				}
				cur[ch] = raw
				seen |= 1 << uint(ch)
			}
		}
		if err != nil {
			return frames, err
		}
	}
	return frames, nil
}

// correlation is the Pearson correlation matrix of the raw channel counts.
// A channel with no variance has NaN in its row and column.
func correlation(samples [][channels]float64) [channels][channels]float64 {
	var out [channels][channels]float64
	n := float64(len(samples))
	var mean [channels]float64
	for _, sample := range samples {
		for c := 0; c < channels; c++ {
			mean[c] += sample[c]
		}
	}
	for c := 0; c < channels; c++ {
		mean[c] /= n
	}
	var cov [channels][channels]float64
	for _, sample := range samples {
		var delta [channels]float64
		for c := 0; c < channels; c++ {
			delta[c] = sample[c] - mean[c]
		}
		for i := 0; i < channels; i++ {
			for j := i; j < channels; j++ {
				cov[i][j] += delta[i] * delta[j]
			}
		}
	}
	for i := 0; i < channels; i++ {
		for j := i; j < channels; j++ {
			den := math.Sqrt(cov[i][i] * cov[j][j])
			r := math.NaN()
			if den != 0 {
				r = cov[i][j] / den
			}
			out[i][j] = r
			out[j][i] = r
		}
	}
	return out
}

func printCorrelation(w io.Writer, frames int, m [channels][channels]float64) {
	fmt.Fprintf(w, "Pearson correlation of raw counts (%d frames)\n\n", frames)
	fmt.Fprintf(w, "%4s", "")
	for c := 0; c < channels; c++ {
		fmt.Fprintf(w, " %7s", fmt.Sprintf("CH%d", c))
	}
	fmt.Fprintln(w)
	for i := 0; i < channels; i++ {
		fmt.Fprintf(w, "CH%-2d", i)
		for j := 0; j < channels; j++ {
			v := m[i][j]
			if math.IsNaN(v) {
				fmt.Fprintf(w, " %7s", "nan")
				continue
			}
			fmt.Fprintf(w, " %7.3f", v)
		}
		fmt.Fprintln(w)
	}
}
