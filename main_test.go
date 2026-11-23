// Copyright 2026 The ADC Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"math"
	"strings"
	"testing"
)

func TestParseLine(t *testing.T) {
	ch, raw, ok := parseLine("CH3:1275\t1.026V")
	if !ok || ch != 3 || raw != 1275 {
		t.Fatalf("parse CH3: got ch=%d raw=%v ok=%v", ch, raw, ok)
	}
	if _, _, ok := parseLine("noise"); ok {
		t.Fatal("noise parsed as a sample")
	}
	if _, _, ok := parseLine("CH10:0001\t0.001V"); ok {
		t.Fatal("channel 10 accepted")
	}
}

func TestReadFramesSkipsPartial(t *testing.T) {
	var b strings.Builder
	b.WriteString("CH3:0009\t0.007V\r\n\r\n")
	for i := 0; i < 2; i++ {
		for ch := 0; ch < channels; ch++ {
			b.WriteString("CH")
			b.WriteString(string(rune('0' + ch)))
			b.WriteString(":000")
			b.WriteString(string(rune('0' + i)))
			b.WriteString("\t0.000V\r\n")
		}
		b.WriteString("\r\n")
	}
	frames, err := readFrames(strings.NewReader(b.String()), 2)
	if err != nil {
		t.Fatal(err)
	}
	if len(frames) != 2 {
		t.Fatalf("frames = %d", len(frames))
	}
	if frames[0][0] != 0 || frames[1][4] != 1 {
		t.Fatalf("frames = %v", frames)
	}
}

func TestCorrelation(t *testing.T) {
	samples := make([][channels]float64, 4)
	// CH0 rises, CH1 falls with it, CH2 matches CH0, CH3 is constant.
	values := []float64{1, 2, 3, 4}
	for i, v := range values {
		samples[i][0] = v
		samples[i][1] = 5 - v
		samples[i][2] = v * 10
		samples[i][3] = 7
	}
	m := correlation(samples)
	if math.Abs(m[0][0]-1) > 1e-9 || math.Abs(m[0][1]+1) > 1e-9 || math.Abs(m[0][2]-1) > 1e-9 {
		t.Fatalf("unexpected matrix row 0: %v", m[0])
	}
	if m[1][2] != m[2][1] {
		t.Fatal("matrix is not symmetric")
	}
	if !math.IsNaN(m[3][0]) || !math.IsNaN(m[3][3]) {
		t.Fatalf("constant channel should be NaN, got %v", m[3])
	}
}
