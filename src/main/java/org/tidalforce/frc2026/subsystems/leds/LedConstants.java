// Team: FRC 1721 - Concord Robotics (Tidal Force)
// Year: 2025-2026
// Code: Public codebase for our REBUILT frc robot
// License: MIT License (See LICENSE file for full text)
//
// Copyright (c) 2025-2026 Concord Robotics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN an ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package org.tidalforce.frc2026.subsystems.leds;

public class LedConstants {
  public static final int length = 40;
  public static final int port = 0;
  public static final boolean prideLeds = false;
  public static final double startupBreathDuration = 1.0;
  public static final double strobeSlowDuration = 0.2;
  public static final double breathFastDuration = 0.5;
  public static final double breathSlowDuration = 1.0;
  public static final double rainbowCycleLength = 25.0;
  public static final double rainbowDuration = 0.25;
  public static final double rainbowStrobeDuration = 0.2;
  public static final double waveExponent = 0.4;
  public static final double waveFastCycleLength = 25.0;
  public static final double waveFastDuration = 0.25;
  public static final double waveDisabledCycleLength = 15.0;
  public static final double waveDisabledDuration = 2.0;
  public static final double strobeDuration = 0.1;

  private LedConstants() {}
}
