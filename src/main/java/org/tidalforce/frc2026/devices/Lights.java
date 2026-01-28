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

package org.tidalforce.frc2026.devices;

import com.ctre.phoenix6.controls.ControlRequest;
import java.util.List;
import org.tidalforce.frc2026.subsystems.leds.LightsIO;

/** Class for simplified Lights implementation */
public class Lights {
  private final LightsIO io;

  /**
   * Constructs Lights.
   *
   * @param io the IO to interact with.
   */
  public Lights(LightsIO io) {
    this.io = io;
  }

  /**
   * Passes ControlRequest to IO layer
   *
   * @param request {@link ControlRequest}
   */
  public void setAnimation(ControlRequest request) {
    io.setAnimation(request);
  }

  /**
   * Passes ControlRequests to IO layer
   *
   * @param requests {@link ControlRequest}
   */
  public void setAnimations(List<ControlRequest> requests) {
    for (ControlRequest request : requests) {
      io.setAnimation(request);
    }
  }
}
