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

package org.tidalforce.frc2026.util.controllers;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for 1721's custom bindings on a Turtle Beach Rematch Advanced controller. */
public class TurtleBeachRematchAdvController extends CommandXboxController {
  public TurtleBeachRematchAdvController(int port) {
    super(port);
  }

  /**
   * Constructs a Trigger instance around the upper left paddle button's digital signal.
   *
   * @return a Trigger instance representing the upper left paddle button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #start(EventLoop)
   */
  public Trigger LeftPaddle() {
    return povUp().or(povUpLeft()).or(povUpRight());
  }

  /**
   * Constructs a Trigger instance around the lower left paddle button's digital signal.
   *
   * @return a Trigger instance representing the lower left paddle button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #start(EventLoop)
   */
  public Trigger RightPaddle() {
    return povDown().or(povDownLeft()).or(povDownRight());
  }
}
