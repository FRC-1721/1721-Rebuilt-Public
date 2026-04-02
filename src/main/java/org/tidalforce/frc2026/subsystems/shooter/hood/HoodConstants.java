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

package org.tidalforce.frc2026.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation2d;

public class HoodConstants {
  public static final int HOODID = 2;
  public static final String CAN_BUS = "rio";

  // ─── Gear ratio ──────────────────────────────────────────────────────────
  // Motor rotations per one full mechanism rotation.
  // Verify from CAD — wrong value means every position command is wrong.
  public static final double GEAR_RATIO = 20.0;

  // ─── Physical limits — from measured hard stop values ───────────────────
  // Measured hard stops: -5° (upper) and -30° (lower)
  // These are set 3° inside each hard stop so software limits catch it first.
  // To widen range: move MAX_ANGLE closer to -5° or MIN_ANGLE closer to -30°.
  // To narrow range: move either value further from its respective hard stop.
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(-36.0);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-5.5);

  // ─── Zeroing ─────────────────────────────────────────────────────────────
  // The angle the sensor is set to after hitting the lower (-30°) hard stop.
  // Should match MIN_ANGLE so the hood knows where it is after zeroing.
  public static final Rotation2d ZERO_ANGLE = Rotation2d.fromDegrees(-5.5);

  // How slowly to drive into the lower hard stop during zeroing.
  // MAKE MORE NEGATIVE (e.g. -2.0) to zero faster.
  // MAKE LESS NEGATIVE (e.g. -0.5) to zero more gently.
  public static final double ZERO_VOLTAGE = -1.0;

  // Current threshold to detect we've hit the hard stop.
  // RAISE (e.g. 12.0) if zeroing triggers too early on inertia.
  // LOWER (e.g. 5.0) if the motor stalls but zeroing never triggers.
  public static final double ZERO_CURRENT_THRESHOLD_AMPS = 8.0;

  // How long current must stay above threshold before accepting the zero.
  // RAISE (e.g. 0.3) if you get false zero triggers.
  // LOWER (e.g. 0.05) if zeroing is slow to confirm.
  public static final double ZERO_STALL_TIME_SECS = 0.15;

  // ─── Position tolerance ──────────────────────────────────────────────────
  // How close the hood must be before atGoal() returns true.
  // TIGHTEN (e.g. 0.5°) for more precise shots.
  // LOOSEN (e.g. 3.0°) to allow shooting sooner while still moving.
  public static final Rotation2d AT_GOAL_TOLERANCE = Rotation2d.fromDegrees(2.0);
}
