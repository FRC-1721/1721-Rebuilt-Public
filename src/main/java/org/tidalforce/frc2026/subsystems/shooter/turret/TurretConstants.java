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

package org.tidalforce.frc2026.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretConstants {
  public static final int TURRET_ID = 3;
  public static final String CAN_BUS = "rio";

  // ─── Gear ratio ──────────────────────────────────────────────────────────
  // Motor rotations per one full turret rotation (360°).
  // Verify from CAD — wrong value means every position command is wrong.
  public static final double GEAR_RATIO = 18.18;

  // ─── Physical limits ─────────────────────────────────────────────────────
  // Turret is robot-relative. 0° = straight forward.
  // Positive = CCW (left), Negative = CW (right).
  // Set 10-20° inside true hard stops. Widen only after verifying cable wrap.
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-60.0);
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180.0);

  // ─── Zeroing ─────────────────────────────────────────────────────────────
  // After hitting the lower hard stop, the sensor is set to this value
  // so that 0° reads as forward. Should equal abs(MIN_ANGLE).
  public static final Rotation2d ZERO_OFFSET_FROM_HARDSTOP = Rotation2d.fromDegrees(120.0);

  // MAKE MORE NEGATIVE (e.g. -2.0) to zero faster.
  // MAKE LESS NEGATIVE (e.g. -0.5) to zero more gently.
  public static final double ZERO_VOLTAGE = -0.8;

  // RAISE (e.g. 15.0) if inertia triggers a false zero.
  // LOWER (e.g. 6.0) if the motor stalls but zeroing never fires.
  public static final double ZERO_CURRENT_THRESHOLD_AMPS = 10.0;

  // RAISE (e.g. 0.3) if you get false zero triggers.
  // LOWER (e.g. 0.05) if zeroing is slow to confirm.
  public static final double ZERO_STALL_TIME_SECS = 0.15;

  // ─── Position tolerance ──────────────────────────────────────────────────
  // TIGHTEN (e.g. 0.5°) for more accurate aiming.
  // LOOSEN (e.g. 2.5°) to allow shooting sooner while tracking.
  public static final Rotation2d AT_GOAL_TOLERANCE = Rotation2d.fromDegrees(1.5);
}
