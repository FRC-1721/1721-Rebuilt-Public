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

package org.tidalforce.frc2026.subsystems.shooter;

import static org.tidalforce.frc2026.subsystems.shooter.LauncherConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.RobotState;
import org.tidalforce.frc2026.util.geometry.AllianceFlipUtil;

/**
 * Computes turret, hood, and flywheel parameters for passing shots.
 *
 * <p>Passing targets are fixed floor spots on either side of the Alliance Zone boundary (~3.2m from
 * the blue wall in WPILib blue-origin coordinates). AllianceFlipUtil mirrors them for red alliance
 * automatically.
 *
 * <p>ALL values below are PLACEHOLDERS. Tune them on the field: 1. Stand at the passing target spot
 * with a tape measure. 2. Note your distance from the turret pivot. 3. Spin up to the listed RPM,
 * adjust until the note lands, log the result. 4. Update PASSING_FLYWHEEL_RPM and
 * PASSING_HOOD_ANGLE_DEG to match.
 *
 * <p>Integrate into LaunchCalculator by calling {@link
 * PassingCalculator#getInstance()}.getParameters() and forwarding the result when the robot is in
 * passing mode.
 */
public class PassingCalculator {

  // ── Passing target floor spots (WPILib blue-origin field coordinates) ──────
  //
  // The 2026 REBUILT field is 16.54m long × 8.07m wide.
  // Alliance Zone: 4.03m deep from each alliance wall.
  //
  // These spots sit just inside the AZ boundary on each guardrail side.
  // Measure them on your actual field and update X/Y before competition.
  //
  //   LEFT  = guardrail side closest to driver station 1 (low Y)
  //   RIGHT = guardrail side closest to driver station 3 (high Y)
  //
  // Blue coordinates (red is auto-flipped):
  //   X ≈ 3.2m  — ~0.8m inside the AZ from the neutral zone boundary
  //   Y ≈ 1.0m  — LEFT side,  ~1m from left guardrail
  //   Y ≈ 7.0m  — RIGHT side, ~1m from right guardrail
  //
  // ⚠ MEASURE THESE ON YOUR FIELD. These are geometric approximations only.
  private static final Translation2d PASSING_TARGET_LEFT_BLUE = new Translation2d(3.2, 1.0);
  private static final Translation2d PASSING_TARGET_RIGHT_BLUE = new Translation2d(3.2, 7.0);

  // ── Shooter parameters for passing — PLACEHOLDERS, tune on field ──────────
  //
  // A passing shot is a high-arc lob; expect:
  //   - Higher hood angle than a hub shot at the same distance (~40–50°)
  //   - Lower flywheel speed than a hub shot (lob, not line drive)
  //
  // Process: activate ShooterTuningCommand, position robot at your shooting
  // spot, lob at the passing target, read ShooterTuning/* from AdvantageScope.
  //
  // If passing distance varies significantly by robot position, you can replace
  // these with a small InterpolatingDoubleTreeMap keyed by distance (same
  // pattern as LaunchCalculator). For now, fixed values are simpler to tune.
  private static final double PASSING_FLYWHEEL_RPM = 300.0; // ← tune me
  private static final double PASSING_HOOD_ANGLE_DEG = 42.0; // ← tune me

  // ── Passing target selection ──────────────────────────────────────────────
  public enum PassingSide {
    LEFT,
    RIGHT
  }

  // ── Singleton ────────────────────────────────────────────────────────────
  private static PassingCalculator instance;

  public static PassingCalculator getInstance() {
    if (instance == null) instance = new PassingCalculator();
    return instance;
  }

  private PassingCalculator() {}

  // ── Parameters record ─────────────────────────────────────────────────────
  /**
   * Output parameters for a passing shot.
   *
   * <p>{@code turretAngle} is field-relative (same convention as LaunchCalculator so
   * Turret.periodic() handles it identically). {@code turretVelocity} is the feedforward rate in
   * rad/s — zero for a stationary pass, non-zero if the robot is moving significantly.
   */
  public record PassingParameters(
      boolean isValid,
      PassingSide side,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngleRads,
      double flywheelSpeedRPM,
      double distanceMeters) {}

  /**
   * Compute passing parameters to the requested side.
   *
   * <p>Call once per loop when in passing mode. Unlike LaunchCalculator, this is NOT cached — it's
   * cheap enough to recompute and always reflects the latest robot pose.
   *
   * @param side which passing target to aim at
   * @return fully populated {@link PassingParameters}
   */
  public PassingParameters getParameters(PassingSide side) {
    // ── Resolve which target to use, flipped for red alliance ────────────
    Translation2d targetBlue =
        side == PassingSide.LEFT ? PASSING_TARGET_LEFT_BLUE : PASSING_TARGET_RIGHT_BLUE;
    Translation2d target = AllianceFlipUtil.apply(targetBlue);

    // ── Get current turret position ───────────────────────────────────────
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Pose2d turretPose =
        robotPose.transformBy(
            new Transform2d(
                new Translation2d(robotToTurret.getX(), robotToTurret.getY()),
                robotToTurret.getRotation().toRotation2d()));
    Translation2d turretTranslation = turretPose.getTranslation();

    // ── Field-relative angle to the target ───────────────────────────────
    Rotation2d turretAngle = target.minus(turretTranslation).getAngle();

    // ── Distance from turret pivot to target ─────────────────────────────
    double distance = target.getDistance(turretTranslation);

    // ── Turret velocity feedforward ───────────────────────────────────────
    // For a stationary pass this is 0. If you want moving-robot compensation,
    // apply the same velocity math from LaunchCalculator here.
    double turretVelocity = 0.0;

    // ── isValid: only if turret can physically reach the angle ────────────
    // The Turret subsystem will clamp anyway, but flag it so callers can
    // gate the shot command.
    boolean isValid = distance > 1.0 && distance < 10.0; // ← tune range if needed

    PassingParameters params =
        new PassingParameters(
            isValid,
            side,
            turretAngle,
            turretVelocity,
            Math.toRadians(PASSING_HOOD_ANGLE_DEG),
            PASSING_FLYWHEEL_RPM,
            distance);

    // ── Log ───────────────────────────────────────────────────────────────
    Logger.recordOutput("PassingCalculator/Side", side.toString());
    Logger.recordOutput("PassingCalculator/TargetX", target.getX());
    Logger.recordOutput("PassingCalculator/TargetY", target.getY());
    Logger.recordOutput("PassingCalculator/TurretAngleDeg", turretAngle.getDegrees());
    Logger.recordOutput("PassingCalculator/DistanceM", distance);
    Logger.recordOutput("PassingCalculator/IsValid", isValid);

    return params;
  }
}
