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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.Constants;
import org.tidalforce.frc2026.FieldConstants;
import org.tidalforce.frc2026.RobotState;
import org.tidalforce.frc2026.subsystems.shooter.PassingCalculator.PassingSide;
import org.tidalforce.frc2026.util.geometry.AllianceFlipUtil;
import org.tidalforce.frc2026.util.geometry.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class LaunchCalculator {

  private static LaunchCalculator instance;

  // ── Velocity filters ──────────────────────────────────────────────────────
  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  // ── Passing mode state ────────────────────────────────────────────────────
  // Commands set this via setPassingMode() each loop they run.
  // It is cleared in clearLaunchingParameters() so hub-shot commands work
  // normally without explicitly disabling passing.
  private boolean passingMode = false;
  private PassingSide passingSide = PassingSide.LEFT;

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  // ── LaunchingParameters record ────────────────────────────────────────────
  /**
   * Output parameters for a single aiming frame.
   *
   * <p>{@code passing} is true when the calculator is targeting a passing spot rather than the hub.
   * Callers (Turret, Hood, Flywheel commands) can gate behaviour on this flag — e.g. the Trigger in
   * RobotContainer that was previously broken because this field didn't exist.
   *
   * <p>{@code turretAngle} is always field-relative. The Turret subsystem converts it to
   * robot-relative internally.
   */
  public record LaunchingParameters(
      boolean isValid,
      boolean passing,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  // ── Lookup tables ─────────────────────────────────────────────────────────
  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;

  // Time-of-flight min/max — stored explicitly because InterpolatingDoubleTreeMap
  // doesn't expose firstKey()/lastKey(). Keep in sync with the map entries below.
  private static double minTimeOfFlight;
  private static double maxTimeOfFlight;

  private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.25;
    maxDistance = 7.5;
    phaseDelay = 0.03;

    launchHoodAngleMap.put(1.25, Rotation2d.fromDegrees(-5.5));
    launchHoodAngleMap.put(3.5, Rotation2d.fromDegrees(-12.5));
    launchHoodAngleMap.put(7.5, Rotation2d.fromDegrees(-30.0));

    launchFlywheelSpeedMap.put(1.25, 215.0);
    launchFlywheelSpeedMap.put(1.5, 225.0);
    launchFlywheelSpeedMap.put(2.0, 255.0);
    launchFlywheelSpeedMap.put(2.5, 290.0);
    launchFlywheelSpeedMap.put(3.0, 335.0);
    launchFlywheelSpeedMap.put(3.5, 395.0);
    launchFlywheelSpeedMap.put(4.0, 530.0);
    launchFlywheelSpeedMap.put(5.5, 620.0);
    launchFlywheelSpeedMap.put(6.0, 675.0);
    launchFlywheelSpeedMap.put(7.5, 405.0);

    // Sorted ascending by distance. Keep minTimeOfFlight/maxTimeOfFlight
    // pointing at the first and last entries respectively.
    timeOfFlightMap.put(1.38, 0.90);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(5.68, 1.16);

    // Update these if you add/remove entries from timeOfFlightMap.
    minTimeOfFlight = 0.90; // value at 1.38m
    maxTimeOfFlight = 1.16; // value at 5.68m
  }

  // ── Static TOF accessors (used by HubShiftUtil) ───────────────────────────
  public static double getMinTimeOfFlight() {
    return minTimeOfFlight;
  }

  public static double getMaxTimeOfFlight() {
    return maxTimeOfFlight;
  }

  // ── Passing mode control ──────────────────────────────────────────────────

  /**
   * Commands call this each loop to put the calculator into passing mode. The next {@link
   * #getParameters()} call will return passing parameters targeting the given side instead of the
   * hub.
   */
  public void disablePassingMode() {
    passingMode = false;
  }

  public void setPassingMode(PassingCalculator.PassingSide side) {
    this.passingMode = true;
    this.passingSide = side;
  }

  // ── Main parameter computation ────────────────────────────────────────────

  public LaunchingParameters getParameters() {

    // ── Passing mode — delegate to PassingCalculator ──────────────────────
    if (passingMode) {
      var passing = PassingCalculator.getInstance().getParameters(passingSide);
      return new LaunchingParameters(
          passing.isValid(),
          true,
          passing.turretAngle(),
          passing.turretVelocity(),
          passing.hoodAngleRads(),
          0.0,
          passing.flywheelSpeedRPM());
    }

    // ── Hub shooting mode ─────────────────────────────────────────────────
    // Phase-delay compensated pose
    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Turret pivot world position
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d turretPosition = estimatedPose.transformBy(robotToTurret.toTransform2d());
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Field-relative turret velocity (for lead-shot and feedforward)
    ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldVelocity();
    double robotAngle = estimatedPose.getRotation().getRadians();
    // Rotate the robot-relative turret pivot offset into field frame first,
    // then apply ω × r to get the turret pivot's field-frame velocity.
    double pivotOffsetFieldX =
        robotToTurret.getX() * Math.cos(robotAngle) - robotToTurret.getY() * Math.sin(robotAngle);
    double pivotOffsetFieldY =
        robotToTurret.getX() * Math.sin(robotAngle) + robotToTurret.getY() * Math.cos(robotAngle);

    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond * (-pivotOffsetFieldY);
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond + robotVelocity.omegaRadiansPerSecond * (pivotOffsetFieldX);

    // Iterative lead-shot lookahead
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    Translation2d diff = target.minus(lookaheadPose.getTranslation());
    Rotation2d rawAngle = diff.getAngle();

    // unwrap angle to prevent ±π discontinuity
    if (lastTurretAngle != null) {
      double delta = rawAngle.minus(lastTurretAngle).getRadians();
      delta = MathUtil.angleModulus(delta);
      turretAngle = lastTurretAngle.plus(new Rotation2d(delta));
    } else {
      // FIRST LOOP: just initialize cleanly
      turretAngle = rawAngle;
      lastTurretAngle = rawAngle; // THIS LINE FIXES YOUR CRASH
    }

    hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();

    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);

    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;

    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return new LaunchingParameters(
        lookaheadTurretToTargetDistance >= minDistance
            && lookaheadTurretToTargetDistance <= maxDistance,
        false,
        turretAngle,
        turretVelocity,
        hoodAngle,
        hoodVelocity,
        launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));
  }
}
