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

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.Constants;
import org.tidalforce.frc2026.FieldConstants;
import org.tidalforce.frc2026.RobotState;
import org.tidalforce.frc2026.util.geometry.AllianceFlipUtil;
import org.tidalforce.frc2026.util.geometry.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class LaunchCalculator {
  private static LaunchCalculator instance;

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private double lastHoodAngle;
  private double hoodAngle;
  private double hoodVelocity;

  private static final double phaseDelay = 0.03; // seconds to account for processing delay
  private static final double minDistance = 1.34; // meters
  private static final double maxDistance = 5.60; // meters

  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<Double, Rotation2d>(
          InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    // Hood angles (meters → radians)
    hoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    hoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    hoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    hoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    hoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    hoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    // Flywheel speeds (meters → RPM)
    flywheelSpeedMap.put(1.34, 210.0);
    flywheelSpeedMap.put(1.78, 220.0);
    flywheelSpeedMap.put(2.17, 220.0);
    flywheelSpeedMap.put(2.81, 230.0);
    flywheelSpeedMap.put(3.82, 250.0);
    flywheelSpeedMap.put(4.09, 255.0);
    flywheelSpeedMap.put(4.40, 260.0);
    flywheelSpeedMap.put(4.77, 265.0);
    flywheelSpeedMap.put(5.57, 275.0);
    flywheelSpeedMap.put(5.60, 290.0);

    // Time-of-flight (meters → seconds)
    timeOfFlightMap.put(1.38, 0.90);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(5.68, 1.16);
  }

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record ShootingParameters(
      boolean isValid, Rotation2d hoodAngle, double hoodVelocity, double flywheelSpeed) {}

  private ShootingParameters latestParameters;

  /** Calculates the shooting parameters for a fixed shooter with robot motion compensation. */
  public ShootingParameters getParameters() {
    if (latestParameters != null) return latestParameters;

    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    // Robot velocity in field frame
    var fieldVelocity = RobotState.getInstance().getFieldVelocity();

    // Phase delay compensation
    Pose2d estimatedPose =
        robotPose.exp(
            new Twist2d(
                fieldVelocity.vxMetersPerSecond * phaseDelay,
                fieldVelocity.vyMetersPerSecond * phaseDelay,
                0.0)); // no rotation for fixed shooter

    // Distance to target
    double distance = target.getDistance(estimatedPose.getTranslation());

    // Lookahead accounting for robot velocity
    Pose2d lookaheadPose = estimatedPose;
    double lookaheadDistance = distance;
    for (int i = 0; i < 20; i++) {
      double tof = timeOfFlightMap.get(lookaheadDistance);
      Translation2d offset =
          new Translation2d(
              fieldVelocity.vxMetersPerSecond * tof, fieldVelocity.vyMetersPerSecond * tof);
      lookaheadPose =
          new Pose2d(estimatedPose.getTranslation().plus(offset), estimatedPose.getRotation());
      lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Hood angle
    hoodAngle = hoodAngleMap.get(lookaheadDistance).getRadians();
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

    hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastHoodAngle = hoodAngle;

    double flywheel = flywheelSpeedMap.get(lookaheadDistance);

    latestParameters =
        new ShootingParameters(
            lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance,
            new Rotation2d(hoodAngle),
            hoodVelocity,
            flywheel);

    // Logging
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/RobotPose", robotPose.toString());
    Logger.recordOutput("LaunchCalculator/Distance", lookaheadDistance);
    Logger.recordOutput("LaunchCalculator/HoodAngle", hoodAngle);
    Logger.recordOutput("LaunchCalculator/FlywheelSpeed", flywheel);

    return latestParameters;
  }

  /** Returns a Pose2d pointing from the robot to the hub, with velocity compensation */
  public Pose2d getFaceHerePose() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    // Robot velocity compensation
    var fieldVelocity = RobotState.getInstance().getFieldVelocity();

    // Phase delay
    Pose2d estimatedPose =
        robotPose.exp(
            new Twist2d(
                fieldVelocity.vxMetersPerSecond * phaseDelay,
                fieldVelocity.vyMetersPerSecond * phaseDelay,
                0.0 // no rotation for fixed shooter
                ));

    // Lookahead to account for robot motion
    Pose2d lookaheadPose = estimatedPose;
    double distance = target.getDistance(estimatedPose.getTranslation());
    for (int i = 0; i < 20; i++) {
      double tof = timeOfFlightMap.get(distance);
      Translation2d offset =
          new Translation2d(
              fieldVelocity.vxMetersPerSecond * tof, fieldVelocity.vyMetersPerSecond * tof);
      lookaheadPose =
          new Pose2d(estimatedPose.getTranslation().plus(offset), estimatedPose.getRotation());
      distance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Return Pose2d pointing from robot to hub
    Rotation2d facing = target.minus(lookaheadPose.getTranslation()).getAngle();
    return new Pose2d(lookaheadPose.getX(), lookaheadPose.getY(), facing);
  }

  /** Clears cached parameters for next loop. */
  public void clearParameters() {
    latestParameters = null;
  }
}
