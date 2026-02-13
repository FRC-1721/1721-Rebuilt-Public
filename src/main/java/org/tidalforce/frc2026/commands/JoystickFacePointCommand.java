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

package org.tidalforce.frc2026.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.tidalforce.frc2026.subsystems.drive.Drive;
import org.tidalforce.frc2026.util.TuneableProfiledPID;

public class JoystickFacePointCommand extends Command {

  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final Supplier<Pose2d> targetSupplier;

  private static final double DEADBAND = 0.1;

  private final TuneableProfiledPID angleController =
      new TuneableProfiledPID("FacePointAngle", 9, 0.0, 0, 3.7, 4);

  public JoystickFacePointCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Pose2d> targetSupplier) {

    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.targetSupplier = targetSupplier;

    angleController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    angleController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    angleController.updatePID();

    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = targetSupplier.get();
    Translation2d target = targetPose.getTranslation();

    // Vector from robot to target
    Translation2d robotToTarget = target.minus(currentPose.getTranslation());

    Rotation2d offset = Rotation2d.fromRadians(0);

    // Compute desired heading
    Rotation2d desiredHeading = new Rotation2d(robotToTarget.getX(), robotToTarget.getY()).plus(offset);

    // Compute angular velocity
    double omega =
        angleController.calculate(
            currentPose.getRotation().getRadians(), desiredHeading.getRadians());

    // Compute translation from joysticks (field relative)
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(-xSupplier.getAsDouble(), -ySupplier.getAsDouble())
            .times(drive.getMaxLinearSpeedMetersPerSec());

    // Convert to chassis speeds
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX(), linearVelocity.getY(), omega, currentPose.getRotation());

    drive.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d direction = new Rotation2d(Math.atan2(y, x));

    magnitude = magnitude * magnitude;

    return new Pose2d(new Translation2d(), direction)
        .transformBy(new Transform2d(magnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
}
