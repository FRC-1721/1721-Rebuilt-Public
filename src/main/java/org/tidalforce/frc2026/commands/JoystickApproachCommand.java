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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.subsystems.drive.Drive;
import org.tidalforce.frc2026.util.TuneableProfiledPID;

public class JoystickApproachCommand extends Command {
  Drive drive;
  DoubleSupplier ySupplier;
  Supplier<Pose2d> targetSupplier;

  Pose2d targetPose2d;
  Pose2d currentPose2d;
  Pose2d relativePose2d;
  Rotation2d targetRotation2d;

  boolean running = false;

  static final double DEADBAND = 0.1;

  TuneableProfiledPID angleController =
      new TuneableProfiledPID("angleController", 4.5, 0.0, 0.4, 8.0, 20.0);

  TuneableProfiledPID alignController =
      new TuneableProfiledPID("alignController", 4, 0.0, 0, 3.7, 4);

  public JoystickApproachCommand(
      Drive drive, DoubleSupplier ySupplier, Supplier<Pose2d> targetSupplier) {
    this.drive = drive;
    this.ySupplier = ySupplier;
    this.targetSupplier = targetSupplier;

    angleController.enableContinuousInput(-Math.PI, Math.PI);
    alignController.setGoal(0);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alignController.reset(0);
    angleController.reset(drive.getPose().getRotation().getRadians());
    targetPose2d = targetSupplier.get();

    Logger.recordOutput("AutoAlign/Approach/Target", targetPose2d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleController.updatePID();
    alignController.updatePID();

    running = true;
    relativePose2d = drive.getPose().relativeTo(targetPose2d);
    targetRotation2d = targetPose2d.getRotation();

    // Calculate lateral linear velocity
    Translation2d offsetVector =
        new Translation2d(0, alignController.calculate(relativePose2d.getY()));

    // Calculate total linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(-ySupplier.getAsDouble(), 0)
            .times(drive.getMaxLinearSpeedMetersPerSec())
            .plus(offsetVector)
            .rotateBy(targetRotation2d);

    // Calculate angular speed
    double omega =
        angleController.calculate(
            drive.getRotation().getRadians(),
            targetRotation2d.rotateBy(Rotation2d.k180deg).getRadians());

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    running = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Returns true when withing a lateral tolerance
  public boolean withinTolerance(double dist) {
    return running ? Math.abs(relativePose2d.getY()) < dist : false;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
}
