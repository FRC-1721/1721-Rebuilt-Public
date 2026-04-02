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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.subsystems.shooter.LaunchCalculator;
import org.tidalforce.frc2026.subsystems.shooter.flywheel.Flywheel;
import org.tidalforce.frc2026.subsystems.shooter.hood.Hood;

/**
 * Manual shooter tuning command for the operator (secondary) controller.
 *
 * <p>Always starts at 225 RPM / 0° hood — matches the existing spin-up command baseline so the
 * operator starts from a known state every activation.
 *
 * <p>Jog rates at FULL stick deflection (~5 RPM/sec flywheel, ~0.5 deg/sec hood): Flywheel : 0.1
 * RPM per 20ms loop Hood : 0.01 deg per 20ms loop
 *
 * <p>Read these keys in AdvantageScope to fill in LaunchCalculator's maps:
 * ShooterTuning/FlywheelTargetRPM → value for launchFlywheelSpeedMap ShooterTuning/HoodTargetDeg →
 * value for launchHoodAngleMap LaunchCalculator/TurretToTargetDistance → distance key for both maps
 *
 * <p>Bind in RobotContainer:
 *
 * <pre>
 *   operatorController
 *       .leftBumper()
 *       .whileTrue(
 *           new ShooterTuningCommand(
 *               flywheel,
 *               hood,
 *               () -> -operatorController.getLeftY(),
 *               () -> -operatorController.getRightY()));
 * </pre>
 */
public class ShooterTuningCommand extends Command {

  // ── Baseline — always starts here, every activation ──────────────────────
  private static final double BASELINE_FLYWHEEL_RPM = 225.0;

  // ── Jog rates per 20ms loop at full stick ─────────────────────────────────
  // 5 RPM/sec ÷ 50 loops/sec  = 0.1 RPM/loop
  // 0.5 deg/sec ÷ 50 loops/sec = 0.01 deg/loop
  private static final double FLYWHEEL_RPM_PER_LOOP = 0.1;
  private static final double HOOD_DEG_PER_LOOP = 0.01;

  // ── Hardware limits ───────────────────────────────────────────────────────
  private static final double FLYWHEEL_MIN_RPM = 0.0;
  private static final double FLYWHEEL_MAX_RPM = 6000.0;
  private static final double HOOD_MIN_DEG = 0.0;
  private static final double HOOD_MAX_DEG = 60.0; // ← set to your physical travel

  // ── Stick deadband ────────────────────────────────────────────────────────
  private static final double STICK_DEADBAND = 0.05;

  private final Flywheel flywheel;
  private final Hood hood;
  private final DoubleSupplier flywheelStick; // left Y, negated by caller — up = faster
  private final DoubleSupplier hoodStick; // right Y, negated by caller — up = higher

  private double flywheelTargetRPM;
  private double hoodTargetDeg;

  public ShooterTuningCommand(
      Flywheel flywheel, Hood hood, DoubleSupplier flywheelStick, DoubleSupplier hoodStick) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.flywheelStick = flywheelStick;
    this.hoodStick = hoodStick;
    addRequirements(flywheel, hood);
  }

  @Override
  public void initialize() {
    // Reset to baseline every time — deterministic starting point.
    flywheelTargetRPM = BASELINE_FLYWHEEL_RPM;
    Logger.recordOutput("ShooterTuning/Active", true);
    Logger.recordOutput("ShooterTuning/FlywheelTargetRPM", flywheelTargetRPM);
  }

  @Override
  public void execute() {
    double flywheelInput = MathUtil.applyDeadband(flywheelStick.getAsDouble(), STICK_DEADBAND);
    double hoodInput = MathUtil.applyDeadband(hoodStick.getAsDouble(), STICK_DEADBAND);

    flywheelTargetRPM =
        MathUtil.clamp(
            flywheelTargetRPM + flywheelInput * FLYWHEEL_RPM_PER_LOOP,
            FLYWHEEL_MIN_RPM,
            FLYWHEEL_MAX_RPM);
    hoodTargetDeg =
        MathUtil.clamp(hoodTargetDeg + hoodInput * HOOD_DEG_PER_LOOP, HOOD_MIN_DEG, HOOD_MAX_DEG);

    flywheel.setGoal(flywheelTargetRPM);
    hood.runFixedCommand(() -> new Rotation2d(hoodTargetDeg));

    Logger.recordOutput("ShooterTuning/FlywheelTargetRPM", flywheelTargetRPM);
    Logger.recordOutput("ShooterTuning/HoodTargetDeg", hoodTargetDeg);
    Logger.recordOutput(
        "ShooterTuning/IsInRange", LaunchCalculator.getInstance().getParameters().isValid());
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.setGoal(0);
    hood.setGoalIdle();
    Logger.recordOutput("ShooterTuning/Active", false);
  }
}
