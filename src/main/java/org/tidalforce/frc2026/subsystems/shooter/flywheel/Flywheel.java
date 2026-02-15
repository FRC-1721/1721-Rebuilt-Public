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

package org.tidalforce.frc2026.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.Constants;
import org.tidalforce.frc2026.Robot;
import org.tidalforce.frc2026.subsystems.shooter.ShotCalculator;
import org.tidalforce.frc2026.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import org.tidalforce.frc2026.util.EqualsUtil;
import org.tidalforce.frc2026.util.FullSubsystem;
import org.tidalforce.frc2026.util.LoggedTunableNumber;

public class Flywheel extends FullSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;

  public static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS");
  public static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV");
  public static final LoggedTunableNumber kVMaxVelocity =
      new LoggedTunableNumber("Flywheel/kVMaxVelocity");
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP");
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD");
  private static final LoggedTunableNumber rateLimiter =
      new LoggedTunableNumber("Flywheel/SlewRateLimiter");

  SlewRateLimiter slewRateLimiter = new SlewRateLimiter(rateLimiter.get());

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  private double goalRadsPerSec = 0.0;

  static {
    switch (Constants.robot) {
      case COMP -> {
        rateLimiter.initDefault(300);
        kS.initDefault(0.0);
        kV.initDefault(0.0);
        kVMaxVelocity.initDefault(0.0);
        kP.initDefault(0.0);
        kD.initDefault(0.0);
      }
      case DEV -> {
        rateLimiter.initDefault(800.0);
        kS.initDefault(10.0);
        kV.initDefault(0.065);
        kVMaxVelocity.initDefault(113.0);
        kP.initDefault(10.0);
        kD.initDefault(0.0);
      }
      default -> {
        rateLimiter.initDefault(0.0);
        kS.initDefault(0.0);
        kV.initDefault(0.0);
        kP.initDefault(0.0);
        kD.initDefault(0.0);
      }
    }
  }

  public Flywheel(FlywheelIO io) {
    this.io = io;

    disconnected = new Alert("Flywheel motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (rateLimiter.hasChanged(hashCode())) {
      slewRateLimiter = new SlewRateLimiter(rateLimiter.get());
    }
    outputs.kP = kP.get();
    outputs.kD = kD.get();

    if (outputs.coast) {
      slewRateLimiter.reset(inputs.velocityRadsPerSec);
    }

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));
  }

  public void setGoal(double velocityRadsPerSec) {
    this.goalRadsPerSec = velocityRadsPerSec;
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRadsPerSec) {
    outputs.coast = false;
    outputs.velocityRadsPerSec = slewRateLimiter.calculate(velocityRadsPerSec);
    outputs.feedForward =
        kS.get() * Math.signum(outputs.velocityRadsPerSec)
            + kV.get() * Math.min(outputs.velocityRadsPerSec, kVMaxVelocity.get());
    atGoal = EqualsUtil.epsilonEquals(velocityRadsPerSec, outputs.velocityRadsPerSec);

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/Setpoint", outputs.velocityRadsPerSec);
  }

  /** Stops the flywheel. */
  private void stop() {
    outputs.velocityRadsPerSec = 0.0;
    outputs.coast = true;
    atGoal = false;
  }

  /** Returns the current velocity in RPM. */
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public Command runTrackTargetCommand() {
    return runEnd(
        () -> runVelocity(ShotCalculator.getInstance().getParameters().flywheelSpeed()),
        this::stop);
  }

  public Command runFixedCommand(DoubleSupplier velocity) {
    return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command runGoalCommand() {
    return runEnd(() -> runVelocity(goalRadsPerSec), this::stop);
  }
}
