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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.Constants;
import org.tidalforce.frc2026.DevBotMech3d;
import org.tidalforce.frc2026.Robot;
import org.tidalforce.frc2026.subsystems.shooter.ShotCalculator;
import org.tidalforce.frc2026.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import org.tidalforce.frc2026.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import org.tidalforce.frc2026.util.EqualsUtil;
import org.tidalforce.frc2026.util.FullSubsystem;
import org.tidalforce.frc2026.util.LoggedTracer;
import org.tidalforce.frc2026.util.LoggedTunableNumber;

public class Hood extends FullSubsystem {
  public static final double minAngle = Units.degreesToRadians(10);
  public static final double maxAngle = Units.degreesToRadians(38);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA");
  private static final LoggedTunableNumber maxVelocityRadPerSec =
      new LoggedTunableNumber("Hood/MaxVelocityRadPerSec", 75.0);
  private static final LoggedTunableNumber maxAccelerationRadPerSec2 =
      new LoggedTunableNumber("Hood/MaxAccelerationRadPerSec2", 85.0);

  static {
    kP.initDefault(2);
    kD.initDefault(0.1);
    kS.initDefault(0);
    kG.initDefault(0);
    kA.initDefault(0);
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  // Connected debouncer
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Hood motor disconnected!", Alert.AlertType.kWarning);

  @Setter private BooleanSupplier coastOverride = () -> false;

  private TrapezoidProfile profile;
  @Getter private State setpoint = new State();

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput(key = "Hood/Profile/AtGoal")
  private boolean atGoal = false;

  private double goalAngle = 0.0;
  private double goalVelocity = 0.0;

  private static double hoodOffset = 0.0;
  private boolean hoodZeroed = false;

  private enum ControlMode {
    PROFILED,
    DIRECT,
    OPEN_LOOP
  }

  private ControlMode controlMode = ControlMode.PROFILED;

  public Hood(HoodIO io) {
    this.io = io;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVelocityRadPerSec.get(), maxAccelerationRadPerSec2.get()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    motorDisconnectedAlert.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));

    // Stop when disabled
    if (DriverStation.isDisabled() || (!hoodZeroed && outputs.mode != HoodIOOutputMode.OPEN_LOOP)) {
      outputs.mode = HoodIOOutputMode.BRAKE;

      if (coastOverride.getAsBoolean()) {
        outputs.mode = HoodIOOutputMode.COAST;
      }
    }

    // Update tunable numbers
    outputs.kP = kP.get();
    outputs.kD = kD.get();

    if (maxVelocityRadPerSec.hasChanged(hashCode())
        || maxAccelerationRadPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  maxVelocityRadPerSec.get(), maxAccelerationRadPerSec2.get()));
    }

    if (DriverStation.isDisabled()) {
      setpoint = new State(getMeasuredAngleRad(), 0.0);
    }

    // Visualize turret in 3D
    DevBotMech3d.getMeasured().setHoodAngle(new Rotation2d(getMeasuredAngleRad()));

    // Record cycle time
    LoggedTracer.record("Hood");
  }

  @Override
  public void periodicAfterScheduler() {
    if (DriverStation.isEnabled()) {

      switch (controlMode) {
        case PROFILED -> {
          var goalState =
              new State(
                  MathUtil.clamp(goalAngle, minAngle, maxAngle),
                  MathUtil.clamp(goalVelocity, 0.0, maxVelocityRadPerSec.get()));

          double previousVelocity = setpoint.velocity;
          setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);

          if (setpoint.position < minAngle || setpoint.position > maxAngle) {
            setpoint =
                new State(
                    MathUtil.clamp(setpoint.position, minAngle, maxAngle),
                    MathUtil.clamp(setpoint.velocity, 0.0, maxVelocityRadPerSec.get()));
          }

          atGoal =
              EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
                  && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);

          double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;

          outputs.positionRad = setpoint.position - hoodOffset;
          outputs.velocityRadsPerSec = setpoint.velocity;

          outputs.feedforward =
              kS.get() * Math.signum(setpoint.velocity)
                  + kG.get() * Math.cos(setpoint.position)
                  + kA.get() * accel;

          outputs.mode = HoodIOOutputMode.CLOSED_LOOP;

          Logger.recordOutput("Hood/Profile/SetpointPositionRad", setpoint.position);
          Logger.recordOutput("Hood/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
          Logger.recordOutput("Hood/Profile/GoalPositionRad", goalState.position);
          Logger.recordOutput("Hood/Profile/GoalVelocityRadPerSec", goalState.velocity);
        }

        case DIRECT -> {
          double clamped = MathUtil.clamp(goalAngle, minAngle, maxAngle);

          outputs.positionRad = clamped - hoodOffset;
          outputs.velocityRadsPerSec = 0.0;

          outputs.feedforward = kG.get() * Math.cos(clamped);

          outputs.mode = HoodIOOutputMode.CLOSED_LOOP;

          atGoal = Math.abs(getMeasuredAngleRad() - clamped) < 0.01;
        }

        case OPEN_LOOP -> {
          outputs.mode = HoodIOOutputMode.OPEN_LOOP;
        }
      }
    }

    io.applyOutputs(outputs);
  }

  private void setGoalParams(double angle, double velocity) {
    goalAngle = angle;
    goalVelocity = velocity;
  }

  @AutoLogOutput(key = "Hood/MeasuredAngleRads")
  public double getMeasuredAngleRad() {
    return inputs.positionRads + hoodOffset;
  }

  private void zero() {
    hoodOffset = minAngle - inputs.positionRads;
    hoodZeroed = true;
  }

  public Command testVoltageCommand(DoubleSupplier volts) {
    return run(
        () -> {
          outputs.mode = HoodIOOutputMode.OPEN_LOOP;
          outputs.appliedVoltage = MathUtil.clamp(volts.getAsDouble(), -8.0, 8.0);
        });
  }

  public Command runTrackTargetCommand() {
    return run(
        () -> {
          var params = ShotCalculator.getInstance().getParameters();
          setGoalParams(params.hoodAngle(), params.hoodVelocity());
        });
  }

  public Command runProfiledCommand(DoubleSupplier angle, DoubleSupplier velocity) {
    return run(
        () -> {
          controlMode = ControlMode.PROFILED;
          setGoalParams(angle.getAsDouble(), velocity.getAsDouble());
        });
  }

  public Command runDirectAngleCommand(DoubleSupplier angle) {
    return run(
        () -> {
          controlMode = ControlMode.DIRECT;
          goalAngle = angle.getAsDouble();
        });
  }

  public Command jogCommand(DoubleSupplier input) {
    return run(
        () -> {
          controlMode = ControlMode.DIRECT;

          double speed = 0.02;
          goalAngle += input.getAsDouble() * speed;

          goalAngle = MathUtil.clamp(goalAngle, minAngle, maxAngle);
        });
  }

  public Command runCharacterizationCommand() {
    return run(
        () -> {
          controlMode = ControlMode.OPEN_LOOP; // prevent profile interference
          outputs.mode = HoodIOOutputMode.CHARACTERIZATION;
        });
  }

  public Command zeroCommand() {
    return run(() -> {
          outputs.mode = HoodIOOutputMode.OPEN_LOOP;
          hoodZeroed = false;
        })
        .andThen(this::zero);
  }
}
