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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.subsystems.shooter.LaunchCalculator;
import org.tidalforce.frc2026.util.LoggedTunableNumber;

public class Hood extends SubsystemBase {

  // ── Single source of truth for all gains ────────────────────────────────
  // These are the ONLY place gain values live. HoodConstants has no gains.
  // Change the default values here. They load into NetworkTables on boot
  // and can be edited live from Glass during testing.
  //
  // HOW TO TUNE:
  //   kP  — Raise until oscillation, back off ~30%. Expect final: 40-80.
  //   kD  — Raise after kP is set to damp oscillation. Expect final: 1-3.
  //   kS  — Raise if hood doesn't reach setpoint on small moves. Max ~1.0.
  //   kG  — Tune with hoodHoldHorizontal() command. Raise if hood sags.
  //   kV  — Leave at placeholder until SysId characterization is run.
  //   kA  — Leave at placeholder until SysId characterization is run.
  //
  // HOW TO CHANGE SPEED:
  //   Faster overall:    raise MM_CruiseVelocityRPS (try 5.0 → 10.0)
  //   Faster ramp-up:    raise MM_AccelerationRPS2 (keep at 2-4x cruise)
  //   Smoother start:    raise MM_JerkRPS3 from 0 (try 200)
  //   Still too slow:    raise PeakForwardVoltage in HoodIOKraken toward 12.0
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", 20.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", 0.5);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS", 0.2);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG", 0.3);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV", 0.12);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA", 0.01);
  private static final LoggedTunableNumber mmCruise =
      new LoggedTunableNumber("Hood/MM_CruiseVelocityRPS", 2.0);
  private static final LoggedTunableNumber mmAccel =
      new LoggedTunableNumber("Hood/MM_AccelerationRPS2", 6.0);
  private static final LoggedTunableNumber mmJerk =
      new LoggedTunableNumber("Hood/MM_JerkRPS3", 0.0);
  private static final int GainsID = 10;
  private static final int ProfileID = 11;

  // ── Goal state machine ───────────────────────────────────────────────────
  public enum Goal {
    TRACK_TARGET,
    FIXED,
    ZEROING,
    ZEROED,
    OPEN_LOOP_TEST,
    IDLE
  }

  @Getter @AutoLogOutput private Goal goal = Goal.IDLE;
  @Getter @AutoLogOutput private Rotation2d setpoint = HoodConstants.MIN_ANGLE;

  private boolean zeroed = false;
  private BooleanSupplier coastOverride = () -> false;
  private double testVoltage = 0.0;
  private double hoodOffsetRads = 0.0;
  private double lastGoalAngleRads = 0.0;

  private boolean hoodZeroed = false;
  private final Timer zeroStallTimer = new Timer();

  // ── IO ───────────────────────────────────────────────────────────────────
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  // ── Alerts ───────────────────────────────────────────────────────────────
  private final Alert faultAlert =
      new Alert("Hood motor has active faults — check Tuner X", Alert.AlertType.kError);
  private final Alert notZeroedAlert =
      new Alert(
          "Hood has not been zeroed — run zero sequence before shooting", Alert.AlertType.kWarning);

  public Hood(HoodIO io) {
    this.io = io;
  }

  // ── Periodic ─────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // ── Push gains to motor ──────────────────────────────────────────────
    // On first loop: gainsInitialized is false, so we always push once.
    // After that: only push when a NT value actually changed in Glass.
    // This guarantees the motor always reflects NT values, never stale
    // constructor values from HoodIOKraken.
    LoggedTunableNumber.ifChanged(
        GainsID,
        values -> io.reconfigureGains(kP.get(), kD.get(), kS.get(), kG.get(), kV.get(), kA.get()));
    LoggedTunableNumber.ifChanged(
        ProfileID, values -> io.reconfigureProfile(mmCruise.get(), mmAccel.get(), mmJerk.get()));

    // ── Coast override when disabled ─────────────────────────────────────
    io.setBrakeMode(!coastOverride.getAsBoolean());

    // ── Goal state machine ───────────────────────────────────────────────
    switch (goal) {
      case OPEN_LOOP_TEST -> io.setVoltage(testVoltage);
      case ZEROING -> runZeroing();
      case ZEROED, FIXED, TRACK_TARGET -> {
        double clamped =
            Math.max(
                HoodConstants.MIN_ANGLE.getRotations(),
                Math.min(HoodConstants.MAX_ANGLE.getRotations(), setpoint.getRotations()));
        io.setPosition(clamped);
      }
      case IDLE -> io.setVoltage(0.0);
    }

    // ── Alerts ───────────────────────────────────────────────────────────
    faultAlert.set(inputs.stickyFaultAny);
    notZeroedAlert.set(!zeroed);

    // ── Logging ──────────────────────────────────────────────────────────
    Logger.recordOutput("Hood/SetpointDeg", setpoint.getDegrees());
    Logger.recordOutput("Hood/PositionDeg", inputs.position.getDegrees());
    Logger.recordOutput("Hood/AtGoal", atGoal());
    Logger.recordOutput("Hood/Zeroed", zeroed);
  }

  private void runZeroing() {
    // Drive slowly into the lower hard stop (-30° side).
    // ZERO_VOLTAGE is negative so it moves in the correct direction.
    io.setVoltage(HoodConstants.ZERO_VOLTAGE);

    if (inputs.supplyCurrentAmps > HoodConstants.ZERO_CURRENT_THRESHOLD_AMPS) {
      zeroStallTimer.start();
    } else {
      zeroStallTimer.reset();
      zeroStallTimer.stop();
    }

    if (zeroStallTimer.hasElapsed(HoodConstants.ZERO_STALL_TIME_SECS)) {
      io.zeroSensor(); // sets sensor = ZERO_ANGLE (-27°)
      io.setVoltage(0.0);
      zeroed = true;
      goal = Goal.ZEROED;
      setpoint = HoodConstants.MIN_ANGLE;
      zeroStallTimer.stop();
      zeroStallTimer.reset();
    }
  }

  // ── Public API ───────────────────────────────────────────────────────────

  @AutoLogOutput
  public boolean atGoal() {
    return Math.abs(inputs.position.minus(setpoint).getDegrees())
        < HoodConstants.AT_GOAL_TOLERANCE.getDegrees();
  }

  public boolean isZeroed() {
    return zeroed;
  }

  public void setCoastOverride(BooleanSupplier coast) {
    this.coastOverride = coast;
  }

  // ── Test command API (called by ShooterTestCommands) ─────────────────────

  /**
   * Bypass the goal state machine entirely for open-loop testing. Only use with startEnd() so
   * voltage is reset to 0 on release.
   */
  public void setTestVoltage(double volts) {
    goal = Goal.OPEN_LOOP_TEST;
    testVoltage = volts;
  }

  /** Set a fixed setpoint directly. Used by test and auto commands. */
  public void setFixedSetpoint(Rotation2d angle) {
    goal = Goal.FIXED;
    setpoint = angle;
  }

  /**
   * Increment or decrement the current setpoint by delta. Clamps to MIN/MAX_ANGLE automatically.
   */
  public void nudgeSetpoint(Rotation2d delta) {
    goal = Goal.FIXED;
    double newDeg = setpoint.getDegrees() + delta.getDegrees();
    newDeg =
        Math.max(
            HoodConstants.MIN_ANGLE.getDegrees(),
            Math.min(HoodConstants.MAX_ANGLE.getDegrees(), newDeg));
    setpoint = Rotation2d.fromDegrees(newDeg);
  }

  /** Stop all motor output and return to idle. */
  public void setGoalIdle() {
    goal = Goal.IDLE;
  }

  /** Current position in degrees for console reporting. */
  public double getPositionDeg() {
    return inputs.position.getDegrees();
  }

  // ── Commands ──────────────────────────────────────────────────────────────

  /**
   * Run the zeroing sequence. Must complete before position commands work. Drives into the lower
   * hard stop, sets sensor, then holds MIN_ANGLE.
   */
  public Command zeroCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  goal = Goal.ZEROING;
                  zeroStallTimer.reset();
                  zeroStallTimer.stop();
                },
                this),
            Commands.waitUntil(() -> goal == Goal.ZEROED),
            Commands.runOnce(() -> setpoint = HoodConstants.MIN_ANGLE))
        .withName("Hood Zero");
  }

  public Command skipZeroCommand() {
    return Commands.runOnce(
            () -> {
              hoodZeroed = true;
              goal = Goal.ZEROED;
              hoodOffsetRads = 0.0;

              // Start lastGoalAngleRads at current LaunchCalculator angle
              lastGoalAngleRads =
                  LaunchCalculator.getInstance().getParameters().turretAngle().getRadians();
            },
            this)
        .withName("Hood Skip Zero");
  }

  /** Go to a fixed angle and hold it while the command is running. */
  public Command runFixedCommand(Supplier<Rotation2d> angleSupplier) {
    return Commands.run(
            () -> {
              goal = Goal.FIXED;
              setpoint = angleSupplier.get();
            },
            this)
        .withName("Hood Fixed");
  }

  /** Continuously track the LaunchCalculator's hood angle. */
  public Command runTrackTargetCommand() {
    return Commands.run(
            () -> {
              goal = Goal.TRACK_TARGET;
              setpoint = new Rotation2d(LaunchCalculator.getInstance().getParameters().hoodAngle());
            },
            this)
        .withName("Hood Track Target");
  }

  /** Open-loop voltage sweep for SysId characterization. */
  public Command runCharacterizationCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> goal = Goal.IDLE, this),
            Commands.run(() -> io.setVoltage(2.0), this))
        .withName("Hood Characterization");
  }
}
