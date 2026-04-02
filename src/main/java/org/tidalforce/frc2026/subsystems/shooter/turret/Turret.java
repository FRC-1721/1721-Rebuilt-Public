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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.DevBotMech3d;
import org.tidalforce.frc2026.RobotState;
import org.tidalforce.frc2026.subsystems.shooter.LaunchCalculator;
import org.tidalforce.frc2026.util.LoggedTunableNumber;

public class Turret extends SubsystemBase {

  // ── Angle limits & tracking margins (mirroring 6328 pattern) ────────────
  // Hard mechanical travel is defined in TurretConstants (MIN/MAX_ANGLE).
  // trackMin/MaxAngle extend ±1 full rotation past center + a small overlap
  // margin, so the turret can track through the ±π discontinuity without
  // snapping to the wrong revolution.
  private static final double MIN_RADS = TurretConstants.MIN_ANGLE.getRadians();
  private static final double MAX_RADS = TurretConstants.MAX_ANGLE.getRadians();
  private static final double TRACK_CENTER_RADS = (MIN_RADS + MAX_RADS) / 2.0;
  private static final double TRACK_OVERLAP_RADS = Math.toRadians(10.0);
  private static final double TRACK_MIN_RADS = TRACK_CENTER_RADS - Math.PI - TRACK_OVERLAP_RADS;
  private static final double TRACK_MAX_RADS = TRACK_CENTER_RADS + Math.PI + TRACK_OVERLAP_RADS;

  // ── Single source of truth for all gains ────────────────────────────────
  // These are the ONLY place gain values live. TurretConstants has no gains.
  // Change the default values here. They load into NetworkTables on boot
  // and can be edited live from Glass during testing.
  //
  // HOW TO TUNE:
  //   kP  — Raise until oscillation, back off ~30%. Expect final: 40-100.
  //   kD  — Raise after kP is set to damp oscillation. Expect final: 1-4.
  //   kS  — Raise if turret doesn't reach setpoint on small moves. Max ~1.5.
  //         This is the fix for consistent 1° undershoot — raise in 0.1 steps.
  //   kI  — Add only if kS alone doesn't close remaining steady-state error.
  //         Keep very small (0.5) and always pair with kIZone.
  //   kV  — Leave at placeholder until SysId characterization is run.
  //   kA  — Leave at placeholder until SysId characterization is run.
  //
  // HOW TO CHANGE SPEED:
  //   Faster overall:    raise MM_CruiseVelocityRPS (try 4.0 → 8.0)
  //   Faster ramp-up:    raise MM_AccelerationRPS2 (keep at 2-4x cruise)
  //   Smoother start:    raise MM_JerkRPS3 from 0 (try 200)
  //   Still too slow:    raise PeakForwardVoltage in TurretIOKraken toward 12.0
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", 25.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", 0.5);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", 0.3);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", 0.12);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA", 0.01);
  private static final LoggedTunableNumber mmCruise =
      new LoggedTunableNumber("Turret/MM_CruiseVelocityRPS", 1.5);
  private static final LoggedTunableNumber mmAccel =
      new LoggedTunableNumber("Turret/MM_AccelerationRPS2", 5.0);
  private static final LoggedTunableNumber mmJerk =
      new LoggedTunableNumber("Turret/MM_JerkRPS3", 0.0);

  private static final int GainsID = 0;
  private static final int ProfileID = 1;

  // ── Enums ────────────────────────────────────────────────────────────────

  /**
   * Lifecycle goal — used exclusively for the zeroing sequence. Once zeroed, all movement is
   * governed by LaunchState + the field-relative goal angle set by the active command.
   */
  public enum Goal {
    ZEROING,
    ZEROED,
    IDLE
  }

  /**
   * Tracking mode — mirrors 6328's LaunchState. Determines which wrap range is legal for
   * multi-revolution selection: ACTIVE_LAUNCHING → full hard-stop range (MIN/MAX_ANGLE) TRACKING →
   * ±(π + overlap) around center, allows crossing ±π
   */
  public enum LaunchState {
    ACTIVE_LAUNCHING,
    TRACKING
  }

  // ── State ────────────────────────────────────────────────────────────────

  @Getter @AutoLogOutput private Goal goal = Goal.IDLE;

  @Getter @Setter @AutoLogOutput private LaunchState launchState = LaunchState.ACTIVE_LAUNCHING;

  // Field-relative goal set by the active command each loop.
  // periodic() converts these to a robot-relative position target.
  private Rotation2d goalAngle = Rotation2d.kZero;
  private double goalVelocityRadPerSec = 0.0;

  // Used by the multi-revolution wrap selector to prefer continuity over
  // the shortest path — prevents sudden 360° snaps when crossing ±π.
  private double lastGoalAngleRads = 0.0;

  // ── Software zeroing offset (6328-style) ─────────────────────────────────
  // Instead of writing to the encoder (io.zeroSensor), we store the
  // difference between raw sensor and the desired logical zero here.
  //   getPositionRads() == inputs.position.getRadians() + turretOffsetRads
  // After zeroing at the hard stop:
  //   getPositionRads() == 0 means forward-facing.
  private double turretOffsetRads = 0.0;
  private boolean turretZeroed = false;
  private final Timer zeroStallTimer = new Timer();

  // ── Coast override ───────────────────────────────────────────────────────
  @Setter private BooleanSupplier coastOverride = () -> false;

  // ── IO ───────────────────────────────────────────────────────────────────
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  // ── Alerts ───────────────────────────────────────────────────────────────
  private final Alert notZeroedAlert =
      new Alert(
          "Turret has not been zeroed — run zero sequence before shooting",
          Alert.AlertType.kWarning);
  private final Alert faultAlert =
      new Alert("Turret motor has active faults — check Tuner X", Alert.AlertType.kError);

  // ── atGoal ───────────────────────────────────────────────────────────────
  // Fluent accessor so callers can write turret.atGoal() like 6328.
  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  // ─────────────────────────────────────────────────────────────────────────
  public Turret(TurretIO io) {
    this.io = io;
  }

  // ── Periodic ─────────────────────────────────────────────────────────────
  //
  // NOTE — periodicAfterScheduler():
  //   6328 runs field-relative tracking math in periodicAfterScheduler() so it
  //   executes *after* RobotState has been updated by the drivetrain. That
  //   requires a custom FullSubsystem base class. We keep everything here for
  //   now. If you add FullSubsystem, move the "Enabled + zeroed" block below
  //   into an override of periodicAfterScheduler().
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // ── Alerts ───────────────────────────────────────────────────────────
    faultAlert.set(inputs.stickyFaultAny);
    notZeroedAlert.set(!turretZeroed);

    // ── Push gains to motor whenever an NT value changes ─────────────────
    LoggedTunableNumber.ifChanged(
        GainsID,
        () -> io.reconfigureGains(kP.get(), kD.get(), kS.get(), kV.get(), kA.get()),
        kP,
        kD,
        kS,
        kV,
        kA);
    LoggedTunableNumber.ifChanged(
        ProfileID,
        () -> io.reconfigureProfile(mmCruise.get(), mmAccel.get(), mmJerk.get()),
        mmCruise,
        mmAccel,
        mmJerk);

    // ── Brake / coast ────────────────────────────────────────────────────
    io.setBrakeMode(!coastOverride.getAsBoolean());

    // ── Zeroing state machine ────────────────────────────────────────────
    if (goal == Goal.ZEROING) {
      runZeroing();
    }

    // ── Disabled or not zeroed: hold still, reset wrap-selector state ─────
    if (DriverStation.isDisabled() || !turretZeroed) {
      if (goal != Goal.ZEROING) {
        io.setVoltage(0.0);
      }
      // Seed lastGoalAngle so the first enabled frame picks the nearest wrap.
      lastGoalAngleRads = getPositionRads();
      atGoal = false;
      logBasic();
      return;
    }

    // ── Enabled + zeroed: field-relative tracking ─────────────────────────

    // Pull robot pose from RobotState (updated by the drivetrain this frame).
    Rotation2d robotAngle = RobotState.getInstance().getRotation();
    double robotAngularVelocityRadPerSec =
        RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond;

    // Convert field-relative goal to robot-relative.
    Rotation2d robotRelativeGoal = goalAngle.minus(robotAngle);
    double robotRelativeVelocity = goalVelocityRadPerSec - robotAngularVelocityRadPerSec;

    // ── Multi-revolution wrap selection ───────────────────────────────────
    // LaunchState picks the legal range. ACTIVE_LAUNCHING uses the full
    // mechanical range; TRACKING allows ±(π + overlap) around center so the
    // turret can cross ±π smoothly without reversing through 360°.
    double minLegal =
        switch (launchState) {
          case ACTIVE_LAUNCHING -> MIN_RADS;
          case TRACKING -> TRACK_MIN_RADS;
        };
    double maxLegal =
        switch (launchState) {
          case ACTIVE_LAUNCHING -> MAX_RADS;
          case TRACKING -> TRACK_MAX_RADS;
        };

    // Iterate ±2 full turns. Pick the legal candidate closest to the
    // previous goal so we don't snap across a revolution boundary.
    boolean hasBestAngle = false;
    double bestAngle = 0.0;
    for (int i = -2; i < 3; i++) {
      double candidate = robotRelativeGoal.getRadians() + Math.PI * 2.0 * i;
      if (candidate < minLegal || candidate > maxLegal) continue;
      if (!hasBestAngle
          || Math.abs(lastGoalAngleRads - candidate) < Math.abs(lastGoalAngleRads - bestAngle)) {
        bestAngle = candidate;
        hasBestAngle = true;
      }
    }
    lastGoalAngleRads = bestAngle; // persist for next frame

    double clampedRads = MathUtil.clamp(bestAngle, minLegal, maxLegal);

    // ── atGoal ───────────────────────────────────────────────────────────
    atGoal =
        Math.abs(getPositionRads() - clampedRads) < TurretConstants.AT_GOAL_TOLERANCE.getRadians();

    // ── Send position command (subtract offset to recover raw sensor target)
    // io.setPosition() expects rotations in raw-sensor space.
    double rawTargetRots = (clampedRads - turretOffsetRads) / (2.0 * Math.PI);
    io.setPosition(rawTargetRots);

    // ── RobotState — timestamp-compensated for vision latency ────────────
    RobotState.getInstance()
        .addTurretObservation(
            new RobotState.TurretObservation(
                Timer.getFPGATimestamp(), new Rotation2d(getPositionRads())));

    // ── Mech3d ───────────────────────────────────────────────────────────
    DevBotMech3d.getMeasured().setTurretAngle(new Rotation2d(getPositionRads()));

    // ── Logging ──────────────────────────────────────────────────────────
    Logger.recordOutput("Turret/GoalPositionDeg", Math.toDegrees(bestAngle));
    Logger.recordOutput("Turret/GoalVelocityRadPerSec", robotRelativeVelocity);
    logBasic();
  }

  /** Shared logging emitted every loop regardless of enabled state. */
  private void logBasic() {
    Logger.recordOutput("Turret/PositionDeg", getPositionDeg());
    Logger.recordOutput("Turret/AtGoal", atGoal);
    Logger.recordOutput("Turret/Zeroed", turretZeroed);
  }

  // ── Zeroing ──────────────────────────────────────────────────────────────
  // Drives slowly into the CW hard stop (ZERO_VOLTAGE is negative).
  // When stall current is sustained for ZERO_STALL_TIME_SECS, we compute
  // turretOffsetRads so that getPositionRads() == 0 means forward-facing.
  // No encoder write — the offset is pure software (6328 style).
  private void runZeroing() {
    io.setVoltage(TurretConstants.ZERO_VOLTAGE);

    if (inputs.supplyCurrentAmps > TurretConstants.ZERO_CURRENT_THRESHOLD_AMPS) {
      zeroStallTimer.start();
    } else {
      zeroStallTimer.reset();
      zeroStallTimer.stop();
    }

    if (zeroStallTimer.hasElapsed(TurretConstants.ZERO_STALL_TIME_SECS)) {
      // At the hard stop: logical position should equal -ZERO_OFFSET_FROM_HARDSTOP.
      // Solving getPositionRads() == -ZERO_OFFSET_FROM_HARDSTOP.getRadians():
      //   inputs.position.getRadians() + turretOffsetRads == -ZERO_OFFSET.getRadians()
      //   turretOffsetRads = -ZERO_OFFSET.getRadians() - inputs.position.getRadians()
      turretOffsetRads =
          TurretConstants.ZERO_OFFSET_FROM_HARDSTOP.unaryMinus().getRadians()
              - inputs.position.getRadians();
      io.setVoltage(0.0);
      turretZeroed = true;
      goal = Goal.ZEROED;
      goalAngle = Rotation2d.kZero;
      lastGoalAngleRads = 0.0;
      zeroStallTimer.stop();
      zeroStallTimer.reset();
    }
  }

  // ── Internal field-relative goal setter ──────────────────────────────────
  /** Called by every command that drives the turret. */
  private void setFieldRelativeTarget(Rotation2d angle, double velocityRadPerSec) {
    this.goalAngle = angle;
    this.goalVelocityRadPerSec = velocityRadPerSec;
  }

  // ── Measured-position accessors ───────────────────────────────────────────

  /** Logical position in radians (raw sensor + software offset). */
  @AutoLogOutput(key = "Turret/MeasuredPositionRad")
  public double getPositionRads() {
    return inputs.position.getRadians() + turretOffsetRads;
  }

  /** Convenience accessor in degrees for console / test commands. */
  public double getPositionDeg() {
    return Math.toDegrees(getPositionRads());
  }

  // ── Public API ────────────────────────────────────────────────────────────

  public boolean isZeroed() {
    return turretZeroed;
  }

  /**
   * Bypass the goal state machine for open-loop voltage testing. Use only inside a startEnd() so
   * voltage resets to 0 on release.
   */
  public void setTestVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Set a fixed field-relative setpoint directly. Used by test commands and ShooterTestCommands.
   */
  public void setFixedSetpoint(Rotation2d angle) {
    setFieldRelativeTarget(angle, 0.0);
    launchState = LaunchState.TRACKING;
  }

  /**
   * Increment or decrement the current field-relative goal by delta. Useful for fine-tuning during
   * testing.
   */
  public void nudgeSetpoint(Rotation2d delta) {
    setFieldRelativeTarget(goalAngle.plus(delta), 0.0);
    launchState = LaunchState.TRACKING;
  }

  /** Stop all motor output and return to idle. */
  public void setGoalIdle() {
    goal = Goal.IDLE;
    io.setVoltage(0.0);
  }

  // ── Commands ──────────────────────────────────────────────────────────────

  /**
   * Run the hard-stop zeroing sequence. Must complete before position commands work. Drives CW into
   * the mechanical stop, detects stall via supply current, then records the software encoder
   * offset. No encoder write — pure software offset.
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
            Commands.waitUntil(() -> goal == Goal.ZEROED))
        .withName("Turret Zero");
  }

  public Command skipZeroCommand() {
    return Commands.runOnce(
            () -> {
              turretZeroed = true;
              goal = Goal.ZEROED;
              turretOffsetRads = 0.0;

              // Start lastGoalAngleRads at current LaunchCalculator angle
              lastGoalAngleRads =
                  LaunchCalculator.getInstance().getParameters().turretAngle().getRadians();
            },
            this)
        .withName("Turret Skip Zero");
  }

  /**
   * Continuously track the hub in TRACKING mode (±(π + overlap) wrap range). Use this during
   * approach / aiming — allows smooth tracking through ±π.
   */
  public Command runTrackTargetCommand() {
    return run(() -> {
          var params = LaunchCalculator.getInstance().getParameters();

          // Clamp to TRACKING wrap range to prevent "moving away from hub"
          double clampedRads =
              MathUtil.clamp(params.turretAngle().getRadians(), TRACK_MIN_RADS, TRACK_MAX_RADS);

          setFieldRelativeTarget(new Rotation2d(clampedRads), params.turretVelocity());
          setLaunchState(LaunchState.TRACKING);
        })
        .withName("Turret Track Target");
  }

  /**
   * Continuously track the hub in ACTIVE_LAUNCHING mode (full mechanical range). Use this during
   * the actual shot — no wrap range restriction so the turret can reach any legal angle.
   */
  public Command runTrackTargetActiveLaunchingCommand() {
    return run(() -> {
          var params = LaunchCalculator.getInstance().getParameters();

          // Clamp to full mechanical range for ACTIVE_LAUNCHING
          double clampedRads =
              MathUtil.clamp(params.turretAngle().getRadians(), MIN_RADS, MAX_RADS);

          setFieldRelativeTarget(new Rotation2d(clampedRads), params.turretVelocity());
          setLaunchState(LaunchState.ACTIVE_LAUNCHING);
        })
        .withName("Turret Track Target Active Launching");
  }

  /**
   * Go to a fixed field-relative angle and hold it while the command runs.
   *
   * @param angleSupplier target field-relative angle (0° = field forward)
   * @param velocity feedforward velocity in rad/s; pass {@code () -> 0.0} if none
   */
  public Command runFixedCommand(Supplier<Rotation2d> angleSupplier, DoubleSupplier velocity) {
    return run(() -> {
          setFieldRelativeTarget(angleSupplier.get(), velocity.getAsDouble());
          setLaunchState(LaunchState.TRACKING);
        })
        .withName("Turret Fixed");
  }
}
