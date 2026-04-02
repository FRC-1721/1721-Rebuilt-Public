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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tidalforce.frc2026.subsystems.shooter.hood.Hood;
import org.tidalforce.frc2026.subsystems.shooter.hood.HoodConstants;
import org.tidalforce.frc2026.subsystems.shooter.turret.Turret;
import org.tidalforce.frc2026.subsystems.shooter.turret.TurretConstants;

public class ShooterTestCommands {

  private ShooterTestCommands() {}

  // =========================================================================
  // PHASE 1 — Open Loop Voltage (run these FIRST, before any closed loop)
  // Purpose: confirm motor direction and that nothing mechanical is wrong.
  // Safe because: very low voltage, stops the instant you release the button.
  // =========================================================================

  /**
   * Drives the hood with a fixed low voltage while held. Use +volts to find which direction is
   * "up", -volts for "down". Watch Hood/PositionDeg in AdvantageScope to confirm direction.
   *
   * <p>HOW TO USE: Bind positive and negative versions to two buttons. Hold positive — if hood goes
   * DOWN, flip InvertedValue in HoodIOKraken. Hold negative — hood should approach the lower hard
   * stop slowly. Note the PositionDeg value at each hard stop for setting MIN/MAX_ANGLE.
   */
  public static Command hoodOpenLoop(Hood hood, double volts) {
    return Commands.run(() -> hood.setTestVoltage(volts), hood)
        .finallyDo(() -> hood.setGoalIdle())
        .withName("Hood Open Loop" + (volts > 0 ? "+" : "") + volts + "V");
  }

  /**
   * Same as above for the turret. Use +volts to find which direction is CCW (positive = CCW is
   * standard). Watch Turret/PositionDeg in AdvantageScope. Note PositionDeg at each hard stop for
   * setting MIN/MAX_ANGLE.
   */
  public static Command turretOpenLoop(Turret turret, double volts) {
    return Commands.run(() -> turret.setTestVoltage(volts), turret)
        .finallyDo(() -> turret.setGoalIdle())
        .withName("Turret Open Loop" + (volts > 0 ? "+" : "") + volts + "V");
  }

  // =========================================================================
  // PHASE 2 — Zeroing
  // Purpose: verify the zeroing sequence works before any position commands.
  // Run this before any Phase 3+ commands — position commands mean nothing
  // until the sensor is zeroed.
  // =========================================================================

  /**
   * Runs the hood zero sequence and prints result to console. Watch Hood/Zeroed in AdvantageScope —
   * should flip to true. After it completes, Hood/PositionDeg should read ~MIN_ANGLE.
   */
  public static Command hoodZeroAndReport(Hood hood) {
    return hood.zeroCommand()
        .andThen(
            Commands.runOnce(
                () ->
                    System.out.println(
                        "[Hood] Zero complete. Position: " + hood.getPositionDeg() + "°")))
        .withName("Hood Zero and Report");
  }

  /**
   * Runs the turret zero sequence and prints result to console. After completion,
   * Turret/PositionDeg should read ~0° (forward).
   */
  public static Command turretZeroAndReport(Turret turret) {
    return turret
        .zeroCommand()
        .andThen(
            Commands.runOnce(
                () ->
                    System.out.println(
                        "[Turret] Zero complete. Position: " + turret.getPositionDeg() + "°")))
        .withName("Turret Zero and Report");
  }

  // =========================================================================
  // PHASE 3 — Fixed Position Commands
  // Purpose: verify closed-loop works and position units are correct.
  // Only run these AFTER zeroing.
  // =========================================================================

  /**
   * Moves the hood to a specific angle and holds it. Releases when button is released, returning to
   * idle (voltage = 0).
   *
   * <p>SAFE ANGLES TO TRY FIRST: MIN_ANGLE + 5° — barely above the hard stop, safe (MIN + MAX) / 2
   * — midpoint, good for testing kG MAX_ANGLE - 5° — near top, confirms full range
   *
   * <p>Watch Hood/PositionDeg vs Hood/SetpointDeg in AdvantageScope. If they don't converge, kP is
   * too low. If PositionDeg overshoots and oscillates, kP is too high or kD too low.
   */
  public static Command hoodGoToAngle(Hood hood, Rotation2d angle) {
    return Commands.startEnd(() -> hood.setFixedSetpoint(angle), () -> hood.setGoalIdle(), hood)
        .withName("Hood Go To " + (int) angle.getDegrees() + "°");
  }

  /**
   * Moves the turret to a robot-relative angle and holds it. 0° = straight forward. Positive = CCW
   * (left). Negative = CW (right).
   *
   * <p>SAFE ANGLES TO TRY FIRST: 0° — forward, should be your zero reference 30° — small CCW move,
   * safe -30° — small CW move, safe 90° — larger move, only after 30° is confirmed working
   *
   * <p>Watch Turret/PositionDeg vs Turret/SetpointDeg in AdvantageScope.
   */
  public static Command turretGoToAngle(Turret turret, Rotation2d angle) {
    return Commands.startEnd(
            () -> turret.setFixedSetpoint(angle), () -> turret.setGoalIdle(), turret)
        .withName("Turret Go To " + (int) angle.getDegrees() + "°");
  }

  // =========================================================================
  // PHASE 4 — Nudge Commands
  // Purpose: fine-tune position interactively without binding every angle.
  // Great for finding the correct kG on the hood.
  // =========================================================================

  /**
   * Increments the hood setpoint by deltaDeg each time the button is pressed. Use +5.0 for "up"
   * button and -5.0 for "down" button. The hood holds whatever position it nudged to when you
   * release.
   *
   * <p>HOW TO USE: Bind hoodNudge(hood, +5.0) to one button. Bind hoodNudge(hood, -5.0) to another
   * button. Press repeatedly to walk the hood to any position. Watch Hood/SetpointDeg to see where
   * you've commanded it.
   */
  public static Command hoodNudge(Hood hood, double deltaDeg) {
    return Commands.runOnce(() -> hood.nudgeSetpoint(Rotation2d.fromDegrees(deltaDeg)), hood)
        .withName("Hood Nudge " + (deltaDeg > 0 ? "+" : "") + deltaDeg + "°");
  }

  /** Same nudge pattern for the turret. Use +5.0 for CCW and -5.0 for CW. */
  public static Command turretNudge(Turret turret, double deltaDeg) {
    return Commands.runOnce(() -> turret.nudgeSetpoint(Rotation2d.fromDegrees(deltaDeg)), turret)
        .withName("Turret Nudge " + (deltaDeg > 0 ? "+" : "") + deltaDeg + "°");
  }

  // =========================================================================
  // PHASE 5 — Slow Sweep
  // Purpose: verify the full range of motion, soft limits, and that the
  // mechanism doesn't bind or skip anywhere in its travel.
  // Only run after fixed position commands are confirmed working.
  // =========================================================================

  /**
   * Slowly sweeps the hood from MIN_ANGLE to MAX_ANGLE and back, once. The sweep pauses 1 second at
   * each end so you can verify atGoal().
   *
   * <p>WHAT TO WATCH: Hood/PositionDeg — should follow Hood/SetpointDeg cleanly Hood/AtGoal —
   * should go true at each end before moving on Hood/appliedVolts — should stay reasonable (< 8V
   * typically) No grinding or binding sounds from the mechanism
   *
   * <p>If atGoal() never goes true at an endpoint, loosen AT_GOAL_TOLERANCE or raise kP.
   */
  public static Command hoodSweep(Hood hood) {
    return Commands.sequence(
            Commands.runOnce(() -> System.out.println("[Hood] Starting sweep to MIN_ANGLE")),
            Commands.startEnd(() -> hood.setFixedSetpoint(HoodConstants.MIN_ANGLE), () -> {}, hood)
                .until(hood::atGoal),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> System.out.println("[Hood] Sweep to MAX_ANGLE")),
            Commands.startEnd(() -> hood.setFixedSetpoint(HoodConstants.MAX_ANGLE), () -> {}, hood)
                .until(hood::atGoal),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> System.out.println("[Hood] Sweep complete")),
            Commands.runOnce(() -> hood.setGoalIdle(), hood))
        .withName("Hood Full Sweep");
  }

  /**
   * Slowly sweeps the turret from MIN_ANGLE to MAX_ANGLE and back, once. Watch for cable wrap
   * issues anywhere in the travel range.
   *
   * <p>WHAT TO WATCH: Turret/PositionDeg tracks Turret/SetpointDeg Turret/AtGoal goes true at each
   * endpoint No cable tension or binding through the travel SoftLimit indicators in Phoenix Tuner —
   * should never trigger if MIN/MAX_ANGLE are set correctly
   */
  public static Command turretSweep(Turret turret) {
    return Commands.sequence(
            Commands.runOnce(() -> System.out.println("[Turret] Starting sweep to MIN_ANGLE")),
            Commands.startEnd(
                    () -> turret.setFixedSetpoint(TurretConstants.MIN_ANGLE), () -> {}, turret)
                .until(turret::atGoal),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> System.out.println("[Turret] Sweep to MAX_ANGLE")),
            Commands.startEnd(
                    () -> turret.setFixedSetpoint(TurretConstants.MAX_ANGLE), () -> {}, turret)
                .until(turret::atGoal),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> System.out.println("[Turret] Returning to zero")),
            Commands.startEnd(() -> turret.setFixedSetpoint(Rotation2d.kZero), () -> {}, turret)
                .until(turret::atGoal),
            Commands.runOnce(() -> System.out.println("[Turret] Sweep complete")),
            Commands.runOnce(() -> turret.setGoalIdle(), turret))
        .withName("Turret Full Sweep");
  }

  // =========================================================================
  // PHASE 6 — Hold Position Under Load
  // Purpose: tune kG on the hood. Not needed for turret.
  // =========================================================================

  /**
   * Moves the hood to 45° (roughly horizontal) and holds it indefinitely. Use this to tune kG:
   *
   * <p>1. Run this command. 2. Watch Hood/appliedVolts in AdvantageScope. 3. Increase Hood/kG in
   * NetworkTables (Glass or AdvantageScope). 4. Stop when appliedVolts ≈ Hood/kS and hood isn't
   * drifting.
   *
   * <p>If the hood slowly sags downward: kG is too low — raise it. If the hood slowly creeps
   * upward: kG is too high — lower it.
   */
  public static Command hoodHoldHorizontal(Hood hood) {
    Rotation2d horizontal =
        Rotation2d.fromDegrees(
            (HoodConstants.MIN_ANGLE.getDegrees() + HoodConstants.MAX_ANGLE.getDegrees()) / 2.0);
    return Commands.startEnd(
            () -> hood.setFixedSetpoint(horizontal), () -> hood.setGoalIdle(), hood)
        .withName("Hood Hold Horizontal (" + (int) horizontal.getDegrees() + "°)");
  }
}
