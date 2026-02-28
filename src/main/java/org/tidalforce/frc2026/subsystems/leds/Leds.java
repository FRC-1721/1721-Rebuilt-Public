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

package org.tidalforce.frc2026.subsystems.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import java.util.TreeSet;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.devices.Lights;
import org.tidalforce.frc2026.util.LoggerHelper;

/**
 * Subsystem that controls the robot's LED lights for visual feedback and animations. Provides
 * commands for different animation patterns during disabled, autonomous, and teleop modes.
 */
public class LEDs extends SubsystemBase {
  /** The states for the lights in order from highest priority to low */
  @Getter
  @RequiredArgsConstructor
  @SuppressWarnings("ImmutableEnumChecker")
  public enum State {
    // list of states with their respective priorities, ie if both RUNNING_AUTO and
    // RUNNING_INTAKE are true it will set to RUNNING_AUTO

    DISABLED(0, LEDsConstants.disabledAnimation),
    RUNNING_AUTO(1, LEDsConstants.autoAnimation),
    PATHFINDING_AGGRESIVE(2, LEDsConstants.solidBlueAnim),
    READY_TO_SHOOT(2, LEDsConstants.flashAnimation),
    RUNNING_INTAKE(3, LEDsConstants.solidGreenAnim),
    NONE(4, LEDsConstants.offAnimation);

    // Lower priority first
    private final int priority;
    private final List<ControlRequest> animation;
  }

  private final Lights lights;

  private final TreeSet<State> stateQueue;
  private State currentState = State.NONE;

  /**
   * Constructs an LEDs subsystem.
   *
   * @param io The lights IO interface for controlling the LED hardware
   */
  public LEDs(LightsIO io) {
    lights = new Lights(io);

    stateQueue = new TreeSet<>((a, b) -> Integer.compare(a.getPriority(), b.getPriority()));

    lights.setAnimations(currentState.getAnimation());
  }

  private Optional<State> getCurrentStateFromQueue() {

    State currentState = stateQueue.isEmpty() ? null : stateQueue.first();

    return Optional.ofNullable(currentState);
  }

  private boolean updateCurrentState() {
    Optional<State> newState = getCurrentStateFromQueue();

    boolean hasChanged = !this.currentState.equals(newState.orElse(null));

    this.currentState = newState.orElse(State.NONE);
    return hasChanged;
  }

  private void updateState() {
    boolean hasChanged = updateCurrentState();
    if (hasChanged) {
      lights.setAnimations(currentState.getAnimation());
    }
  }

  @Override
  public void periodic() {
    updateState();

    LoggerHelper.recordCurrentCommand(LEDsConstants.NAME, this);
    Logger.recordOutput(LEDsConstants.NAME + "/State", currentState.name());
    Logger.recordOutput(
        LEDsConstants.NAME + "/StateQueue",
        stateQueue.stream().map(s -> s.name()).toArray(String[]::new));
  }

  public Command scheduleStateCommand(State state) {
    return this.runOnce(() -> stateQueue.add(state));
  }

  public Command unscheduleStateCommand(State state) {
    return this.runOnce(() -> stateQueue.remove(state));
  }
}
