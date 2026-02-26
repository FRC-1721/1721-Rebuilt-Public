// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.tidalforce.frc2026.util.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for physical override switches on operator console. */
public class OverrideSwitches {
  private final GenericHID joystick;

  public OverrideSwitches(int port) {
    joystick = new GenericHID(port);
  }

  /** Returns whether the controller is connected. */
  public boolean isConnected() {
    return joystick.isConnected();
  }

  /** Gets the state of a driver-side switch (0-2 from left to right). */
  public boolean getDriverSwitch(int index) {
    if (index < 0 || index > 4) {
      throw new RuntimeException(
          "Invalid driver override index " + Integer.toString(index) + ". Must be 0-2.");
    }
    return joystick.getRawButton(index + 8);
  }

  /** Gets the state of the multi-directional switch. */
  public MultiDirectionSwitchState getMultiDirectionSwitch() {
    if (joystick.getRawButton(4)) {
      return MultiDirectionSwitchState.DOWN_2;
    } else if (joystick.getRawButton(5)) {
      return MultiDirectionSwitchState.UP_2;
    } else if (joystick.getRawButton(7)) {
        return MultiDirectionSwitchState.DOWN_1;
    } else if (joystick.getRawButton(6)) {
        return MultiDirectionSwitchState.UP_1;
    } else {
      return MultiDirectionSwitchState.NEUTRAL;
    }
  }

  /** Returns a trigger for a driver-side switch (0-2 from left to right). */
  public Trigger driverSwitch(int index) {
    return new Trigger(() -> getDriverSwitch(index));
  }

  /** Returns a trigger for when the multi-directional switch is pushed up. */
  public Trigger multiDirectionSwitch1Down() {
    return new Trigger(() -> getMultiDirectionSwitch() == MultiDirectionSwitchState.DOWN_1);
  }

  /** Returns a trigger for when the multi-directional switch is pushed up. */
  public Trigger multiDirectionSwitch1Up() {
    return new Trigger(() -> getMultiDirectionSwitch() == MultiDirectionSwitchState.UP_1);
  }

  /** Returns a trigger for when the multi-directional switch is pushed up. */
  public Trigger multiDirectionSwitch2Down() {
    return new Trigger(() -> getMultiDirectionSwitch() == MultiDirectionSwitchState.DOWN_2);
  }

  /** Returns a trigger for when the multi-directional switch is pushed down. */
  public Trigger multiDirectionSwitch2Up() {
    return new Trigger(() -> getMultiDirectionSwitch() == MultiDirectionSwitchState.UP_2);
  }

  /** The state of the multi-directional switch. */
  public static enum MultiDirectionSwitchState {
    UP_1,
    UP_2,
    NEUTRAL,
    DOWN_1,
    DOWN_2
  }
}