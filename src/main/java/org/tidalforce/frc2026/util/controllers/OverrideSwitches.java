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
