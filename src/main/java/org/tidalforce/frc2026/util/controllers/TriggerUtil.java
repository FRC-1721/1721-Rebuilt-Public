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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerUtil {
  private TriggerUtil() {}

  /**
   * Constantly starts the given command while the button is held.
   *
   * <p>{@link Command#schedule()} will be called repeatedly while the trigger is active, and will
   * be canceled when the trigger becomes inactive.
   *
   * @param command The command to start.
   * @return This trigger, so calls can be chained.
   */
  public static void whileTrueContinuous(Trigger trigger, final Command command) {
    CommandScheduler.getInstance()
        .getDefaultButtonLoop()
        .bind(
            new Runnable() {
              private boolean m_pressedLast = trigger.getAsBoolean();

              @Override
              public void run() {
                boolean pressed = trigger.getAsBoolean();

                if (pressed) {
                  CommandScheduler.getInstance().schedule(command);
                } else if (m_pressedLast) {
                  CommandScheduler.getInstance().cancel(command);
                }

                m_pressedLast = pressed;
              }
            });
  }

  /**
   * Tracker that activates only when a button is pressed twice quickly.
   *
   * @param baseTrigger The trigger to wrap.
   * @return The new trigger that activates on double press.
   */
  public static Trigger doublePress(Trigger baseTrigger) {
    var tracker = new DoublePressTracker(baseTrigger);
    return new Trigger(tracker::get);
  }

  /** Tracker that activates only when a button is pressed twice quickly. */
  private static class DoublePressTracker {
    // How long after the first press does the second need to occur?
    public static final double maxLengthSecs = 0.4;

    private final Trigger trigger;
    private final Timer resetTimer = new Timer();
    private DoublePressState state = DoublePressState.IDLE;

    private DoublePressTracker(Trigger baseTrigger) {
      trigger = baseTrigger;
    }

    private boolean get() {
      boolean pressed = trigger.getAsBoolean();
      switch (state) {
        case IDLE:
          if (pressed) {
            state = DoublePressState.FIRST_PRESS;
            resetTimer.reset();
            resetTimer.start();
          }
          break;
        case FIRST_PRESS:
          if (!pressed) {
            if (resetTimer.hasElapsed(maxLengthSecs)) {
              reset();
            } else {
              state = DoublePressState.FIRST_RELEASE;
            }
          }
          break;
        case FIRST_RELEASE:
          if (pressed) {
            state = DoublePressState.SECOND_PRESS;
          } else if (resetTimer.hasElapsed(maxLengthSecs)) {
            reset();
          }
          break;
        case SECOND_PRESS:
          if (!pressed) {
            reset();
          }
      }
      return state == DoublePressState.SECOND_PRESS;
    }

    private void reset() {
      state = DoublePressState.IDLE;
      resetTimer.stop();
    }

    private enum DoublePressState {
      IDLE,
      FIRST_PRESS,
      FIRST_RELEASE,
      SECOND_PRESS
    }
  }
}
