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

package org.tidalforce.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {

  private boolean deployed = false;

  private final IntakePivotIO pivotIO;
  private final IntakePivotIO.IntakePivotIOInputs inputs = new IntakePivotIO.IntakePivotIOInputs();
  private final IntakePivotIO.IntakePivotIOOutputs outputs =
      new IntakePivotIO.IntakePivotIOOutputs();

  /** Constructor takes any implementation of IntakePivotIO (hardware or sim) */
  public IntakePivotSubsystem(IntakePivotIO pivotIO) {
    this.pivotIO = pivotIO;
  }

  /** Move the pivot to the deployed (out) position */
  public void deploy() {
    pivotIO.setOut();
    deployed = true;
  }

  /** Move the pivot to the stowed (in) position */
  public void stow() {
    pivotIO.setIn();
    deployed = false;
  }

  /** Stop the pivot motor */
  public void stopPivot() {
    pivotIO.stop();
  }

  /** Periodic update of inputs and outputs for logging and control */
  @Override
  public void periodic() {
    // Update hardware inputs
    pivotIO.updateInputs(inputs);

    // Apply outputs if needed (in this case, mostly handled by setIn/setOut)
    pivotIO.applyOutputs(outputs);
  }

  /** Convenience getters for testing or simulation */
  public double getPositionRads() {
    return inputs.positionRads;
  }

  public double getVelocityRadsPerSec() {
    return inputs.velocityRadsPerSec;
  }

  public boolean isDeployed() {
    return deployed;
  }
}
