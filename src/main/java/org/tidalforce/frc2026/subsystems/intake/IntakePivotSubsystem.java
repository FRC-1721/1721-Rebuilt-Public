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

  private final IntakePivotIO pivotIO;
  private final IntakePivotIO.IntakePivotIOInputs inputs = new IntakePivotIO.IntakePivotIOInputs();
  private final IntakePivotIO.IntakePivotIOOutputs outputs =
      new IntakePivotIO.IntakePivotIOOutputs();

  private final double inPositionRads;
  private final double outPositionRads;

  private boolean deployed = false;

  public IntakePivotSubsystem(
      IntakePivotIO pivotIO, double inPositionRads, double outPositionRads) {

    this.pivotIO = pivotIO;
    this.inPositionRads = inPositionRads;
    this.outPositionRads = outPositionRads;
  }

  public void deploy() {
    outputs.targetPositionRads = outPositionRads;
    deployed = true;
  }

  public void stow() {
    outputs.targetPositionRads = inPositionRads;
    deployed = false;
  }

  public void stopPivot() {
    outputs.appliedVoltage = 0.0;
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(inputs);
    pivotIO.applyOutputs(outputs);
  }

  public double getPositionRads() {
    return inputs.positionRads;
  }

  public boolean isDeployed() {
    return deployed;
  }
}
