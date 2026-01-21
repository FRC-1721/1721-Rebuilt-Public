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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.tidalforce.frc2026.Constants;

public class HoodIOSim implements HoodIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX44(1);
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          motorModel, 1.0, .004, .33, 0.0, Units.degreesToRadians(45), false, 0, null);

  private double currentOutput = 0.0;
  private double appliedVolts = 0.0;
  private boolean currentControl = false;

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (currentControl) {
      appliedVolts = motorModel.getVoltage(currentOutput, sim.getVelocityRadPerSec());
    } else {
      appliedVolts = 0.0;
    }

    // Update sim state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.motorConnected = true;
    inputs.positionRads = sim.getAngleRads();
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> {
        currentControl = false;
      }
      case COAST -> {
        currentOutput = 0.0;
        currentControl = true;
      }
      case CLOSED_LOOP -> {
        currentOutput =
            outputs.feedforward
                + (sim.getAngleRads() - outputs.positionRad) * outputs.kP
                + (sim.getVelocityRadPerSec() - outputs.velocityRadsPerSec) * outputs.kD;
        currentControl = true;
      }
    }
  }
}
