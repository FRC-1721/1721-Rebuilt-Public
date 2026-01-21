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

package org.tidalforce.frc2026.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.tidalforce.frc2026.Constants;

public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, .025, 1), motorModel);

  private PIDController controller = new PIDController(0, 0, 0, Constants.loopPeriodSecs);
  private double currentOutput = 0.0;
  private double currentOutputAsVolt = 0.0;
  private double appliedVolts = 0.0;

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    currentOutputAsVolt = motorModel.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec());
    appliedVolts = currentOutputAsVolt;

    // Update sim state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    if (outputs.coast) {
      currentOutput = 0.0;
    } else {
      currentOutput = controller.calculate(outputs.velocityRadsPerSec);
    }
  }
}
