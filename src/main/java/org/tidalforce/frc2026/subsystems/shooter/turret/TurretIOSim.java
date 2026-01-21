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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.tidalforce.frc2026.Constants;

public class TurretIOSim implements TurretIO {
  private final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, 100.0), gearbox);

  private double currentOutput = 0.0;
  private double appliedVoltage = 0.0;
  private boolean currentControl = false;

  public TurretIOSim() {}

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (currentControl) {
      appliedVoltage = gearbox.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec());
    }
    sim.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.motorConnected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> {
        currentControl = false;
        appliedVoltage = 0.0;
      }
      case COAST -> {
        currentControl = true;
        currentOutput = 0.0;
      }
      case CLOSED_LOOP -> {
        currentControl = true;
        currentOutput =
            (outputs.position - sim.getAngularPositionRad()) * outputs.kP
                + (outputs.velocity - sim.getAngularVelocityRadPerSec()) * outputs.kD;
      }
    }
  }
}
