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

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

public class HoodIOKraken implements HoodIO {
  private final TalonFX motor;

  private static final double GEAR_RATIO = 25.0; // TODO: tune this

  private double getPositionRad() {
    return (motor.getPosition().getValueAsDouble() / GEAR_RATIO) * 2.0 * Math.PI;
  }

  private double getVelocityRadPerSec() {
    return (motor.getVelocity().getValueAsDouble() / GEAR_RATIO) * 2.0 * Math.PI;
  }

  private double appliedVolts = 0.0;

  public HoodIOKraken(int motorId, String canBus) {
    motor = new TalonFX(motorId, canBus);

    motor.setNeutralMode(NeutralModeValue.Brake);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.connected = motor.isAlive();

    inputs.positionRads = getPositionRad();
    inputs.velocityRadsPerSec = getVelocityRadPerSec();

    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {

    if (DriverStation.isDisabled()) {
      appliedVolts = 0.0;
      motor.setControl(new VoltageOut(0.0));
      return;
    }

    final double DIRECTION = -1.0;

    switch (outputs.mode) {
      case OPEN_LOOP:
        appliedVolts = DIRECTION * outputs.appliedVoltage;
        break;

      case CLOSED_LOOP:
        {
          double position = getPositionRad();
          double velocity = getVelocityRadPerSec();

          double error = outputs.positionRad - position;

          double pid = outputs.kP * error - outputs.kD * velocity;

          appliedVolts = pid + outputs.feedforward;

          appliedVolts *= DIRECTION;
          break;
        }

      case CHARACTERIZATION:
        {
          double step = 0.02;
          appliedVolts += step;
          appliedVolts = MathUtil.clamp(appliedVolts, -2.0, 2.0);

          if (Math.abs(getVelocityRadPerSec()) > 0.01) {
            System.out.println("KS FOUND: " + appliedVolts);
          }
          break;
        }

      case BRAKE:
      case COAST:
      default:
        appliedVolts = 0.0;
        break;
    }

    // Safety clamp
    appliedVolts = MathUtil.clamp(appliedVolts, -2.0, 2.0);

    // Smooth ramp
    double rampRate = 0.05;
    double delta = appliedVolts - this.appliedVolts;
    delta = MathUtil.clamp(delta, -rampRate, rampRate);
    this.appliedVolts += delta;

    motor.setControl(new VoltageOut(this.appliedVolts));
  }
}
