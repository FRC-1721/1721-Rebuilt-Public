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

package org.tidalforce.frc2026.subsystems.rollers;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

public class RollerSystemIOKraken implements RollerSystemIO {
  private final TalonFX motor;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private double appliedVoltage = 0.0;

  public RollerSystemIOKraken(int motorId, String canBus) {
    motor = new TalonFX(motorId, canBus);
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    inputs.connected = motor.isAlive();

    inputs.positionRads = motor.getPosition().getValueAsDouble() * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = motor.getVelocity().getValueAsDouble() * 2.0 * Math.PI;

    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void applyOutputs(RollerSystemIOOutputs outputs) {
    motor.setNeutralMode(
        outputs.brakeModeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);

    if (DriverStation.isDisabled()) {
      appliedVoltage = 0.0;
    } else {
      appliedVoltage = MathUtil.clamp(outputs.appliedVoltage, -12.0, 12.0);
    }

    motor.setControl(voltageRequest.withOutput(appliedVoltage));
  }
}
