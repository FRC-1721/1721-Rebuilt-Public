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

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

public class FlywheelIOKraken implements FlywheelIO {
  private final TalonFX master;
  private final TalonFX follower;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private double appliedVolts = 0.0;

  public FlywheelIOKraken(int masterId, int followerId, String canBus) {
    master = new TalonFX(masterId, canBus);
    follower = new TalonFX(followerId, canBus);

    // Make follower mirror master (same direction)
    follower.setControl(
        new com.ctre.phoenix6.controls.Follower(followerId, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.connected = master.isAlive() && follower.isAlive();

    inputs.positionRads = master.getPosition().getValueAsDouble() * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = master.getVelocity().getValueAsDouble() * 2.0 * Math.PI;

    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps =
        master.getSupplyCurrent().getValueAsDouble()
            + follower.getSupplyCurrent().getValueAsDouble();

    inputs.torqueCurrentAmps =
        master.getTorqueCurrent().getValueAsDouble()
            + follower.getTorqueCurrent().getValueAsDouble();

    inputs.tempCelsius =
        Math.max(
            master.getDeviceTemp().getValueAsDouble(), follower.getDeviceTemp().getValueAsDouble());
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    if (DriverStation.isDisabled() || outputs.coast) {
      appliedVolts = 0.0;
    } else {
      // Your sim uses PID internally — real hardware just takes voltage
      appliedVolts = MathUtil.clamp(outputs.appliedVoltage, -12.0, 12.0);
    }

    master.setControl(voltageRequest.withOutput(appliedVolts));
  }
}
