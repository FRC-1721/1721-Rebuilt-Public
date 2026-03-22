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

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;

public class FlywheelIOKraken implements FlywheelIO {

  private final TalonFX master;
  private final TalonFX follower;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final Follower followerRequest;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Track last applied PID so we don't spam CAN every loop
  private double lastKP = Double.NaN;
  private double lastKD = Double.NaN;

  public FlywheelIOKraken(int masterId, int followerId, String canBus) {

    master = new TalonFX(masterId, canBus);
    follower = new TalonFX(followerId, canBus);

    // IMPORTANT:
    // Use Opposed unless the motors are physically aligned
    followerRequest = new Follower(masterId, MotorAlignmentValue.Opposed);

    // Safe ramp to reduce belt shock during spinup
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

    master.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    // Set follower once
    follower.setControl(followerRequest);

    master.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    inputs.connected = master.isAlive() && follower.isAlive();

    // Phoenix 6 reports rotations and rotations/sec
    inputs.positionRads = Units.rotationsToRadians(master.getPosition().getValueAsDouble());

    inputs.velocityRadsPerSec = Units.rotationsToRadians(master.getVelocity().getValueAsDouble());

    inputs.supplyCurrentAmps =
        master.getSupplyCurrent().getValueAsDouble()
            + follower.getSupplyCurrent().getValueAsDouble();

    inputs.torqueCurrentAmps =
        master.getTorqueCurrent().getValueAsDouble()
            + follower.getTorqueCurrent().getValueAsDouble();

    inputs.tempCelsius =
        Math.max(
            master.getDeviceTemp().getValueAsDouble(), follower.getDeviceTemp().getValueAsDouble());

    inputs.appliedVoltage = master.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {

    if (outputs.coast) {
      master.setControl(neutralRequest);
      return;
    }

    if (outputs.voltageMode) {
      master.setControl(voltageRequest.withOutput(outputs.voltage));
      return;
    }

    double rotationsPerSec = Units.radiansToRotations(outputs.velocityRadsPerSec);

    if (outputs.kP != lastKP || outputs.kD != lastKD) {
      config.Slot0.kP = outputs.kP;
      config.Slot0.kD = outputs.kD;
      master.getConfigurator().apply(config);

      lastKP = outputs.kP;
      lastKD = outputs.kD;
    }

    master.setControl(
        velocityRequest.withVelocity(rotationsPerSec).withFeedForward(outputs.feedForward));
  }
}
