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

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

public class IntakePivotIOKraken implements IntakePivotIO {

  private final TalonFX motor;

  // Motor rotations : mechanism rotation
  private static final double GEAR_RATIO = 25.0;

  // Gravity compensation voltage (starting estimate)
  private static final double kG = 0.35;

  // Safe mechanical limits (adjust after testing)
  private static final double MIN_RAD = -70.0;
  private static final double MAX_RAD = 0.0;

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  public IntakePivotIOKraken(int motorId, String canBus) {

    motor = new TalonFX(motorId, canBus);

    var config = new TalonFXConfiguration();

    // PID starting values
    config.Slot0.kP = 8.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.2;

    // Tell TalonFX the mechanism ratio
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Slow and safe for first tests
    config.Voltage.PeakForwardVoltage = 3.0;
    config.Voltage.PeakReverseVoltage = -3.0;

    // Hold pivot position
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {

    inputs.connected = motor.isAlive();

    double mechanismRotations = motor.getPosition().getValueAsDouble();
    double mechanismVelocity = motor.getVelocity().getValueAsDouble();

    inputs.positionRads = mechanismRotations * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = mechanismVelocity * 2.0 * Math.PI;

    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void applyOutputs(IntakePivotIOOutputs outputs) {

    motor.setNeutralMode(
        outputs.brakeModeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);

    if (DriverStation.isDisabled()) {
      motor.stopMotor();
      return;
    }

    // Clamp pivot targets to safe range
    double targetRads = MathUtil.clamp(outputs.targetPositionRads, MIN_RAD, MAX_RAD);

    // Convert radians -> mechanism rotations
    double targetRotations = targetRads / (2.0 * Math.PI);

    // Gravity compensation
    double gravityFF = kG * Math.cos(targetRads);

    motor.setControl(positionRequest.withPosition(targetRotations).withFeedForward(gravityFF));
  }
}
