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

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class TurretIOKraken implements TurretIO {

  private final TalonFX motor;

  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Unknown reduction, start conservative
  private static final double GEAR_RATIO = 18.0;

  // HARD SPEED LIMIT (prevents turret from ever moving fast)
  private static final double MAX_MOTOR_RPS = 0.25;

  private double lastKP = Double.NaN;
  private double lastKD = Double.NaN;

  public TurretIOKraken(int motorID, String canBus) {

    motor = new TalonFX(motorID, canBus);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // VERY slow ramp for safety
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.4;

    // Current limits so the turret can't break itself
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
    limits.StatorCurrentLimit = 40;
    limits.StatorCurrentLimitEnable = true;

    limits.SupplyCurrentLimit = 30;
    limits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits = limits;

    motor.getConfigurator().apply(config);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    inputs.motorConnected = motor.isAlive();

    double motorRot = motor.getPosition().getValueAsDouble();
    double motorVel = motor.getVelocity().getValueAsDouble();

    double turretRot = motorRot / GEAR_RATIO;
    double turretVel = motorVel / GEAR_RATIO;

    inputs.positionRads = Units.rotationsToRadians(turretRot);
    inputs.velocityRadsPerSec = Units.rotationsToRadians(turretVel);

    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {

    switch (outputs.mode) {
      case BRAKE -> motor.setControl(neutralRequest);

      case COAST -> motor.setVoltage(0);

      case CLOSED_LOOP -> {

        // Update PID only when changed (avoids CAN spam)
        if (outputs.kP != lastKP || outputs.kD != lastKD) {

          config.Slot0.kP = outputs.kP;
          config.Slot0.kD = outputs.kD;

          motor.getConfigurator().apply(config);

          lastKP = outputs.kP;
          lastKD = outputs.kD;
        }

        double motorRotations = Units.radiansToRotations(outputs.position) * GEAR_RATIO;

        double motorVelocity = Units.radiansToRotations(outputs.velocity) * GEAR_RATIO;

        // HARD clamp so turret moves extremely slowly
        motorVelocity = MathUtil.clamp(motorVelocity, -MAX_MOTOR_RPS, MAX_MOTOR_RPS);

        motor.setControl(positionRequest.withPosition(motorRotations).withVelocity(motorVelocity));
      }

      case OPEN_LOOP -> motor.setVoltage(outputs.voltage);
    }
  }
}
