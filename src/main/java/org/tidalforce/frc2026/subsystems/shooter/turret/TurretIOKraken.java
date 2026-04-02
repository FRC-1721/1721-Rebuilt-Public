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

import static org.tidalforce.frc2026.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOKraken implements TurretIO {

  private final TalonFX motor;

  // Cached configurator for live gain updates without full reconfiguration
  private final com.ctre.phoenix6.configs.TalonFXConfigurator configurator;

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  // Fault signals
  private final StatusSignal<Boolean> faultHardware;
  private final StatusSignal<Boolean> faultBootDuringEnable;
  private final StatusSignal<Boolean> faultUnderVoltage;
  private final StatusSignal<Boolean> faultStatorCurrLimit;
  private final StatusSignal<Integer> stickyFaultField;

  // Control requests — FOC disabled, no Pro license required
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(false);

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  private NeutralModeValue lastNeutralMode = null;

  public TurretIOKraken(int id, String canBus) {
    motor = new TalonFX(id, canBus);
    configurator = motor.getConfigurator();

    motor.optimizeBusUtilization();

    var cfg = new TalonFXConfiguration();

    // ── Current limits ────────────────────────────────────────────────────
    cfg.CurrentLimits.StatorCurrentLimit = 40.0;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    // ── Feedback — integrated encoder only ───────────────────────────────
    // Do NOT set FeedbackSensorSource or FeedbackRemoteSensorID.
    // Phoenix defaults to integrated encoder when these are absent.
    cfg.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;

    // ── Motor direction ───────────────────────────────────────────────────
    // Positive output should move turret CCW (left) when viewed from above.
    // If open-loop +voltage moves CW, flip to Clockwise_Positive.
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // ── Voltage clamp — conservative during initial testing ───────────────
    // RAISE PeakForwardVoltage toward 12.0 when ready for full speed.
    // Keep at 6.0 until zeroing and position control are confirmed working.
    cfg.Voltage.PeakForwardVoltage = 6.0;
    cfg.Voltage.PeakReverseVoltage = -6.0;

    // ── Soft limits ───────────────────────────────────────────────────────
    // Forward = CCW = toward MAX_ANGLE (+120°)
    // Reverse = CW  = toward MIN_ANGLE (-120°)
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_ANGLE.getRotations();
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_ANGLE.getRotations();

    // ── NO PID GAINS SET HERE ─────────────────────────────────────────────
    // Gains are pushed by Turret.java on the first periodic() call via
    // reconfigureGains() and reconfigureProfile(). This ensures
    // LoggedTunableNumber values in NetworkTables are always what the
    // motor actually runs with — never stale constructor values.

    tryUntilOk(5, () -> configurator.apply(cfg, 0.25));

    // ── Status signals ────────────────────────────────────────────────────
    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();
    temp = motor.getDeviceTemp();

    // Fault signals at lower frequency — only needed for diagnostics
    faultHardware = motor.getFault_Hardware();
    faultBootDuringEnable = motor.getFault_BootDuringEnable();
    faultUnderVoltage = motor.getFault_Undervoltage();
    faultStatorCurrLimit = motor.getFault_StatorCurrLimit();
    stickyFaultField = motor.getStickyFaultField();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp);
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        faultHardware,
        faultBootDuringEnable,
        faultUnderVoltage,
        faultStatorCurrLimit,
        stickyFaultField);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp);
    BaseStatusSignal.refreshAll(
        faultHardware,
        faultBootDuringEnable,
        faultUnderVoltage,
        faultStatorCurrLimit,
        stickyFaultField);

    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.position = Rotation2d.fromRotations(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();

    inputs.faultHardware = faultHardware.getValue();
    inputs.faultBootDuringEnable = faultBootDuringEnable.getValue();
    inputs.faultUnderVoltage = faultUnderVoltage.getValue();
    inputs.faultStatorCurrLimit = faultStatorCurrLimit.getValue();
    inputs.stickyFaultAny = stickyFaultField.getValueAsDouble() != 0;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double mechanismRotations) {
    motor.setControl(positionRequest.withPosition(mechanismRotations));
  }

  @Override
  public void zeroSensor(double knownPositionRotations) {
    // Sets the sensor to the known position after hitting the hard stop.
    // knownPositionRotations = -ZERO_OFFSET_FROM_HARDSTOP.getRotations()
    // so that 0 rotations = forward-facing.
    motor.setPosition(knownPositionRotations);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    var mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    if (mode != lastNeutralMode) {
      motor.setNeutralMode(mode);
      lastNeutralMode = mode;
    }
  }

  @Override
  public void reconfigureGains(double kP, double kD, double kS, double kV, double kA) {
    var slot = new Slot0Configs();
    slot.kP = kP;
    slot.kI = 0.0;
    slot.kD = kD;
    slot.kS = kS;
    slot.kG = 0.0; // turret has no gravity loading — always zero
    slot.kV = kV;
    slot.kA = kA;
    configurator.apply(slot);
  }

  @Override
  public void reconfigureProfile(
      double cruiseVelocityRPS, double accelerationRPS2, double jerkRPS3) {
    var mm = new MotionMagicConfigs();
    mm.MotionMagicCruiseVelocity = cruiseVelocityRPS;
    mm.MotionMagicAcceleration = accelerationRPS2;
    mm.MotionMagicJerk = jerkRPS3;
    configurator.apply(mm);
  }
}
