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

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  class TurretIOInputs {
    public boolean connected = false;
    public Rotation2d position = Rotation2d.kZero;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    // Fault flags — visible in AdvantageScope and Elastic alerts
    public boolean faultHardware = false;
    public boolean faultBootDuringEnable = false;
    public boolean faultUnderVoltage = false;
    public boolean faultStatorCurrLimit = false;
    public boolean stickyFaultAny = false;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  /** Run open-loop voltage. Used for zeroing, open-loop testing, and SysId. */
  default void setVoltage(double volts) {}

  /** Run closed-loop MotionMagic to a position in mechanism rotations. */
  default void setPosition(double mechanismRotations) {}

  /** Set the motor's sensor to the known position after zeroing. */
  default void zeroSensor(double knownPositionRotations) {}

  /** Switch between brake and coast mode. */
  default void setBrakeMode(boolean brake) {}

  /**
   * Push updated PID gains to the motor. Called by Turret.java whenever a LoggedTunableNumber
   * changes.
   */
  default void reconfigureGains(double kP, double kD, double kS, double kV, double kA) {}

  /**
   * Push updated MotionMagic profile to the motor. Called by Turret.java whenever a
   * LoggedTunableNumber changes.
   */
  default void reconfigureProfile(
      double cruiseVelocityRPS, double accelerationRPS2, double jerkRPS3) {}
}
