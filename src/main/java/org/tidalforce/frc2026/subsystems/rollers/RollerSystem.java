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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.Robot;
import org.tidalforce.frc2026.subsystems.rollers.RollerSystemIO.RollerSystemIOOutputs;
import org.tidalforce.frc2026.util.FullSubsystem;
import org.tidalforce.frc2026.util.LoggedTracer;

public class RollerSystem extends FullSubsystem {
  private final String name;
  private final String inputsName;
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
  private final RollerSystemIOOutputs outputs = new RollerSystemIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;

  @Setter private double volts = 0.0;
  @Setter private boolean brakeModeEnabled = true;

  public RollerSystem(String name, String inputsName, RollerSystemIO io) {
    this.name = name;
    this.inputsName = inputsName;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(inputsName, inputs);
    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));

    // Run roller
    outputs.appliedVoltage = volts;

    // Update brake mode
    outputs.brakeModeEnabled = brakeModeEnabled;

    // Record cycle time
    LoggedTracer.record(name);

    Logger.recordOutput(inputsName + "/BrakeModeEnabled", brakeModeEnabled);
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public void stop() {
    volts = 0.0;
  }
}
