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

import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.tidalforce.frc2026.subsystems.rollers.RollerSystem;
import org.tidalforce.frc2026.subsystems.rollers.RollerSystemIO;
import org.tidalforce.frc2026.util.FullSubsystem;
import org.tidalforce.frc2026.util.LoggedTunableNumber;

public class Intake extends FullSubsystem {
  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Intake/Roller/IntakeVolts", 8.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
      new LoggedTunableNumber("Intake/Roller/OuttakeVolts", -8.0);

  private final RollerSystem roller;

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOP;

  public Intake(RollerSystemIO rollerIO) {
    this.roller = new RollerSystem("Intake roller", "Intake/Roller", rollerIO);
  }

  public void periodic() {

    roller.periodic();

    double rollerVolts = 0.0;
    switch (goal) {
      case INTAKE -> {
        rollerVolts = rollerIntakeVolts.get();
      }

      case OUTTAKE -> {
        rollerVolts = rollerOuttakeVolts.get();
      }
      case STOP -> {
        rollerVolts = 0.0;
      }
    }
    roller.setVolts(rollerVolts);
  }

  @Override
  public void periodicAfterScheduler() {
    roller.periodicAfterScheduler();
  }

  public enum Goal {
    INTAKE,
    OUTTAKE,
    STOP
  }
}
