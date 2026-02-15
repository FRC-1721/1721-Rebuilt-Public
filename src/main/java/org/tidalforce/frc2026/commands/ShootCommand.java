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

package org.tidalforce.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.tidalforce.frc2026.subsystems.kicker.Kicker;
import org.tidalforce.frc2026.subsystems.shooter.ShotCalculator;
import org.tidalforce.frc2026.subsystems.shooter.flywheel.Flywheel;

public class ShootCommand extends SequentialCommandGroup {

  public ShootCommand(Flywheel flywheel, Kicker kicker, ShotCalculator shotCalculator) {

    // 1️⃣ Set the flywheel goal from the shot calculator
    addCommands(
        Commands.runOnce(
            () -> {
              var shot = shotCalculator.getParameters();
              flywheel.setGoal(shot.flywheelSpeed());
            }));

    // 2️⃣ Run the flywheel continuously until we feed balls
    addCommands(
        flywheel
            .runGoalCommand()
            .deadlineWith(
                // 3️⃣ Wait until flywheel is at setpoint
                new WaitUntilCommand(() -> flywheel.atGoal())
                    // 4️⃣ Then run the kicker for a short period
                    .andThen(
                        Commands.runOnce(() -> kicker.setGoal(Kicker.Goal.SHOOT))
                            .withTimeout(0.4)
                            .andThen(Commands.runOnce(() -> kicker.setGoal(Kicker.Goal.STOP))))));
  }
}
