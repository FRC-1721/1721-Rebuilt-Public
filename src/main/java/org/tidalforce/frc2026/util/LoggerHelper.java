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

package org.tidalforce.frc2026.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * A helper class for logging various types of data related to FRC subsystems by Team 604 Quixilver
 */
public class LoggerHelper {
  public static void recordCurrentCommand(String name, SubsystemBase subsystem) {
    final var currentCommand = subsystem.getCurrentCommand();
    Logger.recordOutput(
        name + "/Current Command", currentCommand == null ? "None" : currentCommand.getName());
  }

  /**
   * Records a list of Pose2d objects to the logger with the specified key.
   *
   * @param key the key under which the list of Pose2d objects will be recorded
   * @param list the list of Pose2d objects to be recorded
   */
  public static void recordPose2dList(String key, List<Pose2d> list) {
    Pose2d[] array = new Pose2d[list.size()];
    list.toArray(array);
    Logger.recordOutput(key, array);
  }
}
