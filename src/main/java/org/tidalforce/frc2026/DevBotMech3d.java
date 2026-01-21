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

package org.tidalforce.frc2026;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.subsystems.shooter.ShooterConstants;
import org.tidalforce.frc2026.util.geometry.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class DevBotMech3d {
  private static DevBotMech3d measured;

  public static DevBotMech3d getMeasured() {
    if (measured == null) {
      measured = new DevBotMech3d();
    }
    return measured;
  }

  @Getter @Setter private Rotation2d turretAngle = Rotation2d.kZero; // Robot-relative
  @Getter @Setter private Rotation2d hoodAngle = Rotation2d.kZero; // Relative to the ground

  /** Log the component poses and camera pose. */
  public void log(String key) {
    var turretPose =
        ShooterConstants.robotToTurret
            .toPose3d()
            .transformBy(
                new Transform3d(
                    Translation3d.kZero, new Rotation3d(0.0, 0.0, turretAngle.getRadians())));
    var hoodPose =
        turretPose.transformBy(
            new Transform3d(
                0.105, 0.0, 0.092, new Rotation3d(0.0, -hoodAngle.getRadians(), Math.PI)));
    Logger.recordOutput(key + "/Components", turretPose, hoodPose);

    var cameraPose =
        new Pose3d(RobotState.getInstance().getEstimatedPose())
            .transformBy(turretPose.toTransform3d())
            .transformBy(ShooterConstants.turretToCamera);
    Logger.recordOutput(key + "/CameraPose", cameraPose);
  }
}
