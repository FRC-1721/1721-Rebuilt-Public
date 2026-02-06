package org.tidalforce.frc2026.subsystems.navigation.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.Logger;
import java.util.List;

public class PathLogger {
    public static void log(List<Pose2d> path) {
        Logger.recordOutput("Nav/Path", path.toArray(new Pose2d[0]));
    }
}
