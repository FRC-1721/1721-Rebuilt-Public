package org.tidalforce.frc2026.subsystems.navigation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import org.tidalforce.frc2026.subsystems.navigation.grid.*;
import org.tidalforce.frc2026.subsystems.navigation.planner.*;
import org.tidalforce.frc2026.subsystems.navigation.dynamic.*;
import org.tidalforce.frc2026.subsystems.navigation.path.*;
import org.tidalforce.frc2026.subsystems.navigation.visualization.*;

public class NavigationSubsystem extends SubsystemBase {
    private final OccupancyGrid grid = GridLoader.load();
    private final InflationLayer inflation = new InflationLayer(grid, 0.45);
    private final ADStar planner = new ADStar(grid);
    private final DynamicObstacleDetector detector = new DynamicObstacleDetector(grid, inflation);
    private final ReplanManager replan = new ReplanManager(planner, grid);

    private Pose2d goal = new Pose2d();

    public void setGoal(Pose2d g) {
        goal = g;
    }

    public void update(Pose2d robotPose) {
        detector.update(robotPose);

        if (replan.shouldReplan(robotPose)) {
            replan.replan(robotPose, goal);
        }

        NavGridLogger.log(grid);
        PathLogger.log(replan.getPath());
    }
}
