package org.tidalforce.frc2026.subsystems.navigation.path;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;
import java.util.stream.Collectors;

public class PathPlannerBridge {

    public static PathPlannerPath fromPoses(List<Pose2d> poses) {
        var waypoints = poses.stream()
                .map(p -> new PathPlannerPath.Waypoint(
                        p.getTranslation(),
                        p.getRotation()
                ))
                .collect(Collectors.toList());

        PathConstraints constraints = new PathConstraints(
                4.5,
                3.5,
                10.0,
                10.0
        );

        GoalEndState goalEndState = new GoalEndState(
                0.0,
                poses.get(poses.size() - 1).getRotation()
        );

        return PathPlannerPath.fromWaypoints(
                waypoints,
                constraints,
                goalEndState
        );
    }
}
