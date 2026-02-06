package org.tidalforce.frc2026.subsystems.navigation.path;

import org.tidalforce.frc2026.subsystems.navigation.planner.*;
import org.tidalforce.frc2026.subsystems.navigation.grid.*;
import edu.wpi.first.math.geometry.*;
import java.util.*;

public class ReplanManager {
    private final ADStar planner;
    private final OccupancyGrid grid;
    private List<Pose2d> path = List.of();

    public ReplanManager(ADStar planner, OccupancyGrid grid) {
        this.planner = planner;
        this.grid = grid;
    }

    public boolean shouldReplan(Pose2d robot) {
        if (path.isEmpty()) return true;

        for (Pose2d p : path) {
            int x = (int)(p.getX() / grid.resolution);
            int y = (int)(p.getY() / grid.resolution);
            if (grid.isBlocked(x, y)) return true;
        }

        return robot.getTranslation().getDistance(path.get(0).getTranslation()) > 0.3;
    }

    public void replan(Pose2d start, Pose2d goal) {
        int sx = (int)(start.getX() / grid.resolution);
        int sy = (int)(start.getY() / grid.resolution);
        int gx = (int)(goal.getX() / grid.resolution);
        int gy = (int)(goal.getY() / grid.resolution);

        planner.initialize(sx, sy, gx, gy);
        planner.compute();

        List<Node> raw = planner.extractPath();
        path = PathSimplifier.simplify(raw, grid);

        PathPlannerBridge.send(path, goal);
    }

    public List<Pose2d> getPath() {
        return path;
    }
}
