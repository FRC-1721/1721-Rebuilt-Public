package org.tidalforce.frc2026.subsystems.navigation.planner;

import org.tidalforce.frc2026.subsystems.navigation.grid.*;
import edu.wpi.first.math.geometry.*;
import java.util.*;

public class PathSimplifier {
    public static List<Pose2d> simplify(List<Node> path, OccupancyGrid grid) {
        List<Pose2d> out = new ArrayList<>();
        if (path.isEmpty()) return out;

        Node last = path.get(0);
        out.add(toPose(last, grid));

        for (int i = 2; i < path.size(); i++) {
            Node n = path.get(i);
            if (!lineOfSight(last, n, grid)) {
                Node prev = path.get(i - 1);
                out.add(toPose(prev, grid));
                last = prev;
            }
        }

        out.add(toPose(path.get(path.size() - 1), grid));
        return out;
    }

    private static Pose2d toPose(Node n, OccupancyGrid grid) {
        return new Pose2d(n.x * grid.resolution, n.y * grid.resolution, new Rotation2d());
    }

    private static boolean lineOfSight(Node a, Node b, OccupancyGrid grid) {
        int dx = b.x - a.x;
        int dy = b.y - a.y;
        int steps = Math.max(Math.abs(dx), Math.abs(dy));

        for (int i = 0; i <= steps; i++) {
            int x = a.x + dx * i / steps;
            int y = a.y + dy * i / steps;
            if (grid.isBlocked(x, y)) return false;
        }
        return true;
    }
}
