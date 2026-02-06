package org.tidalforce.frc2026.subsystems.navigation.grid;

public class Costmap {
    private final OccupancyGrid grid;

    public Costmap(OccupancyGrid grid) {
        this.grid = grid;
    }

    public double cost(int x, int y) {
        if (grid.isBlocked(x, y)) return Double.POSITIVE_INFINITY;

        double minDist = Double.POSITIVE_INFINITY;
        for (int dx = -5; dx <= 5; dx++) {
            for (int dy = -5; dy <= 5; dy++) {
                if (grid.isBlocked(x + dx, y + dy)) {
                    double d = Math.hypot(dx, dy);
                    minDist = Math.min(minDist, d);
                }
            }
        }
        return 1.0 + Math.exp(-minDist);
    }
}
