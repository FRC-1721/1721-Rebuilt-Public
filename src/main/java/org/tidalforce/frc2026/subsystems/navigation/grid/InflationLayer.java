package org.tidalforce.frc2026.subsystems.navigation.grid;

public class InflationLayer {
    private final OccupancyGrid grid;
    private final int radiusCells;

    public InflationLayer(OccupancyGrid grid, double radiusMeters) {
        this.grid = grid;
        this.radiusCells = (int)(radiusMeters / grid.resolution);
    }

    public void inflate(int cx, int cy) {
        for (int dx = -radiusCells; dx <= radiusCells; dx++) {
            for (int dy = -radiusCells; dy <= radiusCells; dy++) {
                if (Math.hypot(dx, dy) <= radiusCells) {
                    grid.markInflated(cx + dx, cy + dy);
                }
            }
        }
    }
}
