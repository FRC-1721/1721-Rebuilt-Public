package org.tidalforce.frc2026.subsystems.navigation.grid;

import java.util.List;

public class NavGridData {
    public double resolution;
    public int width;
    public int height;
    public List<Obstacle> staticObstacles;

    public static class Obstacle {
        public int x, y, w, h;
    }
}
