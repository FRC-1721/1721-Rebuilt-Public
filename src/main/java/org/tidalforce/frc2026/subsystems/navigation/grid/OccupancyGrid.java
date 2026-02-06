package org.tidalforce.frc2026.subsystems.navigation.grid;

public class OccupancyGrid {
    public final int width, height;
    public final double resolution;
    private final int[][] grid;

    public OccupancyGrid(int w, int h, double res) {
        width = w;
        height = h;
        resolution = res;
        grid = new int[w][h];
    }

    public void clearDynamic() {
        for (int x = 0; x < width; x++)
            for (int y = 0; y < height; y++)
                if (grid[x][y] == 2) grid[x][y] = 0;
    }

    public void markStatic(int x, int y) {
        if (inBounds(x, y)) grid[x][y] = 1;
    }

    public void markInflated(int x, int y) {
        if (inBounds(x, y) && grid[x][y] == 0) grid[x][y] = 2;
    }

    public boolean isBlocked(int x, int y) {
        return !inBounds(x, y) || grid[x][y] != 0;
    }

    public int get(int x, int y) {
        return grid[x][y];
    }

    private boolean inBounds(int x, int y) {
        return x >= 0 && y >= 0 && x < width && y < height;
    }
}
