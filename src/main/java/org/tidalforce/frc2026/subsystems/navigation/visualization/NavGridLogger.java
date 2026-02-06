package org.tidalforce.frc2026.subsystems.navigation.visualization;

import org.tidalforce.frc2026.subsystems.navigation.grid.*;
import org.littletonrobotics.junction.Logger;

public class NavGridLogger {
    public static void log(OccupancyGrid grid) {
        double[][] data = new double[grid.width][grid.height];
        for (int x = 0; x < grid.width; x++)
            for (int y = 0; y < grid.height; y++)
                data[x][y] = grid.get(x, y);

        Logger.recordOutput("Nav/Grid", data);
    }
}
