package org.tidalforce.frc2026.subsystems.navigation.grid;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.InputStream;

public class GridLoader {
    public static OccupancyGrid load() {
        try {
            ObjectMapper mapper = new ObjectMapper();
            InputStream is = GridLoader.class.getResourceAsStream("/navgrid.json");
            NavGridData data = mapper.readValue(is, NavGridData.class);

            OccupancyGrid grid = new OccupancyGrid(data.width, data.height, data.resolution);

            for (var obs : data.staticObstacles) {
                for (int x = obs.x; x < obs.x + obs.w; x++) {
                    for (int y = obs.y; y < obs.y + obs.h; y++) {
                        grid.markStatic(x, y);
                    }
                }
            }

            return grid;
        } catch (Exception e) {
            throw new RuntimeException("Failed to load navgrid.json", e);
        }
    }
}
