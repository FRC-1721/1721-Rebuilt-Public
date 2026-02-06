package org.tidalforce.frc2026.subsystems.navigation.dynamic;

import org.tidalforce.frc2026.subsystems.navigation.grid.*;
import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.PhotonCamera;

public class DynamicObstacleDetector {
    private final PhotonCamera cam = new PhotonCamera("ObjectCam");
    private final OccupancyGrid grid;
    private final InflationLayer inflation;

    public DynamicObstacleDetector(OccupancyGrid grid, InflationLayer inflation) {
        this.grid = grid;
        this.inflation = inflation;
    }

    public void update(Pose2d robotPose) {
        grid.clearDynamic();

        var result = cam.getLatestResult();
        if (!result.hasTargets()) return;

        for (var t : result.getTargets()) {
            double d = t.getBestCameraToTarget().getTranslation().getNorm();
            double yaw = Math.toRadians(t.getYaw());

            double x = robotPose.getX() + d * Math.cos(robotPose.getRotation().getRadians() + yaw);
            double y = robotPose.getY() + d * Math.sin(robotPose.getRotation().getRadians() + yaw);

            int gx = (int)(x / grid.resolution);
            int gy = (int)(y / grid.resolution);

            inflation.inflate(gx, gy);
        }
    }
}
