// Team: FRC 1721 - Concord Robotics (Tidal Force)
// Year: 2025-2026
// Code: Public codebase for our REBUILT frc robot
// License: MIT License (See LICENSE file for full text)
//
// Copyright (c) 2025-2026 Concord Robotics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN an ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package org.tidalforce.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.tidalforce.frc2026.subsystems.drive.DriveConstants;
import org.tidalforce.frc2026.subsystems.vision.VisionConstants;
import org.tidalforce.frc2026.util.EqualsUtil;
import org.tidalforce.frc2026.util.FuelSim;
import org.tidalforce.frc2026.util.LoggedTunableNumber;
import org.tidalforce.frc2026.util.geometry.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class ObjectDetection {
  private Set<FuelPoseRecord> fuelPoses = new HashSet<>();

  private static final LoggedTunableNumber fuelPersistanceTime =
      new LoggedTunableNumber("ObjectDetection/FuelPersistanceTime", 15.0);
  private static final LoggedTunableNumber allowedRoll =
      new LoggedTunableNumber("ObjectDetection/AllowedRoll", Units.degreesToRadians(5));
  private static final LoggedTunableNumber allowedPitch =
      new LoggedTunableNumber("ObjectDetection/AllowedPitch", Units.degreesToRadians(5));
  private static final LoggedTunableNumber fuelOverlap =
      new LoggedTunableNumber("ObjectDetection/FuelOverlap", FieldConstants.fuelDiameter / 2.0);
  private static final double maxPossibleFuel = 504;

  private static final LoggedTunableNumber densityRadius =
      new LoggedTunableNumber("ObjectDetection/DensityRadius", 1.0);

  private static final LoggedTunableNumber densityThreshold =
      new LoggedTunableNumber("ObjectDetection/DensityThreshold", 2.0);

  private static final LoggedTunableNumber densityDistanceWeight =
      new LoggedTunableNumber("ObjectDetection/DensityDistanceWeight", 0.4);

  private static ObjectDetection instance;
  @Setter private static FuelSim fuelSim;

  public static ObjectDetection getInstance() {
    if (instance == null) instance = new ObjectDetection();
    return instance;
  }

  /** Clears fuel that is too old or in the robot. */
  public void clearOldFuelPoses() {
    Rectangle2d robot =
        new Rectangle2d(
            RobotState.getInstance()
                .getEstimatedPose()
                .transformBy(
                    new Translation2d(
                            ((DriveConstants.intakeFarX - DriveConstants.frameWidthX) / 2.0), 0.0)
                        .toTransform2d()),
            DriveConstants.intakeFarX,
            DriveConstants.frameWidthY);
    fuelPoses =
        fuelPoses.stream()
            .filter((x) -> Timer.getTimestamp() - x.timestamp() < fuelPersistanceTime.get())
            .filter((x) -> !robot.contains(x.translation()))
            .collect(Collectors.toSet());
  }

  /** Clears fuel that is within the FOV of the specified camera. */
  public void clearFuelInCameraFOV(double timestamp, int camera) {
    // Get field to robot
    var fieldToRobotOptional = RobotState.getInstance().getEstimatedPoseAtTimestamp(timestamp);
    if (fieldToRobotOptional.isEmpty()) {
      return;
    }
    Pose2d fieldToRobot = fieldToRobotOptional.get();

    // Get field to camera
    var robotToCameraOptional = VisionConstants.cameras[camera].poseFunction().apply(timestamp);
    if (robotToCameraOptional.isEmpty()) {
      return;
    }
    Pose3d robotToCamera = robotToCameraOptional.get();
    Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera.toPose2d().toTransform2d());

    // Clear poses
    fuelPoses =
        fuelPoses.stream()
            .filter(
                (x) ->
                    Math.abs(
                            new Transform2d(
                                    fieldToCamera, new Pose2d(x.translation, Rotation2d.kZero))
                                .getTranslation()
                                .getAngle()
                                .getRadians())
                        > VisionConstants.cameras[camera].fovRads() / 2.0)
            .collect(Collectors.toSet());
  }

  public void addFuelTxTyObservation(FuelTxTyObservation observation) {
    Optional<Rotation3d> estimatedRotation3d =
        RobotState.getInstance().getEstimatedRotation3dAtTimestamp(observation.timestamp);
    if (estimatedRotation3d.isPresent()) {
      if (!(EqualsUtil.epsilonEquals(estimatedRotation3d.get().getX(), 0, allowedRoll.get())
          && EqualsUtil.epsilonEquals(estimatedRotation3d.get().getY(), 0, allowedPitch.get()))) {
        return;
      }
    }
    if (fuelPoses.size() >= maxPossibleFuel) {
      return;
    }

    var fieldToRobotOptional =
        RobotState.getInstance().getEstimatedPoseAtTimestamp(observation.timestamp());
    if (fieldToRobotOptional.isEmpty()) {
      return;
    }
    Pose2d fieldToRobot = fieldToRobotOptional.get();

    Pose3d robotToCamera =
        VisionConstants.cameras[observation.camera()]
            .poseFunction()
            .apply(observation.timestamp())
            .get();

    // Find midpoint of width of top tx ty
    double tx = (observation.tx()[0] + observation.tx()[1]) / 2;
    double ty = (observation.ty()[0] + observation.ty()[1]) / 2;

    // Account for camera roll
    Translation2d txyxTranslation =
        new Translation2d(Math.tan(tx), Math.tan(-ty))
            .rotateBy(new Rotation2d(-robotToCamera.getRotation().getX()));
    tx = Math.atan(txyxTranslation.getX());
    ty = -Math.atan(txyxTranslation.getY());

    // No fuel above camera
    double cameraToFuelAngle = -robotToCamera.getRotation().getY() - ty;
    if (cameraToFuelAngle >= 0) {
      return;
    }

    // Top down distance to fuel from camera
    double cameraToFuelNorm =
        (-robotToCamera.getZ() + FieldConstants.fuelDiameter)
            / Math.tan(-robotToCamera.getRotation().getY() - ty)
            / Math.cos(tx);

    Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera.toPose2d().toTransform2d());
    Pose2d fieldToFuel =
        fieldToCamera
            .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-tx)))
            .transformBy(new Transform2d(new Translation2d(cameraToFuelNorm, 0), Rotation2d.kZero));

    Translation2d fieldToFuelTranslation2d = fieldToFuel.getTranslation();

    FuelPoseRecord fuelPoseRecord =
        new FuelPoseRecord(fieldToFuelTranslation2d, observation.timestamp());

    fuelPoses =
        fuelPoses.stream()
            .filter((x) -> x.translation.getDistance(fieldToFuelTranslation2d) > fuelOverlap.get())
            .collect(Collectors.toSet());

    fuelPoses.add(fuelPoseRecord);
  }

  public Set<Translation2d> getFuelTranslations() {
    if (Constants.getMode() == Constants.Mode.SIM) {
      return fuelSim.getFuels();
    }
    return fuelPoses.stream().map(FuelPoseRecord::translation).collect(Collectors.toSet());
  }

  public Optional<Translation2d> getDensestFuelClusterCenter() {
    var fuels = getFuelTranslations();
    if (fuels.isEmpty()) return Optional.empty();

    Translation2d robotPosition = RobotState.getInstance().getEstimatedPose().getTranslation();

    double bestScore = Double.NEGATIVE_INFINITY;
    Translation2d bestCenter = null;

    for (var candidate : fuels) {
      double radius = densityRadius.get();
      int count = 0;
      double sumX = 0;
      double sumY = 0;

      for (var other : fuels) {
        if (candidate.getDistance(other) <= radius) {
          count++;
          sumX += other.getX();
          sumY += other.getY();
        }
      }

      if (count >= densityThreshold.get()) {
        Translation2d clusterCenter = new Translation2d(sumX / count, sumY / count);

        double distanceFromRobot = clusterCenter.getDistance(robotPosition);

        double score = count - (densityDistanceWeight.get() * distanceFromRobot);

        if (score > bestScore) {
          bestScore = score;
          bestCenter = clusterCenter;
        }
      }
    }

    return Optional.ofNullable(bestCenter);
  }

  public Optional<Pose2d> getDensestFuelClusterPose() {
    return getDensestFuelClusterCenter()
        .map(t -> new Pose2d(t, RobotState.getInstance().getEstimatedPose().getRotation()));
  }

  public record FuelTxTyObservation(int camera, double[] tx, double[] ty, double timestamp) {}

  public record FuelPoseRecord(Translation2d translation, double timestamp) {}
}
