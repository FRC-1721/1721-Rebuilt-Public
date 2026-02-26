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

package org.tidalforce.frc2026.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import java.util.function.Function;
import lombok.Builder;
import org.tidalforce.frc2026.Constants;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.01;
  public static final double thetaStdDevCoefficient = 0.03;

  public static final double fuelDetectConfidenceThreshold = 0.2;

  private static int monoExposure = 1800;
  private static double monoGain = 15.0;
  private static double monoDenoise = 1.0;
  private static int colorExposure = 4500;
  private static double colorGain = 5.0;

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.28, 0.25, 0.2, new Rotation3d(0.0, Units.degreesToRadians(-22.5), 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          0.28,
          -0.25,
          0.2,
          new Rotation3d(0.0, Units.degreesToRadians(22.5), Units.degreesToRadians(4)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static CameraConfig[] cameras =
      switch (Constants.robot) {
        case COMP ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(-10.343),
                                Units.inchesToMeters(-8.102),
                                Units.inchesToMeters(20.940),
                                new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(-22.5),
                                    Units.degreesToRadians(175.0))));
                      })
                  .id("40530395")
                  .width(1800)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(75.0))
                  .build()
            };
        case DEV ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(-28.0 / 2.0 + 2.5),
                                Units.inchesToMeters(-28.0 / 2.0 + 2.75),
                                Units.inchesToMeters(18.75),
                                new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(-27.0),
                                    Units.degreesToRadians(-152.5))));
                      })
                  .id("40530395")
                  .width(1800)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(75.0))
                  .build(),
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(-28.0 / 2.0 + 2.25),
                                Units.inchesToMeters(-28.0 / 2.0 + 2.75),
                                Units.inchesToMeters(18.0),
                                new Rotation3d(
                                    Units.degreesToRadians(11.0),
                                    Units.degreesToRadians(15.0),
                                    Units.degreesToRadians(-167.0))));
                      })
                  .id("24737133")
                  .width(1280)
                  .height(960)
                  .exposure(colorExposure)
                  .gain(colorGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(90.0))
                  .build()
            };
        default -> new CameraConfig[] {};
      };

  @Builder
  public record CameraConfig(
      Function<Double, Optional<Pose3d>> poseFunction,
      String id,
      int width,
      int height,
      int autoExposure,
      int exposure,
      double gain,
      double denoise,
      double stdDevFactor,
      double fovRads) {}

  private VisionConstants() {}
}
