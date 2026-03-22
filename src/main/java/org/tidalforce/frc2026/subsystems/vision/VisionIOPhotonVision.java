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

import static org.tidalforce.frc2026.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;

  /** Dynamic supplier because camera is mounted on a rotating turret */
  private final Supplier<Transform3d> robotToCameraSupplier;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCameraSupplier Supplier returning the robot->camera transform.
   */
  public VisionIOPhotonVision(String name, Supplier<Transform3d> robotToCameraSupplier) {
    camera = new PhotonCamera(name);
    this.robotToCameraSupplier = robotToCameraSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Get the latest camera pose relative to robot
    Transform3d robotToCamera = robotToCameraSupplier.get();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (var result : camera.getAllUnreadResults()) {

      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      // ---------------- MULTITAG RESULT ----------------
      if (result.multitagResult.isPresent()) {

        var multitagResult = result.multitagResult.get();

        // Camera pose relative to field
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;

        // Convert to robot pose
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());

        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Track tag IDs used
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                robotPose,
                multitagResult.estimatedPose.ambiguity,
                multitagResult.fiducialIDsUsed.size(),
                totalTagDistance / result.targets.size(),
                PoseObservationType.PHOTONVISION));

      }

      // ---------------- SINGLE TAG RESULT ----------------
      else if (!result.targets.isEmpty()) {

        var target = result.targets.get(0);

        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);

        if (tagPose.isPresent()) {

          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());

          Transform3d cameraToTarget = target.bestCameraToTarget;

          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());

          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());

          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          tagIds.add((short) target.fiducialId);

          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(),
                  robotPose,
                  target.poseAmbiguity,
                  1,
                  cameraToTarget.getTranslation().getNorm(),
                  PoseObservationType.PHOTONVISION));
        }
      }
    }

    // Save pose observations
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
