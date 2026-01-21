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

package org.tidalforce.frc2026.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

// NOTE: This file is available at
// https://gist.github.com/mjansen4857/a8024b55eb427184dbd10ae8923bd57d

public class LocalADStarAK implements Pathfinder {
  private final ADStarIO io = new ADStarIO();

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
    if (!Logger.hasReplaySource()) {
      io.updateIsNewPathAvailable();
    }

    Logger.processInputs("LocalADStarAK", io);

    return io.isNewPathAvailable;
  }

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    if (!Logger.hasReplaySource()) {
      io.updateCurrentPathPoints(constraints, goalEndState);
    }

    Logger.processInputs("LocalADStarAK", io);

    if (io.currentPathPoints.isEmpty()) {
      return null;
    }

    return PathPlannerPath.fromPathPoints(io.currentPathPoints, constraints, goalEndState);
  }

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setStartPosition(startPosition);
    }
  }

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setGoalPosition(goalPosition);
    }
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path to properly avoid obstacles
   */
  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setDynamicObstacles(obs, currentRobotPos);
    }
  }

  private static class ADStarIO implements LoggableInputs {
    public LocalADStar adStar = new LocalADStar();
    public boolean isNewPathAvailable = false;
    public List<PathPoint> currentPathPoints = Collections.emptyList();

    @Override
    public void toLog(LogTable table) {
      table.put("IsNewPathAvailable", isNewPathAvailable);

      double[] pointsLogged = new double[currentPathPoints.size() * 2];
      int idx = 0;
      for (PathPoint point : currentPathPoints) {
        pointsLogged[idx] = point.position.getX();
        pointsLogged[idx + 1] = point.position.getY();
        idx += 2;
      }

      table.put("CurrentPathPoints", pointsLogged);
    }

    @Override
    public void fromLog(LogTable table) {
      isNewPathAvailable = table.get("IsNewPathAvailable", false);

      double[] pointsLogged = table.get("CurrentPathPoints", new double[0]);

      List<PathPoint> pathPoints = new ArrayList<>();
      for (int i = 0; i < pointsLogged.length; i += 2) {
        pathPoints.add(
            new PathPoint(new Translation2d(pointsLogged[i], pointsLogged[i + 1]), null));
      }

      currentPathPoints = pathPoints;
    }

    public void updateIsNewPathAvailable() {
      isNewPathAvailable = adStar.isNewPathAvailable();
    }

    public void updateCurrentPathPoints(PathConstraints constraints, GoalEndState goalEndState) {
      PathPlannerPath currentPath = adStar.getCurrentPath(constraints, goalEndState);

      if (currentPath != null) {
        currentPathPoints = currentPath.getAllPathPoints();
      } else {
        currentPathPoints = Collections.emptyList();
      }
    }
  }
}
