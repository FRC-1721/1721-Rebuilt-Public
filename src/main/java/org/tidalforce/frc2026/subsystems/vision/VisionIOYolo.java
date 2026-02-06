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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;

public class VisionIOYolo implements VisionIO {

  private final NetworkTable table;

  public VisionIOYolo() {
    table = NetworkTableInstance.getDefault().getTable("yolo");
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = true;

    double[] raw = table.getEntry("fuel").getDoubleArray(new double[0]);

    List<YoloDetection> detections = new ArrayList<>();

    // Expected format: cx, cy, conf, timestamp repeating
    for (int i = 0; i + 3 < raw.length; i += 4) {
      double cx = raw[i];
      double cy = raw[i + 1];
      double conf = raw[i + 2];
      double ts = raw[i + 3];

      detections.add(new YoloDetection(cx, cy, conf, ts));
    }

    inputs.fuelDetections = detections.toArray(new YoloDetection[0]);
  }
}
