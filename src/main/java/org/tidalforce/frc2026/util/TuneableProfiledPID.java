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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class TuneableProfiledPID extends ProfiledPIDController {

  // Instance name for tagging log values
  String m_name;

  // Tunable numbers
  private LoggedTunableNumber m_kP, m_kI, m_kD, m_maxV, m_maxA;

  public TuneableProfiledPID(
      String name, double kP, double kI, double kD, double maxV, double maxA) {
    super(kP, kI, kD, new TrapezoidProfile.Constraints(maxV, maxA));

    m_name = name;

    // Tunable numbers for PID and motion gain constants
    m_kP = new LoggedTunableNumber(m_name + "/kP", kP);
    m_kI = new LoggedTunableNumber(m_name + "/kI", kI);
    m_kD = new LoggedTunableNumber(m_name + "/kD", kD);

    m_maxV = new LoggedTunableNumber(m_name + "/maxV", maxV);
    m_maxA = new LoggedTunableNumber(m_name + "/maxA", maxA);
  }

  public void updatePID() {
    // If changed, update controller constants from Tuneable Numbers
    if (m_kP.hasChanged(hashCode()) || m_kI.hasChanged(hashCode()) || m_kD.hasChanged(hashCode())) {
      this.setPID(m_kP.get(), m_kI.get(), m_kD.get());
    }

    if (m_maxV.hasChanged(hashCode()) || m_maxA.hasChanged(hashCode())) {
      this.setConstraints(new TrapezoidProfile.Constraints(m_maxV.get(), m_maxA.get()));
    }
  }
}
