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

package org.tidalforce.frc2026.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.tidalforce.frc2026.Constants;
import org.tidalforce.frc2026.Constants.RobotType;

public class DriveConstants {
  public static final double COMPTrackWidthXInches = 20.25;
  public static final double COMPTrackWidthYInches = 20.25;
  public static final double COMPFrameWidthXInches = 26;
  public static final double COMPFrameWidthYInches = 26;
  public static final double COMPMaxLinearSpeed = 4.2; // Theoretical max is 4.4196
  public static final double COMPMaxTrajectoryLinearSpeed = 4.0;
  public static final double COMPMaxAngularSpeed =
      5.374; // maxLinearSpeed / driveBaseRadius (0.7814886979 meters)
  public static final double COMPWheelRadiusInches = 1.99588001;
  public static final double COMPTrajectoryWheelRadiusInches = 2.0;
  public static final double COMPMaxTrajectoryWheelTorque = 4.0; // N * m
  public static final double COMPMassLbs = 150.0;
  public static final double COMPWheelCOF = 1.5;
  public static final double COMPRotationMOI = 6.0; // kg * m^2

  // SDS MK5i modules, R1 reduction
  public static final double COMPDriveReduction = 7.03125;
  public static final double COMPTurnReductionFL = 26.0;
  public static final double COMPTurnReductionFR = 26.0;
  public static final double COMPTurnReductionBL = 26.0;
  public static final double COMPTurnReductionBR = 26.0;
  // public static final double COMPTurnReductionBR = 26.09090909091; // MK5n

  public static final String COMPCanBus = "*";
  public static final int COMPGyroId = 30;
  public static final int COMPDriveMotorIdFL = 11;
  public static final int COMPDriveMotorIdFR = 13;
  public static final int COMPDriveMotorIdBL = 0;
  public static final int COMPDriveMotorIdBR = 23;

  public static final int COMPTurnMotorIdFL = 10;
  public static final int COMPTurnMotorIdFR = 12;
  public static final int COMPTurnMotorIdBL = 1;
  public static final int COMPTurnMotorIdBR = 22;

  public static final int COMPEncoderIdFL = 40;
  public static final int COMPEncoderIdFR = 41;
  public static final int COMPEncoderIdBL = 42;
  public static final int COMPEncoderIdBR = 43;

  public static final double COMPEncoderOffsetFL = -1.530913;
  public static final double COMPEncoderOffsetFR = 2.078544;
  public static final double COMPEncoderOffsetBL = -2.42369;
  public static final double COMPEncoderOffsetBR = -2.863942;

  // MARK: - DEV Constants

  public static final double DEVTrackWidthXInches = 22.75;
  public static final double DEVTrackWidthYInches = 22.75;
  public static final double DEVFrameWidthXInches = 28.0;
  public static final double DEVFrameWidthYInches = 28.0;
  public static final double DEVFullWidthXInches = 34.5;
  public static final double DEVFullWidthYInches = 34.5;
  public static final double DEVMaxLinearSpeed = 4.69;
  public static final double DEVMaxTrajectoryLinearSpeed = 4.0;
  public static final double DEVMaxAngularSpeed = 6.46; // maxLinearSpeed / driveBaseRadius
  public static final double DEVWheelRadiusInches = 1.8796;
  public static final double DEVTrajectoryWheelRadiusInches = 2.0;
  public static final double DEVMaxTrajectoryWheelTorque = 3.0; // N * m
  public static final double DEVMassLbs = 140.0;
  public static final double DEVWheelCOF = 1.5;
  public static final double DEVRotationMOI = 6.0; // kg * m^2

  // SDS MK4i modules, L3 reduction
  public static final double DEVDriveReduction = 6.1224489796;
  public static final double DEVTurnReductionFL = 21.4285714286;
  public static final double DEVTurnReductionFR = 21.4285714286;
  public static final double DEVTurnReductionBL = 21.4285714286;
  public static final double DEVTurnReductionBR = 21.4285714286;

  public static final String DEVCanBus = "*";
  public static final int DEVGyroId = 1;
  public static final int DEVDriveMotorIdFL = 30;
  public static final int DEVDriveMotorIdFR = 2;
  public static final int DEVDriveMotorIdBL = 1;
  public static final int DEVDriveMotorIdBR = 3;

  public static final int DEVTurnMotorIdFL = 4;
  public static final int DEVTurnMotorIdFR = 7;
  public static final int DEVTurnMotorIdBL = 5;
  public static final int DEVTurnMotorIdBR = 6;

  public static final int DEVEncoderIdFL = 0;
  public static final int DEVEncoderIdFR = 1;
  public static final int DEVEncoderIdBL = 2;
  public static final int DEVEncoderIdBR = 3;

  public static final double DEVEncoderOffsetFL = 0.012886;
  public static final double DEVEncoderOffsetFR = 0.866168;
  public static final double DEVEncoderOffsetBL = -1.050078;
  public static final double DEVEncoderOffsetBR = -2.801255;

  // MARK: - Shared Constants

  public static final double driveKs = 5.0;
  public static final double driveKv = 0.0;
  public static final double driveKp = 35.0;
  public static final double driveKd = 0.0;
  public static final double turnKp = 4000.0;
  public static final double turnKd = 50.0;
  public static final double turnDeadbandDegrees = 0.3;
  public static final double driveCurrentLimitAmps = 80;
  public static final double driveSupplyCurrentLimitAmps = 40;
  public static final double turnCurrentLimitAmps = 40;

  public static final double trackWidthX =
      Units.inchesToMeters(
          Constants.robot == RobotType.DEV ? DEVTrackWidthXInches : COMPTrackWidthXInches);
  public static final double trackWidthY =
      Units.inchesToMeters(
          Constants.robot == RobotType.DEV ? DEVTrackWidthYInches : COMPTrackWidthYInches);
  public static final double frameWidthX =
      Units.inchesToMeters(
          Constants.robot == RobotType.DEV ? DEVFrameWidthXInches : COMPFrameWidthXInches);
  public static final double frameWidthY =
      Units.inchesToMeters(
          Constants.robot == RobotType.DEV ? DEVFrameWidthYInches : COMPFrameWidthYInches);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxLinearSpeed =
      Constants.robot == RobotType.DEV ? DEVMaxLinearSpeed : COMPMaxLinearSpeed;
  public static final double maxTrajectoryLinearSpeed =
      Constants.robot == RobotType.DEV ? DEVMaxTrajectoryLinearSpeed : COMPMaxTrajectoryLinearSpeed;
  public static final double maxAngularSpeed =
      Constants.robot == RobotType.DEV ? DEVMaxAngularSpeed : COMPMaxAngularSpeed;
  public static final double wheelRadiusInches =
      Constants.robot == RobotType.DEV ? DEVWheelRadiusInches : COMPWheelRadiusInches;
  public static final double trajectoryWheelRadiusInches =
      Constants.robot == RobotType.DEV
          ? DEVTrajectoryWheelRadiusInches
          : COMPTrajectoryWheelRadiusInches;
  public static final double maxTrajectoryWheelTorque =
      Constants.robot == RobotType.DEV ? DEVMaxTrajectoryWheelTorque : COMPMaxTrajectoryWheelTorque;
  public static final double wheelRadius = Units.inchesToMeters(wheelRadiusInches);
  public static final double trajectoryWheelRadius =
      Units.inchesToMeters(trajectoryWheelRadiusInches);
  public static final double mass =
      Units.lbsToKilograms(Constants.robot == RobotType.DEV ? DEVMassLbs : COMPMassLbs);
  public static final double wheelCOF =
      Constants.robot == RobotType.DEV ? DEVWheelCOF : COMPWheelCOF;
  public static final double rotationMOI =
      Constants.robot == RobotType.DEV ? DEVRotationMOI : COMPRotationMOI;
  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double driveReduction =
      Constants.robot == RobotType.DEV ? DEVDriveReduction : COMPDriveReduction;
  public static final double turnReductionFL =
      Constants.robot == RobotType.DEV ? DEVTurnReductionFL : COMPTurnReductionFL;
  public static final double turnReductionFR =
      Constants.robot == RobotType.DEV ? DEVTurnReductionFR : COMPTurnReductionFR;
  public static final double turnReductionBL =
      Constants.robot == RobotType.DEV ? DEVTurnReductionBL : COMPTurnReductionBL;
  public static final double turnReductionBR =
      Constants.robot == RobotType.DEV ? DEVTurnReductionBR : COMPTurnReductionBR;

  public static final double intakeFarX = frameWidthX / 2.0 + Units.inchesToMeters(12.0);
  public static final double intakeWidth = frameWidthY;
}
