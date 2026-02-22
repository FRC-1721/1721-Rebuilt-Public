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

import static org.tidalforce.frc2026.subsystems.vision.VisionConstants.camera0Name;
import static org.tidalforce.frc2026.subsystems.vision.VisionConstants.camera1Name;
import static org.tidalforce.frc2026.subsystems.vision.VisionConstants.robotToCamera0;
import static org.tidalforce.frc2026.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.tidalforce.frc2026.FieldConstants.AprilTagLayoutType;
import org.tidalforce.frc2026.commands.DriveCommands;
import org.tidalforce.frc2026.commands.JoystickApproachCommand;
import org.tidalforce.frc2026.commands.JoystickFacePointCommand;
import org.tidalforce.frc2026.generated.TunerConstants;
// import org.tidalforce.frc2026.subsystems.battery.BatteryIO;
// import org.tidalforce.frc2026.subsystems.battery.BatteryIOReal;
import org.tidalforce.frc2026.subsystems.drive.Drive;
import org.tidalforce.frc2026.subsystems.drive.GyroIO;
import org.tidalforce.frc2026.subsystems.drive.GyroIOPigeon2;
import org.tidalforce.frc2026.subsystems.drive.ModuleIOSim;
import org.tidalforce.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.tidalforce.frc2026.subsystems.hopper.Hopper;
import org.tidalforce.frc2026.subsystems.intake.Intake;
import org.tidalforce.frc2026.subsystems.intake.IntakePivotIO;
import org.tidalforce.frc2026.subsystems.intake.IntakePivotSubsystem;
import org.tidalforce.frc2026.subsystems.kicker.Kicker;
import org.tidalforce.frc2026.subsystems.leds.LEDs;
import org.tidalforce.frc2026.subsystems.leds.LEDsConstants;
import org.tidalforce.frc2026.subsystems.rollers.RollerSystemIO;
import org.tidalforce.frc2026.subsystems.shooter.flywheel.Flywheel;
import org.tidalforce.frc2026.subsystems.shooter.flywheel.FlywheelIO;
import org.tidalforce.frc2026.subsystems.shooter.hood.Hood;
import org.tidalforce.frc2026.subsystems.shooter.hood.HoodIO;
import org.tidalforce.frc2026.subsystems.shooter.turret.Turret;
import org.tidalforce.frc2026.subsystems.shooter.turret.TurretIO;
import org.tidalforce.frc2026.subsystems.shooter.turret.TurretIOSim;
import org.tidalforce.frc2026.subsystems.vision.Vision;
import org.tidalforce.frc2026.subsystems.vision.VisionIOPhotonVision;
import org.tidalforce.frc2026.subsystems.vision.VisionIOPhotonVisionSim;
import org.tidalforce.frc2026.util.LoggedTunableNumber;
import org.tidalforce.frc2026.util.controllers.TriggerUtil;
import org.tidalforce.frc2026.util.controllers.TurtleBeachRematchAdvController;
import org.tidalforce.frc2026.util.geometry.AllianceFlipUtil;

// import org.tidalforce.lib.BatteryTracker;

@ExtensionMethod({TriggerUtil.class})
public class RobotContainer {
  // Subsystems
  public Drive drive;
  private Intake intake;
  private Hopper hopper;
  private Kicker kicker;
  private Hood hood;
  private Flywheel flywheel;
  private Turret turret;
  private Vision vision;
  private LEDs leds;
  private IntakePivotSubsystem intakePivot;

  private final LoggedDashboardChooser<Boolean> m_flipChooser;

  // Controllers
  private final TurtleBeachRematchAdvController TBC = new TurtleBeachRematchAdvController(0);
  private final CommandXboxController secondary = new CommandXboxController(1);

  // Battery Tracker
  // private BatteryTracker batteryTracker;

  public LoggedTunableNumber speedMultiplier =
      new LoggedTunableNumber("Drivebase Speed Multiplier", 1.1);
  private LoggedTunableNumber alignPredictionSeconds =
      new LoggedTunableNumber("Align Prediction Seconds", 0.3);

  private final Alert TBCDisconnected =
      new Alert("TBC controller disconnected (port 0).", AlertType.kWarning);
  private final Alert secondaryDisconnected =
      new Alert("Secondary controller disconnected (port 1).", AlertType.kWarning);

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    m_flipChooser = new LoggedDashboardChooser<>("Side");

    m_flipChooser.addOption("Right", false);
    m_flipChooser.addDefaultOption("Left", true);

    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.robot) {
        case COMP -> {
          drive =
              new Drive(
                  new GyroIOPigeon2() {},
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          //   vision =
          //       new Vision(
          //           drive::addVisionMeasurement,
          //           new VisionIOPhotonVision(camera0Name, robotToCamera0),
          //           new VisionIOPhotonVision(camera1Name, robotToCamera1));
          // leds = LEDsConstants.get();
          //   leds =
          //       new LEDs(
          //           new LightsIOCandle(
          //               org.tidalforce.frc2026.subsystems.leds.LEDsConstants.NAME,
          //               org.tidalforce.frc2026.subsystems.leds.LEDsConstants.CANDLE_ID,
          //               org.tidalforce.frc2026.subsystems.leds.LEDsConstants.CANDLE_CONFIG));
          //   hood =
          //       new Hood(
          //           new HoodIOKraken(
          //               org.tidalforce.frc2026.subsystems.shooter.hood.HoodConstants.HOODID,
          //               org.tidalforce.frc2026.subsystems.shooter.hood.HoodConstants.CAN_BUS));
          //   flywheel =
          //       new Flywheel(
          //           new FlywheelIOKraken(
          //               org.tidalforce.frc2026.subsystems.shooter.flywheel.FlywheelConstants
          //                   .FYLWHEELIDMAINID,
          //               org.tidalforce.frc2026.subsystems.shooter.flywheel.FlywheelConstants
          //                   .FLYWHEELFOLLOWID,
          //               org.tidalforce.frc2026.subsystems.shooter.flywheel.FlywheelConstants
          //                   .CAN_BUS));
          //   kicker =
          //       new Kicker(
          //           new RollerSystemIOKraken(
          //               org.tidalforce.frc2026.subsystems.rollers.RollerConstants.KICKER_ID,
          //               org.tidalforce.frc2026.subsystems.rollers.RollerConstants.CAN_BUS),
          //           new RollerSystemIOKraken(
          //               org.tidalforce.frc2026.subsystems.rollers.RollerConstants.KICKER_ID,
          //               org.tidalforce.frc2026.subsystems.rollers.RollerConstants.CAN_BUS));
          //   hopper =
          //       new Hopper(
          //           new RollerSystemIOKraken(
          //               org.tidalforce.frc2026.subsystems.rollers.RollerConstants.HOPPER_ID,
          //               org.tidalforce.frc2026.subsystems.rollers.RollerConstants.CAN_BUS));
          //   intake =
          //       new Intake(
          //           new RollerSystemIOKraken(
          //               org.tidalforce.frc2026.subsystems.rollers.RollerConstants.INTAKE_ID,
          //               org.tidalforce.frc2026.subsystems.rollers.RollerConstants.CAN_BUS));
          //   intakePivot =
          //       new IntakePivotSubsystem(
          //           new IntakePivotIOKraken(
          //               org.tidalforce.frc2026.subsystems.intake.IntakeConstants.INTAKEPIVOT_ID,
          //               org.tidalforce.frc2026.subsystems.intake.IntakeConstants.CAN_BUS,
          //               org.tidalforce.frc2026.subsystems.intake.IntakeConstants.IN_POSITION,
          //               org.tidalforce.frc2026.subsystems.intake.IntakeConstants.OUT_POSITION));
          //   batteryTracker = new BatteryTracker(new BatteryIOReal());
        }
        case DEV -> {
          drive =
              new Drive(
                  new GyroIOPigeon2() {},
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0));
          // batteryTracker = new BatteryTracker(new BatteryIOReal());
        }
        case SIM -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(TunerConstants.FrontLeft),
                  new ModuleIOSim(TunerConstants.FrontRight),
                  new ModuleIOSim(TunerConstants.BackLeft),
                  new ModuleIOSim(TunerConstants.BackRight));
          turret = new Turret(new TurretIOSim());
          leds = LEDsConstants.get();
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                  new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
          // batteryTracker = new BatteryTracker(new BatteryIO() {});
        }
      }
    }

    if (intake == null) intake = new Intake(new RollerSystemIO() {});
    if (hopper == null) hopper = new Hopper(new RollerSystemIO() {});
    if (hood == null) hood = new Hood(new HoodIO() {});
    if (flywheel == null) flywheel = new Flywheel(new FlywheelIO() {});
    if (turret == null) turret = new Turret(new TurretIO() {});
    if (kicker == null) kicker = new Kicker(new RollerSystemIO() {}, new RollerSystemIO() {});
    if (intakePivot == null) intakePivot = new IntakePivotSubsystem(new IntakePivotIO() {});

    turret.setDefaultCommand(turret.runTrackTargetCommand());
    hood.setDefaultCommand(hood.runTrackTargetCommand());
    flywheel.setDefaultCommand(flywheel.runTrackTargetCommand());

    registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));

    SmartDashboard.putData("AutoChooser", autoChooser.getSendableChooser());

    configureButtonBindings();

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -TBC.getLeftY() * speedMultiplier.getAsDouble() - secondary.getLeftY(),
            () -> -TBC.getLeftX() * speedMultiplier.getAsDouble() - secondary.getLeftX(),
            () -> -TBC.getRightX() - secondary.getRightX()));

    if (drive != null) {
      // Create a field object to send to Elastic
      edu.wpi.first.wpilibj.smartdashboard.Field2d field =
          new edu.wpi.first.wpilibj.smartdashboard.Field2d();
      SmartDashboard.putData("Field", field);

      // Log the active path (including Pathfinder paths)
      PathPlannerLogging.setLogActivePathCallback(
          (poses) -> {
            field.getObject("path").setPoses(poses);
          });

      // Log the target pose (the "ghost" robot Elastic will show)
      PathPlannerLogging.setLogTargetPoseCallback(
          (pose) -> {
            field.getObject("targetPose").setPose(pose);
          });
    }
    ;
  }

  private Command joystickApproach(Supplier<Pose2d> approachPose) {
    return new JoystickApproachCommand(
        drive, () -> -TBC.getLeftY() * speedMultiplier.getAsDouble(), approachPose);
  }

  private Command joystickFaceCommand(Supplier<Pose2d> facePose) {
    return new JoystickFacePointCommand(
        drive,
        () -> {
          boolean isBlue =
              DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
                  == DriverStation.Alliance.Blue;

          double value = TBC.getLeftY() * speedMultiplier.getAsDouble();
          return isBlue ? value : -value;
        },
        () -> {
          boolean isBlue =
              DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
                  == DriverStation.Alliance.Blue;

          double value = TBC.getLeftX() * speedMultiplier.getAsDouble();
          return isBlue ? value : -value;
        },
        facePose);
  }

  private Command testPathfindTo(Supplier<Pose2d> pose) {
    return Commands.defer(
        () -> AutoBuilder.pathfindToPose(pose.get(), DriveCommands.testPathConstraints()),
        Set.of(drive));
  }

  private Command compPathfindTo(Supplier<Pose2d> pose) {
    return Commands.defer(
        () -> AutoBuilder.pathfindToPose(pose.get(), DriveCommands.compPathConstraints()),
        Set.of(drive));
  }

  private Pose2d getFuturePose(double seconds) {
    return drive.getPose().exp(drive.getChassisSpeeds().toTwist2d(seconds));
  }

  private void configureButtonBindings() {
    TBC.povDown().whileTrue(Commands.runOnce(() -> intakePivot.deploy(), intakePivot));

    TBC.povUp().whileTrue(Commands.runOnce(() -> intakePivot.stow(), intakePivot));

    TBC.x().onTrue(hood.zeroCommand().alongWith(turret.zeroCommand()));

    TBC.b()
        .whileTrue(joystickFaceCommand(() -> AllianceFlipUtil.apply(FieldConstants.Hub.hubCenter)));

    TBC.leftTrigger()
        .onTrue(
            Commands.either(
                Commands.none(),
                Commands.runOnce(() -> intakePivot.deploy(), intakePivot),
                () -> intakePivot.isDeployed()));

    TBC.leftTrigger()
        .whileTrue(
            Commands.startEnd(
                () -> intake.setGoal(Intake.Goal.INTAKE),
                () -> intake.setGoal(Intake.Goal.STOP),
                intake));

    TBC.rightTrigger()
        .whileTrue(
            Commands.parallel(
                Commands.startEnd(
                    () -> hopper.setGoal(Hopper.Goal.SHOOT),
                    () -> hopper.setGoal(Hopper.Goal.STOP),
                    hopper),
                Commands.startEnd(
                    () -> kicker.setGoal(Kicker.Goal.SHOOT),
                    () -> kicker.setGoal(Kicker.Goal.STOP),
                    kicker),
                Commands.startEnd(
                    () -> flywheel.runGoalCommand(), () -> flywheel.setGoal(0), flywheel)));

    TBC.leftBumper()
        .whileTrue(
            joystickApproach(
                () ->
                    AllianceFlipUtil.apply(
                        FieldConstants.LeftTrench.getNearestLeftTrench(
                            getFuturePose(alignPredictionSeconds.get())))));

    TBC.rightBumper()
        .whileTrue(
            joystickApproach(
                () ->
                    AllianceFlipUtil.apply(
                        FieldConstants.RightTrench.getNearestRightTrench(
                            getFuturePose(alignPredictionSeconds.get())))));

    // TBC.a()
    //    .whileTrue(
    //        joystickApproach(
    //            () ->
    //                FieldConstants.Hub.getNearestHubCenter(
    //                    getFuturePose(alignPredictionSeconds.get()))));

    // My magnum opus

    TBC.LeftPaddle()
        .whileTrue(
            testPathfindTo(() -> AllianceFlipUtil.apply(FieldConstants.LeftTrench.leftTest)));

    TBC.RightPaddle()
        .whileTrue(
            compPathfindTo(() -> AllianceFlipUtil.apply(FieldConstants.RightTrench.rightTest)));

    TBC.RightPaddle()
        .multiPress(2, 0.1)
        .debounce(0.05)
        .onTrue(
            Commands.run(
                () ->
                    compPathfindTo(
                        () -> AllianceFlipUtil.apply(FieldConstants.RightTrench.rightTest)),
                drive));

    TBC.y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));
  }

  private void registerNamedCommands() {
    switch (Constants.currentMode) {
      default:

        // Shoot
        NamedCommands.registerCommand(
            "Shoot",
            Commands.parallel(
                    Commands.runEnd(
                        () ->
                            joystickFaceCommand(
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.hubCenter)),
                        () ->
                            joystickFaceCommand(
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.hubCenter)),
                        drive),
                    Commands.startEnd(
                        () -> hopper.setGoal(Hopper.Goal.SHOOT),
                        () -> hopper.setGoal(Hopper.Goal.STOP),
                        hopper),
                    Commands.startEnd(
                        () -> kicker.setGoal(Kicker.Goal.SHOOT),
                        () -> kicker.setGoal(Kicker.Goal.STOP),
                        kicker),
                    Commands.startEnd(
                        () -> flywheel.runGoalCommand(), () -> flywheel.setGoal(0), flywheel))
                .withTimeout(4));

        // Intake Out
        NamedCommands.registerCommand(
            "IntakePivotOut", Commands.runOnce(() -> intakePivot.deploy(), intakePivot));

        // Intake In
        NamedCommands.registerCommand(
            "IntakePivotIn", Commands.runOnce(() -> intakePivot.deploy(), intakePivot));

        // Intake Roll In And Pivot Out
        NamedCommands.registerCommand(
            "IntakePivotOutPlusIntake",
            Commands.parallel(
                    Commands.runOnce(() -> intakePivot.deploy(), intakePivot),
                    Commands.startEnd(
                        () -> intake.setGoal(Intake.Goal.INTAKE),
                        () -> intake.setGoal(Intake.Goal.STOP),
                        intake))
                .withTimeout(0.25));

        // Intake Roll In
        NamedCommands.registerCommand(
            "IntakeIn",
            Commands.startEnd(
                    () -> intake.setGoal(Intake.Goal.INTAKE),
                    () -> intake.setGoal(Intake.Goal.STOP),
                    intake)
                .withTimeout(1.86));
    }
  }

  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    TBCDisconnected.set(!DriverStation.isJoystickConnected(TBC.getHID().getPort()));
    secondaryDisconnected.set(!DriverStation.isJoystickConnected(secondary.getHID().getPort()));
  }

  // public void updateBatteryTelemetry() {
  //   batteryTracker.periodic();
  // }

  public Optional<Pose2d> getFirstAutoPose() {
    var autoCommandName = getAutonomousCommand().getName();
    if (AutoBuilder.getAllAutoNames().contains(autoCommandName)) {
      try {
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoCommandName);

        var firstPath = pathGroup.get(0);
        if (m_flipChooser.get()) {
          firstPath = firstPath.mirrorPath();
        }
        return Optional.of(
            new Pose2d(
                firstPath.getPathPoses().get(0).getTranslation(),
                firstPath.getIdealStartingState().rotation()));
      } catch (Exception e) {
        return Optional.empty();
      }
    }
    return Optional.empty();
  }

  public AprilTagLayoutType getSelectedAprilTagLayout() {
    return FieldConstants.defaultAprilTagType;
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Boolean shouldMirrorPath() {
    return m_flipChooser.get();
  }
}
