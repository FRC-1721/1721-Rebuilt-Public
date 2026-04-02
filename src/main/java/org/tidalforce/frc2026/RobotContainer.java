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

import static org.tidalforce.frc2026.subsystems.vision.VisionConstants.targetingCamera;
// import static org.tidalforce.frc2026.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
import org.tidalforce.frc2026.commands.ShooterTestCommands;
import org.tidalforce.frc2026.commands.ShooterTuningCommand;
import org.tidalforce.frc2026.generated.TunerConstants;
// import org.tidalforce.frc2026.subsystems.battery.BatteryIO;
// import org.tidalforce.frc2026.subsystems.battery.BatteryIOReal;
import org.tidalforce.frc2026.subsystems.drive.Drive;
import org.tidalforce.frc2026.subsystems.drive.GyroIO;
import org.tidalforce.frc2026.subsystems.drive.GyroIOPigeon2;
import org.tidalforce.frc2026.subsystems.drive.ModuleIO;
import org.tidalforce.frc2026.subsystems.drive.ModuleIOSim;
import org.tidalforce.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.tidalforce.frc2026.subsystems.hopper.Hopper;
import org.tidalforce.frc2026.subsystems.intake.Intake;
import org.tidalforce.frc2026.subsystems.intake.Intake.Goal;
import org.tidalforce.frc2026.subsystems.intake.IntakePivotIOKraken;
import org.tidalforce.frc2026.subsystems.intake.IntakePivotSubsystem;
import org.tidalforce.frc2026.subsystems.kicker.Kicker;
import org.tidalforce.frc2026.subsystems.leds.LEDs;
import org.tidalforce.frc2026.subsystems.leds.LEDsConstants;
import org.tidalforce.frc2026.subsystems.rollers.RollerSystemIO;
import org.tidalforce.frc2026.subsystems.rollers.RollerSystemIOKraken;
import org.tidalforce.frc2026.subsystems.shooter.LaunchCalculator;
import org.tidalforce.frc2026.subsystems.shooter.LauncherConstants;
import org.tidalforce.frc2026.subsystems.shooter.PassingCalculator;
import org.tidalforce.frc2026.subsystems.shooter.flywheel.Flywheel;
import org.tidalforce.frc2026.subsystems.shooter.flywheel.FlywheelIO;
import org.tidalforce.frc2026.subsystems.shooter.flywheel.FlywheelIOKraken;
import org.tidalforce.frc2026.subsystems.shooter.hood.Hood;
import org.tidalforce.frc2026.subsystems.shooter.hood.HoodConstants;
import org.tidalforce.frc2026.subsystems.shooter.hood.HoodIO;
import org.tidalforce.frc2026.subsystems.shooter.hood.HoodIOKraken;
import org.tidalforce.frc2026.subsystems.shooter.turret.Turret;
import org.tidalforce.frc2026.subsystems.shooter.turret.TurretIO;
import org.tidalforce.frc2026.subsystems.shooter.turret.TurretIOKraken;
// import org.tidalforce.frc2026.subsystems.shooter.turret.TurretIOSim;
import org.tidalforce.frc2026.subsystems.vision.Vision;
import org.tidalforce.frc2026.subsystems.vision.VisionIOPhotonVision;
// import org.tidalforce.frc2026.subsystems.vision.VisionIOPhotonVisionSim;
import org.tidalforce.frc2026.util.FuelSim;
import org.tidalforce.frc2026.util.HubShiftUtil;
import org.tidalforce.frc2026.util.LoggedTunableNumber;
import org.tidalforce.frc2026.util.controllers.OverrideSwitches;
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
  private org.photonvision.PhotonCamera fuelCamera;
  //   private Intake turrethaha;

  private final Trigger isAutonomous = new Trigger(DriverStation::isAutonomous);
  private final Trigger isDisabled = new Trigger(DriverStation::isDisabled);

  private final LoggedDashboardChooser<Boolean> m_flipChooser;

  // Controllers
  private final TurtleBeachRematchAdvController TBC = new TurtleBeachRematchAdvController(0);
  private final CommandXboxController secondary = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(2);

  // Driver Overrides
  private final Trigger coast = overrides.driverSwitch(0);
  private final Trigger ignoreHubState = overrides.driverSwitch(1);
  private final Trigger wonAutoOverride = secondary.povUp();
  private final Trigger lostAutoOverride = secondary.povDown();
  private final Trigger aggresivePathfinding = overrides.multiDirectionSwitch2Down();
  private final Trigger passivePathfinding = overrides.multiDirectionSwitch2Up();

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
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 2).", AlertType.kWarning);
  private final Alert autoWinnerNotSet = new Alert("!!! AUTO WINNER NOT SET !!!", AlertType.kError);

  private final LoggedDashboardChooser<Command> autoChooser;

  private boolean coastOverride = false;

  private enum PathMode {
    AGGRESSIVE,
    MID,
    PASSIVE,
    FUEL,
    NONE
  }

  private PathMode currentPathMode = PathMode.NONE;

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
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(
                      targetingCamera,
                      () ->
                          LauncherConstants.robotToTurret
                              .plus(
                                  new Transform3d(
                                      new edu.wpi.first.math.geometry.Translation3d(),
                                      new Rotation3d(
                                          0, 0, turret.getPositionRads()))) // <-- live turret angle
                              .plus(LauncherConstants.turretToCamera),
                      true));
          //   new VisionIOPhotonVision(localizationCamera, () -> robotToCamera1, true));

          //   fuelCamera = new org.photonvision.PhotonCamera(targetingCamera);
          // leds = LEDsConstants.get();
          //   leds =
          //       new LEDs(
          //           new LightsIOCandle(
          //               org.tidalforce.frc2026.subsystems.leds.LEDsConstants.NAME,
          //               org.tidalforce.frc2026.subsystems.leds.LEDsConstants.CANDLE_ID,
          //               org.tidalforce.frc2026.subsystems.leds.LEDsConstants.CANDLE_CONFIG));
          hood =
              new Hood(
                  new HoodIOKraken(
                      org.tidalforce.frc2026.subsystems.shooter.hood.HoodConstants.HOODID,
                      org.tidalforce.frc2026.subsystems.shooter.hood.HoodConstants.CAN_BUS));
          flywheel =
              new Flywheel(
                  new FlywheelIOKraken(
                      org.tidalforce.frc2026.subsystems.shooter.flywheel.FlywheelConstants
                          .FYLWHEELIDMAINID,
                      org.tidalforce.frc2026.subsystems.shooter.flywheel.FlywheelConstants
                          .FLYWHEELFOLLOWID,
                      org.tidalforce.frc2026.subsystems.shooter.flywheel.FlywheelConstants
                          .CAN_BUS));
          turret =
              new Turret(
                  new TurretIOKraken(
                      org.tidalforce.frc2026.subsystems.shooter.turret.TurretConstants.TURRET_ID,
                      org.tidalforce.frc2026.subsystems.shooter.turret.TurretConstants.CAN_BUS));
          kicker =
              new Kicker(
                  new RollerSystemIOKraken(
                      org.tidalforce.frc2026.subsystems.rollers.RollerConstants.KICKER_ID,
                      org.tidalforce.frc2026.subsystems.rollers.RollerConstants.CAN_BUS));
          hopper =
              new Hopper(
                  new RollerSystemIOKraken(
                      org.tidalforce.frc2026.subsystems.rollers.RollerConstants.HOPPER_ID,
                      org.tidalforce.frc2026.subsystems.rollers.RollerConstants.CAN_BUS));
          intake =
              new Intake(
                  new RollerSystemIOKraken(
                      org.tidalforce.frc2026.subsystems.rollers.RollerConstants.INTAKE_ID,
                      org.tidalforce.frc2026.subsystems.rollers.RollerConstants.CAN_BUS));
          intakePivot =
              new IntakePivotSubsystem(
                  new IntakePivotIOKraken(
                      org.tidalforce.frc2026.subsystems.intake.IntakeConstants.INTAKEPIVOT_ID,
                      org.tidalforce.frc2026.subsystems.intake.IntakeConstants.CAN_BUS),
                  org.tidalforce.frc2026.subsystems.intake.IntakeConstants.IN_POSITION,
                  org.tidalforce.frc2026.subsystems.intake.IntakeConstants.OUT_POSITION);
        }
        case DEV -> {
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
          //           new VisionIOPhotonVision(localizationCamera, robotToCamera0));

          fuelCamera = new org.photonvision.PhotonCamera(targetingCamera);
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
          // turret = new Turret(new TurretIOSim());
          leds = LEDsConstants.get();
          //   vision =
          //       new Vision(
          //           drive::addVisionMeasurement,
          //           new VisionIOPhotonVisionSim(localizationCamera, robotToCamera0,
          // drive::getPose),
          //           new VisionIOPhotonVisionSim(targetingCamera, robotToCamera1,
          // drive::getPose));
          // batteryTracker = new BatteryTracker(new BatteryIO() {});
          fuelCamera = new org.photonvision.PhotonCamera(targetingCamera);
          ObjectDetection.setFuelSim(new FuelSim());
        }
      }
    }
    if (Constants.getMode() == Constants.Mode.REPLAY) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    if (intake == null) intake = new Intake(new RollerSystemIO() {});
    if (hopper == null) hopper = new Hopper(new RollerSystemIO() {});
    if (hood == null) hood = new Hood(new HoodIO() {});
    if (flywheel == null) flywheel = new Flywheel(new FlywheelIO() {});
    if (turret == null) turret = new Turret(new TurretIO() {});
    if (kicker == null) kicker = new Kicker(new RollerSystemIO() {});

    // Set up overrides
    hood.setCoastOverride(() -> coastOverride);
    hopper.setCoastOverride(() -> coastOverride);
    intake.setCoastOverride(() -> coastOverride);
    kicker.setCoastOverride(() -> coastOverride);
    HubShiftUtil.setAllianceWinOverride(
        () -> {
          if (lostAutoOverride.getAsBoolean()) {
            return Optional.of(false);
          }
          if (wonAutoOverride.getAsBoolean()) {
            return Optional.of(true);
          }
          return Optional.empty();
        });

    // turret.setDefaultCommand(turret.runTrackTargetCommand());
    // hood.setDefaultCommand();
    // flywheel.setDefaultCommand(flywheel.runFixedCommand(() -> 900));

    registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption("FF Characterization", DriveCommands.feedforwardCharacterization(drive));

    SmartDashboard.putData("AutoChooser", autoChooser.getSendableChooser());

    configureButtonBindings();
    configureTestBindings();
    // configureLEDTriggers();

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -TBC.getLeftY() * speedMultiplier.getAsDouble(),
            () -> -TBC.getLeftX() * speedMultiplier.getAsDouble(),
            () -> -TBC.getRightX()));

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

  private Command smartShootCommand() {
    return Commands.parallel(
            // ── Set mode ONCE ─────────────────────────────
            Commands.runOnce(
                () -> {
                  var calc = LaunchCalculator.getInstance();

                  if (!HubShiftUtil.getShiftedShiftInfo().active()) {
                    calc.setPassingMode(PassingCalculator.PassingSide.LEFT);
                  } else {
                    calc.disablePassingMode(); // REQUIRED
                  }
                }),

            // ── Aim + spin up ─────────────────────────────
            turret.runTrackTargetCommand(),
            hood.runTrackTargetCommand(),
            flywheel.runTrackTargetCommand(),

            // ── Feed when ready ───────────────────────────
            Commands.waitUntil(() -> hood.atGoal() && flywheel.atGoal())
                .andThen(
                    Commands.parallel(
                        Commands.startEnd(
                            () -> hopper.setGoal(Hopper.Goal.SHOOT),
                            () -> hopper.setGoal(Hopper.Goal.STOP),
                            hopper),
                        Commands.startEnd(
                            () -> kicker.setGoal(Kicker.Goal.SHOOT),
                            () -> kicker.setGoal(Kicker.Goal.STOP),
                            kicker))))
        .withName("SmartShoot");
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

  private Command slowPathfindTo(Supplier<Pose2d> pose) {
    return Commands.defer(
            () -> AutoBuilder.pathfindToPose(pose.get(), DriveCommands.slowPathConstraints()),
            Set.of(drive))
        .beforeStarting(() -> currentPathMode = PathMode.PASSIVE)
        .finallyDo(() -> currentPathMode = PathMode.NONE);
  }

  private Command midLevelPathfindTo(Supplier<Pose2d> pose) {
    return Commands.defer(
            () -> AutoBuilder.pathfindToPose(pose.get(), DriveCommands.midLevelPathConstraints()),
            Set.of(drive))
        .beforeStarting(() -> currentPathMode = PathMode.MID)
        .finallyDo(() -> currentPathMode = PathMode.NONE);
  }

  private Command compPathfindTo(Supplier<Pose2d> pose) {
    return Commands.defer(
            () -> AutoBuilder.pathfindToPose(pose.get(), DriveCommands.compPathConstraints()),
            Set.of(drive))
        .beforeStarting(() -> currentPathMode = PathMode.AGGRESSIVE)
        .finallyDo(() -> currentPathMode = PathMode.NONE);
  }

  private Command shootCommand() {
    return Commands.parallel(
            Commands.startEnd(
                () -> hopper.setGoal(Hopper.Goal.SHOOT),
                () -> hopper.setGoal(Hopper.Goal.STOP),
                hopper),
            Commands.startEnd(
                () -> kicker.setGoal(Kicker.Goal.SHOOT),
                () -> kicker.setGoal(Kicker.Goal.STOP),
                kicker),
            flywheel.runTrackTargetCommand())
        .withName("ShootCommand");
  }

  private Pose2d getFuturePose(double seconds) {
    return drive.getPose().exp(drive.getChassisSpeeds().toTwist2d(seconds));
  }

  //   private PassingCalculator.PassingSide getAutoPassingSide() {
  //   var launchCalc = LaunchCalculator.getInstance();

  //   // Get current turret angle (field-relative)
  //   var currentAngle = turret.getAngle(); // MUST be field-relative

  //   // Get both passing options
  //   var leftParams =
  //       PassingCalculator.getInstance().getParameters(PassingCalculator.PassingSide.LEFT);
  //   var rightParams =
  //       PassingCalculator.getInstance().getParameters(PassingCalculator.PassingSide.RIGHT);

  //   // Compute angular difference
  //   double leftError =
  //       Math.abs(currentAngle.minus(leftParams.turretAngle()).getRadians());
  //   double rightError =
  //       Math.abs(currentAngle.minus(rightParams.turretAngle()).getRadians());

  //   return leftError < rightError
  //       ? PassingCalculator.PassingSide.LEFT
  //       : PassingCalculator.PassingSide.RIGHT;
  // }

  private void configureButtonBindings() {
    if (Constants.tuningMode) {
      TBC.b().whileTrue(drive.sysIdDriveDynamic(Direction.kForward));
      TBC.x().whileTrue(drive.sysIdDriveDynamic(Direction.kReverse));
      TBC.y().whileTrue(drive.sysIdDriveQuasistatic(Direction.kForward));
      TBC.a().whileTrue(drive.sysIdDriveQuasistatic(Direction.kReverse));

      TBC.povUp().whileTrue(drive.sysIdSteerQuasistatic(Direction.kForward));
      TBC.povDown().whileTrue(drive.sysIdSteerQuasistatic(Direction.kReverse));
      TBC.povRight().whileTrue(drive.sysIdSteerDynamic(Direction.kForward));
      TBC.povLeft().whileTrue(drive.sysIdSteerDynamic(Direction.kReverse));
    } else {
      Trigger hubActiveOrPassing =
          new Trigger(
              () ->
                  HubShiftUtil.getShiftedShiftInfo().active()
                      || LaunchCalculator.getInstance().getParameters().passing());

      TBC.povUp().whileTrue(DriveCommands.wheelRadiusCharacterization(drive));

      TBC.povDown().whileTrue(Commands.runOnce(() -> intakePivot.stow(), intakePivot));

      // TBC.povDown().whileTrue(Commands.runOnce(() -> intakePivot.deploy(), intakePivot));

      // TBC.povUp().whileTrue(Commands.runOnce(() -> intakePivot.stow(), intakePivot));

      TBC.a().whileTrue(turret.runTrackTargetCommand());

      TBC.povRight()
          .whileTrue(
              Commands.parallel(
                  Commands.runEnd(
                      () -> kicker.setGoal(Kicker.Goal.SHOOT),
                      () -> kicker.setGoal(Kicker.Goal.STOP),
                      kicker),
                  Commands.runEnd(
                      () -> hopper.setGoal(Hopper.Goal.SHOOT),
                      () -> hopper.setGoal(Hopper.Goal.STOP),
                      hopper)));

      TBC.povRight()
          .onFalse(
              Commands.parallel(
                      Commands.runEnd(
                          () -> kicker.setGoal(Kicker.Goal.OUTTAKE),
                          () -> kicker.setGoal(Kicker.Goal.STOP),
                          kicker),
                      Commands.runEnd(
                          () -> hopper.setGoal(Hopper.Goal.OUTTAKE),
                          () -> hopper.setGoal(Hopper.Goal.STOP),
                          hopper))
                  .withTimeout(0.5));

      TBC.povLeft().whileTrue(hood.runCharacterizationCommand());

      TBC.x().onTrue(hood.zeroCommand().andThen(turret.zeroCommand()).withName("Zero Shooter"));

      // TBC.povLeft()
      //     .whileTrue(
      //         compPathfindTo(
      //             () ->
      //                 ObjectDetection.getInstance()
      //                     .getDensestFuelClusterPose()
      //                     .orElse(RobotState.getInstance().getEstimatedPose())));

      // TBC.a().whileTrue(turret.runFixedCommand(() -> new Rotation2d(90), () -> 1));
      // TBC.a().whileFalse(turret.idle());

      // TBC.x().onTrue(hood.zeroCommand());
      // .alongWith(turret.zeroCommand()));
      // TBC.x().whileTrue(
      //     Commands.runEnd(
      //         () -> turrethaha.setGoal(Intake.Goal.INTAKE),
      //         () -> turrethaha.setGoal(Intake.Goal.STOP),
      //         turrethaha)
      // );
      // TBC.leftBumper().whileTrue(
      //     Commands.runEnd(
      //         () -> turrethaha.setGoal(Intake.Goal.OUTTAKE),
      //         () -> turrethaha.setGoal(Intake.Goal.STOP),
      //         turrethaha)
      // );

      TBC.b()
          .whileTrue(
              joystickFaceCommand(() -> AllianceFlipUtil.apply(FieldConstants.Hub.hubCenter)));

      TBC.leftTrigger()
          .onTrue(
              Commands.either(
                  Commands.none(),
                  Commands.runOnce(() -> intakePivot.deploy(), intakePivot),
                  () -> intakePivot.isDeployed()));
      TBC.leftTrigger()
          .onFalse(
              Commands.startEnd(
                      () -> intake.setGoal(Intake.Goal.OUTTAKE),
                      () -> intake.setGoal(Goal.STOP),
                      intake)
                  .withTimeout(0.25));

      TBC.leftTrigger()
          .whileTrue(
              Commands.either(
                  Commands.startEnd(
                      () -> intake.setGoal(Intake.Goal.INTAKE),
                      () -> intake.setGoal(Intake.Goal.STOP),
                      intake),
                  Commands.none(),
                  () -> intakePivot.isDeployed()));

      TBC.leftBumper()
          .whileTrue(
              joystickApproach(
                  () ->
                      AllianceFlipUtil.apply(
                          FieldConstants.LeftTrench.getNearestLeftTrench(
                              getFuturePose(alignPredictionSeconds.get())))));

      // TBC.rightBumper()
      //     .whileTrue(
      //         joystickApproach(
      //             () ->
      //                 AllianceFlipUtil.apply(
      //                     FieldConstants.RightTrench.getNearestRightTrench(
      //                         getFuturePose(alignPredictionSeconds.get())))));

      // TBC.a()
      //    .whileTrue(
      //        joystickApproach(
      //            () ->
      //                FieldConstants.Hub.getNearestHubCenter(
      //                    getFuturePose(alignPredictionSeconds.get()))));

      // My magnum opus

      TBC.RightPaddle()
          .whileTrue(
              Commands.either(
                  // AGGRESSIVE
                  compPathfindTo(
                      () -> AllianceFlipUtil.apply(FieldConstants.RightTrench.rightTest)),

                  // If not aggressive → check middle
                  Commands.either(
                      // MIDDLE
                      midLevelPathfindTo(
                          () -> AllianceFlipUtil.apply(FieldConstants.RightTrench.rightTest)),

                      // PASSIVE (fallback)
                      slowPathfindTo(
                          () -> AllianceFlipUtil.apply(FieldConstants.RightTrench.rightTest)),
                      passivePathfinding),
                  aggresivePathfinding));

      TBC.y().toggleOnTrue(flywheel.runTrackTargetCommand());

      TBC.rightTrigger().toggleOnTrue(turret.runTrackTargetActiveLaunchingCommand());

      TBC.rightBumper()
          .whileTrue(Commands.sequence(hood.skipZeroCommand(), hood.runTrackTargetCommand()));

      secondary
          .leftTrigger()
          .whileTrue(
              new ShooterTuningCommand(
                  flywheel, hood, () -> -secondary.getLeftY(), () -> -secondary.getRightY()));

      coast
          .onTrue(
              Commands.runOnce(
                      () -> {
                        if (DriverStation.isDisabled()) {
                          coastOverride = true;
                          // leds.scheduleStateCommand(LEDs.State.DISABLED);
                        }
                      })
                  .withName("Superstructure Coast")
                  .ignoringDisable(true))
          .onFalse(
              Commands.runOnce(
                      () -> {
                        coastOverride = false;
                        // leds.scheduleStateCommand(LEDs.State.NONE);
                      })
                  .withName("Superstructure Uncoast")
                  .ignoringDisable(true));

      Timer teleopElapsedTimer = new Timer();
      RobotModeTriggers.teleop()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    teleopElapsedTimer.restart();
                  }));
      // RobotModeTriggers.teleop()
      //     .and(() -> !(DriverStation.getGameSpecificMessage().length() > 0))
      //     .and(() -> HubShiftUtil.getAllianceWinOverride().isEmpty())
      //     .and(() -> teleopElapsedTimer.hasElapsed(1.0))
      //     .whileTrue(
      //         Commands.runEnd(
      //             () -> {
      //               TBC.setRumble(RumbleType.kBothRumble, 1);
      //               secondary.setRumble(RumbleType.kBothRumble, 1);
      //             },
      //             () -> {
      //               TBC.setRumble(RumbleType.kBothRumble, 0);
      //               secondary.setRumble(RumbleType.kBothRumble, 0);
      //             }))
      //     .whileTrue(
      //         Commands.startEnd(() -> autoWinnerNotSet.set(true), () ->
      // autoWinnerNotSet.set(false)));

      // End-of-shift warning
      for (int i = 1; i <= 5; i++) {
        double time = i;
        Trigger shiftAboutToEnd =
            new Trigger(() -> (HubShiftUtil.getShiftedShiftInfo().remainingTime() < time));
        shiftAboutToEnd
            .and(RobotModeTriggers.teleop())
            .and(ignoreHubState.negate())
            .onTrue(
                Commands.runEnd(
                        () -> TBC.setRumble(RumbleType.kRightRumble, 1.0),
                        () -> TBC.setRumble(RumbleType.kBothRumble, 0.0))
                    .withTimeout(0.25));
      }
    }
  }

  private void configureTestBindings() {
    // ── Phase 1: Open Loop ─────────────────────────────────────────────
    // Hold to jog slowly. Release = stop.
    // Do this with robot on blocks, hand ready to e-stop.
    secondary.a().whileTrue(ShooterTestCommands.hoodOpenLoop(hood, 1.5));
    secondary.b().whileTrue(ShooterTestCommands.hoodOpenLoop(hood, -1.5));
    secondary.x().whileTrue(ShooterTestCommands.turretOpenLoop(turret, 1.5));
    secondary.y().whileTrue(ShooterTestCommands.turretOpenLoop(turret, -1.5));

    // ── Phase 2: Zeroing ───────────────────────────────────────────────
    // Press once — runs to completion automatically.
    secondary
        .start()
        .onTrue(
            ShooterTestCommands.hoodZeroAndReport(hood)
                .andThen(ShooterTestCommands.turretZeroAndReport(turret)));

    // ── Phase 3: Fixed Positions ───────────────────────────────────────
    // Hold button = hold position. Release = idle.
    secondary
        .leftBumper()
        .whileTrue(ShooterTestCommands.hoodGoToAngle(hood, HoodConstants.MIN_ANGLE));
    secondary
        .rightBumper()
        .whileTrue(ShooterTestCommands.hoodGoToAngle(hood, HoodConstants.MAX_ANGLE));
    secondary
        .leftTrigger()
        .whileTrue(ShooterTestCommands.turretGoToAngle(turret, Rotation2d.fromDegrees(-3.33)));
    secondary
        .rightTrigger()
        .whileTrue(ShooterTestCommands.turretGoToAngle(turret, Rotation2d.fromDegrees(24.0)));

    // ── Phase 4: Nudge ─────────────────────────────────────────────────
    // Each press moves 5°. Holds the nudged position.
    secondary.povUp().onTrue(ShooterTestCommands.hoodNudge(hood, 5.0));
    secondary.povDown().onTrue(ShooterTestCommands.hoodNudge(hood, -5.0));
    secondary.povRight().onTrue(ShooterTestCommands.turretNudge(turret, 5.0));
    secondary.povLeft().onTrue(ShooterTestCommands.turretNudge(turret, -5.0));

    // ── Phase 5: Sweep (runs automatically, press once) ────────────────
    // Only bind these after Phase 3 is confirmed working.
    // Comment out until ready.
    // secondary.back().onTrue(ShooterTestCommands.hoodSweep(hood));
    // secondary.back().onTrue(ShooterTestCommands.turretSweep(turret));

    // ── Phase 6: kG Tuning Hold ────────────────────────────────────────
    // Hold to run, release to idle. Tune Hood/kG in Glass while held.
    // secondary.leftStick().whileTrue(
    //     ShooterTestCommands.hoodHoldHorizontal(hood));
  }

  private void configureLEDTriggers() {
    isDisabled
        .onTrue(leds.scheduleStateCommand(LEDs.State.DISABLED))
        .onFalse(leds.unscheduleStateCommand(LEDs.State.DISABLED));
    isAutonomous
        .onTrue(leds.scheduleStateCommand(LEDs.State.RUNNING_AUTO))
        .onFalse(leds.unscheduleStateCommand(LEDs.State.RUNNING_AUTO));

    new Trigger(() -> LaunchCalculator.getInstance().getParameters().isValid())
        .onTrue(leds.scheduleStateCommand(LEDs.State.READY_TO_SHOOT))
        .onFalse(leds.unscheduleStateCommand(LEDs.State.READY_TO_SHOOT));

    new Trigger(() -> intake.getGoal() == Intake.Goal.INTAKE)
        .onTrue(leds.scheduleStateCommand(LEDs.State.RUNNING_INTAKE))
        .onFalse(leds.unscheduleStateCommand(LEDs.State.RUNNING_INTAKE));

    new Trigger(() -> currentPathMode == PathMode.AGGRESSIVE)
        .onTrue(leds.scheduleStateCommand(LEDs.State.PATHFINDING_AGGRESIVE))
        .onFalse(leds.unscheduleStateCommand(LEDs.State.PATHFINDING_AGGRESIVE));

    new Trigger(() -> currentPathMode == PathMode.MID)
        .onTrue(leds.scheduleStateCommand(LEDs.State.PATHFINDING_MID))
        .onFalse(leds.unscheduleStateCommand(LEDs.State.PATHFINDING_MID));

    new Trigger(() -> currentPathMode == PathMode.PASSIVE)
        .onTrue(leds.scheduleStateCommand(LEDs.State.PATHFINDING_PASSIVE))
        .onFalse(leds.unscheduleStateCommand(LEDs.State.PATHFINDING_PASSIVE));

    new Trigger(() -> currentPathMode == PathMode.FUEL)
        .onTrue(leds.scheduleStateCommand(LEDs.State.PATHFINDING_FUEL))
        .onFalse(leds.unscheduleStateCommand(LEDs.State.PATHFINDING_FUEL));
  }

  private void registerNamedCommands() {
    switch (Constants.currentMode) {
      default:

        // Shoot
        NamedCommands.registerCommand(
            "Shoot",
            Commands.parallel(
                    Commands.sequence(
                        Commands.waitSeconds(0.75),
                        Commands.runEnd(
                            () -> kicker.setGoal(Kicker.Goal.SHOOT),
                            () -> kicker.setGoal(Kicker.Goal.STOP),
                            kicker)),
                    Commands.runEnd(
                        () -> hopper.setGoal(Hopper.Goal.SHOOT),
                        () -> hopper.setGoal(Hopper.Goal.STOP),
                        hopper),
                    flywheel.runTrackTargetCommand())
                .withTimeout(10));

        // Intake Out
        NamedCommands.registerCommand(
            "IntakePivotOut", Commands.runOnce(() -> intakePivot.deploy(), intakePivot));

        // Intake In
        NamedCommands.registerCommand(
            "IntakePivotIn", Commands.runOnce(() -> intakePivot.stow(), intakePivot));

        // Intake Roll In
        NamedCommands.registerCommand(
            "IntakeIn",
            Commands.startEnd(
                    () -> intake.setGoal(Intake.Goal.INTAKE),
                    () -> intake.setGoal(Intake.Goal.STOP),
                    intake)
                .withTimeout(1.25));

        NamedCommands.registerCommand(
            "IntakeSlowOutpost",
            Commands.startEnd(
                () -> intake.setGoal(Intake.Goal.OUTPOST),
                () -> intake.setGoal(Intake.Goal.STOP),
                intake));
    }
  }

  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    TBCDisconnected.set(!DriverStation.isJoystickConnected(TBC.getHID().getPort()));
    secondaryDisconnected.set(!DriverStation.isJoystickConnected(secondary.getHID().getPort()));
    overrideDisconnected.set(!overrides.isConnected());
  }

  public void updateFuelCamera() {
    if (fuelCamera == null) return;

    var result = fuelCamera.getLatestResult();

    if (result.hasTargets()) {
      var target = result.getBestTarget();

      ObjectDetection.getInstance()
          .addFuelTxTyObservation(
              new ObjectDetection.FuelTxTyObservation(
                  1,
                  new double[] {target.getYaw()},
                  new double[] {target.getPitch()},
                  result.getTimestampSeconds()));
    }
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
