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

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import java.io.IOException;
import java.lang.reflect.Field;
import java.net.InetAddress;
import java.net.UnknownHostException;
// import java.nio.file.Files;
// import java.nio.file.Path;
// import java.nio.file.StandardCopyOption;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.tidalforce.frc2026.Constants.Mode;
import org.tidalforce.frc2026.Constants.RobotType;
import org.tidalforce.frc2026.subsystems.shooter.ShotCalculator;
import org.tidalforce.frc2026.util.FuelSim;
import org.tidalforce.frc2026.util.FullSubsystem;
import org.tidalforce.frc2026.util.LocalADStarAK;
import org.tidalforce.frc2026.util.LoggedTracer;
import org.tidalforce.frc2026.util.VirtualSubsystem;

public class Robot extends LoggedRobot {
  private static final double lowBatteryVoltage = 11.0;
  private static final double lowBatteryDisabledTime = 2.0;

  private Command autonomousCommand;
  private double autoStart;
  private boolean autoMessagePrinted;
  private RobotContainer robotContainer;
  private FuelSim fuelSim;
  private StructArrayPublisher<Translation3d> fuelPublisher;

  private final Timer disabledTimer = new Timer();
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, turn off the robot or replace the battery to avoid damage.",
          AlertType.kWarning);

  public Robot() {
    super(Constants.loopPeriodSecs);

    /*--------------------------------------------------------------------------*/
    /* AdvantageKit Metadata                                                    */
    /*--------------------------------------------------------------------------*/

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    Logger.recordMetadata("Robot", Constants.robot.toString());
    Logger.recordMetadata("Mode", Constants.getMode().toString());
    Logger.recordMetadata("LoopPeriodSecs", Double.toString(Constants.loopPeriodSecs));

    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    try {
      Logger.recordMetadata("Hostname", InetAddress.getLocalHost().getHostName());
    } catch (UnknownHostException e) {
      Logger.recordMetadata("Hostname", "Unknown");
    }

    Logger.recordMetadata(
        "Platform",
        "%s %s (%s)"
            .formatted(
                System.getProperty("os.name").replace(" ", ""),
                System.getProperty("os.version"),
                System.getProperty("os.arch")));

    /*--------------------------------------------------------------------------*/
    /* Logging Setup                                                            */
    /*--------------------------------------------------------------------------*/

    switch (Constants.getMode()) {
      case REAL:
        // Log to USB stick
        Logger.addDataReceiver(new WPILOGWriter("/U/logs"));

        // Also publish to NetworkTables for AdvantageScope live viewing
        Logger.addDataReceiver(new NT4Publisher());

        System.out.println("[AdvantageKit] Logging to /U/logs");
        break;

      case SIM:
        // Simulation only publishes to NT
        Logger.addDataReceiver(new NT4Publisher());
        System.out.println("[AdvantageKit] Simulation logging via NT4");
        break;

      case REPLAY:
        // Disable real-time timing so replay runs as fast as possible
        setUseTiming(false);

        // Automatically locate replay log
        // String logPath = LogFileUtil.findReplayLog();
        String logPath = LogFileUtil.findReplayLog();

        Logger.setReplaySource(new WPILOGReader(logPath));

        // Write a new log with replay outputs
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));

        System.out.println("[AdvantageKit] Replaying log:");
        System.out.println(logPath);
        break;
    }

    /*--------------------------------------------------------------------------*/
    /* Start Logger                                                             */
    /*--------------------------------------------------------------------------*/

    Logger.start();

    System.out.println("[AdvantageKit] Logger started");

    /*--------------------------------------------------------------------------*/
    /* Watchdog Configuration                                                   */
    /*--------------------------------------------------------------------------*/

    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(Constants.loopPeriodWatchdogSecs);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to adjust loop watchdog.", false);
    }

    CommandScheduler.getInstance().setPeriod(Constants.loopPeriodWatchdogSecs);

    /*--------------------------------------------------------------------------*/
    /* Driver Station Settings                                                  */
    /*--------------------------------------------------------------------------*/

    DriverStation.silenceJoystickConnectionWarning(true);

    /*--------------------------------------------------------------------------*/
    /* Command Logging                                                          */
    /*--------------------------------------------------------------------------*/

    Map<String, Integer> commandCounts = new HashMap<>();

    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();

          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);

          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);

          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };

    CommandScheduler.getInstance()
        .onCommandInitialize(command -> logCommandFunction.accept(command, true));

    CommandScheduler.getInstance()
        .onCommandFinish(command -> logCommandFunction.accept(command, false));

    CommandScheduler.getInstance()
        .onCommandInterrupt(command -> logCommandFunction.accept(command, false));

    /*--------------------------------------------------------------------------*/
    /* Simulation Setup                                                         */
    /*--------------------------------------------------------------------------*/

    RoboRioSim.setTeamNumber(1721);

    if (Constants.robot == RobotType.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
      DriverStationSim.notifyNewData();
    }

    /*--------------------------------------------------------------------------*/
    /* Initialize Robot                                                         */
    /*--------------------------------------------------------------------------*/

    disabledTimer.restart();

    AutoLogOutputManager.addObject(RobotState.getInstance());

    robotContainer = new RobotContainer();

    // Load default navgrid
    // useNavgrid("navgrid_auto.json");

    // Set AdvantageKit-compatible pathfinder
    Pathfinding.setPathfinder(new LocalADStarAK());

    PathfindingCommand.warmupCommand().schedule();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {

    // Main periodic functions
    LoggedTracer.reset();
    VirtualSubsystem.runAllPeriodic();
    LoggedTracer.record("Commands");
    VirtualSubsystem.runAllPeriodicAfterScheduler();
    FullSubsystem.runAllPeriodicAfterScheduler();
    // LoggedTracer.record("PeriodicAfterScheduler");
    // robotContainer.updateBatteryTelemetry();
    CommandScheduler.getInstance().run();
    robotContainer.updateFuelCamera();

    // Print auto duration
    if (autonomousCommand != null) {
      if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }

    // Low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() > 0.0
        && RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      lowBatteryAlert.set(true);
    }

    // Update RobotContainer dashboard outputs
    robotContainer.updateDashboardOutputs();

    // Log Mechanism3d data
    DevBotMech3d.getMeasured().log("Mechanism3d/Alpha");

    // Record cycle time
    LoggedTracer.record("RobotPeriodic");
  }

  /** Whether to display alerts related to hardware faults. */
  public static boolean showHardwareAlerts() {
    return Constants.getMode() != Mode.SIM && Timer.getTimestamp() > 30.0;
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  // private void useNavgrid(String name) {
  //   try {
  //     Path deploy = Filesystem.getDeployDirectory().toPath();
  //     Path src = deploy.resolve("pathplanner/" + name);
  //     Path dest = deploy.resolve("pathplanner/navgrid.json");

  //     Files.copy(src, dest, StandardCopyOption.REPLACE_EXISTING);

  //     System.out.println("[PathPlanner] Loaded navgrid: " + name);
  //   } catch (IOException e) {
  //     DriverStation.reportError("Failed to load navgrid: " + name, e.getStackTrace());
  //   }
  // }

  @Override
  public void autonomousInit() {
    autoStart = Timer.getTimestamp();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    fuelSim = new FuelSim();

    fuelSim.spawnStartingFuel();
    fuelSim.start();
    fuelSim.registerRobot(
        Units.inchesToMeters(26), // robot width
        Units.inchesToMeters(26), // robot length
        Units.inchesToMeters(22), // bumper height
        () -> robotContainer.drive.getPose(),
        () -> robotContainer.drive.getChassisSpeeds());

    ObjectDetection.setFuelSim(fuelSim);

    fuelPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("FuelSim/Fuels", Translation3d.struct)
            .publish();

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    Pose2d simPose = robotContainer.drive.getPose();
    RobotState.getInstance().resetPose(simPose);
    RobotState.getInstance().setRobotVelocity(robotContainer.drive.getChassisSpeeds());

    ShotCalculator.getInstance().clearShootingParameters();

    // Update fuel physics
    if (fuelSim != null) {
      fuelSim.updateSim();

      // Publish for AdvantageScope
      Translation3d[] allFuel =
          fuelSim.getFuels().stream()
              .map(t -> new Translation3d(t.getX(), t.getY(), 0.2)) // use real Z
              .toArray(Translation3d[]::new);

      fuelPublisher.set(allFuel);
    }
  }
}
