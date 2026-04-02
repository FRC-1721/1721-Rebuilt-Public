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

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.tidalforce.frc2026.RobotState;
import org.tidalforce.frc2026.generated.TunerConstants;
import org.tidalforce.frc2026.util.LocalADStarAK;
import org.tidalforce.frc2026.util.LoggedTunableNumber;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  public static final LoggedTunableNumber kPTranslation =
      new LoggedTunableNumber("PathPlanner/kPTranslation", 5);
  public static final LoggedTunableNumber kDTranslation =
      new LoggedTunableNumber("PathPlanner/kDTranslation", 0.2);
  public static final LoggedTunableNumber kPRotation =
      new LoggedTunableNumber("PathPlanner/kPRotation", 5);
  public static final LoggedTunableNumber kDRotation =
      new LoggedTunableNumber("PathPlanner/kDRotation", 0.2);

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 49.9;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final Field2d m_field = new Field2d();
  private final Pigeon2 m_gryo;

  private Pigeon2SimState m_gyrosim;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysIdDrive;
  private final SysIdRoutine sysIDSteer;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private PPHolonomicDriveController ppController = buildPPController();

  private PPHolonomicDriveController buildPPController() {
    return new PPHolonomicDriveController(
        new PIDConstants(kPTranslation.get(), 0.0, kDTranslation.get()),
        new PIDConstants(kPRotation.get(), 0.0, kDRotation.get()));
  }

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  private final DifferentialDrivetrainSim sim =
      DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);
    m_gryo = new Pigeon2(0);

    SmartDashboard.putData(m_field);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        ppController,
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId

    // In the Drive constructor, replace the existing sysId instantiation:
    sysIdDrive =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second), // ramp rate: 0.5 V/s for quasistatic
                Volts.of(3.0), // step voltage for dynamic test
                Second.of(10.0), // timeout
                (state) -> Logger.recordOutput("Drive/SysId/DriveState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  // Lock all modules to 0° and apply voltage to drive motors only
                  for (var module : modules) {
                    module.runSysIdDrive((voltage.in(Volts)));
                  }
                },
                null,
                this));

    sysIDSteer =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second), // ramp rate
                Volts.of(3.0), // step voltage
                Second.of(10.0), // timeout
                (state) -> Logger.recordOutput("Drive/SysId/SteerState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  // Hold drive motors still, apply voltage to steer motors only
                  for (var module : modules) {
                    module.runSysIdSteer(voltage.in(Volts));
                  }
                },
                null,
                this));

    if (RobotBase.isSimulation()) {
      m_gyrosim = m_gryo.getSimState();
    }
  }

  public void run(double xAxis, double zAxis) {
    sim.setInputs((xAxis - zAxis) * 12.0, (xAxis + zAxis) * 12.0);
  }

  @Override
  public void simulationPeriodic() {
    if (RobotBase.isSimulation() && m_gyrosim != null) {

      // 1. Get the current chassis speeds from your odometry or physics engine.
      // The SwerveDriveKinematics object can convert current module *states* back into *chassis
      // speeds*.
      // The result will contain an 'omega' field in RADIANS per second.
      ChassisSpeeds currentSpeeds =
          kinematics.toChassisSpeeds(
              modules[0].getState(),
              modules[1].getState(),
              modules[2].getState(),
              modules[3].getState());

      // 2. Get the angular velocity (omega) in radians/second
      double omegaRadiansPerSec = currentSpeeds.omegaRadiansPerSecond;

      // 3. Convert radians/second to degrees/second
      double omegaDegreesPerSec = Math.toDegrees(omegaRadiansPerSec);

      // 4. Add the incremental change to the Pigeon Sim State
      // We multiply by the loop time (0.02s or 20ms) to get the delta angle for this cycle.
      m_gyrosim.addYaw(omegaDegreesPerSec * 0.020);

      // Pose2d simPose = getPose();

      // // 1. Update RobotState with the simulated pose
      // RobotState.getInstance().resetPose(simPose);
    }
  }

  @Override
  public void periodic() {
    odometryLock.lock();

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }

    // Cache the FULL array reference inside the lock, not just the length.
    // The odometry thread cannot replace these arrays while we hold the lock,
    // and after we release it we use our local references only — never the
    // getters — so there is no race condition.
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;

    // Cache per-module position arrays and gyro samples while lock is still held
    SwerveModulePosition[][] allModulePositions = new SwerveModulePosition[4][];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      allModulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions();
    }

    // Cache gyro samples — same reasoning
    Rotation2d[] gyroSamples = gyroInputs.odometryYawPositions;

    odometryLock.unlock();
    // ── Lock released — use only cached references below, never getters ──

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // ── Single correct odometry update loop ──────────────────────────────
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        // Guard against mismatched array lengths between modules
        if (i < allModulePositions[moduleIndex].length) {
          modulePositions[moduleIndex] = allModulePositions[moduleIndex][i];
        } else {
          modulePositions[moduleIndex] = lastModulePositions[moduleIndex];
        }
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Use per-sample gyro angle — eliminates accumulated heading drift.
      // Falls back to the latest reading if gyro queue ran short.
      if (gyroInputs.connected && gyroSamples != null && i < gyroSamples.length) {
        rawGyroRotation = gyroSamples[i];
      } else {
        rawGyroRotation = gyroInputs.yawPosition;
      }

      poseEstimator.updateWithTime(
          sampleTimestamps[i], // use cached reference, not the getter
          rawGyroRotation,
          modulePositions);
    }

    // Visualization, alerts, RobotState, logging below — unchanged
    m_field.setRobotPose(getPose());
    gyroDisconnectedAlert.set(!gyroInputs.connected && !RobotBase.isSimulation());
    RobotState.getInstance().resetPose(getPose());

    if (kPTranslation.hasChanged(hashCode())
        || kDTranslation.hasChanged(hashCode())
        || kPRotation.hasChanged(hashCode())
        || kDRotation.hasChanged(hashCode())) {
      ppController = buildPPController();
    }

    Pose2d pose = getPose();
    Logger.recordOutput(
        "Odometry/Robot3d",
        new Pose3d(pose.getX(), pose.getY(), 0.0, new Rotation3d(pose.getRotation())));
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.05);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // Drive characterization commands — bind these to controller buttons
  public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> {
          for (var module : modules) {
            module.runSysIdDrive(0.0); // pre-orient wheels forward
          }
        })
        .withTimeout(1.0)
        .andThen(sysIdDrive.quasistatic(direction))
        .withName("SysId Drive Quasistatic " + direction);
  }

  public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
    return run(() -> {
          for (var module : modules) {
            module.runSysIdDrive(0.0);
          }
        })
        .withTimeout(1.0)
        .andThen(sysIdDrive.dynamic(direction))
        .withName("SysId Drive Dynamic " + direction);
  }

  public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> {
          for (var module : modules) {
            module.runSysIdSteer(0.0);
          }
        })
        .withTimeout(1.0)
        .andThen(sysIDSteer.quasistatic(direction))
        .withName("SysId Steer Quasistatic " + direction);
  }

  public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
    return run(() -> {
          for (var module : modules) {
            module.runSysIdSteer(0.0);
          }
        })
        .withTimeout(1.0)
        .andThen(sysIDSteer.dynamic(direction))
        .withName("SysId Steer Dynamic " + direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    if (poseEstimator == null) {
      throw new IllegalStateException("PoseEstimator is not initialized.");
    }
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    if (poseEstimator == null) {
      throw new IllegalStateException("PoseEstimator is not initialized.");
    }
    poseEstimator.resetPosition(
        getRotation2d(), // Ensure this method is implemented and returns a valid Rotation2d
        getModulePositions(), // Ensure this method is implemented and returns valid module
        // positions
        pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  private Rotation2d getRotation2d() {
    if (gyroInputs.connected) {
      return gyroInputs.yawPosition;
    } else {
      // Fallback to kinematics-based heading estimation
      ChassisSpeeds chassisSpeeds = getChassisSpeeds();
      double deltaThetaRadians = chassisSpeeds.omegaRadiansPerSecond / ODOMETRY_FREQUENCY;
      return rawGyroRotation.plus(new Rotation2d(deltaThetaRadians));
    }
  }
}
