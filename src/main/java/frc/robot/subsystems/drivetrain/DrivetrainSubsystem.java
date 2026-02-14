// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Mode;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Chassis;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.SysIdUtil;
import frc.robot.util.SysIdUtil.SysIdType;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DrivetrainSubsystem extends SubsystemBase implements Vision.VisionConsumer {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine driveSysId;
  private final SysIdRoutine azimuthSysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // Kinematics
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Chassis.moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  // Motion Profiling
  private final SwerveSetpointGenerator swerveSetpointGenerator =
      new SwerveSetpointGenerator(Chassis.ppConfig, Drive.max_mPs / Chassis.trackRadius_m);
  private SwerveSetpoint lastSetpoint = new SwerveSetpoint(
      new ChassisSpeeds(),
      new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.kZero),
        new SwerveModuleState(0, Rotation2d.kZero),
        new SwerveModuleState(0, Rotation2d.kZero),
        new SwerveModuleState(0, Rotation2d.kZero)
      },
      new DriveFeedforwards(
          new double[] {0, 0, 0, 0},
          new double[] {0, 0, 0, 0},
          new double[] {0, 0, 0, 0},
          new double[] {0, 0, 0, 0},
          new double[] {0, 0, 0, 0}));

  public DrivetrainSubsystem(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    NovaOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        this::setGoalVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        Chassis.ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        // () -> {
        //   return false;
        // },
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback((activePath) -> {
      Logger.recordOutput(
          DrivetrainConstants.NAME + "/Trajectory",
          activePath.toArray(new Pose2d[activePath.size()]));
    });
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
      Logger.recordOutput(DrivetrainConstants.NAME + "/TrajectorySetpoint", targetPose);
    });

    // Configure SysId
    driveSysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput(
                DrivetrainConstants.NAME + "/DriveSysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runDriveSysId(voltage.in(Volts)), null, this));
    azimuthSysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput(
                DrivetrainConstants.NAME + "/AzimuthSysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runAzimuthSysId(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("6_Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving and log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }

      Logger.recordOutput(
          DrivetrainConstants.NAME + "/ModuleSetpoints", new SwerveModuleState[] {});
      Logger.recordOutput(DrivetrainConstants.NAME + "/ChassisSetpoint", new ChassisSpeeds());
      Logger.recordOutput(
          DrivetrainConstants.NAME + "/ModuleSetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(Rotation2d.fromRadians(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && RobotConstants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the next step to reach the desired velocity. Uses 254's
   * SwerveSetpointGenerator.
   *
   * @param speeds_mps Target speeds in meters/sec
   */
  public void setGoalVelocity(ChassisSpeeds speeds_mps) {
    Logger.recordOutput("testspeeds", speeds_mps);
    lastSetpoint = swerveSetpointGenerator.generateSetpoint(lastSetpoint, speeds_mps, 0.02);
    // Log unoptimized setpoints
    Logger.recordOutput(DrivetrainConstants.NAME + "/ModuleSetpoints", lastSetpoint.moduleStates());
    Logger.recordOutput(
        DrivetrainConstants.NAME + "/ChassisSetpoint", lastSetpoint.robotRelativeSpeeds());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].setNextState(
          lastSetpoint.moduleStates()[i], lastSetpoint.feedforwards().accelerationsMPSSq()[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput(
        DrivetrainConstants.NAME + "/ModuleSetpointsOptimized", lastSetpoint.moduleStates());
    // Logger.recordOutput("SwerveSetpoint", lastSetpoint);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runDriveSysId(double output_V) {
    for (int i = 0; i < 4; i++) {
      modules[i].runDriveSysId(output_V);
    }
  }

  public void runAzimuthSysId(double output_V) {
    for (int i = 0; i < 4; i++) {
      modules[i].runAzimuthSysId(output_V);
    }
  }

  /** Stops the drive. */
  public void stop() {
    setGoalVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and azimuths the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = Chassis.moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a drive sysId test with the specified type. */
  public Command getDriveSysId(SysIdType type) {
    return run(() -> runDriveSysId(0.0))
        .withTimeout(1.0)
        .andThen(SysIdUtil.getSysIdCommand(driveSysId, type));
  }

  /** Returns a command to run a drive sysId test with the specified type. */
  public Command getAzimuthSysId(SysIdType type) {
    return SysIdUtil.getSysIdCommand(driveSysId, type);
  }

  /** Returns the module states (azimuth angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = DrivetrainConstants.NAME + "/ModulesMeasured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (azimuth angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = DrivetrainConstants.NAME + "/ChassisMeasured")
  private ChassisSpeeds getChassisSpeeds() {
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

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = DrivetrainConstants.NAME + "/EstimatedPose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void resetOdometry(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  @Override
  public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return Drive.max_mPs;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return Drive.max_mPs / Chassis.trackRadius_m;
  }
}
