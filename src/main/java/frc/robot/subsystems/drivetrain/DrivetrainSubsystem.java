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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Chassis;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Drive;
import frc.robot.subsystems.drivetrain.GyroIO.GyroIOInputs;
import frc.robot.util.LocalADStarAK;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DrivetrainSubsystem extends SubsystemBase {
  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  // Pose estimation
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Chassis.moduleTranslations);
  private Rotation2d rawGyroYaw = Rotation2d.kZero;
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroYaw, lastModulePositions, new Pose2d());
  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  // Motion Profiling
  private final SwerveSetpointGenerator swerveSetpointGenerator =
      new SwerveSetpointGenerator(Chassis.ppConfig, 15);
  private SwerveSetpoint lastSetpoint;

  // Module heading control
  private boolean controllingHeadings = false;
  private final Rotation2d[] goalHeadings = new Rotation2d[4];

  // Pose resetting
  private Pose2d prevVisionPose = null;
  private boolean resetPoseWithVision = true;
  private boolean resetTranslationWithVision = false;
  private final double poseLinTolerance = Drive.max_mPs * 0.1;
  private final double poseAngTolerance = Drive.max_mPs / Chassis.trackRadius_m * 0.1;

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

    // Start odometry thread
    NovaOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    lastSetpoint =
        new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        this::setGoalVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.4), new PIDConstants(6, 0, 0.55)),
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
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
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
        rawGyroYaw = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroYaw = rawGyroYaw.plus(Rotation2d.fromRadians(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroYaw, modulePositions);

      DrivetrainCommands.updatePoses(getPose()); // maybe get this from somewhere that also considers the vison est???
    }
  }

  /**
   * Runs the drive at the next step to reach the desired velocity. Uses pathplanner's
   * SwerveSetpointGenerator.
   *
   * @param goalSpeeds_mps Target speeds in meters/sec
   */
  public void setGoalVelocity(ChassisSpeeds goalSpeeds_mps) {
    Logger.recordOutput(DrivetrainConstants.NAME + "/1_Goal/Chassis", goalSpeeds_mps);
    // swerve setpoint generator
    lastSetpoint = swerveSetpointGenerator.generateSetpoint(lastSetpoint, goalSpeeds_mps, 0.02);

    Logger.recordOutput(
        DrivetrainConstants.NAME + "/2_Next/Chassis", lastSetpoint.robotRelativeSpeeds());
    Logger.recordOutput(DrivetrainConstants.NAME + "/2_Next/Modules", lastSetpoint.moduleStates());
    Logger.recordOutput(
        DrivetrainConstants.NAME + "/2_Next/Acc_mPs2",
        lastSetpoint.feedforwards().accelerationsMPSSq());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      if (controllingHeadings
          && lastSetpoint.moduleStates()[i].speedMetersPerSecond < 1e-6
          && lastSetpoint.feedforwards().accelerationsMPSSq()[i] < 1e-6) {
        // double headingError_rad = MathUtil.angleModulus(
        //     goalHeadings[i].minus(lastSetpoint.moduleStates()[i].angle).getRadians());
        // lastSetpoint.moduleStates()[i].angle = lastSetpoint.moduleStates()[i].angle.plus(
        //     Rotation2d.fromRadians(Math.copySign(
        //         Math.min(15 * RobotConstants.CODE_PERIOD_s, Math.abs(headingError_rad)),
        //         headingError_rad)));
        lastSetpoint.moduleStates()[i].angle = goalHeadings[i];
      }
      modules[i].setNextState(
          lastSetpoint.moduleStates()[i], lastSetpoint.feedforwards().accelerationsMPSSq()[i]);
    }
    if (!controllingHeadings) {
      SwerveModuleState[] goalModules = new SwerveModuleState[4];
      Arrays.fill(goalModules, new SwerveModuleState(Double.NaN, new Rotation2d(Double.NaN)));
      Logger.recordOutput(DrivetrainConstants.NAME + "/1_Goal/Modules", goalModules);
    } else {
      controllingHeadings = false;
    }

    // DrivetrainCommands.resetAngleProfileFromSetpointGenerator(lastSetpoint);
  }

  /**
   * Stops the drive and azimuths the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithHeadings(Rotation2d[] goalHeadings) {
    Logger.recordOutput(
        DrivetrainConstants.NAME + "/1_Goal/Modules",
        Arrays.stream(goalHeadings)
            .map(rotation -> new SwerveModuleState(0, rotation))
            .toArray(SwerveModuleState[]::new));
    controllingHeadings = true;
    for (int i = 0; i < 4; i++) {
      if (!MathUtil.isNear(
          modules[i].getAngle(), goalHeadings[i].getRadians(), Math.PI / 2, -Math.PI, Math.PI)) {
        goalHeadings[i] = goalHeadings[i].rotateBy(Rotation2d.k180deg);
      }
      this.goalHeadings[i] = goalHeadings[i];
    }
    setGoalVelocity(new ChassisSpeeds());
  }

  // /** Runs the drive motors at the specified voltage while controlling the heading with pure
  // feedback. */
  public void setDriveVoltage(double output_V, double[] headings_rad) {
    for (int i = 0; i < 4; i++) {
      modules[i].runDriveVoltage(output_V, headings_rad[i]);
    }
  }

  public void setAzimuthVoltage(double output_V) {
    for (int i = 0; i < 4; i++) {
      modules[i].runAzimuthVoltage(output_V);
    }
  }

  /** Returns the module states (azimuth angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = DrivetrainConstants.NAME + "/0_Measured/Modules")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = DrivetrainConstants.NAME + "/0_Measured/Chassis")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = DrivetrainConstants.NAME + "/EstimatedPose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Resets the current odometry pose. */
  public void resetOdometry(Pose2d pose) {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = modules[i].getPosition();
    }
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroYaw, modulePositions, pose);
  }

  /** Returns the robot's orientation according to the NavX. */
  public Rotation3d getNavXOrientation() {
    return gyroInputs.orientation;
  }

  /** Returns the robot's angular velocity according to the NavX. */
  public AngularVelocity[] getNavXAngularVel() {
    return gyroInputs.angularVel;
  }

  /** Returns the robot's linear acceleration according to the NavX. */
  public LinearAcceleration[] getNavXLinearAcc() {
    return gyroInputs.linearAcc;
  }

  /** Returns the NavX yaw angle. */
  public Rotation2d getNavXYaw() {
    return gyroInputs.orientation.toRotation2d();
  }

  /** Returns the robot's angle from horizontal according to the NavX. */
  public Angle getNavXAngleFromHorizontal() {
    return gyroInputs.angleFromHorizontal;
  }

  /** Adds a new timestamped vision measurement. */
  public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    if (prevVisionPose != null) {
      if (resetPoseWithVision
          && visionRobotPoseMeters.getTranslation().getDistance(prevVisionPose.getTranslation())
              < poseLinTolerance
          && MathUtil.isNear(
              prevVisionPose.getRotation().getRadians(),
              visionRobotPoseMeters.getRotation().getRadians(),
              poseAngTolerance,
              0,
              2 * Math.PI)) {
        poseEstimator.resetPose(visionRobotPoseMeters);
        resetPoseWithVision = false;
        resetTranslationWithVision = false;
      } else if (resetTranslationWithVision
          && visionRobotPoseMeters.getTranslation().getDistance(prevVisionPose.getTranslation())
              < poseLinTolerance) {
        poseEstimator.resetPose(new Pose2d(
            visionRobotPoseMeters.getTranslation(),
            poseEstimator.getEstimatedPosition().getRotation()));
        resetTranslationWithVision = false;
      }
    }
    prevVisionPose = visionRobotPoseMeters;
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters,
        timestampSeconds,
        gyroInputs.connected
            ? visionMeasurementStdDevs
            : VecBuilder.fill(
                visionMeasurementStdDevs.get(0, 0) / 3,
                visionMeasurementStdDevs.get(1, 0) / 3,
                visionMeasurementStdDevs.get(2, 0) / 8));
    // TODO: when should this be run? Max's answer: DrivetrainSubsystem.accept() should be run each
    // timestep that photonvision was able to predict the robot pose
  }

  public void resetPoseWithVision() {
    resetPoseWithVision = true;
  }

  public void resetTranslationWithVision() {
    resetTranslationWithVision = true;
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }
}
