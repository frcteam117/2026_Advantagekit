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

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Chassis;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.SysIdUtil;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.UnitUtil;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.logging.TunableBoolean;
import frc.robot.util.logging.TunableDouble;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DrivetrainCommands {
  private static final String TUNING_NT_KEY = "Tuning/" + DrivetrainConstants.NAME + "/Commands";
  private static final BooleanSupplier TUNABLE =
      new TunableBoolean(TUNING_NT_KEY + "/.Tunable", true);
  private static final double JOYSTICK_DEADBAND = 0.04;
  private static final ProfiledPIDController angleProfiledPID =
      new ProfiledPIDController(6, 0, 0.55, new Constraints(6, 12));
  private static final PIDController anglePID = new PIDController(6, 0, 0.55);
  // private static final double ANGLE_KP = 0.5;
  // private static final double ANGLE_KD = 0;
  // private static final double ANGLE_MAX_VELOCITY = 4.0;
  // private static final double ANGLE_MAX_ACCELERATION = 12.0;
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  // private static final double DRIVE_MAX_ACCELERATION = 6;

  // private static final DoubleSupplier errorOffset =
  //     new TunableDouble(TUNING_NT_KEY + "/errorOffset", 0.15);
  private static final DoubleSupplier DRIVE_MAX_VELOCITY =
      new TunableDouble(TUNING_NT_KEY + "/DRIVE_MAX_VELOCITY", 4);
  private static final DoubleSupplier DRIVE_MAX_ANGULAR_VELOCITY =
      new TunableDouble(TUNING_NT_KEY + "/DRIVE_MAX_ANGULAR_VELOCITY", 15);
  private static final Rotation2d[] X_MODULE_HEADINGS = new Rotation2d[] {
    Chassis.moduleTranslations[0].getAngle(),
    Chassis.moduleTranslations[1].getAngle(),
    Chassis.moduleTranslations[2].getAngle(),
    Chassis.moduleTranslations[3].getAngle()
  };
  public static Rotation2d targetTagRotation2d;
  public static Rotation2d robotRotation2d[];
  public static Rotation2d intendedDirection = new Rotation2d();

  static {
    angleProfiledPID.enableContinuousInput(-Math.PI, Math.PI);
    angleProfiledPID.setTolerance(0.05);
    LogUtil.createTunableProfiledPID(
        TUNING_NT_KEY + "/AngleProfiledPID", angleProfiledPID, TUNABLE);
    anglePID.enableContinuousInput(-Math.PI, Math.PI);
    anglePID.setTolerance(0.05);
    LogUtil.createTunablePID(TUNING_NT_KEY + "/AnglePID", anglePID, TUNABLE);
    // angleController.enableContinuousInput(-Math.PI, Math.PI);
    // angleController.setTolerance(0.025, .3);
    // LogUtil.createTunableProfiledPID(TUNING_NT_KEY + "/AnglePID", angleController, TUNABLE);
  }

  private DrivetrainCommands() {} // Stops DrivetrainCommands from being instantiated

  public static Command stopAndShootToward(
      DrivetrainSubsystem drivetrain,
      Supplier<Translation2d> targetSupplier,
      Supplier<Translation2d> centerOfRotationSupplier) {
    return drivetrain.startRun(
        () -> {
          resetAngleController(drivetrain);
        },
        () -> {
          ChassisSpeeds speeds = new ChassisSpeeds(
              0,
              0,
              angularVelFromAngleProfile(
                  drivetrain.getPose().getRotation(),
                  targetSupplier
                      .get()
                      .minus(drivetrain.getPose().getTranslation())
                      .getAngle()
                      .plus(Rotation2d.k180deg)));
          if (isAutoAimReady(drivetrain)) {
            intendedDirection = drivetrain.getPose().getRotation();
            drivetrain.stopWithHeadings(X_MODULE_HEADINGS);
          } else {
            intendedDirection = drivetrain.getPose().getRotation();
            setGoalVelocity(drivetrain, speeds, centerOfRotationSupplier.get(), false);
          }
        });
  }

  @AutoLogOutput(key = TUNING_NT_KEY + "/isAutoAimReady")
  public static boolean isAutoAimReady(DrivetrainSubsystem drivetrain) {
    // if (!stopAndFacePosition(drivetrain, null)
    //     .equals(CommandScheduler.getInstance().requiring(drivetrain))) {
    //   return false;
    // }
    if (DrivetrainSubsystem.useSwerveSetpointGenerator()) {
      return anglePID.atSetpoint();
    }
    return angleProfiledPID.atSetpoint();
    // return MathUtil.isNear(
    // goalOrientation, drivetrain.getPose().getRotation().getRadians(), 0.02, -Math.PI, Math.PI);
  }

  private static final double[] linearDriveModuleHeadings_rad = new double[] {0.0, 0.0, 0.0, 0.0};

  /** Returns a command to run a drive sysId test with the specified type. */
  public static Command getLinearDriveSysId(DrivetrainSubsystem drivetrain, SysIdType type) {
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.75).per(Second),
            Volts.of(2),
            Seconds.of(1.5),
            (state) -> Logger.recordOutput(
                DrivetrainConstants.NAME + "/LinearDriveSysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) ->
                drivetrain.setDriveVoltage(voltage.in(Volts), linearDriveModuleHeadings_rad),
            null,
            drivetrain));
    return drivetrain
        .run(() -> drivetrain.setDriveVoltage(0.0, linearDriveModuleHeadings_rad))
        .withTimeout(1.0)
        .andThen(SysIdUtil.getSysIdCommand(routine, type));
  }

  // private static final double[] angularDriveModuleHeadings_rad = new double[] {
  //   Chassis.moduleTranslations[0].minus(Chassis.cmPosition).getAngle().getRadians() + Math.PI /
  // 2,
  //   Chassis.moduleTranslations[1].minus(Chassis.cmPosition).getAngle().getRadians() + Math.PI /
  // 2,
  //   Chassis.moduleTranslations[2].minus(Chassis.cmPosition).getAngle().getRadians() + Math.PI /
  // 2,
  //   Chassis.moduleTranslations[3].minus(Chassis.cmPosition).getAngle().getRadians() + Math.PI / 2
  // };

  // /** Returns a command to run a drive sysId test with the specified type. */
  // public static Command getAngularDriveSysId(DrivetrainSubsystem drivetrain, SysIdType type) {
  //   SysIdRoutine routine = new SysIdRoutine(
  //         new SysIdRoutine.Config(
  //             Volts.of(0.6).per(Second),
  //             Volts.of(2),
  //             Seconds.of(2),
  //             (state) -> Logger.recordOutput(
  //                 DrivetrainConstants.NAME + "/AngularDriveSysIdState", state.toString())),
  //         new SysIdRoutine.Mechanism(
  //             (voltage) -> drivetrain.setDriveVoltage(voltage.in(Volts),
  // angularDriveModuleHeadings_rad), null, drivetrain));
  //   return drivetrain
  //       .run(() -> drivetrain.setDriveVoltage(0.0, angularDriveModuleHeadings_rad))
  //       .withTimeout(1.0)
  //       .andThen(SysIdUtil.getSysIdCommand(routine, type));
  // }

  /** Returns a command to run a drive sysId test with the specified type. */
  public static Command getAzimuthSysId(DrivetrainSubsystem drivetrain, SysIdType type) {
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.4).per(Second),
            Volts.of(4),
            Seconds.of(5),
            (state) -> Logger.recordOutput(
                DrivetrainConstants.NAME + "/AzimuthSysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> drivetrain.setAzimuthVoltage(voltage.in(Volts)), null, drivetrain));
    return SysIdUtil.getSysIdCommand(routine, type);
  }

  private static final double joystickDrive_velTolerance = .03;
  private static double joystickDrive_target_rad;

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      DrivetrainSubsystem drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Translation2d> centerOfRotationSupplier,
      BooleanSupplier driveAssistSupplier) {
    return drivetrain
        .run(() -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), JOYSTICK_DEADBAND);

          // Square rotation value for more precise control
          omega = MathUtil.copyDirectionPow(omega, 2);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * DRIVE_MAX_VELOCITY.getAsDouble(),
              linearVelocity.getY() * DRIVE_MAX_VELOCITY.getAsDouble(),
              omega * DRIVE_MAX_ANGULAR_VELOCITY.getAsDouble());
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              isFlipped
                  ? drivetrain.getPose().getRotation().plus(Rotation2d.k180deg)
                  : drivetrain.getPose().getRotation());

          if (Math.abs(speeds.omegaRadiansPerSecond) > joystickDrive_velTolerance) {
            resetAngleController(drivetrain);
            joystickDrive_target_rad = drivetrain.getPose().getRotation().getRadians();
            setGoalVelocity(
                drivetrain,
                speeds,
                centerOfRotationSupplier.get(),
                driveAssistSupplier.getAsBoolean());
          } else {
            speeds.omegaRadiansPerSecond = angularVelFromAngleProfile(
                drivetrain.getPose().getRotation(),
                Rotation2d.fromRadians(joystickDrive_target_rad));
            setGoalVelocity(
                drivetrain,
                speeds,
                centerOfRotationSupplier.get(),
                driveAssistSupplier.getAsBoolean());
          }
        })
        .beforeStarting(() -> {
          resetAngleController(drivetrain);
          joystickDrive_target_rad = drivetrain.getPose().getRotation().getRadians();
        })
        .withName("JoystickDrive");
    // return drivetrain.run(() -> {
    //   // Get linear velocity
    //   Translation2d linearVelocity =
    //       getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    //   // Apply rotation deadband
    //   double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), JOYSTICK_DEADBAND);

    //   // Square rotation value for more precise control
    //   omega = MathUtil.copyDirectionPow(omega, 2);

    //   // Convert to field relative speeds & send command
    //   ChassisSpeeds speeds = new ChassisSpeeds(
    //       linearVelocity.getX() * DRIVE_MAX_VELOCITY.getAsDouble(),
    //       linearVelocity.getY() * DRIVE_MAX_VELOCITY.getAsDouble(),
    //       omega * DRIVE_MAX_ANGULAR_VELOCITY);
    //   boolean isFlipped = DriverStation.getAlliance().isPresent()
    //       && DriverStation.getAlliance().get() == Alliance.Red;
    //   speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    //       speeds,
    //       isFlipped
    //           ? drivetrain.getPose().getRotation().plus(Rotation2d.k180deg)
    //           : drivetrain.getPose().getRotation());
    //   if (Math.abs(speeds.omegaRadiansPerSecond) < 0.01
    //       && Math.abs(speeds.vxMetersPerSecond) < 0.01
    //       && Math.abs(speeds.vyMetersPerSecond) < 0.01) {
    //     drivetrain.stopWithHeadings(X_MODULE_HEADINGS);
    //   } else {
    //     // 7 in extra from intake
    //     // move center of rotation by 3.5 in in the x direction
    //     // omega cross r = v
    //     // r is (-3.5 in, 0.0 in) and omega is vertical, so positive omega results in negative y
    //     // velocity at the old center of rotation
    //     speeds.vyMetersPerSecond = speeds.vyMetersPerSecond
    //         - speeds.omegaRadiansPerSecond
    //             * UnitUtil.inTom(3.5)
    //             * pivotFractionLoweredSupplier.getAsDouble();
    //     drivetrain.setGoalVelocity(speeds);
    //   }
    // });
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      DrivetrainSubsystem drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier,
      Supplier<Translation2d> centerOfRotationSupplier,
      BooleanSupplier driveAssistSupplier) {

    // Construct command
    return drivetrain
        .run(() -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * DRIVE_MAX_VELOCITY.getAsDouble(),
              linearVelocity.getY() * DRIVE_MAX_VELOCITY.getAsDouble(),
              angularVelFromAngleProfile(
                  drivetrain.getPose().getRotation(),
                  isFlipped
                      ? rotationSupplier.get().plus(Rotation2d.k180deg)
                      : rotationSupplier.get()));
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              isFlipped
                  ? drivetrain.getPose().getRotation().plus(Rotation2d.k180deg)
                  : drivetrain.getPose().getRotation());
          setGoalVelocity(
              drivetrain,
              speeds,
              centerOfRotationSupplier.get(),
              driveAssistSupplier.getAsBoolean());
        })
        // Reset PID controller when command starts
        .beforeStarting(() -> resetAngleController(drivetrain));
  }

  public static Command joystickNormalTurning(
      DrivetrainSubsystem drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      Supplier<Translation2d> centerOfRotationSupplier,
      BooleanSupplier driveAssistSupplier) {

    return drivetrain
        .run(() -> {
          // Apply deadband and power curve to rotation input
          double omegaInput =
              MathUtil.applyDeadband(rotationSupplier.getAsDouble(), JOYSTICK_DEADBAND);
          omegaInput = MathUtil.copyDirectionPow(omegaInput, 2);

          // Calculate desired feedforward angular velocity in rad/s
          double omegaFeedforward = omegaInput * DRIVE_MAX_ANGULAR_VELOCITY.getAsDouble();

          // Integrate target position step using loop period (0.02s)
          intendedDirection = intendedDirection.plus(new Rotation2d(omegaFeedforward * 0.02));

          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

          // Pass BOTH target position (intendedDirection) AND feedforward velocity
          // (omegaFeedforward)
          // into angularVelFromAngleProfile via TrapezoidProfile.State
          double angularVelocity = angularVelFromAngleProfile(
              drivetrain.getPose().getRotation(),
              new TrapezoidProfile.State(intendedDirection.getRadians(), omegaFeedforward));

          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * DRIVE_MAX_VELOCITY.getAsDouble(),
              linearVelocity.getY() * DRIVE_MAX_VELOCITY.getAsDouble(),
              angularVelocity);

          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              isFlipped
                  ? drivetrain.getPose().getRotation().plus(Rotation2d.k180deg)
                  : drivetrain.getPose().getRotation());

          setGoalVelocity(
              drivetrain,
              speeds,
              centerOfRotationSupplier.get(),
              driveAssistSupplier.getAsBoolean());
        })
        .beforeStarting(() -> {
          resetAngleController(drivetrain);
          intendedDirection = drivetrain.getPose().getRotation(); // Synchronize heading on start
        });
  }

  public static Command joystickNormallTurning(
      DrivetrainSubsystem drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      Supplier<Translation2d> centerOfRotationSupplier,
      BooleanSupplier driveAssistSupplier) {
    return joystickNormalTurning(
        drivetrain,
        xSupplier,
        ySupplier,
        rotationSupplier,
        centerOfRotationSupplier,
        driveAssistSupplier);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveFacingTarget(
      DrivetrainSubsystem drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> targetPosSupplier,
      Supplier<Translation2d> targetVelSupplier,
      Supplier<Translation2d> centerOfRotationSupplier,
      BooleanSupplier driveAssistSupplier) {

    // Construct command
    return drivetrain
        .run(() -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

          Translation2d robotCenteredTarget =
              targetPosSupplier.get().minus(drivetrain.getPose().getTranslation());
          double omegaFF = robotCenteredTarget.cross(targetVelSupplier
                  .get()
                  .minus(new Translation2d(
                          drivetrain.getChassisSpeeds().vxMetersPerSecond,
                          drivetrain.getChassisSpeeds().vyMetersPerSecond)
                      .rotateBy(drivetrain.getPose().getRotation())))
              / (robotCenteredTarget.getNorm() * robotCenteredTarget.getNorm());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * DRIVE_MAX_VELOCITY.getAsDouble(),
              linearVelocity.getY() * DRIVE_MAX_VELOCITY.getAsDouble(),
              angularVelFromAngleProfile(
                  drivetrain.getPose().getRotation().plus(Rotation2d.k180deg),
                  new TrapezoidProfile.State(
                      robotCenteredTarget.getAngle().getRadians(), omegaFF)));
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              isFlipped
                  ? drivetrain.getPose().getRotation().plus(Rotation2d.k180deg)
                  : drivetrain.getPose().getRotation());
          setGoalVelocity(
              drivetrain,
              speeds,
              centerOfRotationSupplier.get(),
              driveAssistSupplier.getAsBoolean());
        })
        // Reset PID controller when command starts
        .beforeStarting(() -> resetAngleController(drivetrain));
  }

  private static Rotation2d joystickDriveAtAngle_rotTarget;
  private static double[] snapToAngle1Targets_deg = new double[] {
    0, // 30, 60,
    90, // 120, 150,
    180, // 210, 240,
    270, // 300, 330,
    360
  };
  private static double snapToAngle2TargetsDegFrom90 = 20;
  private static double[] snapToAngle2Targets_deg = new double[] {
    snapToAngle2TargetsDegFrom90,
    90 - snapToAngle2TargetsDegFrom90,
    90 + snapToAngle2TargetsDegFrom90,
    180 - snapToAngle2TargetsDegFrom90,
    180 + snapToAngle2TargetsDegFrom90,
    270 - snapToAngle2TargetsDegFrom90,
    270 + snapToAngle2TargetsDegFrom90,
    360 - snapToAngle2TargetsDegFrom90
  };
  private static double[] snapToAngle1targetBorders_deg = new double[] { // 15,
    45, // 75, 105,
    135, // 165, 195,
    225, // 255, 285,
    315, // 345, 375,
    405
  };
  private static double[] snapToAngle2targetBorders_deg =
      new double[] {45, 90, 135, 180, 225, 270, 315, 360};

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      DrivetrainSubsystem drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotXSupplier,
      DoubleSupplier rotYSupplier,
      double deadband,
      BooleanSupplier snapToAngle1,
      BooleanSupplier snapToAngle2,
      Supplier<Translation2d> centerOfRotationSupplier,
      BooleanSupplier driveAssistSupplier) {

    // Construct command
    return drivetrain
        .run(() -> {
          boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
          if (Math.hypot(rotYSupplier.getAsDouble(), rotXSupplier.getAsDouble()) > deadband) {
            joystickDriveAtAngle_rotTarget = Rotation2d.fromRadians(
                Math.atan2(rotYSupplier.getAsDouble(), rotXSupplier.getAsDouble()));
            if (snapToAngle2.getAsBoolean()) {
              double snapToAngleJoystickTarget_deg =
                  MathUtil.inputModulus(joystickDriveAtAngle_rotTarget.getDegrees(), 0, 360);
              for (int i = 0; i < snapToAngle2targetBorders_deg.length; i++) {
                if (snapToAngleJoystickTarget_deg < snapToAngle2targetBorders_deg[i]) {
                  joystickDriveAtAngle_rotTarget =
                      Rotation2d.fromDegrees(snapToAngle2Targets_deg[i]);
                  break;
                }
              }
            } else if (snapToAngle1.getAsBoolean()) {
              double snapToAngleJoystickTarget_deg =
                  MathUtil.inputModulus(joystickDriveAtAngle_rotTarget.getDegrees(), 0, 360);
              for (int i = 0; i < snapToAngle1targetBorders_deg.length; i++) {
                if (snapToAngleJoystickTarget_deg < snapToAngle1targetBorders_deg[i]) {
                  joystickDriveAtAngle_rotTarget =
                      Rotation2d.fromDegrees(snapToAngle1Targets_deg[i]);
                  break;
                }
              }
            }
          } else {
            if (snapToAngle1.getAsBoolean()) {
              joystickDriveAtAngle_rotTarget = MathUtil.isNear(
                      0,
                      drivetrain.getPose().getRotation().getRadians(),
                      Math.PI / 2,
                      -Math.PI,
                      Math.PI)
                  ? Rotation2d.kZero
                  : Rotation2d.k180deg;
              joystickDriveAtAngle_rotTarget = joystickDriveAtAngle_rotTarget.plus(
                  isFlipped ? Rotation2d.k180deg : Rotation2d.kZero);
            }
          }

          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * DRIVE_MAX_VELOCITY.getAsDouble(),
              linearVelocity.getY() * DRIVE_MAX_VELOCITY.getAsDouble(),
              angularVelFromAngleProfile(
                  drivetrain.getPose().getRotation(),
                  isFlipped
                      ? joystickDriveAtAngle_rotTarget.plus(Rotation2d.k180deg)
                      : joystickDriveAtAngle_rotTarget));
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              isFlipped
                  ? drivetrain.getPose().getRotation().plus(Rotation2d.k180deg)
                  : drivetrain.getPose().getRotation());
          setGoalVelocity(
              drivetrain,
              speeds,
              centerOfRotationSupplier.get(),
              driveAssistSupplier.getAsBoolean());
        })
        // Reset PID controller when command starts
        .beforeStarting(() -> {
          resetAngleController(drivetrain);
          if (Math.hypot(rotYSupplier.getAsDouble(), rotXSupplier.getAsDouble()) > deadband) {
            joystickDriveAtAngle_rotTarget = Rotation2d.fromRadians(
                Math.atan2(rotYSupplier.getAsDouble(), rotXSupplier.getAsDouble()));
          } else {
            joystickDriveAtAngle_rotTarget = drivetrain
                .getPose()
                .getRotation()
                .plus(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero);
          }
        });
  }
  // TODO: ask max about tuning rot PID
  public static Command joystickDriveAtAngleRegularTurning(
      DrivetrainSubsystem drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotXSupplier,
      DoubleSupplier rotYSupplier,
      double deadband,
      BooleanSupplier snapToAngle1,
      BooleanSupplier snapToAngle2,
      Supplier<Translation2d> centerOfRotationSupplier,
      BooleanSupplier driveAssistSupplier) {

    // Construct command
    return drivetrain.run(() -> {
      boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
      if (Math.hypot(rotYSupplier.getAsDouble(), rotXSupplier.getAsDouble()) > deadband) {
        joystickDriveAtAngle_rotTarget = Rotation2d.fromRadians(
            Math.atan2(rotYSupplier.getAsDouble(), rotXSupplier.getAsDouble()));
        if (snapToAngle2.getAsBoolean()) {
          double snapToAngleJoystickTarget_deg =
              MathUtil.inputModulus(joystickDriveAtAngle_rotTarget.getDegrees(), 0, 360);
          for (int i = 0; i < snapToAngle2targetBorders_deg.length; i++) {
            if (snapToAngleJoystickTarget_deg < snapToAngle2targetBorders_deg[i]) {
              joystickDriveAtAngle_rotTarget = Rotation2d.fromDegrees(snapToAngle2Targets_deg[i]);
              break;
            }
          }
        } else if (snapToAngle1.getAsBoolean()) {
          double snapToAngleJoystickTarget_deg =
              MathUtil.inputModulus(joystickDriveAtAngle_rotTarget.getDegrees(), 0, 360);
          for (int i = 0; i < snapToAngle1targetBorders_deg.length; i++) {
            if (snapToAngleJoystickTarget_deg < snapToAngle1targetBorders_deg[i]) {
              joystickDriveAtAngle_rotTarget = Rotation2d.fromDegrees(snapToAngle1Targets_deg[i]);
              break;
            }
          }
        }
      } else {
        if (snapToAngle1.getAsBoolean()) {
          joystickDriveAtAngle_rotTarget = MathUtil.isNear(
                  0,
                  drivetrain.getPose().getRotation().getRadians(),
                  Math.PI / 2,
                  -Math.PI,
                  Math.PI)
              ? Rotation2d.kZero
              : Rotation2d.k180deg;
          joystickDriveAtAngle_rotTarget = joystickDriveAtAngle_rotTarget.plus(
              isFlipped ? Rotation2d.k180deg : Rotation2d.kZero);
        }
      }

      // Get linear velocity
      Translation2d linearVelocity =
          getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

      // Convert to field relative speeds & send command
      Logger.recordOutput("rotYSupplier", rotXSupplier.getAsDouble());
      double turningSpeed =
          rotXSupplier.getAsDouble() * DRIVE_MAX_ANGULAR_VELOCITY.getAsDouble() * 0.3;
      turningSpeed = MathUtil.applyDeadband(turningSpeed, deadband);
      if (snapToAngle2.getAsBoolean()) {
        Rotation2d currentRot = drivetrain.getPose().getRotation();
        Rotation2d allianceRot = currentRot.plus(isFlipped ? Rotation2d.k180deg : Rotation2d.kZero);
        Rotation2d targetAllianceRot = (Math.abs(allianceRot.getRadians()) <= Math.PI / 2)
            ? Rotation2d.kZero
            : Rotation2d.k180deg;
        Rotation2d targetRot =
            targetAllianceRot.plus(isFlipped ? Rotation2d.k180deg : Rotation2d.kZero);
        turningSpeed = angularVelFromAngleProfile(currentRot, targetRot);
      } else {
        turningSpeed -= (drivetrain.getChassisSpeeds().omegaRadiansPerSecond - turningSpeed);
      }
      ChassisSpeeds speeds = new ChassisSpeeds(
          linearVelocity.getX() * DRIVE_MAX_VELOCITY.getAsDouble(),
          linearVelocity.getY() * DRIVE_MAX_VELOCITY.getAsDouble(),
          turningSpeed);
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds,
          isFlipped
              ? drivetrain.getPose().getRotation().plus(Rotation2d.k180deg)
              : drivetrain.getPose().getRotation());
      setGoalVelocity(
          drivetrain, speeds, centerOfRotationSupplier.get(), driveAssistSupplier.getAsBoolean());
    });
  }
  ;

  private Translation2d shootWhileMoving_shooterVelocity = new Translation2d();

  public static Command shootWhileMoving(
      DrivetrainSubsystem drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      BooleanSupplier driveAssistSupplier) {

    return Commands.none();
  }

  private static final DoubleSupplier pathOverBump_VELOCITY =
      new TunableDouble(TUNING_NT_KEY + "pathOverBump_VELOCITY", 2);
  private static final DoubleSupplier pathOverBump_DEGREES_FROM_ALIGNED =
      new TunableDouble(TUNING_NT_KEY + "pathOverBump_DEGREES_FROM_ALIGNED", 20);
  private static Rotation2d pathOverBump_angleOffset_rad;
  private static Rotation2d pathOverBump_angleTarget;
  private static boolean pathOverBump_driveForward;
  private static boolean pathOverBump_onBump = false;
  private static boolean pathOverBump_overBump = false;
  private static final Debouncer pathOverBump_debouncer = new Debouncer(.1, DebounceType.kRising);

  public static Command pathOverBump(DrivetrainSubsystem drivetrain) {
    return drivetrain
        .startRun(
            () -> {
              pathOverBump_onBump = false;
              pathOverBump_overBump = false;
              double yawMod = MathUtil.inputModulus(
                  drivetrain.getPose().getRotation().getRadians(), 0, 2 * Math.PI);
              pathOverBump_angleOffset_rad =
                  drivetrain.getPose().getRotation().minus(drivetrain.getNavXYaw());
              pathOverBump_angleTarget = Rotation2d.fromDegrees(
                  yawMod < Math.PI
                      ? (yawMod < Math.PI / 2
                          ? pathOverBump_DEGREES_FROM_ALIGNED.getAsDouble()
                          : 180 - pathOverBump_DEGREES_FROM_ALIGNED.getAsDouble())
                      : (yawMod < 3 * Math.PI / 2
                          ? 180 + pathOverBump_DEGREES_FROM_ALIGNED.getAsDouble()
                          : 360 - pathOverBump_DEGREES_FROM_ALIGNED.getAsDouble()));
              if (drivetrain.getPose().getX() < 8.05) {
                pathOverBump_driveForward = drivetrain.getPose().getX() < 4.7;
              } else {
                pathOverBump_driveForward = drivetrain.getPose().getX() < 11.4;
              }
              resetAngleController(drivetrain);
            },
            () -> {
              boolean isLevel =
                  drivetrain.getNavXAngleFromHorizontal().isNear(Radians.zero(), Radians.of(0.07));
              if (!isLevel) {
                pathOverBump_onBump = true;
              }
              if (pathOverBump_onBump && pathOverBump_debouncer.calculate(isLevel)) {
                pathOverBump_overBump = true;
              }
              Rotation2d robotOrientation =
                  drivetrain.getNavXYaw().plus(pathOverBump_angleOffset_rad);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds = new ChassisSpeeds(
                  pathOverBump_driveForward
                      ? pathOverBump_VELOCITY.getAsDouble()
                      : -pathOverBump_VELOCITY.getAsDouble(),
                  0.0,
                  angularVelFromAngleProfile(robotOrientation, pathOverBump_angleTarget));

              speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotOrientation);
              drivetrain.setGoalVelocity(speeds, new Translation2d());
            })
        .onlyIf(() -> {
          double x = drivetrain.getPose().getX();
          return (x > 1.7 && x < 7.7) || (x > 8.4 && x < 14.4);
        })
        .finallyDo(() -> drivetrain.resetTranslationWithVision())
        .until(() -> pathOverBump_overBump);
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(DrivetrainSubsystem drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> {
              limiter.reset(0.0);
            }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.setGoalVelocity(new ChassisSpeeds(0.0, 0.0, speed), new Translation2d());
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(() -> {
              state.positions = drive.getWheelRadiusCharacterizationPositions();
              state.lastAngle = drive.getNavXYaw();
              state.gyroDelta = 0.0;
            }),

            // Update gyro delta
            Commands.run(() -> {
                  var rotation = drive.getNavXYaw();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                })

                // When cancelled, calculate and print results
                .finallyDo(() -> {
                  double[] positions = drive.getWheelRadiusCharacterizationPositions();
                  double wheelDelta = 0.0;
                  for (int i = 0; i < 4; i++) {
                    wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                  }
                  double wheelRadius = (state.gyroDelta * Chassis.trackRadius_m) / wheelDelta;

                  NumberFormat formatter = new DecimalFormat("#0.000");
                  System.out.println("********** Wheel Radius Characterization Results **********");
                  System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                  System.out.println(
                      "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                  System.out.println("\tWheel Radius: "
                      + formatter.format(wheelRadius)
                      + " meters, "
                      + formatter.format(Units.metersToInches(wheelRadius))
                      + " inches");
                })));
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), JOYSTICK_DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    // linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  private static void resetAngleController(DrivetrainSubsystem drivetrain) {
    anglePID.reset();
    angleProfiledPID.reset(new State(
        drivetrain.getPose().getRotation().getRadians(),
        drivetrain.getChassisSpeeds().omegaRadiansPerSecond));
  }

  // private static final Translation2d[] driveAssistRobotCorners = new Translation2d[] {
  //   new Translation2d(
  //       (DrivetrainConstants.Chassis.bumperLength_m / 2) + UnitUtil.inTom(4),
  //       DrivetrainConstants.Chassis.bumperWidth_m / 2),
  //   new Translation2d(
  //       (DrivetrainConstants.Chassis.bumperLength_m / 2) + UnitUtil.inTom(4),
  //       -DrivetrainConstants.Chassis.bumperWidth_m / 2),
  //   new Translation2d(
  //       -(DrivetrainConstants.Chassis.bumperLength_m / 2),
  //       DrivetrainConstants.Chassis.bumperWidth_m / 2),
  //   new Translation2d(
  //       -(DrivetrainConstants.Chassis.bumperLength_m / 2),
  //       -DrivetrainConstants.Chassis.bumperWidth_m / 2),
  // };
  // private static final DriveAssistWall[] DRIVE_ASSIST_MAP = new DriveAssistWall[] {
  //   new DriveAssistWall(
  //       new Translation2d(0, 0),
  //       new Translation2d(8, 0),
  //       new Translation2d(1, Rotation2d.kCCW_90deg),
  //       false),
  //   new DriveAssistWall(
  //       new Translation2d(0, 0),
  //       new Translation2d(0, 5),
  //       new Translation2d(1, Rotation2d.kZero),
  //       false),
  //   new DriveAssistWall(
  //       new Translation2d(8, 0),
  //       new Translation2d(8, 5),
  //       new Translation2d(1, Rotation2d.k180deg),
  //       false),
  //   new DriveAssistWall(
  //       new Translation2d(0, 5),
  //       new Translation2d(8, 5),
  //       new Translation2d(1, Rotation2d.kCW_90deg),
  //       false)
  // };
  private static double prevRobotOmega = 0;

  private static void setGoalVelocity(
      DrivetrainSubsystem drivetrain,
      ChassisSpeeds speeds,
      Translation2d centerOfRotation,
      boolean driveAssist) {
    // if (Math.abs(speeds.omegaRadiansPerSecond) < joystickDrive_velTolerance
    //     && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
    //         < joystickDrive_velTolerance) {
    // drivetrain.stopWithHeadings(X_MODULE_HEADINGS);
    // } else {
    // Translation2d discretizedLinVelFromAngVel = centerOfRotation
    //     .minus(centerOfRotation.rotateBy(
    //         Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * RobotConstants.CODE_PERIOD_s)))
    //     .div(RobotConstants.CODE_PERIOD_s);
    // speeds.vxMetersPerSecond += discretizedLinVelFromAngVel.getX();
    // speeds.vyMetersPerSecond += discretizedLinVelFromAngVel.getY();
    // if (driveAssist) {
    //   final Translation2d robotVelocity =
    //       new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    //   // Translation2d[] cornerSpeeds = new Translation2d[4];
    //   Translation2d[] fieldCenteredRobotCorners = new Translation2d[4];
    //   Translation2d robotSpeedsDelta = new Translation2d();
    //   // List<DriveAssistWall> relavantMap = new ArrayList<>();

    //   for (int i = 0; i < 4; i++) {
    //     fieldCenteredRobotCorners[i] = driveAssistRobotCorners[i]
    //         .rotateBy(drivetrain.getPose().getRotation())
    //         .plus(drivetrain.getPose().getTranslation());
    //   }

    //   final double maxFreeDistance = DRIVE_MAX_VELOCITY.getAsDouble()
    //       * DRIVE_MAX_VELOCITY.getAsDouble()
    //       / (2 * DRIVE_MAX_ACCELERATION);
    //   double xmin = Math.min(
    //       Math.min(fieldCenteredRobotCorners[0].getX(), fieldCenteredRobotCorners[1].getX()),
    //       Math.min(fieldCenteredRobotCorners[2].getX(), fieldCenteredRobotCorners[3].getX()));
    //   if (xmin < maxFreeDistance
    //       && robotVelocity.getX() < -Math.sqrt(2 * DRIVE_MAX_ACCELERATION * Math.max(0, xmin)))
    // {
    //     robotSpeedsDelta = new Translation2d(
    //         Math.sqrt(2 * DRIVE_MAX_ACCELERATION * Math.max(0, xmin)) - robotVelocity.getX(),
    //         robotSpeedsDelta.getY());
    //   }
    //   double xmax = Math.max(
    //       Math.max(fieldCenteredRobotCorners[0].getX(), fieldCenteredRobotCorners[1].getX()),
    //       Math.max(fieldCenteredRobotCorners[2].getX(), fieldCenteredRobotCorners[3].getX()));
    //   if (xmax > 16.5 - maxFreeDistance
    //       && robotVelocity.getX()
    //           > Math.sqrt(2 * DRIVE_MAX_ACCELERATION * Math.max(0, 16.5 - xmax))) {
    //     robotSpeedsDelta = new Translation2d(
    //         -Math.sqrt(2 * DRIVE_MAX_ACCELERATION * Math.max(0, 16.5 - xmax))
    //             - robotVelocity.getX(),
    //         robotSpeedsDelta.getY());
    //   }
    //   double ymin = Math.min(
    //       Math.min(fieldCenteredRobotCorners[0].getY(), fieldCenteredRobotCorners[1].getY()),
    //       Math.min(fieldCenteredRobotCorners[2].getY(), fieldCenteredRobotCorners[3].getY()));
    //   if (ymin < maxFreeDistance
    //       && robotVelocity.getY() < -Math.sqrt(2 * DRIVE_MAX_ACCELERATION * Math.max(0, ymin)))
    // {
    //     robotSpeedsDelta = new Translation2d(
    //         robotSpeedsDelta.getX(),
    //         Math.sqrt(2 * DRIVE_MAX_ACCELERATION * Math.max(0, ymin)) - robotVelocity.getY());
    //   }
    //   double ymax = Math.max(
    //       Math.max(fieldCenteredRobotCorners[0].getY(), fieldCenteredRobotCorners[1].getY()),
    //       Math.max(fieldCenteredRobotCorners[2].getY(), fieldCenteredRobotCorners[3].getY()));
    //   if (ymax > 8.1 - maxFreeDistance
    //       && robotVelocity.getY()
    //           > Math.sqrt(2 * DRIVE_MAX_ACCELERATION * Math.max(0, 8.1 - ymax))) {
    //     robotSpeedsDelta = new Translation2d(
    //         robotSpeedsDelta.getX(),
    //         -Math.sqrt(2 * DRIVE_MAX_ACCELERATION * Math.max(0, 8.1 - ymax))
    //             - robotVelocity.getY());
    //   }
    //   speeds.plus(new ChassisSpeeds(robotSpeedsDelta.getX(), robotSpeedsDelta.getY(), 0));
    // }
    // double scaleFactor = Math.max(
    //     1.0,
    //     Math.max(
    //         speeds.omegaRadiansPerSecond / DRIVE_MAX_ANGULAR_VELOCITY,
    //         Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) /
    // DRIVE_MAX_VELOCITY.getAsDouble()));
    // speeds.times(1 / scaleFactor);
    final double tempPrevRobotOmega = speeds.omegaRadiansPerSecond;
    prevRobotOmega = tempPrevRobotOmega;
    drivetrain.setGoalVelocity(speeds, centerOfRotation);
    // }
  }

  // public static record DriveAssistWall(
  //     Translation2d p1, Translation2d p2, Translation2d unitNormalVector, boolean checkCorners)
  // {}

  public static Translation2d pivotBasedCenterOfRotation(Angle pivotPos) {
    return new Translation2d(UnitUtil.inTom(3.5), 0.0)
        .times(1
            - MathUtil.inverseInterpolate(
                IntakeConstants.Pivot.MIN_POS.in(Radians),
                IntakeConstants.Pivot.MAX_POS.in(Radians),
                pivotPos.in(Radians)));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  private static final double[] prevAngleSetpoints = new double[3];
  private static TrapezoidProfile.State angleSetpoint = new TrapezoidProfile.State();

  private static double angularVelFromAngleProfile(
      Rotation2d robotOrientation, Rotation2d goalOrientation) {
    return angularVelFromAngleProfile(
        robotOrientation, new TrapezoidProfile.State(goalOrientation.getRadians(), 0));
  }

  private static double goalOrientation = 0;

  private static double angularVelFromAngleProfile(
      Rotation2d robotOrientation, TrapezoidProfile.State goalState) {
    // Math from ProfiledPIDController
    // Get error which is the smallest distance between goal and measurement
    // double errorBound = Math.PI;
    // double goalMinDistance = MathUtil.inputModulus(
    //     goalState.position - robotOrientation.getRadians(), -errorBound, errorBound);
    // double setpointMinDistance = MathUtil.inputModulus(
    //     angleSetpoint.position - robotOrientation.getRadians(), -errorBound, errorBound);

    // // Recompute the profile goal with the smallest error, thus giving the shortest path. The
    // goal
    // // may be outside the input range after this operation, but that's OK because the controller
    // // will still go there and report an error of zero. In other words, the setpoint only needs
    // to
    // // be offset from the measurement by the input range modulus; they don't need to be equal.
    // goalState.position = goalMinDistance + robotOrientation.getRadians();
    // angleSetpoint.position = setpointMinDistance + robotOrientation.getRadians();

    // angleSetpoint = angleProfile.calculate(RobotConstants.CODE_PERIOD_s, angleSetpoint,
    // goalState);
    // Logger.recordOutput(DrivetrainConstants.NAME + "/AngleProfile", angleSetpoint);
    goalOrientation = goalState.position;
    // double error = MathUtil.inputModulus(
    //     goalState.position - robotOrientation.getRadians(), -Math.PI, Math.PI);
    // double output = MathUtil.clamp(
    //         Math.copySign(
    //             Math.sqrt(2
    //                 * (Math.abs(error) < errorOffset.getAsDouble()
    //                     ? Math.max(0, (Math.abs(error) - 0.005))
    //                         * DRIVE_MAX_ANGULAR_ACC2.getAsDouble()
    //                     : Math.max(
    //                         0,
    //                         (Math.abs(error) - errorOffset.getAsDouble())
    //                             * DRIVE_MAX_ANGULAR_ACC.getAsDouble()))),
    //             error),
    //         -DRIVE_MAX_ANGULAR_VELOCITY.getAsDouble(),
    //         DRIVE_MAX_ANGULAR_VELOCITY.getAsDouble())
    //     + goalState.velocity;
    double output = MathUtil.clamp(
        (DrivetrainSubsystem.useSwerveSetpointGenerator()
            ? anglePID.calculate(robotOrientation.getRadians(), goalState.position)
            : angleProfiledPID.calculate(robotOrientation.getRadians(), goalState.position)),
        -DRIVE_MAX_ANGULAR_VELOCITY.getAsDouble(),
        DRIVE_MAX_ANGULAR_VELOCITY.getAsDouble());
    Logger.recordOutput(
        DrivetrainConstants.NAME + "/AngleProfile/Error",
        (DrivetrainSubsystem.useSwerveSetpointGenerator()
            ? anglePID.getError()
            : angleProfiledPID.getPositionError()));
    prevAngleSetpoints[0] = prevAngleSetpoints[1];
    prevAngleSetpoints[1] = prevAngleSetpoints[2];
    prevAngleSetpoints[2] = angleSetpoint.position;
    return output
        + (DrivetrainSubsystem.useSwerveSetpointGenerator()
            ? goalState.velocity
            : angleProfiledPID.getSetpoint().velocity);
  }

  private static TrapezoidProfile.State prevAngleState = new TrapezoidProfile.State();

  // public
}
