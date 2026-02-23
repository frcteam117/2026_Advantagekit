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

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Chassis;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.Drive;
import frc.robot.util.SysIdUtil;
import frc.robot.util.SysIdUtil.SysIdType;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DrivetrainCommands {
  private static final double JOYSTICK_DEADBAND = 0.04;
  private static final double ANGLE_KP = 0.5;
  private static final double ANGLE_KD = 0;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static final double DRIVE_MAX_VELOCITY = Drive.max_mPs;
  private static final double DRIVE_MAX_ANGULAR_VELOCITY = Drive.max_mPs / Chassis.trackRadius_m;
  private static final Rotation2d[] X_MODULE_HEADINGS = new Rotation2d[] {
    Chassis.moduleTranslations[0].getAngle(),
    Chassis.moduleTranslations[1].getAngle(),
    Chassis.moduleTranslations[2].getAngle(),
    Chassis.moduleTranslations[3].getAngle()
  };

  private DrivetrainCommands() {} // Stops DrivetrainCommands from being instantiated

  /** Returns a command to run a drive sysId test with the specified type. */
  public static Command getDriveSysId(DrivetrainSubsystem drivetrain, SysIdType type) {
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput(
                DrivetrainConstants.NAME + "/DriveSysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> drivetrain.setForwardDriveVoltage(voltage.in(Volts)), null, drivetrain));
    return drivetrain
        .run(() -> drivetrain.setForwardDriveVoltage(0.0))
        .withTimeout(1.0)
        .andThen(SysIdUtil.getSysIdCommand(routine, type));
  }

  /** Returns a command to run a drive sysId test with the specified type. */
  public static Command getAzimuthSysId(DrivetrainSubsystem drivetrain, SysIdType type) {
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput(
                DrivetrainConstants.NAME + "/AzimuthSysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> drivetrain.setAzimuthVoltage(voltage.in(Volts)), null, drivetrain));
    return SysIdUtil.getSysIdCommand(routine, type);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      DrivetrainSubsystem drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return drivetrain.run(() -> {
      // Get linear velocity
      Translation2d linearVelocity =
          getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

      // Apply rotation deadband
      double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), JOYSTICK_DEADBAND);

      // Square rotation value for more precise control
      omega = MathUtil.copyDirectionPow(omega, 2);

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds = new ChassisSpeeds(
          linearVelocity.getX() * DRIVE_MAX_VELOCITY,
          linearVelocity.getY() * DRIVE_MAX_VELOCITY,
          omega * DRIVE_MAX_ANGULAR_VELOCITY);
      boolean isFlipped = DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red;
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds,
          isFlipped
              ? drivetrain.getPose().getRotation().plus(Rotation2d.k180deg)
              : drivetrain.getPose().getRotation());
      if (speeds.equals(new ChassisSpeeds())) {
        drivetrain.stopWithHeadings(X_MODULE_HEADINGS);
      } else {
        drivetrain.setGoalVelocity(speeds);
      }
    });
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
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController = new ProfiledPIDController(
        ANGLE_KP,
        0.0,
        ANGLE_KD,
        new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return drivetrain
        .run(() -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Calculate angular speed
          double omega = angleController.calculate(
              drivetrain.getPose().getRotation().getRadians(),
              rotationSupplier.get().getRadians());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * DRIVE_MAX_VELOCITY,
              linearVelocity.getY() * DRIVE_MAX_VELOCITY,
              omega);
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              isFlipped
                  ? drivetrain.getPose().getRotation().plus(new Rotation2d(Math.PI))
                  : drivetrain.getPose().getRotation());
          drivetrain.setGoalVelocity(speeds);
        })

        // Reset PID controller when command starts
        .beforeStarting(
            () -> angleController.reset(drivetrain.getPose().getRotation().getRadians()));
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
                  drive.setGoalVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(() -> {
              state.positions = drive.getWheelRadiusCharacterizationPositions();
              state.lastAngle = drive.getPose().getRotation();
              state.gyroDelta = 0.0;
            }),

            // Update gyro delta
            Commands.run(() -> {
                  var rotation = drive.getPose().getRotation();
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
    double linearMagnitude =
        MathUtil.clamp(MathUtil.applyDeadband(Math.hypot(x, y), JOYSTICK_DEADBAND), -1, 1);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  // public static Command pathToReef(Supplier<Pose2d> poseSupplier, BooleanSupplier getButton) {
  //   Pose2d waypoint;
  //   Pose2d waypoint1;
  //   Pose2d waypoint2;
  //   double x = poseSupplier.get().getX();
  //   double y = poseSupplier.get().getY();

  //   if (x < 4.489) {
  //     if (y < (0.5774 * x) + 1.4343) {
  //       waypoint = new Pose2d(3.389, 2.1207, Rotation2d.fromDegrees(60));
  //       waypoint1 = new Pose2d(3.534, 2.373, Rotation2d.fromDegrees(60));
  //       waypoint2 = new Pose2d(3.809, 2.849, Rotation2d.fromDegrees(60));
  //     } else if (y < (-0.5774 * x) + 6.6177) {
  //       waypoint = new Pose2d(2.289, 4.026, Rotation2d.fromDegrees(0));
  //       waypoint1 = new Pose2d(2.58, 4.026, Rotation2d.fromDegrees(0));
  //       waypoint2 = new Pose2d(3.13, 4.026, Rotation2d.fromDegrees(0));
  //     } else {
  //       waypoint = new Pose2d(3.389, 5.93126, Rotation2d.fromDegrees(300));
  //       waypoint1 = new Pose2d(3.534, 5.679, Rotation2d.fromDegrees(300));
  //       waypoint2 = new Pose2d(3.809, 5.203, Rotation2d.fromDegrees(300));
  //     }
  //   } else if (x < 8.775) {
  //     if (y < (-0.5774 * x) + 6.6177) {
  //       waypoint = new Pose2d(5.589, 2.12074, Rotation2d.fromDegrees(120));
  //       waypoint1 = new Pose2d(5.444, 2.373, Rotation2d.fromDegrees(120));
  //       waypoint2 = new Pose2d(5.169, 2.849, Rotation2d.fromDegrees(120));
  //     } else if (y < (0.5774 * x) + 1.4343) {
  //       waypoint = new Pose2d(6.689, 4.026, Rotation2d.fromDegrees(180));
  //       waypoint1 = new Pose2d(6.398, 4.026, Rotation2d.fromDegrees(180));
  //       waypoint2 = new Pose2d(5.848, 4.026, Rotation2d.fromDegrees(180));
  //     } else {
  //       waypoint = new Pose2d(5.589, 5.93126, Rotation2d.fromDegrees(240));
  //       waypoint1 = new Pose2d(5.444, 5.679, Rotation2d.fromDegrees(240));
  //       waypoint2 = new Pose2d(5.169, 5.203, Rotation2d.fromDegrees(240));
  //     }
  //   } else if (x > 13.061) {
  //     if (y < (-0.5774 * x) + 11.56677) {
  //       waypoint = new Pose2d(14.161, 2.12074, Rotation2d.fromDegrees(120));
  //       waypoint1 = new Pose2d(14.016, 2.373, Rotation2d.fromDegrees(120));
  //       waypoint2 = new Pose2d(13.741, 2.849, Rotation2d.fromDegrees(120));
  //     } else if (y < (0.5774 * x) - 3.51477) {
  //       waypoint = new Pose2d(15.261, 4.026, Rotation2d.fromDegrees(180));
  //       waypoint1 = new Pose2d(14.97, 4.026, Rotation2d.fromDegrees(180));
  //       waypoint2 = new Pose2d(14.42, 4.026, Rotation2d.fromDegrees(180));
  //     } else {
  //       waypoint = new Pose2d(14.161, 5.93126, Rotation2d.fromDegrees(240));
  //       waypoint1 = new Pose2d(14.016, 5.679, Rotation2d.fromDegrees(240));
  //       waypoint2 = new Pose2d(13.741, 5.203, Rotation2d.fromDegrees(240));
  //     }
  //   } else if (x > 8.775) {
  //     if (y < (0.5774 * x) - 3.51477) {
  //       waypoint = new Pose2d(11.961, 2.12074, Rotation2d.fromDegrees(60));
  //       waypoint1 = new Pose2d(12.106, 2.373, Rotation2d.fromDegrees(60));
  //       waypoint2 = new Pose2d(12.381, 2.849, Rotation2d.fromDegrees(60));
  //     } else if (y < (-0.5774 * x) + 11.56677) {
  //       waypoint = new Pose2d(10.861, 4.026, Rotation2d.fromDegrees(0));
  //       waypoint1 = new Pose2d(11.152, 4.026, Rotation2d.fromDegrees(0));
  //       waypoint2 = new Pose2d(11.702, 4.026, Rotation2d.fromDegrees(0));
  //     } else {
  //       waypoint = new Pose2d(11.961, 5.93126, Rotation2d.fromDegrees(300));
  //       waypoint1 = new Pose2d(12.106, 5.679, Rotation2d.fromDegrees(300));
  //       waypoint2 = new Pose2d(12.381, 5.203, Rotation2d.fromDegrees(300));
  //     }
  //   } else {
  //     return Commands.none();
  //   }

  //   PathConstraints constraints = new PathConstraints(5, 8, 12, 15, 12);
  //   PathConstraints constraintsSlow = new PathConstraints(1.8, 1.2, 3, 2, 12);
  //   PathPlannerPath path = new PathPlannerPath(
  //       PathPlannerPath.waypointsFromPoses(waypoint1, waypoint2),
  //       constraints,
  //       null,
  //       new GoalEndState(0.0, waypoint2.getRotation()),
  //       false);
  //   path.preventFlipping = true;
  //   // PathPlannerPath path =
  //   //     new PathPlannerPath(
  //   //         waypoints,
  //   //         constraints,
  //   //         null, // The ideal starting state, this is only relevant for pre-planned paths, so
  //   // can
  //   //         // be null for on-the-fly paths.
  //   //         new GoalEndState(
  //   //             0.0,
  //   //             Rotation2d.fromDegrees(
  //   //                 0)) // Goal end state. You can set a holonomic rotation here. If using a
  //   //         // differential drivetrain, the rotation will have no effect.
  //   //         );
  //   // path.preventFlipping = true;
  //   // Command pathCommand = AutoBuilder.pathfindToPose(waypoint, constraints,
  //   // constraintsSlow.maxVelocity());
  //   // PathPlannerAuto pathAuto = new PathPlannerAuto(pathCommand);
  //   // if (algaePosition == 2) {
  //   // pathAuto
  //   //     .nearFieldPosition(waypoint1.getTranslation(), 0.2)
  //   //     .onTrue(
  //   //         AutoBuilder.pathfindThenFollowPath(path, constraintsSlow));
  //   // .alongWith(elevator.commandElevatorTo(ElevatorConstants.posAlgaeOnReef2))
  //   // .alongWith(arm.commandArmTo(ArmConstants.posDown))
  //   // .alongWith(intake.intakeAlgae())
  //   // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
  //   // .until(
  //   //     () -> {
  //   //       return !getButton.getAsBoolean();
  //   //     }));
  //   // } else {
  //   // .until(() -> {return !getButton.getAsBoolean();})
  //   //     );
  //   // }
  //   // pathAuto
  //   // pathAuto.until(
  //   //     () -> {
  //   //       return !getButton.getAsBoolean();
  //   //     });

  //   // WrapperCommand wrapperAuto = pathAuto.finallyDo(() -> {
  //   //     List<Waypoint> waypointsSlow = PathPlannerPath.waypointsFromPoses(poseSupplier.get(),
  //   // waypoint1,
  //   // waypoint2);
  //   //     PathPlannerPath pathSlow = new PathPlannerPath(
  //   //             waypointsSlow,
  //   //             constraintsSlow,
  //   //             null, // The ideal starting state, this is only relevant for pre-planned
  //   //             // paths, so
  //   //             // can
  //   //             // be null for on-the-fly paths.
  //   //             new GoalEndState(
  //   //                     0.0,
  //   //                     waypoint2.getRotation()) // Goal end state. You can set a holonomic
  //   // rotation here. If
  //   // using
  //   //             // a
  //   //             // differential drivetrain, the rotation will have no effect.
  //   //             );
  //   //     pathSlow.preventFlipping = true;
  //   //     Command pathSlowCommand = AutoBuilder.followPath(pathSlow);
  //   //     // .alongWith(elevator.commandElevatorTo(ElevatorConstants.posAlgaeOnReef1))
  //   //     // .alongWith(arm.commandArmTo(ArmConstants.posDown))
  //   //     // .alongWith(intake.intakeAlgae())
  //   //     ParallelRaceGroup pathSlowCommandGroup = pathSlowCommand
  //   //             .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
  //   //             .onlyWhile(getButton);

  //   //     pathSlowCommandGroup.schedule();
  //   // });
  //   // ParallelRaceGroup finalPath = wrapperAuto
  //   //         .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
  //   //         .until(pathAuto.nearFieldPosition(waypoint1.getTranslation(), 0.65))
  //   //         .onlyWhile(getButton);
  //   return AutoBuilder.pathfindThenFollowPath(path, constraints).onlyWhile(getButton);
  // }
}
