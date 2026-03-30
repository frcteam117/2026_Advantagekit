package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.FieldType;
import frc.robot.subsystems.drivetrain.DrivetrainCommands;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerCommands;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotCommands {
  private static boolean isAutoShooting = false;
  private static final Translation2d blueHub = RobotConstants.FIELD_TYPE.equals(FieldType.WELDED)
      ? new Translation2d(0.0254 * (182.11), 0.0254 * (317.69 / 2))
      : new Translation2d(0.0254 * (181.56), 0.0254 * (316.64 / 2));
  private static final Translation2d redHub = RobotConstants.FIELD_TYPE.equals(FieldType.WELDED)
      ? new Translation2d(0.0254 * (651.22 - 182.11), 0.0254 * (317.69 / 2))
      : new Translation2d(0.0254 * (650.12 - 181.56), 0.0254 * (316.64 / 2));
  private static final Translation2d redLPassingTarget = new Translation2d(13.1, 6.1);
  private static final Translation2d redRPassingTarget = new Translation2d(13.1, 2);
  private static final Translation2d blueLPassingTarget = new Translation2d(3, 6.6);
  private static final Translation2d blueRPassingTarget = new Translation2d(3, 1.5);

  public static Command autoAim(
      DrivetrainSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      Supplier<Translation2d> centerOfRotationSupplier,
      BooleanSupplier shootWhenReady) {
    return Commands.defer(
        () -> {
          boolean isBlueAlliance =
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
          boolean isPassing = isBlueAlliance
              ? drivetrain.getPose().getX() > 4.7
              : drivetrain.getPose().getX() < 11.4;
          return Commands.parallel(
                  ShooterCommands.autoAim(
                      shooter,
                      drivetrain::getPose,
                      () -> getTarget(drivetrain.getPose()),
                      () -> isPassing,
                      () -> false),
                  DrivetrainCommands.stopAndShootToward(
                      drivetrain, () -> getTarget(drivetrain.getPose()), centerOfRotationSupplier),
                  IndexerCommands.conditionalRunForward(
                      indexer,
                      () -> shootWhenReady.getAsBoolean()
                          && ShooterCommands.isAutoAimReady(shooter)
                          && DrivetrainCommands.isAutoAimReady(drivetrain)),
                  Commands.run(() -> IntakeCommands.shooting = shootWhenReady.getAsBoolean()
                      && ShooterCommands.isAutoAimReady(shooter)
                      && DrivetrainCommands.isAutoAimReady(drivetrain)))
              .beforeStarting(() -> isAutoShooting = true)
              .finallyDo(() -> {
                isAutoShooting = false;
                IntakeCommands.shooting = false;
              });
        },
        Set.of(drivetrain, shooter, indexer));
  }

  public static Command setPointRevThenShoot(ShooterSubsystem shooter, IndexerSubsystem indexer) {
    Pose2d pose = new Pose2d(1.578, 4.008, Rotation2d.fromDegrees(180));
    return Commands.parallel(
            ShooterCommands.autoAim(shooter, () -> pose, () -> blueHub, () -> false, () -> false),
            IndexerCommands.conditionalRunForward(
                indexer, () -> ShooterCommands.isAutoAimReady(shooter)),
            Commands.run(() -> IntakeCommands.shooting = ShooterCommands.isAutoAimReady(shooter)))
        .beforeStarting(() -> isAutoShooting = true)
        .finallyDo(() -> {
          isAutoShooting = false;
          IntakeCommands.shooting = false;
        });
  }

  public static Command faceHubAndDrive(
      DrivetrainSubsystem drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> centerOfRotationSupplier) {
    return DrivetrainCommands.joystickDriveFacingTarget(
        drivetrain,
        xSupplier,
        ySupplier,
        () -> getTarget(drivetrain.getPose()),
        () -> new Translation2d(),
        centerOfRotationSupplier,
        () -> false);
  }

  public static Command autoFaceTarget(
      DrivetrainSubsystem drivetrain, Supplier<Translation2d> centerOfRotationSupplier) {
    return Commands.defer(
        () -> {
          return DrivetrainCommands.stopAndShootToward(
              drivetrain, () -> getTarget(drivetrain.getPose()), centerOfRotationSupplier);
        },
        Set.of(drivetrain));
  }

  public static Command autoAimRevFlywheels(
      Supplier<Pose2d> robotPoseSupplier, ShooterSubsystem shooter) {
    return ShooterCommands.autoAim(
            shooter,
            robotPoseSupplier,
            () -> getTarget(robotPoseSupplier.get()),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? robotPoseSupplier.get().getX() > 4.7
                : robotPoseSupplier.get().getX() < 12,
            () -> true)
        .onlyIf(() -> !isAutoShooting);
  }

  public static Translation2d getTarget(Pose2d robotPose) {
    boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    boolean isPassing = // false;
        isBlueAlliance ? robotPose.getX() > 6.2 : robotPose.getX() < 9.9;
    Translation2d target = isPassing
        ? (robotPose.getY() < 4.05
            ? (isBlueAlliance ? blueRPassingTarget : redRPassingTarget)
            : (isBlueAlliance ? blueLPassingTarget : redLPassingTarget))
        : (isBlueAlliance ? blueHub : redHub);
    Logger.recordOutput(
        "Commands/alliance",
        DriverStation.getAlliance().isEmpty() ? "Empty" : (isBlueAlliance ? "Blue" : "Red"));
    Logger.recordOutput("Commands/isPassing", isPassing ? "True" : "False");
    Logger.recordOutput("Commands/shootingTarget", target);
    return target;
  }
  ;
}
