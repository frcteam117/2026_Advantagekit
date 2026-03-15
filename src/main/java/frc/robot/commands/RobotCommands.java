package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
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
import java.util.function.Supplier;

public class RobotCommands {
  private static final Translation2d blueHub = RobotConstants.FIELD_TYPE.equals(FieldType.WELDED)
      ? new Translation2d(0.0254 * (182.11), 0.0254 * (317.69 / 2))
      : new Translation2d(0.0254 * (181.56), 0.0254 * (316.64 / 2));
  private static final Translation2d redHub = RobotConstants.FIELD_TYPE.equals(FieldType.WELDED)
      ? new Translation2d(0.0254 * (651.22 - 182.11), 0.0254 * (317.69 / 2))
      : new Translation2d(0.0254 * (650.12 - 181.56), 0.0254 * (316.64 / 2));
  private static final Translation2d redLPassingTarget = new Translation2d(15, 6.1);
  private static final Translation2d redRPassingTarget = new Translation2d(15, 2);
  private static final Translation2d blueLPassingTarget = new Translation2d(.75, 6.6);
  private static final Translation2d blueRPassingTarget = new Translation2d(.75, 1.5);

  public static Command autoAim(
      DrivetrainSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      BooleanSupplier shootWhenReady) {
    return Commands.defer(
        () -> {
          boolean isBlueAlliance =
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
          boolean isPassing = isBlueAlliance
              ? drivetrain.getPose().getX() > 4.7
              : drivetrain.getPose().getX() < 12;
          Translation2d target = isPassing
              ? (drivetrain.getPose().getY() < 4.05
                  ? (isBlueAlliance ? blueRPassingTarget : redRPassingTarget)
                  : (isBlueAlliance ? blueLPassingTarget : redLPassingTarget))
              : (isBlueAlliance ? blueHub : redHub);
          // Logger.recordOutput(
          //     "Commands/alliance",
          //     DriverStation.getAlliance().isEmpty() ? "Empty" : (isBlueAlliance ? "Blue" :
          // "Red"));
          // Logger.recordOutput("Commands/isPassing", isPassing ? "True" : "False");
          // Logger.recordOutput("Commands/shootingTarget", target);
          // Logger.recordOutput("Commands/posex", drivetrain.getPose().getX());
          // Logger.recordOutput("Commands/posey", drivetrain.getPose().getY());
          return Commands.parallel(
                  ShooterCommands.autoAim(
                      shooter, drivetrain::getPose, () -> target, () -> isPassing, () -> false),
                  DrivetrainCommands.stopAndShootToward(drivetrain, () -> target),
                  IndexerCommands.conditionalRunForward(
                      indexer,
                      () -> shootWhenReady.getAsBoolean()
                          && ShooterCommands.isAutoAimReady(shooter)
                          && DrivetrainCommands.isAutoAimReady(drivetrain)),
                  Commands.run(() -> IntakeCommands.shooting = shootWhenReady.getAsBoolean()
                      && ShooterCommands.isAutoAimReady(shooter)
                      && DrivetrainCommands.isAutoAimReady(drivetrain)))
              .finallyDo(() -> IntakeCommands.shooting = false);
        },
        Set.of(drivetrain, shooter, indexer));
  }

  public static Command autoAimRevFlywheels(
      Supplier<Pose2d> robotPoseSupplier, ShooterSubsystem shooter) {
    return Commands.defer(
        () -> {
          boolean isBlueAlliance =
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
          boolean isPassing = isBlueAlliance
              ? robotPoseSupplier.get().getX() > 4.7
              : robotPoseSupplier.get().getX() < 12;
          Translation2d target = isPassing
              ? (robotPoseSupplier.get().getY() < 4.05
                  ? (isBlueAlliance ? blueRPassingTarget : redRPassingTarget)
                  : (isBlueAlliance ? blueLPassingTarget : redLPassingTarget))
              : (isBlueAlliance ? blueHub : redHub);
          // Logger.recordOutput(
          //     "Commands/alliance",
          //     DriverStation.getAlliance().isEmpty() ? "Empty" : (isBlueAlliance ? "Blue" :
          // "Red"));
          // Logger.recordOutput("Commands/isPassing", isPassing ? "True" : "False");
          // Logger.recordOutput("Commands/shootingTarget", target);
          // Logger.recordOutput("Commands/posex", robotPoseSupplier.get().getX());
          // Logger.recordOutput("Commands/posey", robotPoseSupplier.get().getY());
          return ShooterCommands.autoAim(
              shooter, robotPoseSupplier, () -> target, () -> isPassing, () -> true);
        },
        Set.of(shooter));
  }
}
