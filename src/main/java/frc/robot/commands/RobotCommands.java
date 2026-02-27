package frc.robot.commands;

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
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class RobotCommands {
  private static final Translation2d blueHub = RobotConstants.FIELD_TYPE.equals(FieldType.WELDED)
      ? new Translation2d(0.0254 * (182.11), 0.0254 * (317.69 / 2))
      : new Translation2d(0.0254 * (181.56), 0.0254 * (316.64 / 2));
  private static final Translation2d redHub = RobotConstants.FIELD_TYPE.equals(FieldType.WELDED)
      ? new Translation2d(0.0254 * (651.22 - 182.11), 0.0254 * (317.69 / 2))
      : new Translation2d(0.0254 * (650.12 - 181.56), 0.0254 * (316.64 / 2));

  public static Command hubAutoShoot(
      DrivetrainSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      BooleanSupplier shootWhenReady) {
    Translation2d target =
        DriverStation.getAlliance().get().equals(Alliance.Blue) ? blueHub : redHub;
    return Commands.parallel(
        ShooterCommands.hubAutoAim(shooter, drivetrain::getPose, () -> target),
        DrivetrainCommands.stopAndFacePosition(drivetrain, () -> target),
        IndexerCommands.conditionalRunForward(
            indexer,
            () -> shootWhenReady.getAsBoolean()
                && ShooterCommands.isAutoAimReady(shooter)
                && DrivetrainCommands.isAutoAimReady(drivetrain)));
  }
}
