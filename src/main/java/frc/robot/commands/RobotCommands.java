package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.FieldType;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class RobotCommands {
  private static final Translation2d blueHub = RobotConstants.FIELD_TYPE.equals(FieldType.WELDED)
      ? new Translation2d(0.0254 * (182.11), 0.0254 * (317.69 / 2))
      : new Translation2d(0.0254 * (181.56), 0.0254 * (316.64 / 2));
  private static final Translation2d redHub = RobotConstants.FIELD_TYPE.equals(FieldType.WELDED)
      ? new Translation2d(0.0254 * (651.22 - 182.11), 0.0254 * (317.69 / 2))
      : new Translation2d(0.0254 * (650.12 - 181.56), 0.0254 * (316.64 / 2));
  public static Command controllerRumble(CommandPS5Controller controller) {
    
    return Commands.run(
        () -> {
            double time = DriverStation.getMatchTime();
            List<Double> matchCountdownTimes = Arrays.asList(145.0, 135.0, 110.0,
                                                            85.0, 60.0, 35.0, 5.0);
            Boolean condition = false;
            for (int i = 0; i < matchCountdownTimes.size(); i++) {
            if (time >= matchCountdownTimes.get(i) - 0.1 && time <= matchCountdownTimes.get(i) + 0.1) {
                condition = true;
                break;
            }
            }
           if (condition) {
             // Java - Trigger rumble (0.0 to 1.0)
             controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);

             } else {
             // Java - Trigger rumble (0.0 to 1.0)
             controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
           }
        });
  }
  public static Command hubAutoShoot(
      DrivetrainSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      BooleanSupplier shootWhenReady) {
    Translation2d target =
        DriverStation.getAlliance().get().equals(Alliance.Blue) ? blueHub : redHub;
    Logger.recordOutput("Tuning/alliance", DriverStation.getAlliance().get().name());
    return Commands.parallel(
        ShooterCommands.hubAutoAim(shooter, drivetrain::getPose, () -> target));
    // DrivetrainCommands.stopAndFacePosition(drivetrain, () -> target),
    // IndexerCommands.conditionalRunForward(
    //     indexer,
    //     () -> shootWhenReady.getAsBoolean()
    //         && ShooterCommands.isAutoAimReady(shooter)
    //         && DrivetrainCommands.isAutoAimReady(drivetrain)));
  }
}
