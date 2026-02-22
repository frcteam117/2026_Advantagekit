package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.BooleanSupplier;

public class SysIdUtil {
  public static enum SysIdType {
    Quasistatic,
    QuasistaticForward,
    QuasistaticReverse,
    Dynamic,
    DynamicForward,
    DynamicReverse
  }

  private static BooleanSupplier shouldStop;

  public static void registerController(CommandXboxController controller) {
    shouldStop = controller
        .button(0)
        .or(controller.button(1))
        .or(controller.button(2))
        .or(controller.button(3));
  }

  public static Command getSysIdCommand(SysIdRoutine routine, SysIdType type) {
    switch (type) {
      case Quasistatic -> {
        return routine
            .quasistatic(Direction.kForward)
            .until(shouldStop)
            .andThen(Commands.idle().withTimeout(.75))
            .andThen(routine.quasistatic(Direction.kReverse).until(shouldStop));
      }
      case QuasistaticForward -> {
        return routine.quasistatic(Direction.kForward).until(shouldStop);
      }
      case QuasistaticReverse -> {
        return routine.quasistatic(Direction.kReverse).until(shouldStop);
      }
      case Dynamic -> {
        return routine
            .dynamic(Direction.kForward)
            .until(shouldStop)
            .andThen(Commands.idle().withTimeout(.75))
            .andThen(routine.dynamic(Direction.kReverse).until(shouldStop));
      }
      case DynamicForward -> {
        return routine.dynamic(Direction.kForward).until(shouldStop);
      }
      case DynamicReverse -> {
        return routine.dynamic(Direction.kReverse).until(shouldStop);
      }
      default -> {
        return Commands.none();
      }
    }
  }
}
