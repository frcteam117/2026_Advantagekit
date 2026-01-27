package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SysIdUtil {
  public static enum SysIdType {
    Quasistatic,
    QuasistaticForward,
    QuasistaticReverse,
    Dynamic,
    DynamicForward,
    DynamicReverse
  }

  public static Command getSysIdCommand(SysIdRoutine routine, SysIdType type) {
    switch (type) {
      case Quasistatic -> {
        return routine
            .quasistatic(Direction.kForward)
            .andThen(routine.quasistatic(Direction.kReverse));
      }
      case QuasistaticForward -> {
        return routine.quasistatic(Direction.kForward);
      }
      case QuasistaticReverse -> {
        return routine.quasistatic(Direction.kReverse);
      }
      case Dynamic -> {
        return routine.dynamic(Direction.kForward).andThen(routine.dynamic(Direction.kReverse));
      }
      case DynamicForward -> {
        return routine.dynamic(Direction.kForward);
      }
      case DynamicReverse -> {
        return routine.dynamic(Direction.kReverse);
      }
      default -> {
        return Commands.none();
      }
    }
  }
}
