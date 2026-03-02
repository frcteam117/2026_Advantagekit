package frc.robot.subsystems.led;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LedCommands {
  public static Command ledCommand(BooleanSupplier rio_FlywheelSpeed) {

    return Commands.none();
  }

  public static double doublee() {
    return 0.4324;
  }
}
