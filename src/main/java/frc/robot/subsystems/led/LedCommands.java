package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.states.premade.RadVel_State;

// theres def a more efficient way to write this
// it could be better...

public class LedCommands {
  public static Command ledCommand(BooleanSupplier rio_FlywheelSpeed) {

    return Commands.none();
  }

  public static Command updateShooterLEDs(ShooterSubsystem shooter, double targetSpeedRadPerSec) {

    double rioSpeed = shooter.getRIOFlywheelState().vel(RadiansPerSecond);
    double pdhSpeed = shooter.getPDHFlywheelState().vel(RadiansPerSecond);
    boolean atSpeed = Math.abs(rioSpeed - targetSpeedRadPerSec) < tolerance
        && Math.abs(pdhSpeed - targetSpeedRadPerSec) < tolerance;

    return Commands.run(() -> {
      if (targetSpeedRadPerSec <= 0) { // 0
        // shooter off
        setAll(0, 0, 0);
      } else if (atSpeed) {
        // ready = green
        setAll(0, 255, 0);
      } else {
        // spinning up = white
        setAll(255, 255, 255);
      }
      led.setData(buffer);
    });
  }

  private static void setAll(int r, int g, int b) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, r, g, b);
      // this is really inefficient but it works (i think...)
    }
  }
}
