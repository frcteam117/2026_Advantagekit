package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// theres def a more efficient way to write this
// it could be better...

public class LedCommands {

  private static AddressableLED led;
  private static AddressableLEDBuffer buffer;
  // private final ShooterSubsystem shooter;

  private static final double tolerance = 10; // allowable error, should i change this?

  public LedCommands(int pwmPort, int length) { // ShooterSubsystem shooter) {
    // this.shooter = shooter;

    led = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(length);

    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }

  public static Command updateShooterLEDs(ShooterSubsystem shooter, double targetSpeedRadPerSec) {

    double rioSpeed = shooter.getRIOFlywheelState().vel(RadiansPerSecond);
    double pdhSpeed = shooter.getPDHFlywheelState().vel(RadiansPerSecond);

    boolean atSpeed = Math.abs(rioSpeed - targetSpeedRadPerSec) < tolerance
        && Math.abs(pdhSpeed - targetSpeedRadPerSec) < tolerance;

    return Commands.run(() -> {
      if (targetSpeedRadPerSec <= -1) { // 0
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
