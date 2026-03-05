package frc.robot.subsystems.led;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.states.premade.RadVel_State;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

//LEDs should be green for ready, white for starting up 

// public class LedCommands {
//   private static double autoTargetSpeed = 0;
//   private static final String TUNING_NT_KEY = "Tuning/" + LOG_NAME + "/Commands";
//   private static final DoubleSupplier maxAllowableErrorRadPS = new TunableDouble(TUNING_NT_KEY + "/AutoAimAllowableError", 10, () -> true);
//   private AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(120);
//   private AddressableLEDBufferView m_left = m_buffer.createView(0, 59);
//   private AddressableLEDBufferView m_right = m_buffer.createView(60, 119).reversed();
//   static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
//   m_led = new AddressableLED(9);
// // m_ledBuffer = new AddressableLEDBuffer(60);

// static LEDPattern green = LEDPattern.solid(Color.kGreen);
//  public LedCommands() {
//   green.applyTo(m_ledBuffer);
//   m_led.setData(m_ledBuffer);
//  }
// // Apply the LED pattern to the data buffer
// //green.applyTo(m_ledBuffer);

// // Write the data to the LED strip
// // m_led.setData(m_ledBuffer);

// //static {m_ledBuffer = new AddressableLEDBuffer(60);}
// //figure ts out bro idk whats up w it

// public static Command ledCommand(FloatSupplier rio_FlywheelSpeed) {
//     return Commands.none();
//   }

//   //public static Command ledsGreen() {
//     //if (LEDsTurnGreen == True) {
//     //  TURN LEDS ON
//     //}
//   //}
  
//   // public static float floatt() {
//   //   return (float) 0.4324;
//   // }

//   public static boolean LEDsTurnGreen(ShooterSubsystem shooter) {
//     return Math.abs(shooter.getRIOFlywheelState().vel(RadiansPerSecond) - autoTargetSpeed)
//             < maxAllowableErrorRadPS.getAsDouble()
//         && Math.abs(shooter.getPDHFlywheelState().vel(RadiansPerSecond) - autoTargetSpeed)
//             < maxAllowableErrorRadPS.getAsDouble();

//   }
  
// }

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedCommands {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    public LedCommands(int pwmPort, int length) {
        m_led = new AddressableLED(pwmPort);
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void updateShooterLEDs(double currentRPM, double targetRPM) {
        // Define a tolerance (e.g., within 50 RPM of target is "at speed")
        double tolerance = 50.0; 
        
        if (targetRPM <= 0) {
            // Shooter is off - turn LEDs off
            setAll(0, 0, 0); 
        } else if (Math.abs(targetRPM - currentRPM) < tolerance) {
            // AT SPEED: Solid Green
            setAll(0, 255, 0); 
        } else {
            // POWERING UP: Solid White (Red, Green, and Blue at 255)
            setAll(255, 255, 255); 
        }
        
        m_led.setData(m_ledBuffer);
    }

    private void setAll(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
    }
}

