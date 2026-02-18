package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class PivotConstants {
  public static class HardwareConstants {
    public static final int canId = 12;
    public static final MotorType motorType = MotorType.kBrushless;
    public static final DCMotor motor = DCMotor.getNEO(1);
    public static final boolean kInverted = false;
    public static final ControlMode kControlMode = ControlMode.CLOSED_LOOP;
  }

  public static class PIDConstants {
    public static final double rioP = 8.0;
    public static final double rioI = 0.0;
    public static final double rioD = 0.0;

    public static final double sparkP = 4.0;
    public static final double sparkI = 0.0;
    public static final double sparkD = 0.0;

    public static final double kS = 0.1;
    public static final double kG = 0.75;
    public static final double kV = 0.25;
    public static final double kA = 0.05;
  }

  public static class ProfileConstants {
    public static final double kRioMaxVelocity = 2.45; // Rad/s
    public static final double kRioMaxAcceleration = 2.45; // Rad/s^2
    public static final AngularVelocity kSparkMaxVelocity = DegreesPerSecond.of(90);
    public static final AngularAcceleration kSparkMaxAcceleration =
        DegreesPerSecondPerSecond.of(45);
  }

  public static class MechanismConstants {
    public static final double kGearRatioStage1 = 3;
    public static final double kGearRatioStage2 = 4;
    public static final Distance kArmLength = Feet.of(3);
    public static final Mass kArmMass = Pounds.of(1);
    public static final Angle kMinSoftLimit = Radians.of(0.2);
    public static final Angle kMaxSoftLimit = Radians.of(1.4);
    public static final Angle kMinHardLimit = Radians.of(0.0);
    public static final Angle kMaxHardLimit = Radians.of(1.6);
    public static final Angle kStartingPosition = Radians.of(0);
  }

  public static class SafetyConstants {
    public static final Current kCurrentLimit = Amps.of(20);
    public static final Time kRampRate = Seconds.of(0.25);
  }

  public static class TelemetryConstants {
    public static final String kMotorName = "PivotMotor";
    public static final String kArmName = "Pivot";
    public static final TelemetryVerbosity kTelemetryVerbosity = TelemetryVerbosity.HIGH;
  }
}
