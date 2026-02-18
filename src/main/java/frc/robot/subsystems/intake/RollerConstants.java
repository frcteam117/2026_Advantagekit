package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class RollerConstants {
  public static class HardwareConstants {
    public static final int canId = 11; // check all CANIDs!!!!!!!!
    public static final MotorType motorType = MotorType.kBrushless;
    public static final DCMotor motor = DCMotor.getNEO(1);
    public static final boolean kInverted = false;
    public static final ControlMode kControlMode = ControlMode.OPEN_LOOP;
    public static final MotorMode kIdleMode = MotorMode.BRAKE;
  }

  public static class PIDConstants {
    public static final double controllerP = 1.0;
    public static final double controllerI = 0.0;
    public static final double controllerD = 0.0;

    public static final double feedForwardS = 0.0;
    public static final double feedForwardV = 0.0;
    public static final double feedForwardA = 0.0;
  }

  public static class MechanismConstants {
    public static final double kGearRatioStage1 = 3;
    public static final double kGearRatioStage2 = 4;
    public static final Distance kDiameter = Inches.of(1.25);
    public static final Mass kMass = Pounds.of(1);
    public static final AngularVelocity kMaxSpeed = RPM.of(1000);
  }

  public static class SafetyConstants {
    public static final Current kCurrentLimit = Amps.of(20);
    public static final Time kRampRate = Seconds.of(0.25);
  }

  public static class TelemetryConstants {
    public static final String kMotorName = "IntakeMotor";
    public static final String kIntakeName = "IntakeMech";
    public static final TelemetryVerbosity kTelemetryVerbosity = TelemetryVerbosity.HIGH;
  }
}
