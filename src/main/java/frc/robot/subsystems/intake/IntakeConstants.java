package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public class IntakeConstants {
  public static final String NT_KEY = "2_Intake";

  public static class Pivot {
    public static final String NT_KEY = IntakeConstants.NT_KEY + "/Pivot";
    public static final int CAN_ID = 12;
    public static final SparkMaxConfig SPARK_MAX_CONFIG = new SparkMaxConfig();
    public static final double REDUCTION = 55.55555555555555555555555;
    public static final MomentOfInertia MOI = null;
    public static final DCMotor GEARBOX = DCMotor.getNEO(1).withReduction(REDUCTION);
    public static final Angle MIN_POS = Radians.of(-1.495396);
    public static final Angle MAX_POS = Radians.of(0.0);

    static {
      SPARK_MAX_CONFIG
          .disableVoltageCompensation()
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(80)
          .inverted(true)
          .encoder
          .quadratureMeasurementPeriod(10)
          .quadratureAverageDepth(4);
      // .velocityConversionFactor(2 * Math.PI / 60);
    }
  }

  public static class Roller {
    public static final String NT_KEY = IntakeConstants.NT_KEY + "/Roller";
    public static final int CAN_ID = 11;
    public static final SparkMaxConfig SPARK_MAX_CONFIG = new SparkMaxConfig();
    public static final double REDUCTION = 24 / 15;
    public static final Distance RADIUS = Inches.of(1.0);
    public static final MomentOfInertia MOI = null;
    public static final DCMotor GEARBOX = DCMotor.getNEO(1).withReduction(REDUCTION);

    static {
      SPARK_MAX_CONFIG
          .disableVoltageCompensation()
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(50)
          .openLoopRampRate(.2)
          .encoder
          .quadratureMeasurementPeriod(10)
          .quadratureAverageDepth(4);
      // .positionConversionFactor(2 * Math.PI)
      // .velocityConversionFactor(2 * Math.PI / 60);
    }
  }
}
