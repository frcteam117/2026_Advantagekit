package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ShooterConstants {
  public static final String LOG_NAME = "4_Shooter";

  public static class Hood {
    public static final String NT_KEY = ShooterConstants.LOG_NAME + "/Hood";
    public static final int CAN_ID = 17;
    public static final SparkMaxConfig SPARK_MAX_CONFIG = new SparkMaxConfig();
    public static final double REDUCTION = (100.0 / 9.0) * 32;
    public static final MomentOfInertia MOI = null;
    public static final DCMotor GEARBOX = DCMotor.getNEO(1).withReduction(REDUCTION);
    public static final Angle MIN_POS = Radians.of(0.0);
    public static final Angle MAX_POS = Radians.of(0.646);

    static {
      SPARK_MAX_CONFIG
          .disableVoltageCompensation()
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(30)
          .encoder
          .quadratureMeasurementPeriod(10)
          .quadratureAverageDepth(4);
      // .positionConversionFactor(2 * Math.PI)
      // .velocityConversionFactor(2 * Math.PI / 60);
    }
  }

  public static class RIO_Flywheel {
    public static final String NT_KEY = ShooterConstants.LOG_NAME + "/RIO_Flywheel";
    public static final int LEADER_CAN_ID = 13;
    public static final int FOLLOWER_CAN_ID = 14;
    public static final SparkMaxConfig SPARK_MAX_CONFIG = new SparkMaxConfig();
    public static final double REDUCTION = 1.0;
    public static final Distance RADIUS = Inches.of(2.0);
    public static final MomentOfInertia MOI = null;
    public static final DCMotor GEARBOX = DCMotor.getNeoVortex(2).withReduction(REDUCTION);

    static {
      SPARK_MAX_CONFIG
          .disableVoltageCompensation()
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(60)
          .encoder
          .quadratureMeasurementPeriod(10)
          .quadratureAverageDepth(4);
      // .positionConversionFactor(2 * Math.PI)
      // .velocityConversionFactor(2 * Math.PI / 60);
    }
  }

  public static class PDH_Flywheel {
    public static final String NT_KEY = ShooterConstants.LOG_NAME + "/PDH_Flywheel";
    public static final int LEADER_CAN_ID = 15;
    public static final int FOLLOWER_CAN_ID = 16;
    public static final SparkMaxConfig SPARK_MAX_CONFIG = new SparkMaxConfig();
    public static final double REDUCTION = 1.0;
    public static final Distance RADIUS = Inches.of(2.0);
    public static final MomentOfInertia MOI = null;
    public static final DCMotor GEARBOX = DCMotor.getNeoVortex(2).withReduction(REDUCTION);

    static {
      SPARK_MAX_CONFIG
          .disableVoltageCompensation()
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(60)
          .encoder
          .quadratureMeasurementPeriod(10)
          .quadratureAverageDepth(4);
      // .positionConversionFactor(2 * Math.PI)
      // .velocityConversionFactor(2 * Math.PI / 60);
    }
  }
}
