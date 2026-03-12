package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotConstants;

public class IndexerConstants {
  public static final String LOG_NAME = "3_Indexer";

  public static class Hopper {
    public static final String NT_KEY = IndexerConstants.LOG_NAME + "/Hopper";
    public static final String TUNING_NT_KEY = RobotConstants.TUNING_PREFIX + NT_KEY;
    public static final int CAN_ID = 9;
    public static final SparkMaxConfig SPARK_MAX_CONFIG = new SparkMaxConfig();
    public static final double REDUCTION = 3.0;
    public static final Distance RADIUS = Inches.of(1.0);
    public static final DCMotor GEARBOX = DCMotor.getNEO(1).withReduction(REDUCTION);

    static {
      SPARK_MAX_CONFIG
          .disableVoltageCompensation()
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(40)
          .openLoopRampRate(.2)
          .encoder
          .quadratureMeasurementPeriod(10)
          .quadratureAverageDepth(4);
      // .positionConversionFactor(2 * Math.PI)
      // .velocityConversionFactor(2 * Math.PI / 60);
    }
  }

  public static class Kicker extends Hopper {
    public static final String NT_KEY = IndexerConstants.LOG_NAME + "/Kicker";
    public static final String TUNING_NT_KEY = RobotConstants.TUNING_PREFIX + NT_KEY;
    public static final int CAN_ID = 10;
  }
}
