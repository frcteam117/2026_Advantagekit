package frc.robot.subsystems.shooter;

import static frc.robot.Constants.nominalVoltage_V;
import static frc.robot.Constants.robotPeriod_s;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.UnitUtil;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.logging.TunableBoolean;

public class ShooterConstants {
  public static final String name = "Shooter";
  public static final TunableBoolean tunable =
      new TunableBoolean("Tunable/" + name + "/.Tunable", false, () -> true);

  public static class Flywheel {
    public static final String name = ShooterConstants.name + "/Flywheel";
    // physical constants
    public static final double moi_kgm2 = 0.01;
    public static final int canId = 9;
    public static final double reduction = 1;
    public static final DCMotor gearbox = DCMotor.getNEO(1);

    // software limits
    public static final double max_radPs2 = 10000;
    public static final double max_radPs3 = 30000;
    public static final int maxStator_A = 30;
    public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

    // motion profiling
    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0, robotPeriod_s),
        simFF = new SimpleMotorFeedforward(0.0, 0.0198426988, 0.0, robotPeriod_s);
    public static final PIDController realPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s),
        simPID = new PIDController(.5, 0.0, 0.0, robotPeriod_s);

    static {
      motorConfig
          .voltageCompensation(nominalVoltage_V)
          .smartCurrentLimit(maxStator_A)
          .encoder
          .positionConversionFactor(UnitUtil.rotTorad(1) / reduction)
          .velocityConversionFactor(UnitUtil.RPMToradPs(1) / reduction);
      LogUtil.createTunablePID("Tunable/" + name + "/real", realPID, tunable::getAsBoolean);
      LogUtil.createTunableFF("Tunable/" + name + "/real", realFF, tunable::getAsBoolean);
      LogUtil.createTunablePID("Tunable/" + name + "/sim", simPID, tunable::getAsBoolean);
      LogUtil.createTunableFF("Tunable/" + name + "/sim", simFF, tunable::getAsBoolean);
    }
  }

  public static class Hood {
    public static final String name = ShooterConstants.name + "/Hood";
    // physical constants
    public static final double moi_kgm2 = 0.05;
    public static final double mass_kg = 0.5;
    public static final double cmRadius_m = 0.1;
    /** Measured when the hood is at an angle of 0 radians. */
    public static final double cmAngle_rad = -0.3;

    public static final int canId = 10;
    public static final double reduction = 10;
    public static final DCMotor gearbox = DCMotor.getNEO(1);

    // software limits
    public static final double start_rad = 0;
    public static final double min_rad = 0;
    public static final double max_rad = 2;
    public static final double max_radPs = 4;
    public static final double max_radPs2 = 6;
    public static final int maxStator_A = 30;
    public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

    public static final ArmFeedforward
        realFF = new ArmFeedforward(0.0, 0.0, 0.0, 0.0, robotPeriod_s),
        simFF = new ArmFeedforward(0.0, 1.69615385, 0.16, .02, robotPeriod_s);
    public static final PIDController realPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s),
        simPID = new PIDController(5, 0.0, 0.0, robotPeriod_s);

    static {
      motorConfig
          .voltageCompensation(nominalVoltage_V)
          .smartCurrentLimit(maxStator_A)
          .encoder
          .positionConversionFactor(UnitUtil.rotTorad(1) / reduction)
          .velocityConversionFactor(UnitUtil.RPMToradPs(1) / reduction);
      LogUtil.createTunablePID("Tunable/" + name + "/real", realPID, tunable::getAsBoolean);
      LogUtil.createTunableFF("Tunable/" + name + "/real", realFF, tunable::getAsBoolean);
      LogUtil.createTunablePID("Tunable/" + name + "/sim", simPID, tunable::getAsBoolean);
      LogUtil.createTunableFF("Tunable/" + name + "/sim", simFF, tunable::getAsBoolean);
    }
  }

  public static class Turret {
    public static final String name = ShooterConstants.name + "/Turret";
    // physical constants
    public static final double moi_kgm2 = 0.05;
    public static final double mass_kg = 0.5;
    public static final int canId = 11;
    public static final double reduction = 20;
    public static final DCMotor gearbox = DCMotor.getNEO(1);

    // software limits
    public static final double start_rad = 0;
    public static final double min_rad = 0;
    public static final double max_rad = 2 * Math.PI;
    public static final double max_radPs = 4;
    public static final double max_radPs2 = 6;
    public static final int maxStator_A = 30;
    public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0, robotPeriod_s),
        simFF = new SimpleMotorFeedforward(0.013818, 0.39684, 0.010878, robotPeriod_s);
    public static final PIDController realPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s),
        simPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s);

    static {
      motorConfig
          .voltageCompensation(nominalVoltage_V)
          .smartCurrentLimit(maxStator_A)
          .encoder
          .positionConversionFactor(UnitUtil.rotTorad(1) / reduction)
          .velocityConversionFactor(UnitUtil.RPMToradPs(1) / reduction);

      realPID.enableContinuousInput(0, UnitUtil.rotTorad(1));
      simPID.enableContinuousInput(0, UnitUtil.rotTorad(1));
      LogUtil.createTunablePID("Tunable/" + name + "/real", realPID, tunable::getAsBoolean);
      LogUtil.createTunableFF("Tunable/" + name + "/real", realFF, tunable::getAsBoolean);
      LogUtil.createTunablePID("Tunable/" + name + "/sim", simPID, tunable::getAsBoolean);
      LogUtil.createTunableFF("Tunable/" + name + "/sim", simFF, tunable::getAsBoolean);
    }
  }
}
