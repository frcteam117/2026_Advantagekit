package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotConstants;
import frc.robot.util.components.bases.ComponentBase;
import frc.robot.util.components.bases.ComponentSimBase;
import frc.robot.util.components.bases.ComponentStates.Motor_State;
import frc.robot.util.components.controllers.SparkMaxController;
import frc.robot.util.components.simulators.DCMotorSimulator;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.control_functions.feedback.SimplePIDF;
import frc.robot.util.control_functions.profiling.TrapezoidProfileFunction;
import frc.robot.util.mechanisms.MechanismBase.MechanismConfig;
import frc.robot.util.mechanisms.MechanismConstants;
import frc.robot.util.states.StateValue;
import frc.robot.util.states.bases.PosVel_State;
import frc.robot.util.states.bases.Pos_State;
import frc.robot.util.states.bases.VelAcc_State;

public class ShooterConstants {
  public static final String LOG_NAME = "4_Shooter";

  public static final MechanismConstants<PosVel_State> HOOD_CONSTANTS = new MechanismConstants<>();
  public static final MechanismConfig<PosVel_State> HOOD_CONFIG =
      new MechanismConfig<PosVel_State>();

  public static final MechanismConstants<PosVel_State> FLYWHEEL_CONSTANTS =
      new MechanismConstants<>();
  public static final MechanismConfig<PosVel_State> FLYWHEEL_CONFIG =
      new MechanismConfig<PosVel_State>();

  //   public static final MechanismConstants<PosVel_State> R_FLYWHEEL_CONSTANTS;
  //   public static final MechanismConfig<PosVel_State> R_FLYWHEEL_CONFIG =
  //       new MechanismConfig<PosVel_State>();

  static {
    // Miscellaneous
    HOOD_CONSTANTS.outputsLogName = LOG_NAME + "/Hood";
    HOOD_CONSTANTS.tuningLogName = RobotConstants.TUNING_PREFIX + HOOD_CONSTANTS.outputsLogName;
    HOOD_CONSTANTS.codePeriod_s = RobotConstants.CODE_PERIOD_s;
    HOOD_CONSTANTS.mass_kg = 0.5;
    HOOD_CONSTANTS.cmOffset_rad = -0.1;
    // Motor
    HOOD_CONSTANTS.motorCanIds = new int[] {17};
    HOOD_CONSTANTS.revMotorType = MotorType.kBrushless;
    HOOD_CONSTANTS.baseSparkConfig = new SparkMaxConfig();
    HOOD_CONSTANTS
        .baseSparkConfig
        .voltageCompensation(RobotConstants.NOMINAL_V)
        .smartCurrentLimit(30);
    // HOOD_CONSTANTS.useAlternateEncoder = true;
    // Motor properties
    HOOD_CONSTANTS.reduction = (100.0 / 9.0) * 32;
    HOOD_CONSTANTS.gearbox = DCMotor.getNEO(1).withReduction(HOOD_CONSTANTS.reduction);
    HOOD_CONSTANTS.moi_kgm2 = .3;
    // Profiling
    HOOD_CONSTANTS.start_State =
        PosVel_State.create(new StateValue(0, Radians), new StateValue(0, RadiansPerSecond));
    HOOD_CONSTANTS.min_Pos = Pos_State.create(new StateValue(0, Radians));
    HOOD_CONSTANTS.max_Pos = Pos_State.create(new StateValue(0.76, Radians));
    HOOD_CONSTANTS.limits_State = VelAcc_State.create(
        new StateValue(4, RadiansPerSecond), new StateValue(12, RadiansPerSecondPerSecond));
    HOOD_CONSTANTS.isLoop = false;
    // Feedback
    HOOD_CONSTANTS.pid = RobotBase.isReal()
        ? new PIDController(0, 0, 0, HOOD_CONSTANTS.codePeriod_s)
        : new PIDController(0, 0, 0, HOOD_CONSTANTS.codePeriod_s);
    HOOD_CONSTANTS.simpleFF = RobotBase.isReal()
        ? new SimpleMotorFeedforward(0, 0, 0, HOOD_CONSTANTS.codePeriod_s)
        : new SimpleMotorFeedforward(0, 0, 0, HOOD_CONSTANTS.codePeriod_s);

    HOOD_CONFIG.constants = HOOD_CONSTANTS;
    HOOD_CONFIG.realComponents = new ComponentBase[0]; // {absoluteEncoderComponent};
    HOOD_CONFIG.simComponents = new ComponentSimBase[0]; // {absoluteEncoderSimComponent};
    HOOD_CONFIG.realController = new SparkMaxController(HOOD_CONSTANTS);
    HOOD_CONFIG.simController = new DCMotorSimulator(HOOD_CONSTANTS);
    HOOD_CONFIG.profiles = new ControlFunctionBase[] {new TrapezoidProfileFunction(HOOD_CONSTANTS)};
    HOOD_CONFIG.feedbacks = new ControlFunctionBase[] {new SimplePIDF(HOOD_CONSTANTS)};
    HOOD_CONFIG.componentsToState = componentStates -> PosVel_State.create(
        new StateValue(
            ((Motor_State) componentStates[0]).rad() / HOOD_CONSTANTS.reduction, Radians),
        new StateValue(
            ((Motor_State) componentStates[0]).radPs() / HOOD_CONSTANTS.reduction,
            RadiansPerSecond));
  }

  static {
    // Miscellaneous
    FLYWHEEL_CONSTANTS.outputsLogName = LOG_NAME + "/Flywheel";
    FLYWHEEL_CONSTANTS.tuningLogName =
        RobotConstants.TUNING_PREFIX + FLYWHEEL_CONSTANTS.outputsLogName;
    FLYWHEEL_CONSTANTS.codePeriod_s = RobotConstants.CODE_PERIOD_s;
    // Motor
    FLYWHEEL_CONSTANTS.motorCanIds = new int[] {13, 14, 15, 16};
    FLYWHEEL_CONSTANTS.followerInversions = new boolean[] {false, true, false, true};
    FLYWHEEL_CONSTANTS.revMotorType = MotorType.kBrushless;
    FLYWHEEL_CONSTANTS.baseSparkConfig = new SparkMaxConfig();
    FLYWHEEL_CONSTANTS
        .baseSparkConfig
        .voltageCompensation(RobotConstants.NOMINAL_V)
        .smartCurrentLimit(30);
    // Motor properties
    FLYWHEEL_CONSTANTS.reduction = 1d;
    FLYWHEEL_CONSTANTS.gearbox =
        DCMotor.getNeoVortex(2).withReduction(FLYWHEEL_CONSTANTS.reduction);
    FLYWHEEL_CONSTANTS.moi_kgm2 = .05;
    // Profiling
    FLYWHEEL_CONSTANTS.start_State =
        PosVel_State.create(new StateValue(0, Radians), new StateValue(0, RadiansPerSecond));
    FLYWHEEL_CONSTANTS.min_Pos = Pos_State.create(new StateValue(-Double.MAX_VALUE, Radians));
    FLYWHEEL_CONSTANTS.max_Pos = Pos_State.create(new StateValue(Double.MAX_VALUE, Radians));
    FLYWHEEL_CONSTANTS.limits_State = VelAcc_State.create(
        new StateValue(628, RadiansPerSecond), new StateValue(250, RadiansPerSecondPerSecond));
    FLYWHEEL_CONSTANTS.isLoop = true;
    // Feedback
    FLYWHEEL_CONSTANTS.pid = RobotBase.isReal()
        ? new PIDController(0, 0, 0, FLYWHEEL_CONSTANTS.codePeriod_s)
        : new PIDController(0, 0, 0, FLYWHEEL_CONSTANTS.codePeriod_s);
    FLYWHEEL_CONSTANTS.simpleFF = RobotBase.isReal()
        ? new SimpleMotorFeedforward(0.15, 0.017, 0, FLYWHEEL_CONSTANTS.codePeriod_s)
        : new SimpleMotorFeedforward(0, 0, 0, FLYWHEEL_CONSTANTS.codePeriod_s);

    FLYWHEEL_CONFIG.constants = FLYWHEEL_CONSTANTS;
    FLYWHEEL_CONFIG.realComponents = new ComponentBase[0];
    FLYWHEEL_CONFIG.simComponents = new ComponentSimBase[0];
    FLYWHEEL_CONFIG.realController = new SparkMaxController(FLYWHEEL_CONSTANTS);
    FLYWHEEL_CONFIG.simController = new DCMotorSimulator(FLYWHEEL_CONSTANTS);
    FLYWHEEL_CONFIG.profiles =
        new ControlFunctionBase[] {new TrapezoidProfileFunction(FLYWHEEL_CONSTANTS)};
    FLYWHEEL_CONFIG.feedbacks = new ControlFunctionBase[] {new SimplePIDF(FLYWHEEL_CONSTANTS)};
    FLYWHEEL_CONFIG.componentsToState = componentStates -> PosVel_State.create(
        new StateValue(
            ((Motor_State) componentStates[0]).rad() / FLYWHEEL_CONSTANTS.reduction, Radians),
        new StateValue(
            ((Motor_State) componentStates[0]).radPs() / FLYWHEEL_CONSTANTS.reduction,
            RadiansPerSecond));
  }

  //   static {
  //     R_FLYWHEEL_CONSTANTS = L_FLYWHEEL_CONSTANTS;
  //     R_FLYWHEEL_CONSTANTS.outputsLogName = LOG_NAME + "/R Flywheel";
  //     R_FLYWHEEL_CONSTANTS.tuningLogName =
  //         RobotConstants.TUNING_PREFIX + R_FLYWHEEL_CONSTANTS.outputsLogName;
  //     R_FLYWHEEL_CONSTANTS.motorCanIds = new int[] {15, 16};

  //     R_FLYWHEEL_CONFIG.constants = R_FLYWHEEL_CONSTANTS;
  //     R_FLYWHEEL_CONFIG.realComponents = new ComponentBase[0];
  //     R_FLYWHEEL_CONFIG.simComponents = new ComponentSimBase[0];
  //     R_FLYWHEEL_CONFIG.realController = new SparkMaxController(R_FLYWHEEL_CONSTANTS);
  //     R_FLYWHEEL_CONFIG.simController = new DCMotorSimulator(R_FLYWHEEL_CONSTANTS);
  //     R_FLYWHEEL_CONFIG.profiles =
  //         new ControlFunctionBase[] {new TrapezoidProfileFunction(R_FLYWHEEL_CONSTANTS)};
  //     R_FLYWHEEL_CONFIG.feedbacks = new ControlFunctionBase[] {new
  // SimplePIDF(R_FLYWHEEL_CONSTANTS)};
  //     R_FLYWHEEL_CONFIG.componentsToState = componentStates -> PosVel_State.create(
  //         new StateValue(
  //             ((Motor_State) componentStates[0]).rad() / R_FLYWHEEL_CONSTANTS.reduction,
  // Radians),
  //         new StateValue(
  //             ((Motor_State) componentStates[0]).radPs() / R_FLYWHEEL_CONSTANTS.reduction,
  //             RadiansPerSecond));
  //   }
}
