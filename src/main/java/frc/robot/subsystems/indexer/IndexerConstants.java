package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotConstants;
import frc.robot.util.States.AngularPV_State;
import frc.robot.util.States.AngularP_State;
import frc.robot.util.States.AngularVA_State;
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

public class IndexerConstants {
  public static final String LOG_NAME = "3.Indexer";

  public static final MechanismConstants HOPPER_CONSTANTS = new MechanismConstants();
  public static final MechanismConfig<AngularPV_State> HOPPER_CONFIG =
      new MechanismConfig<AngularPV_State>();

  public static final MechanismConstants KICKER_CONSTANTS = new MechanismConstants();
  public static final MechanismConfig<AngularPV_State> KICKER_CONFIG =
      new MechanismConfig<AngularPV_State>();

  static {
    // Miscellaneous
    HOPPER_CONSTANTS.outputsLogName = LOG_NAME + "/Hopper";
    HOPPER_CONSTANTS.tuningLogName = RobotConstants.TUNING_PREFIX + HOPPER_CONSTANTS.outputsLogName;
    HOPPER_CONSTANTS.codePeriod_s = RobotConstants.CODE_PERIOD_s;
    // Motor
    HOPPER_CONSTANTS.motorCanIds = new int[] {14};
    HOPPER_CONSTANTS.revMotorType = MotorType.kBrushless;
    HOPPER_CONSTANTS.baseSparkConfig = new SparkMaxConfig();
    HOPPER_CONSTANTS
        .baseSparkConfig
        .voltageCompensation(RobotConstants.NOMINAL_V)
        .smartCurrentLimit(30);
    // Motor properties
    HOPPER_CONSTANTS.reduction = 3d;
    HOPPER_CONSTANTS.gearbox = DCMotor.getNEO(1).withReduction(HOPPER_CONSTANTS.reduction);
    HOPPER_CONSTANTS.moi_kgm2 = .03;
    // Profiling
    HOPPER_CONSTANTS.start_State = new AngularPV_State(0, 0);
    HOPPER_CONSTANTS.min_Pos = new AngularP_State(-Double.MAX_VALUE);
    HOPPER_CONSTANTS.max_Pos = new AngularP_State(Double.MAX_VALUE);
    HOPPER_CONSTANTS.limits_State = new AngularVA_State(200, 500);
    HOPPER_CONSTANTS.isContinuous = true;
    // Feedback
    HOPPER_CONSTANTS.pid = RobotBase.isReal()
        ? new PIDController(0, 0, 0, HOPPER_CONSTANTS.codePeriod_s)
        : new PIDController(0, 0, 0, HOPPER_CONSTANTS.codePeriod_s);
    HOPPER_CONSTANTS.simpleFF = RobotBase.isReal()
        ? new SimpleMotorFeedforward(0, 0, 0, HOPPER_CONSTANTS.codePeriod_s)
        : new SimpleMotorFeedforward(0, 0, 0, HOPPER_CONSTANTS.codePeriod_s);

    HOPPER_CONFIG.logName = HOPPER_CONSTANTS.outputsLogName;
    HOPPER_CONFIG.tuningLogName = HOPPER_CONSTANTS.tuningLogName;
    HOPPER_CONFIG.realComponents = new ComponentBase[0];
    HOPPER_CONFIG.simComponents = new ComponentSimBase[0];
    HOPPER_CONFIG.realController = new SparkMaxController(HOPPER_CONSTANTS);
    HOPPER_CONFIG.simController = new DCMotorSimulator(HOPPER_CONSTANTS);
    HOPPER_CONFIG.profiles =
        new ControlFunctionBase[] {new TrapezoidProfileFunction(HOPPER_CONSTANTS)};
    HOPPER_CONFIG.feedbacks = new ControlFunctionBase[] {new SimplePIDF(HOPPER_CONSTANTS)};
    HOPPER_CONFIG.componentsToState = componentStates -> new AngularPV_State(
        ((Motor_State) componentStates[0]).rad() / HOPPER_CONSTANTS.reduction,
        ((Motor_State) componentStates[0]).radPs() / HOPPER_CONSTANTS.reduction);
  }

  static {
    // Miscellaneous
    KICKER_CONSTANTS.outputsLogName = LOG_NAME + "/Kicker";
    KICKER_CONSTANTS.tuningLogName = RobotConstants.TUNING_PREFIX + KICKER_CONSTANTS.outputsLogName;
    KICKER_CONSTANTS.codePeriod_s = RobotConstants.CODE_PERIOD_s;
    // Motor
    KICKER_CONSTANTS.motorCanIds = new int[] {15};
    KICKER_CONSTANTS.revMotorType = MotorType.kBrushless;
    KICKER_CONSTANTS.baseSparkConfig = new SparkMaxConfig();
    KICKER_CONSTANTS
        .baseSparkConfig
        .voltageCompensation(RobotConstants.NOMINAL_V)
        .smartCurrentLimit(30);
    // Motor properties
    KICKER_CONSTANTS.reduction = 3d;
    KICKER_CONSTANTS.gearbox = DCMotor.getNEO(1).withReduction(KICKER_CONSTANTS.reduction);
    KICKER_CONSTANTS.moi_kgm2 = .03;
    // Profiling
    KICKER_CONSTANTS.start_State = new AngularPV_State(0, 0);
    KICKER_CONSTANTS.min_Pos = new AngularP_State(-Double.MAX_VALUE);
    KICKER_CONSTANTS.max_Pos = new AngularP_State(Double.MAX_VALUE);
    KICKER_CONSTANTS.limits_State = new AngularVA_State(200, 500);
    KICKER_CONSTANTS.isContinuous = true;
    // Feedback
    KICKER_CONSTANTS.pid = RobotBase.isReal()
        ? new PIDController(0, 0, 0, KICKER_CONSTANTS.codePeriod_s)
        : new PIDController(0, 0, 0, KICKER_CONSTANTS.codePeriod_s);
    KICKER_CONSTANTS.simpleFF = RobotBase.isReal()
        ? new SimpleMotorFeedforward(0, 0, 0, KICKER_CONSTANTS.codePeriod_s)
        : new SimpleMotorFeedforward(0, 0, 0, KICKER_CONSTANTS.codePeriod_s);

    KICKER_CONFIG.logName = KICKER_CONSTANTS.outputsLogName;
    KICKER_CONFIG.tuningLogName = KICKER_CONSTANTS.tuningLogName;
    KICKER_CONFIG.realComponents = new ComponentBase[0];
    KICKER_CONFIG.simComponents = new ComponentSimBase[0];
    KICKER_CONFIG.realController = new SparkMaxController(KICKER_CONSTANTS);
    KICKER_CONFIG.simController = new DCMotorSimulator(KICKER_CONSTANTS);
    KICKER_CONFIG.profiles =
        new ControlFunctionBase[] {new TrapezoidProfileFunction(KICKER_CONSTANTS)};
    KICKER_CONFIG.feedbacks = new ControlFunctionBase[] {new SimplePIDF(KICKER_CONSTANTS)};
    KICKER_CONFIG.componentsToState = componentStates -> new AngularPV_State(
        ((Motor_State) componentStates[0]).rad() / KICKER_CONSTANTS.reduction,
        ((Motor_State) componentStates[0]).radPs() / KICKER_CONSTANTS.reduction);
  }
}
