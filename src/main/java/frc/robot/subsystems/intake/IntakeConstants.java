package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
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
import frc.robot.util.components.simulators.ArmSimulator;
import frc.robot.util.components.simulators.DCMotorSimulator;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.control_functions.feedback.ArmPIDF;
import frc.robot.util.control_functions.feedback.SimplePIDF;
import frc.robot.util.control_functions.profiling.TrapezoidProfileFunction;
import frc.robot.util.mechanisms.MechanismBase.MechanismConfig;
import frc.robot.util.mechanisms.MechanismConstants;

public class IntakeConstants {
  public static final String LOG_NAME = "2.Intake";

  public static final MechanismConstants PIVOT_CONSTANTS = new MechanismConstants();
  public static final MechanismConfig<AngularPV_State> PIVOT_CONFIG = new MechanismConfig<>();

  public static final MechanismConstants ROLLER_CONSTANTS = new MechanismConstants();
  public static final MechanismConfig<AngularPV_State> ROLLER_CONFIG = new MechanismConfig<>();

  static {
    // Miscellaneous
    PIVOT_CONSTANTS.outputsLogName = LOG_NAME + "/Pivot";
    PIVOT_CONSTANTS.tuningLogName = RobotConstants.TUNING_PREFIX + PIVOT_CONSTANTS.outputsLogName;
    PIVOT_CONSTANTS.codePeriod_s = RobotConstants.CODE_PERIOD_s;
    PIVOT_CONSTANTS.mass_kg = 0.5;
    PIVOT_CONSTANTS.cmOffset_Pos = new AngularP_State(-0.2);
    // Motor
    PIVOT_CONSTANTS.motorCanIds = new int[] {12};
    PIVOT_CONSTANTS.revMotorType = MotorType.kBrushless;
    PIVOT_CONSTANTS.baseSparkConfig = new SparkMaxConfig();
    PIVOT_CONSTANTS
        .baseSparkConfig
        .voltageCompensation(RobotConstants.NOMINAL_V)
        .smartCurrentLimit(30);
    // Motor properties
    PIVOT_CONSTANTS.reduction = 30d;
    PIVOT_CONSTANTS.gearbox = DCMotor.getNEO(1).withReduction(PIVOT_CONSTANTS.reduction);
    PIVOT_CONSTANTS.moi_kgm2 = .3;
    // Profiling
    PIVOT_CONSTANTS.start_State = new AngularPV_State(0.5, 0);
    PIVOT_CONSTANTS.min_Pos = new AngularP_State(0.5);
    PIVOT_CONSTANTS.max_Pos = new AngularP_State(2.1);
    PIVOT_CONSTANTS.limits_State = new AngularVA_State(4, 12);
    PIVOT_CONSTANTS.isContinuous = false;
    // Feedback
    PIVOT_CONSTANTS.pid = RobotBase.isReal()
        ? new PIDController(0, 0, 0, PIVOT_CONSTANTS.codePeriod_s)
        : new PIDController(0, 0, 0, PIVOT_CONSTANTS.codePeriod_s);
    PIVOT_CONSTANTS.armFF = RobotBase.isReal()
        ? new ArmFeedforward(0, 0, 0, 0, PIVOT_CONSTANTS.codePeriod_s)
        : new ArmFeedforward(0, 0, 0, 0, PIVOT_CONSTANTS.codePeriod_s);

    PIVOT_CONFIG.logName = PIVOT_CONSTANTS.outputsLogName;
    PIVOT_CONFIG.tuningLogName = PIVOT_CONSTANTS.tuningLogName;
    PIVOT_CONFIG.realComponents = new ComponentBase[0]; // {absoluteEncoderComponent};
    PIVOT_CONFIG.simComponents = new ComponentSimBase[0]; // {absoluteEncoderSimComponent};
    PIVOT_CONFIG.realController = new SparkMaxController(PIVOT_CONSTANTS);
    PIVOT_CONFIG.simController = new ArmSimulator(PIVOT_CONSTANTS);
    PIVOT_CONFIG.profiles =
        new ControlFunctionBase[] {new TrapezoidProfileFunction(PIVOT_CONSTANTS)};
    PIVOT_CONFIG.feedbacks = new ControlFunctionBase[] {new ArmPIDF(PIVOT_CONSTANTS)};
    PIVOT_CONFIG.componentsToState = componentStates -> new AngularPV_State(
        ((Motor_State) componentStates[0]).rad() / PIVOT_CONSTANTS.reduction,
        ((Motor_State) componentStates[0]).radPs() / PIVOT_CONSTANTS.reduction);
  }

  static {
    // Miscellaneous
    ROLLER_CONSTANTS.outputsLogName = LOG_NAME + "/Roller";
    ROLLER_CONSTANTS.tuningLogName = RobotConstants.TUNING_PREFIX + ROLLER_CONSTANTS.outputsLogName;
    ROLLER_CONSTANTS.codePeriod_s = RobotConstants.CODE_PERIOD_s;
    // Motor
    ROLLER_CONSTANTS.motorCanIds = new int[] {13};
    ROLLER_CONSTANTS.revMotorType = MotorType.kBrushless;
    ROLLER_CONSTANTS.baseSparkConfig = new SparkMaxConfig();
    ROLLER_CONSTANTS
        .baseSparkConfig
        .voltageCompensation(RobotConstants.NOMINAL_V)
        .smartCurrentLimit(30);
    // Motor properties
    ROLLER_CONSTANTS.reduction = 5d;
    ROLLER_CONSTANTS.gearbox = DCMotor.getNEO(1).withReduction(ROLLER_CONSTANTS.reduction);
    ROLLER_CONSTANTS.moi_kgm2 = .03;
    // Profiling
    ROLLER_CONSTANTS.start_State = new AngularPV_State(0, 0);
    ROLLER_CONSTANTS.min_Pos = new AngularP_State(-Double.MAX_VALUE);
    ROLLER_CONSTANTS.max_Pos = new AngularP_State(Double.MAX_VALUE);
    ROLLER_CONSTANTS.limits_State = new AngularVA_State(100, 400);
    ROLLER_CONSTANTS.isContinuous = true;
    // Feedback
    ROLLER_CONSTANTS.pid = RobotBase.isReal()
        ? new PIDController(0, 0, 0, ROLLER_CONSTANTS.codePeriod_s)
        : new PIDController(0, 0, 0, ROLLER_CONSTANTS.codePeriod_s);
    ROLLER_CONSTANTS.simpleFF = RobotBase.isReal()
        ? new SimpleMotorFeedforward(0, 0, 0, ROLLER_CONSTANTS.codePeriod_s)
        : new SimpleMotorFeedforward(0, 0, 0, ROLLER_CONSTANTS.codePeriod_s);

    ROLLER_CONFIG.logName = ROLLER_CONSTANTS.outputsLogName;
    ROLLER_CONFIG.tuningLogName = ROLLER_CONSTANTS.tuningLogName;
    ROLLER_CONFIG.realComponents = new ComponentBase[0];
    ROLLER_CONFIG.simComponents = new ComponentSimBase[0];
    ROLLER_CONFIG.realController = new SparkMaxController(ROLLER_CONSTANTS);
    ROLLER_CONFIG.simController = new DCMotorSimulator(ROLLER_CONSTANTS);
    ROLLER_CONFIG.profiles =
        new ControlFunctionBase[] {new TrapezoidProfileFunction(ROLLER_CONSTANTS)};
    ROLLER_CONFIG.feedbacks = new ControlFunctionBase[] {new SimplePIDF(ROLLER_CONSTANTS)};
    ROLLER_CONFIG.componentsToState = componentStates -> new AngularPV_State(
        ((Motor_State) componentStates[0]).rad() / ROLLER_CONSTANTS.reduction,
        ((Motor_State) componentStates[0]).radPs() / ROLLER_CONSTANTS.reduction);
  }
}
