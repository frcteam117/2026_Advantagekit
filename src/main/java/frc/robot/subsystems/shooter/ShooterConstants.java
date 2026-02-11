package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotConstants;
import frc.robot.util.StateUtil.AngularPV_State;
import frc.robot.util.StateUtil.AngularP_State;
import frc.robot.util.StateUtil.AngularVA_State;
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

public class ShooterConstants {
  public static final String LOG_NAME = "4_Shooter";

  public static final MechanismConstants HOOD_CONSTANTS = new MechanismConstants();
  public static final MechanismConfig<AngularPV_State> HOOD_CONFIG =
      new MechanismConfig<AngularPV_State>();

  public static final MechanismConstants FLYWHEEL_CONSTANTS = new MechanismConstants();
  public static final MechanismConfig<AngularPV_State> FLYWHEEL_CONFIG =
      new MechanismConfig<AngularPV_State>();

  static {
    // Miscellaneous
    HOOD_CONSTANTS.outputsLogName = LOG_NAME + "/Hood";
    HOOD_CONSTANTS.tuningLogName = RobotConstants.TUNING_PREFIX + HOOD_CONSTANTS.outputsLogName;
    HOOD_CONSTANTS.codePeriod_s = RobotConstants.CODE_PERIOD_s;
    HOOD_CONSTANTS.mass_kg = 0.5;
    HOOD_CONSTANTS.cmOffset_Pos = new AngularP_State(-0.1);
    // Motor
    HOOD_CONSTANTS.motorCanIds = new int[] {9};
    HOOD_CONSTANTS.revMotorType = MotorType.kBrushless;
    HOOD_CONSTANTS.baseSparkConfig = new SparkMaxConfig();
    HOOD_CONSTANTS
        .baseSparkConfig
        .voltageCompensation(RobotConstants.NOMINAL_V)
        .smartCurrentLimit(30);
    // Motor properties
    HOOD_CONSTANTS.reduction = 30d;
    HOOD_CONSTANTS.gearbox = DCMotor.getNEO(1).withReduction(HOOD_CONSTANTS.reduction);
    HOOD_CONSTANTS.moi_kgm2 = .3;
    // Profiling
    HOOD_CONSTANTS.start_State = new AngularPV_State(0, 0);
    HOOD_CONSTANTS.min_Pos = new AngularP_State(0);
    HOOD_CONSTANTS.max_Pos = new AngularP_State(0.76);
    HOOD_CONSTANTS.limits_State = new AngularVA_State(4, 12);
    HOOD_CONSTANTS.isContinuous = false;
    // Feedback
    HOOD_CONSTANTS.pid = RobotBase.isReal()
        ? new PIDController(0, 0, 0, HOOD_CONSTANTS.codePeriod_s)
        : new PIDController(0, 0, 0, HOOD_CONSTANTS.codePeriod_s);
    HOOD_CONSTANTS.armFF = RobotBase.isReal()
        ? new ArmFeedforward(0, 0, 0, 0, HOOD_CONSTANTS.codePeriod_s)
        : new ArmFeedforward(0, 0, 0, 0, HOOD_CONSTANTS.codePeriod_s);

    HOOD_CONFIG.logName = HOOD_CONSTANTS.outputsLogName;
    HOOD_CONFIG.tuningLogName = HOOD_CONSTANTS.tuningLogName;
    HOOD_CONFIG.realComponents = new ComponentBase[0]; // {absoluteEncoderComponent};
    HOOD_CONFIG.simComponents = new ComponentSimBase[0]; // {absoluteEncoderSimComponent};
    HOOD_CONFIG.realController = new SparkMaxController(HOOD_CONSTANTS);
    HOOD_CONFIG.simController = new ArmSimulator(HOOD_CONSTANTS);
    HOOD_CONFIG.profiles = new ControlFunctionBase[] {new TrapezoidProfileFunction(HOOD_CONSTANTS)};
    HOOD_CONFIG.feedbacks = new ControlFunctionBase[] {new ArmPIDF(HOOD_CONSTANTS)};
    HOOD_CONFIG.componentsToState = componentStates -> new AngularPV_State(
        ((Motor_State) componentStates[0]).rad() / HOOD_CONSTANTS.reduction,
        ((Motor_State) componentStates[0]).radPs() / HOOD_CONSTANTS.reduction);
  }

  static {
    // Miscellaneous
    FLYWHEEL_CONSTANTS.outputsLogName = LOG_NAME + "/Flywheel";
    FLYWHEEL_CONSTANTS.tuningLogName =
        RobotConstants.TUNING_PREFIX + FLYWHEEL_CONSTANTS.outputsLogName;
    FLYWHEEL_CONSTANTS.codePeriod_s = RobotConstants.CODE_PERIOD_s;
    // Motor
    FLYWHEEL_CONSTANTS.motorCanIds = new int[] {10, 11};
    FLYWHEEL_CONSTANTS.followerInversions = new boolean[] {false, true};
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
    FLYWHEEL_CONSTANTS.start_State = new AngularPV_State(0, 0);
    FLYWHEEL_CONSTANTS.min_Pos = new AngularP_State(-Double.MAX_VALUE);
    FLYWHEEL_CONSTANTS.max_Pos = new AngularP_State(Double.MAX_VALUE);
    FLYWHEEL_CONSTANTS.limits_State = new AngularVA_State(628, 2000);
    FLYWHEEL_CONSTANTS.isContinuous = true;
    // Feedback
    FLYWHEEL_CONSTANTS.pid = RobotBase.isReal()
        ? new PIDController(0, 0, 0, FLYWHEEL_CONSTANTS.codePeriod_s)
        : new PIDController(0, 0, 0, FLYWHEEL_CONSTANTS.codePeriod_s);
    FLYWHEEL_CONSTANTS.simpleFF = RobotBase.isReal()
        ? new SimpleMotorFeedforward(0, 0, 0, FLYWHEEL_CONSTANTS.codePeriod_s)
        : new SimpleMotorFeedforward(0, 0, 0, FLYWHEEL_CONSTANTS.codePeriod_s);

    FLYWHEEL_CONFIG.logName = FLYWHEEL_CONSTANTS.outputsLogName;
    FLYWHEEL_CONFIG.tuningLogName = FLYWHEEL_CONSTANTS.tuningLogName;
    FLYWHEEL_CONFIG.realComponents = new ComponentBase[0];
    FLYWHEEL_CONFIG.simComponents = new ComponentSimBase[0];
    FLYWHEEL_CONFIG.realController = new SparkMaxController(FLYWHEEL_CONSTANTS);
    FLYWHEEL_CONFIG.simController = new DCMotorSimulator(FLYWHEEL_CONSTANTS);
    FLYWHEEL_CONFIG.profiles =
        new ControlFunctionBase[] {new TrapezoidProfileFunction(FLYWHEEL_CONSTANTS)};
    FLYWHEEL_CONFIG.feedbacks = new ControlFunctionBase[] {new SimplePIDF(FLYWHEEL_CONSTANTS)};
    FLYWHEEL_CONFIG.componentsToState = componentStates -> new AngularPV_State(
        ((Motor_State) componentStates[0]).rad() / FLYWHEEL_CONSTANTS.reduction,
        ((Motor_State) componentStates[0]).radPs() / FLYWHEEL_CONSTANTS.reduction);
  }
}
