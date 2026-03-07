package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotConstants;
import frc.robot.util.components.bases.ComponentBase;
import frc.robot.util.components.bases.ComponentSimBase;
import frc.robot.util.components.bases.ComponentStates.Motor_State;
import frc.robot.util.components.controllers.SparkMaxController;
import frc.robot.util.components.simulators.ArmSimulator;
import frc.robot.util.components.simulators.DCMotorSimulator;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.control_functions.feedback.ArbitraryPIDSVAF;
import frc.robot.util.control_functions.feedback.SimplePIDF;
import frc.robot.util.control_functions.profiling.TrapezoidProfileFunction;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.mechanisms.MechanismBase.MechanismConfig;
import frc.robot.util.mechanisms.MechanismConstants;
import frc.robot.util.states.StateValue;
import frc.robot.util.states.bases.PosVel_State;
import frc.robot.util.states.bases.VelAcc_State;
import frc.robot.util.states.premade.RadPosVel_State;
import frc.robot.util.states.premade.RadPos_State;

public class IntakeConstants {
  public static final String LOG_NAME = "2_Intake";

  public static final MechanismConstants<PosVel_State> PIVOT_CONSTANTS = new MechanismConstants<>();
  public static final MechanismConfig<PosVel_State> PIVOT_CONFIG = new MechanismConfig<>();

  public static final MechanismConstants<PosVel_State> ROLLER_CONSTANTS =
      new MechanismConstants<>();
  public static final MechanismConfig<PosVel_State> ROLLER_CONFIG = new MechanismConfig<>();

  static {
    // Miscellaneous
    PIVOT_CONSTANTS.outputsLogName = LOG_NAME + "/Pivot";
    PIVOT_CONSTANTS.tuningLogName = RobotConstants.TUNING_PREFIX + PIVOT_CONSTANTS.outputsLogName;
    PIVOT_CONSTANTS.codePeriod_s = RobotConstants.CODE_PERIOD_s;
    PIVOT_CONSTANTS.mass_kg = 0.5;
    PIVOT_CONSTANTS.cmOffset_rad = (-108 * Math.PI / 180); // - ((1.43256625 + 1.54566358557) / 2);
    // Motor
    PIVOT_CONSTANTS.motorCanIds = new int[] {12};
    PIVOT_CONSTANTS.revMotorType = MotorType.kBrushless;
    PIVOT_CONSTANTS.baseSparkConfig = new SparkMaxConfig();
    PIVOT_CONSTANTS
        .baseSparkConfig
        .disableVoltageCompensation()
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(80)
        .inverted(true)
        .encoder
        .quadratureMeasurementPeriod(10)
        .quadratureAverageDepth(4);
    // Motor properties
    PIVOT_CONSTANTS.reduction = 23.80952381;
    PIVOT_CONSTANTS.gearbox = DCMotor.getNEO(1).withReduction(PIVOT_CONSTANTS.reduction);
    PIVOT_CONSTANTS.moi_kgm2 = .3;
    // Profiling
    PIVOT_CONSTANTS.start_State = new RadPosVel_State(
        0, 0); // 5.643, 5.929, -0.214, 0.214 rotor rotations with positive being farther down
    PIVOT_CONSTANTS.min_Pos = new RadPos_State(-1.495396); // -1.54566358557);
    PIVOT_CONSTANTS.max_Pos = new RadPos_State(0);
    PIVOT_CONSTANTS.limits_State = VelAcc_State.create(
        new StateValue(4, RadiansPerSecond), new StateValue(12, RadiansPerSecondPerSecond));
    PIVOT_CONSTANTS.isLoop = false;
    // Feedback
    PIVOT_CONSTANTS.pid = RobotBase.isReal()
        ? new PIDController(0, 0, 0, PIVOT_CONSTANTS.codePeriod_s)
        : new PIDController(0, 0, 0, PIVOT_CONSTANTS.codePeriod_s);
    PIVOT_CONSTANTS.simpleFF = RobotBase.isReal()
        ? new SimpleMotorFeedforward(0, 0, 0, PIVOT_CONSTANTS.codePeriod_s)
        : new SimpleMotorFeedforward(0, 0, 0, PIVOT_CONSTANTS.codePeriod_s);
    PIVOT_CONSTANTS.maxPosError = .02;
    PIVOT_CONSTANTS.minVel = .02;
    PIVOT_CONSTANTS.arbitraryForwardS = new InterpolatingDoubleTreeMap();
    PIVOT_CONSTANTS.arbitraryReverseS = new InterpolatingDoubleTreeMap();
    PIVOT_CONSTANTS.arbitraryForwardS.put(0.1, 0.0);
    PIVOT_CONSTANTS.arbitraryForwardS.put(-0.1, 0.0);
    PIVOT_CONSTANTS.arbitraryForwardS.put(-0.4, 0.0);
    PIVOT_CONSTANTS.arbitraryForwardS.put(-0.7, 0.0);
    PIVOT_CONSTANTS.arbitraryForwardS.put(-1.0, 0.0);
    PIVOT_CONSTANTS.arbitraryForwardS.put(-1.3, 0.0);
    PIVOT_CONSTANTS.arbitraryForwardS.put(-1.6, 0.0);
    PIVOT_CONSTANTS.arbitraryReverseS.put(0.1, 0.0);
    PIVOT_CONSTANTS.arbitraryReverseS.put(-0.1, 0.0);
    PIVOT_CONSTANTS.arbitraryReverseS.put(-0.4, 0.0);
    PIVOT_CONSTANTS.arbitraryReverseS.put(-0.7, 0.0);
    PIVOT_CONSTANTS.arbitraryReverseS.put(-1.0, 0.0);
    PIVOT_CONSTANTS.arbitraryReverseS.put(-1.3, 0.0);
    PIVOT_CONSTANTS.arbitraryReverseS.put(-1.6, 0.0);
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryForwardS/_0.1",
        PIVOT_CONSTANTS.arbitraryForwardS.get(0.1),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryForwardS.put(0.1, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryForwardS/-0.1",
        PIVOT_CONSTANTS.arbitraryForwardS.get(-0.1),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryForwardS.put(-0.1, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryForwardS/-0.4",
        PIVOT_CONSTANTS.arbitraryForwardS.get(-0.4),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryForwardS.put(-0.4, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryForwardS/-0.7",
        PIVOT_CONSTANTS.arbitraryForwardS.get(-0.7),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryForwardS.put(-0.7, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryForwardS/-1.0",
        PIVOT_CONSTANTS.arbitraryForwardS.get(-1.0),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryForwardS.put(-1.0, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryForwardS/-1.3",
        PIVOT_CONSTANTS.arbitraryForwardS.get(-1.3),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryForwardS.put(-1.3, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryForwardS/-1.6",
        PIVOT_CONSTANTS.arbitraryForwardS.get(-1.6),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryForwardS.put(-1.6, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryReverseS/_0.1",
        PIVOT_CONSTANTS.arbitraryReverseS.get(0.1),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryReverseS.put(0.1, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryReverseS/-0.1",
        PIVOT_CONSTANTS.arbitraryReverseS.get(-0.1),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryReverseS.put(-0.1, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryReverseS/-0.4",
        PIVOT_CONSTANTS.arbitraryReverseS.get(-0.4),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryReverseS.put(-0.4, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryReverseS/-0.7",
        PIVOT_CONSTANTS.arbitraryReverseS.get(-0.7),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryReverseS.put(-0.7, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryReverseS/-1.0",
        PIVOT_CONSTANTS.arbitraryReverseS.get(-1.0),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryReverseS.put(-1.0, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryReverseS/-1.3",
        PIVOT_CONSTANTS.arbitraryReverseS.get(-1.3),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryReverseS.put(-1.3, value));
    new TunableDouble(
        "Tuning/" + LOG_NAME + "/ArbitraryReverseS/-1.6",
        PIVOT_CONSTANTS.arbitraryReverseS.get(-1.6),
        () -> true,
        value -> PIVOT_CONSTANTS.arbitraryReverseS.put(-1.6, value));

    PIVOT_CONFIG.constants = PIVOT_CONSTANTS;
    PIVOT_CONFIG.realComponents = new ComponentBase[0]; // {absoluteEncoderComponent};
    PIVOT_CONFIG.simComponents = new ComponentSimBase[0]; // {absoluteEncoderSimComponent};
    PIVOT_CONFIG.realController = new SparkMaxController(PIVOT_CONSTANTS);
    PIVOT_CONFIG.simController = new ArmSimulator(PIVOT_CONSTANTS);
    PIVOT_CONFIG.profiles =
        new ControlFunctionBase[] {new TrapezoidProfileFunction(PIVOT_CONSTANTS)};
    PIVOT_CONFIG.feedbacks = new ControlFunctionBase[] {new ArbitraryPIDSVAF(PIVOT_CONSTANTS)};
    PIVOT_CONFIG.componentsToState = componentStates -> new RadPosVel_State(
        ((Motor_State) componentStates[0]).rad() / PIVOT_CONSTANTS.reduction,
        ((Motor_State) componentStates[0]).radPs() / PIVOT_CONSTANTS.reduction);
  }

  static {
    // Miscellaneous
    ROLLER_CONSTANTS.outputsLogName = LOG_NAME + "/Roller";
    ROLLER_CONSTANTS.tuningLogName = RobotConstants.TUNING_PREFIX + ROLLER_CONSTANTS.outputsLogName;
    ROLLER_CONSTANTS.codePeriod_s = RobotConstants.CODE_PERIOD_s;
    // Motor
    ROLLER_CONSTANTS.motorCanIds = new int[] {11};
    ROLLER_CONSTANTS.revMotorType = MotorType.kBrushless;
    ROLLER_CONSTANTS.baseSparkConfig = new SparkMaxConfig();
    ROLLER_CONSTANTS
        .baseSparkConfig
        .disableVoltageCompensation()
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(60)
        .encoder
        .quadratureMeasurementPeriod(10)
        .quadratureAverageDepth(4);
    // Motor properties
    ROLLER_CONSTANTS.reduction = 5d;
    ROLLER_CONSTANTS.gearbox = DCMotor.getNEO(1).withReduction(ROLLER_CONSTANTS.reduction);
    ROLLER_CONSTANTS.moi_kgm2 = .03;
    // Profiling
    ROLLER_CONSTANTS.start_State =
        PosVel_State.create(new StateValue(0, Radians), new StateValue(0, RadiansPerSecond));
    ROLLER_CONSTANTS.min_Pos = new RadPos_State(-Double.MAX_VALUE);
    ROLLER_CONSTANTS.max_Pos = new RadPos_State(Double.MAX_VALUE);
    ROLLER_CONSTANTS.limits_State = VelAcc_State.create(
        new StateValue(100, RadiansPerSecond), new StateValue(300, RadiansPerSecondPerSecond));
    ROLLER_CONSTANTS.isLoop = true;
    // Feedback
    ROLLER_CONSTANTS.pid = RobotBase.isReal()
        ? new PIDController(0.02, 0, 0, ROLLER_CONSTANTS.codePeriod_s)
        : new PIDController(0, 0, 0, ROLLER_CONSTANTS.codePeriod_s);
    ROLLER_CONSTANTS.simpleFF = RobotBase.isReal()
        ? new SimpleMotorFeedforward(0, 0.113, 0, ROLLER_CONSTANTS.codePeriod_s)
        : new SimpleMotorFeedforward(0, 0, 0, ROLLER_CONSTANTS.codePeriod_s);

    ROLLER_CONFIG.constants = ROLLER_CONSTANTS;
    ROLLER_CONFIG.realComponents = new ComponentBase[0];
    ROLLER_CONFIG.simComponents = new ComponentSimBase[0];
    ROLLER_CONFIG.realController = new SparkMaxController(ROLLER_CONSTANTS);
    ROLLER_CONFIG.simController = new DCMotorSimulator(ROLLER_CONSTANTS);
    ROLLER_CONFIG.profiles =
        new ControlFunctionBase[] {new TrapezoidProfileFunction(ROLLER_CONSTANTS)};
    ROLLER_CONFIG.feedbacks = new ControlFunctionBase[] {new SimplePIDF(ROLLER_CONSTANTS)};
    ROLLER_CONFIG.componentsToState = componentStates -> new RadPosVel_State(
        ((Motor_State) componentStates[0]).rad() / ROLLER_CONSTANTS.reduction,
        ((Motor_State) componentStates[0]).radPs() / ROLLER_CONSTANTS.reduction);
  }
}
