package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
import frc.robot.util.states.premade.RadPosVel_State;
import frc.robot.util.states.premade.RadPos_State;

public class ShooterConstants {
  public static final String LOG_NAME = "4_Shooter";

  public static final MechanismConstants<PosVel_State> HOOD_CONSTANTS = new MechanismConstants<>();
  public static final MechanismConfig<PosVel_State> HOOD_CONFIG =
      new MechanismConfig<PosVel_State>();

  public static final MechanismConstants<PosVel_State> RIO_FLYWHEEL_CONSTANTS =
      new MechanismConstants<>();
  public static final MechanismConfig<PosVel_State> RIO_FLYWHEEL_CONFIG =
      new MechanismConfig<PosVel_State>();

  public static final MechanismConstants<PosVel_State> PDH_FLYWHEEL_CONSTANTS =
      new MechanismConstants<>();
  public static final MechanismConfig<PosVel_State> PDH_FLYWHEEL_CONFIG =
      new MechanismConfig<PosVel_State>();

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
        .disableVoltageCompensation()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
    // HOOD_CONSTANTS.useAlternateEncoder = true;
    // Motor properties
    HOOD_CONSTANTS.reduction = (100.0 / 9.0) * 32;
    HOOD_CONSTANTS.gearbox = DCMotor.getNEO(1).withReduction(HOOD_CONSTANTS.reduction);
    HOOD_CONSTANTS.moi_kgm2 = .3;
    // Profiling
    HOOD_CONSTANTS.start_State = new RadPosVel_State(0, 0);
    HOOD_CONSTANTS.min_Pos = new RadPos_State(0); // -0.477
    HOOD_CONSTANTS.max_Pos = new RadPos_State(0.646); // .169
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
    RIO_FLYWHEEL_CONSTANTS.outputsLogName = LOG_NAME + "/RIO_Flywheel";
    RIO_FLYWHEEL_CONSTANTS.tuningLogName =
        RobotConstants.TUNING_PREFIX + RIO_FLYWHEEL_CONSTANTS.outputsLogName;
    RIO_FLYWHEEL_CONSTANTS.codePeriod_s = RobotConstants.CODE_PERIOD_s;
    // Motor
    RIO_FLYWHEEL_CONSTANTS.motorCanIds = new int[] {13, 14};
    RIO_FLYWHEEL_CONSTANTS.followerInversions = new boolean[] {false, true};
    RIO_FLYWHEEL_CONSTANTS.revMotorType = MotorType.kBrushless;
    RIO_FLYWHEEL_CONSTANTS.baseSparkConfig = new SparkMaxConfig();
    RIO_FLYWHEEL_CONSTANTS
        .baseSparkConfig
        .disableVoltageCompensation()
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(55);
    // Motor properties
    RIO_FLYWHEEL_CONSTANTS.reduction = 1d;
    RIO_FLYWHEEL_CONSTANTS.gearbox =
        DCMotor.getNeoVortex(2).withReduction(RIO_FLYWHEEL_CONSTANTS.reduction);
    RIO_FLYWHEEL_CONSTANTS.moi_kgm2 = .05;
    // Profiling
    RIO_FLYWHEEL_CONSTANTS.start_State =
        PosVel_State.create(new StateValue(0, Radians), new StateValue(0, RadiansPerSecond));
    RIO_FLYWHEEL_CONSTANTS.min_Pos = Pos_State.create(new StateValue(-Double.MAX_VALUE, Radians));
    RIO_FLYWHEEL_CONSTANTS.max_Pos = Pos_State.create(new StateValue(Double.MAX_VALUE, Radians));
    RIO_FLYWHEEL_CONSTANTS.limits_State = VelAcc_State.create(
        new StateValue(628, RadiansPerSecond), new StateValue(250, RadiansPerSecondPerSecond));
    // 75% of top speed should be 471 rad/sec (tuning)
    RIO_FLYWHEEL_CONSTANTS.isLoop = true;
    // Feedback
    RIO_FLYWHEEL_CONSTANTS.pid = RobotBase.isReal()
        ? new PIDController(0.006, 0, 0, RIO_FLYWHEEL_CONSTANTS.codePeriod_s)
        : new PIDController(0, 0, 0, RIO_FLYWHEEL_CONSTANTS.codePeriod_s);
    RIO_FLYWHEEL_CONSTANTS.simpleFF = RobotBase.isReal()
        ? new SimpleMotorFeedforward(.24, .0175, 0, RIO_FLYWHEEL_CONSTANTS.codePeriod_s)
        // from tuning: 0.13, 0.017, N/A
        : new SimpleMotorFeedforward(0, 0, 0, RIO_FLYWHEEL_CONSTANTS.codePeriod_s);

    RIO_FLYWHEEL_CONFIG.constants = RIO_FLYWHEEL_CONSTANTS;
    RIO_FLYWHEEL_CONFIG.realComponents = new ComponentBase[0];
    RIO_FLYWHEEL_CONFIG.simComponents = new ComponentSimBase[0];
    RIO_FLYWHEEL_CONFIG.realController = new SparkMaxController(RIO_FLYWHEEL_CONSTANTS);
    RIO_FLYWHEEL_CONFIG.simController = new DCMotorSimulator(RIO_FLYWHEEL_CONSTANTS);
    RIO_FLYWHEEL_CONFIG.profiles =
        new ControlFunctionBase[] {new TrapezoidProfileFunction(RIO_FLYWHEEL_CONSTANTS)};
    RIO_FLYWHEEL_CONFIG.feedbacks =
        new ControlFunctionBase[] {new SimplePIDF(RIO_FLYWHEEL_CONSTANTS, "RIO_SimplePIDF")};
    RIO_FLYWHEEL_CONFIG.componentsToState = componentStates -> PosVel_State.create(
        new StateValue(
            ((Motor_State) componentStates[0]).rad() / RIO_FLYWHEEL_CONSTANTS.reduction, Radians),
        new StateValue(
            ((Motor_State) componentStates[0]).radPs() / RIO_FLYWHEEL_CONSTANTS.reduction,
            RadiansPerSecond));
  }

  static {
    // Miscellaneous
    PDH_FLYWHEEL_CONSTANTS.outputsLogName = LOG_NAME + "/PDH_Flywheel";
    PDH_FLYWHEEL_CONSTANTS.tuningLogName =
        RobotConstants.TUNING_PREFIX + PDH_FLYWHEEL_CONSTANTS.outputsLogName;
    PDH_FLYWHEEL_CONSTANTS.codePeriod_s = RIO_FLYWHEEL_CONSTANTS.codePeriod_s;
    // Motor
    PDH_FLYWHEEL_CONSTANTS.motorCanIds = new int[] {15, 16};
    PDH_FLYWHEEL_CONSTANTS.followerInversions = new boolean[] {false, true};
    PDH_FLYWHEEL_CONSTANTS.revMotorType = RIO_FLYWHEEL_CONSTANTS.revMotorType;
    PDH_FLYWHEEL_CONSTANTS.baseSparkConfig = new SparkMaxConfig();
    PDH_FLYWHEEL_CONSTANTS
        .baseSparkConfig
        .disableVoltageCompensation()
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(55);
    // Motor properties
    PDH_FLYWHEEL_CONSTANTS.reduction = RIO_FLYWHEEL_CONSTANTS.reduction;
    PDH_FLYWHEEL_CONSTANTS.gearbox = RIO_FLYWHEEL_CONSTANTS.gearbox;
    PDH_FLYWHEEL_CONSTANTS.moi_kgm2 = RIO_FLYWHEEL_CONSTANTS.moi_kgm2;
    // Profiling
    PDH_FLYWHEEL_CONSTANTS.start_State = RIO_FLYWHEEL_CONSTANTS.start_State;
    PDH_FLYWHEEL_CONSTANTS.min_Pos = RIO_FLYWHEEL_CONSTANTS.min_Pos;
    PDH_FLYWHEEL_CONSTANTS.max_Pos = RIO_FLYWHEEL_CONSTANTS.max_Pos;
    PDH_FLYWHEEL_CONSTANTS.limits_State = RIO_FLYWHEEL_CONSTANTS.limits_State;
    // 75% of top speed should be 471 rad/sec (tuning)
    PDH_FLYWHEEL_CONSTANTS.isLoop = true;
    // Feedback
    PDH_FLYWHEEL_CONSTANTS.pid = RobotBase.isReal()
        ? new PIDController(0, 0, 0, PDH_FLYWHEEL_CONSTANTS.codePeriod_s)
        : new PIDController(0, 0, 0, PDH_FLYWHEEL_CONSTANTS.codePeriod_s);
    PDH_FLYWHEEL_CONSTANTS.simpleFF = RobotBase.isReal()
        ? new SimpleMotorFeedforward(0.13, 0.01704, 0, PDH_FLYWHEEL_CONSTANTS.codePeriod_s)
        // from tuning: 0.13, 0.017, N/A
        : new SimpleMotorFeedforward(0, 0, 0, PDH_FLYWHEEL_CONSTANTS.codePeriod_s);

    PDH_FLYWHEEL_CONFIG.constants = PDH_FLYWHEEL_CONSTANTS;
    PDH_FLYWHEEL_CONFIG.realComponents = new ComponentBase[0];
    PDH_FLYWHEEL_CONFIG.simComponents = new ComponentSimBase[0];
    PDH_FLYWHEEL_CONFIG.realController = new SparkMaxController(PDH_FLYWHEEL_CONSTANTS);
    PDH_FLYWHEEL_CONFIG.simController = new DCMotorSimulator(PDH_FLYWHEEL_CONSTANTS);
    PDH_FLYWHEEL_CONFIG.profiles =
        new ControlFunctionBase[] {new TrapezoidProfileFunction(PDH_FLYWHEEL_CONSTANTS)};
    PDH_FLYWHEEL_CONFIG.feedbacks =
        new ControlFunctionBase[] {new SimplePIDF(PDH_FLYWHEEL_CONSTANTS, "PDH_SimplePIDF")};
    PDH_FLYWHEEL_CONFIG.componentsToState = componentStates -> PosVel_State.create(
        new StateValue(
            ((Motor_State) componentStates[0]).rad() / PDH_FLYWHEEL_CONSTANTS.reduction, Radians),
        new StateValue(
            ((Motor_State) componentStates[0]).radPs() / PDH_FLYWHEEL_CONSTANTS.reduction,
            RadiansPerSecond));
  }
}
