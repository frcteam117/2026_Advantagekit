package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.Robot;
import frc.robot.util.States.AngularPV_State;
import frc.robot.util.States.AngularP_State;
import frc.robot.util.States.AngularVA_State;
import frc.robot.util.components.bases.ComponentBase;
import frc.robot.util.components.bases.ComponentSimBase;
import frc.robot.util.components.bases.ComponentStates.Motor_State;
import frc.robot.util.components.controllers.SparkMaxController;
import frc.robot.util.components.controllers.SparkMaxController.SparkMaxControllerConfig;
import frc.robot.util.components.simulators.ArmSimulator;
import frc.robot.util.components.simulators.ArmSimulator.ArmSimulatorConfig;
import frc.robot.util.components.simulators.DCMotorSimulator;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.control_functions.feedback.ArmPIDF;
import frc.robot.util.control_functions.feedback.SimplePIDF;
import frc.robot.util.control_functions.profiling.TrapezoidProfileFunction;
import frc.robot.util.control_functions.profiling.TrapezoidProfileFunction.TrapezoidProfileConfig;
import frc.robot.util.mechanisms.MechanismBase.MechanismConfig;

public class IntakeConstants {
  public static final String logName = "Intake";

  public static final MechanismConfig<AngularPV_State> armConfig =
      new MechanismConfig<AngularPV_State>();

  public static final MechanismConfig<AngularPV_State> rollerConfig =
      new MechanismConfig<AngularPV_State>();

  static {
    // 6 bar constants
    String logName = IntakeConstants.logName + "/Arm";
    String tuningLogName = "Tuning/" + armConfig.logName;
    int[] canIds = new int[] {9};
    MotorType revMotorType = MotorType.kBrushless;
    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    double reduction = 30;
    DCMotor dcMotor = DCMotor.getNEO(1).withReduction(reduction);
    double moi_kgm2 = .02;
    double cmOffset_rad = -.1;
    double length_m =
        0.18; // change this to be a function of cm radius and moi make simulation realistic
    double period_s = Robot.codePeriod_s;
    double min_rad = 0;
    double max_rad = Math.PI / 1.8;
    AngularPV_State start_State = new AngularPV_State(Math.PI / 1.8, 0);
    double max_radPs = 4;
    double max_radPs2 = 12;
    PIDController realPID = new PIDController(0, 0, 0, period_s);
    PIDController simPID = new PIDController(0, 0, 0, period_s);
    ArmFeedforward realFF = new ArmFeedforward(0, 0, 0, 0, period_s);
    ArmFeedforward simFF = new ArmFeedforward(0, 0, 0, 0, period_s);

    sparkConfig.voltageCompensation(Robot.nominal_V).smartCurrentLimit(30);

    armConfig.logName = logName;
    armConfig.realComponents = new ComponentBase[0]; // {absoluteEncoderComponent};
    armConfig.simComponents = new ComponentSimBase[0]; // {absoluteEncoderSimComponent};

    SparkMaxControllerConfig sparkMaxControllerConfig = new SparkMaxControllerConfig();
    sparkMaxControllerConfig.canIds = canIds;
    sparkMaxControllerConfig.baseMotorType = revMotorType;
    sparkMaxControllerConfig.baseSparkConfig = sparkConfig;
    armConfig.realController = new SparkMaxController(sparkMaxControllerConfig);

    ArmSimulatorConfig hoodSimConfig = new ArmSimulatorConfig();
    hoodSimConfig.canIds = canIds;
    hoodSimConfig.cmOffset_rad = cmOffset_rad;
    hoodSimConfig.reduction = reduction;
    hoodSimConfig.gearbox = dcMotor;
    hoodSimConfig.length_m = length_m;
    hoodSimConfig.min_rad = min_rad;
    hoodSimConfig.max_rad = max_rad;
    hoodSimConfig.start_rad = start_State.rad();
    hoodSimConfig.plant = LinearSystemId.createSingleJointedArmSystem(dcMotor, moi_kgm2, reduction);
    armConfig.simController = new ArmSimulator(hoodSimConfig);

    TrapezoidProfileConfig trapezoidProfileConfig = new TrapezoidProfileConfig();
    trapezoidProfileConfig.mechanismTuningLogName = tuningLogName;
    trapezoidProfileConfig.period_s = period_s;
    trapezoidProfileConfig.start = start_State;
    trapezoidProfileConfig.constraints = new AngularVA_State(max_radPs, max_radPs2);
    trapezoidProfileConfig.min = new AngularP_State(min_rad);
    trapezoidProfileConfig.max = new AngularP_State(max_rad);
    armConfig.profiles =
        new ControlFunctionBase[] {new TrapezoidProfileFunction(trapezoidProfileConfig)};
    armConfig.feedbacks = new ControlFunctionBase[] {
      new ArmPIDF(
          RobotBase.isReal() ? realPID : simPID,
          RobotBase.isReal() ? realFF : simFF,
          start_State,
          cmOffset_rad,
          tuningLogName)
    };
    armConfig.componentsToState = componentStates -> new AngularPV_State(
        ((Motor_State) componentStates[0]).rad() / reduction,
        ((Motor_State) componentStates[0]).radPs() / reduction);
  }

  static {
    // Roller Constants
    String logName = IntakeConstants.logName + "/Roller";
    String tuningLogName = "Tuning/" + rollerConfig.logName;
    int[] canIds = new int[] {10, 11};
    boolean[] followerInversions = new boolean[] {false, true};
    MotorType revMotorType = MotorType.kBrushless;
    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    double reduction = 5;
    DCMotor dcMotor = DCMotor.getNEO(1).withReduction(reduction);
    double moi_kgm2 = .03;
    double period_s = Robot.codePeriod_s;
    AngularPV_State start_State = new AngularPV_State(0, 0);
    double max_radPs = 100;
    double max_radPs2 = 400;
    PIDController realPID = new PIDController(0, 0, 0, period_s);
    PIDController simPID = new PIDController(0, 0, 0, period_s);
    SimpleMotorFeedforward realFF = new SimpleMotorFeedforward(0, 0, 0, period_s);
    SimpleMotorFeedforward simFF = new SimpleMotorFeedforward(0, 0, 0, period_s);

    sparkConfig.voltageCompensation(Robot.nominal_V).smartCurrentLimit(30);

    rollerConfig.logName = logName;
    rollerConfig.realComponents = new ComponentBase[0];
    rollerConfig.simComponents = new ComponentSimBase[0];

    SparkMaxControllerConfig sparkMaxControllerConfig = new SparkMaxControllerConfig();
    sparkMaxControllerConfig.canIds = canIds;
    sparkMaxControllerConfig.followerInversions = followerInversions;
    sparkMaxControllerConfig.baseMotorType = revMotorType;
    sparkMaxControllerConfig.baseSparkConfig = sparkConfig;
    rollerConfig.realController = new SparkMaxController(sparkMaxControllerConfig);

    rollerConfig.simController = new DCMotorSimulator(
        new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, moi_kgm2, reduction), dcMotor),
        canIds);

    TrapezoidProfileConfig trapezoidProfileConfig = new TrapezoidProfileConfig();
    trapezoidProfileConfig.mechanismTuningLogName = tuningLogName;
    trapezoidProfileConfig.period_s = period_s;
    trapezoidProfileConfig.start = start_State;
    trapezoidProfileConfig.constraints = new AngularVA_State(max_radPs, max_radPs2);
    trapezoidProfileConfig.min = new AngularP_State(-Double.MAX_VALUE);
    trapezoidProfileConfig.max = new AngularP_State(Double.MAX_VALUE);
    trapezoidProfileConfig.enableContinuousInput = true;
    rollerConfig.profiles =
        new ControlFunctionBase[] {new TrapezoidProfileFunction(trapezoidProfileConfig)};
    rollerConfig.feedbacks = new ControlFunctionBase[] {
      new SimplePIDF(
          RobotBase.isReal() ? realPID : simPID,
          RobotBase.isReal() ? realFF : simFF,
          start_State,
          tuningLogName)
    };
    rollerConfig.componentsToState = componentStates -> new AngularPV_State(
        ((Motor_State) componentStates[0]).rad() / reduction,
        ((Motor_State) componentStates[0]).radPs() / reduction);
  }
}
