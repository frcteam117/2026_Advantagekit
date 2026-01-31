package frc.robot.subsystems.kicker;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
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
import frc.robot.util.components.simulators.DCMotorSimulator;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.control_functions.feedback.SimplePIDF;
import frc.robot.util.control_functions.profiling.TrapezoidProfileFunction;
import frc.robot.util.control_functions.profiling.TrapezoidProfileFunction.TrapezoidProfileConfig;
import frc.robot.util.mechanisms.MechanismBase.MechanismConfig;

public class KickerContants {
  public static final String logName = "Kicker";

  public static final MechanismConfig<AngularPV_State> rollerConfig =
      new MechanismConfig<AngularPV_State>();

  static {
    String logName = KickerContants.logName + "/Roller";
    String tuningLogName = "Tuning/" + rollerConfig.logName;
    int[] canIds = new int[] {10, 11};
    boolean[] followerInversions = new boolean[] {false, true};
    MotorType revMotorType = MotorType.kBrushless;
    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    double reduction = 3;
    DCMotor dcMotor = DCMotor.getNEO(1).withReduction(reduction);
    double moi_kgm2 = .05;
    double period_s = Robot.codePeriod_s;
    AngularPV_State start_State = new AngularPV_State(0, 0);
    double max_radPs = 200;
    double max_radPs2 = 500;
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
