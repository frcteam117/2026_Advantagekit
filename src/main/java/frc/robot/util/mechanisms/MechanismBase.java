package frc.robot.util.mechanisms;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Robot;
import frc.robot.util.States.State;
import frc.robot.util.States.Voltage_State;
import frc.robot.util.SysIdUtil;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.components.bases.ComponentBase;
import frc.robot.util.components.bases.ComponentControllerBase;
import frc.robot.util.components.bases.ComponentSimBase;
import frc.robot.util.components.bases.ComponentSimControllerBase;
import frc.robot.util.components.bases.ComponentStates.ComponentState;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.TunableBoolean;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MechanismBase<Output_State extends State> {
  private final List<ComponentBase> components;
  private final List<ComponentSimBase> simulators;
  private final ComponentControllerBase controller;

  private ComponentState[] componentStates;
  private final String[] componentNames;
  private final Function<ComponentState[], Output_State> componentsToState;
  private Output_State mechanism_State;

  private final ControlFunctionBase[] profiles;
  private final ControlFunctionBase[] feedbacks;
  private int activeProfile = 0;
  private int activeFeedback = 0;
  private String goalStateName = "";
  private String nextStateName = "";
  private String outputStateName = "";
  private State goalStateNaN;
  private State nextStateNaN;
  private State outputStateNaN;

  private final SysIdRoutine sysId;
  private final Timer timer = new Timer();
  private final double[] times = new double[11];

  private final String logName;
  private final ComponentStatesLogger componentStatesLogger = new ComponentStatesLogger();

  public MechanismBase(MechanismConfig<Output_State> config, Subsystem subsystem) {
    logName = config.logName;
    profiles = config.profiles;
    feedbacks = config.feedbacks;
    if (RobotBase.isReal()) {
      simulators = new ArrayList<>(0);
      components = new ArrayList<>(config.realComponents.length + 1);
      for (ComponentBase component : config.realComponents) {
        components.add(component);
      }
      components.add(config.realController);
      controller = config.realController;
    } else {
      simulators = new ArrayList<>(config.simComponents.length + 1);
      for (ComponentSimBase component : config.simComponents) {
        simulators.add(component);
      }
      simulators.add(config.simController);
      components =
          simulators.stream().map(simulator -> (ComponentBase) simulator).toList();
      // if (DCMotorSimulator.class.isInstance(config.simController)) {
      //   simulators.remove(0);
      // }
      controller = config.simController;
    }
    componentsToState = config.componentsToState;

    List<String> componentNamesList = components.stream()
        .flatMap(component -> Arrays.stream(component.getComponentNames()))
        .toList();
    componentNames = new String[componentNamesList.size()];
    for (int i = 0; i < componentNames.length; i++) {
      componentNames[i] = componentNamesList.get(i);
    }
    componentStates = new ComponentState[componentNames.length];

    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(config.sysIdRampRate_VPs).per(Second),
            Volts.of(config.sysIdStepVoltage_V),
            Seconds.of(config.sysIdTimeout_s),
            (state) -> Logger.recordOutput(logName + "/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> setVoltageSysId(voltage), null, subsystem));

    new TunableBoolean(config.tuningLogName + "/.Tunable", false, () -> true, value -> {
      Arrays.stream(profiles).forEach(profile -> profile.setTunable(value));
      Arrays.stream(feedbacks).forEach(feedback -> feedback.setTunable(value));
    });
  }

  public void update() {
    int i = 0;
    for (ComponentBase component : components) {
      for (ComponentState state : component.getState()) {
        componentStates[i] = state;
        i++;
      }
    }
    Logger.processInputs(logName, componentStatesLogger);
    mechanism_State = componentsToState.apply(componentStates);
    Logger.recordOutput(logName + "/State", (Record) mechanism_State);
    // Logger.recordOutput("Tuning/test", 1);
    for (ComponentSimBase simulator : simulators) {
      simulator.updateState(Robot.codePeriod_s);
    }
    // MechanismState newMechanismState = componentStateAverager.apply(componentStates);
    // if (mechanismState == null) {
    //   mechanismState = newMechanismState;
    // }
    // if (!newMechanismState.getClass().isInstance(mechanismState)) {
    //   Logger.recordOutput(logName + "/State_" + mechanismState.getShortName(), (Record)
    //       mechanismState.getNaNState());
    // }
    // mechanismState = newMechanismState;
    // Logger.recordOutput(
    //     logName + "/State_" + mechanismState.getShortName(), (Record) mechanismState);
  }

  public void setGoal(State goalState, int usedProfile, int usedFeedback) {
    activeProfile = usedProfile;
    activeFeedback = usedFeedback;
    setGoal(goalState);
  }

  public void setGoal(State goal_State) {
    State next_State = nextStateNaN;
    State output_State = outputStateNaN;
    for (int i = 0; i < profiles.length; i++) {
      if (i == activeProfile) {
        next_State = profiles[i].calculate(goal_State, mechanism_State);
      } else {
        profiles[i].updateState(mechanism_State);
      }
    }
    for (int i = 0; i < feedbacks.length; i++) {
      if (i == activeFeedback) {
        output_State = feedbacks[i].calculate(next_State, mechanism_State);
      } else {
        feedbacks[i].updateState(mechanism_State);
      }
    }
    Logger.recordOutput(
        logName + "/Active_Profile", profiles[activeProfile].getControlFunctionName());
    Logger.recordOutput(
        logName + "/Active_Feedback", feedbacks[activeFeedback].getControlFunctionName());

    if (goalStateName != goal_State.getShortName()) {
      if (goalStateName != null) {
        Logger.recordOutput(logName + "/Goal_" + goalStateName, (Record) goalStateNaN);
      }
      goalStateName = goal_State.getShortName();
      goalStateNaN = goal_State.getNaNState();
    }
    if (nextStateName != next_State.getShortName()) {
      if (nextStateName != null) {
        Logger.recordOutput(logName + "/Next_" + nextStateName, (Record) nextStateNaN);
      }
      nextStateName = next_State.getShortName();
      nextStateNaN = next_State.getNaNState();
    }
    if (outputStateName != output_State.getShortName()) {
      if (outputStateName != null) {
        Logger.recordOutput(logName + "/Input_" + outputStateName, (Record) outputStateNaN);
      }
      outputStateName = output_State.getShortName();
      outputStateNaN = output_State.getNaNState();
    }

    Logger.recordOutput(logName + "/Goal_" + goalStateName, (Record) goal_State);
    Logger.recordOutput(logName + "/Next_" + nextStateName, (Record) next_State);
    Logger.recordOutput(logName + "/Input_" + outputStateName, (Record) output_State);
    controller.setInput(output_State);
  }

  protected void setVoltageSysId(Voltage V) {
    Voltage_State voltage_State = new Voltage_State(V.in(Volts));
    Arrays.stream(profiles).forEach(profile -> profile.updateState(mechanism_State));
    Arrays.stream(feedbacks).forEach(feedback -> feedback.updateState(mechanism_State));

    Logger.recordOutput(logName + "/Active_Profile", "SysIdRoutine");
    Logger.recordOutput(logName + "/Active_Feedback", "SysIdRoutine");
    Logger.recordOutput(logName + "/Goal_" + goalStateName, (Record) goalStateNaN);
    Logger.recordOutput(logName + "/Next_" + nextStateName, (Record) nextStateNaN);
    Logger.recordOutput(logName + "/Input_" + outputStateName, (Record) outputStateNaN);
    Logger.recordOutput(logName + "/Input_" + voltage_State.getShortName(), voltage_State);
    controller.setInput(voltage_State);
  }

  public Command getSysIdCommand(SysIdType type) {
    return SysIdUtil.getSysIdCommand(sysId, type);
  }

  public Output_State getState() {
    return mechanism_State;
  }

  public class ComponentStatesLogger implements LoggableInputs {
    @Override
    public void toLog(LogTable table) {
      for (int i = 0; i < componentStates.length; i++) {
        table.put(componentNames[i], (Record) componentStates[i]);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      for (int i = 0; i < componentStates.length; i++) {
        componentStates[i] =
            (ComponentState) table.get(componentNames[i], (Record) componentStates[i]);
      }
    }
  }

  // public static <T, U, V> Object[] zipMap(T[] arg1, U[] arg2, BiFunction<T, U, V>
  // mappingFunction) {
  //   List<V> output = new ArrayList<>(Math.min(arg1.length, arg2.length));
  //   for (int i = 0; i < Math.min(arg1.length, arg2.length); i++) {
  //     output.set(i, mappingFunction.apply(arg1[i], arg2[i]));
  //   }
  //   return output.toArray();
  // }

  // public static <T, U> void zip(T[] arg1, U[] arg2, BiConsumer<T, U> mappingFunction) {
  //   for (int i = 0; i < Math.min(arg1.length, arg2.length); i++) {
  //     mappingFunction.accept(arg1[i], arg2[i]);
  //   }
  // }

  public static class MechanismConfig<Output_State extends State> {
    public String logName;
    public String tuningLogName;
    public ComponentBase[] realComponents = new ComponentBase[0];
    public ComponentControllerBase realController;
    public ComponentSimBase[] simComponents = new ComponentSimBase[0];
    public ComponentSimControllerBase simController;
    public int activeProfile = 0;
    /** An array with elements corresponding to arrays of profile classes with the same order and length as the controllers.  */
    public ControlFunctionBase[] profiles;

    public int activeFeedback = 0;
    /** An array with elements corresponding to arrays of feedback classes with the same order and length as the controllers.  */
    public ControlFunctionBase[] feedbacks;

    public Function<ComponentState[], Output_State> componentsToState;
    public double sysIdRampRate_VPs = 1;
    public double sysIdStepVoltage_V = 7;
    public double sysIdTimeout_s = 10;

    // // physical constants
    // public double moi_kgm2;
    // public double mass_kg;
    // public int canId;
    // public double reduction;
    // public DCMotor gearbox;

    // // software limits
    // public ProfileConfig<AngularPV_State> profileConfig;
    // public ComponentSimControllerBase[] componentSimControllers;
    // public ComponentControllerBase[] componentControllers;

    // public double start_rad;
    // public double start_radPs;
    // public double min_rad;
    // public double max_rad;
    // public double max_radPs;
    // public double max_radPs2;
    // public int maxStator_A;
    // public SparkMaxConfig motorConfig;

    // public SimpleMotorFeedforward realFF, simFF;
    // public PIDController realPID, simPID;
  }
}
