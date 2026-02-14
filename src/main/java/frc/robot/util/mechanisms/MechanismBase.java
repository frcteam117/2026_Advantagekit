package frc.robot.util.mechanisms;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.util.SysIdUtil;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.components.bases.ComponentBase;
import frc.robot.util.components.bases.ComponentControllerBase;
import frc.robot.util.components.bases.ComponentSimBase;
import frc.robot.util.components.bases.ComponentSimControllerBase;
import frc.robot.util.components.bases.ComponentStates.ComponentState;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.TunableBoolean;
import frc.robot.util.states.LoggableStateInputs;
import frc.robot.util.states.State;
import frc.robot.util.states.StateUtil;
import frc.robot.util.states.Voltage_State;
import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class MechanismBase<Output_State extends State> {
  public static String mechanismStateLogName = "0_State";
  public static String goalStateLogName = "1_Goal";
  public static String nextStateLogName = "2_Next";
  public static String inputStateLogName = "3_Input";

  private final List<ComponentBase> components;
  private final List<ComponentSimBase> simulators;
  private final ComponentControllerBase controller;

  private State[] componentStates;
  private final String[] componentNames;
  private final Function<State[], Output_State> componentsToState;
  private Output_State mechanism_State;

  private final ControlFunctionBase[] profiles;
  private final ControlFunctionBase[] feedbacks;
  private int activeProfile = 0;
  private int activeFeedback = 0;
  private State goal_State = State.create();
  private State next_State = State.create();
  private State input_State = State.create();

  private final SysIdRoutine sysId;

  private final String logName;
  private final ComponentStatesLogger componentStatesLogger = new ComponentStatesLogger();

  public MechanismBase(MechanismConfig<Output_State> config, Subsystem subsystem) {
    mechanism_State = config.constants.start_State;
    logName = config.constants.outputsLogName;
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

    new TunableBoolean(config.constants.tuningLogName + "/.Tunable", false, () -> true, value -> {
      Arrays.stream(profiles).forEach(profile -> profile.setTunable(value));
      Arrays.stream(feedbacks).forEach(feedback -> feedback.setTunable(value));
    });
  }

  public void update() {
    int i = 0;
    for (ComponentBase component : components) {
      for (State state : component.getState()) {
        componentStates[i] = state;
        i++;
      }
    }
    Logger.processInputs(logName, componentStatesLogger);
    mechanism_State = componentsToState.apply(componentStates);
    StateUtil.recordOutput(logName + "/" + mechanismStateLogName, mechanism_State);
    for (ComponentSimBase simulator : simulators) {
      simulator.updateState(RobotConstants.CODE_PERIOD_s);
    }
  }

  public void setGoal(State goalState, int usedProfile, int usedFeedback) {
    if (profiles.length <= usedProfile
        || usedProfile < 0
        || feedbacks.length <= usedFeedback
        || usedFeedback < 0) {
      throw new InvalidParameterException();
    }
    activeProfile = usedProfile;
    activeFeedback = usedFeedback;
    setGoal(goalState);
  }

  public void setGoal(State goal_State) {
    final State oldNext_State = next_State;
    final State oldInput_State = input_State;

    // Logging goal_State
    StateUtil.overrideOutput(logName + "/" + goalStateLogName, goal_State, this.goal_State);
    this.goal_State = goal_State;

    // Calculating and logging next_State
    for (int i = 0; i < profiles.length; i++) {
      if (i == activeProfile) {
        next_State = profiles[i].calculate(goal_State, mechanism_State);
      } else {
        profiles[i].updateState(mechanism_State);
      }
    }
    Logger.recordOutput(logName + "/_Profile", profiles[activeProfile].getControlFunctionName());
    StateUtil.overrideOutput(logName + "/" + nextStateLogName, next_State, oldNext_State);

    // Calculating and logging input_State
    for (int i = 0; i < feedbacks.length; i++) {
      if (i == activeFeedback) {
        input_State = feedbacks[i].calculate(next_State, mechanism_State);
      } else {
        feedbacks[i].updateState(mechanism_State);
      }
    }
    Logger.recordOutput(logName + "/_Feedback", feedbacks[activeFeedback].getControlFunctionName());
    StateUtil.overrideOutput(logName + "/" + inputStateLogName, input_State, oldInput_State);

    // Giving input_State to the controller
    controller.setInput(input_State);
  }

  protected void setVoltageSysId(Voltage V) {
    Voltage_State voltage_State = new Voltage_State(V.in(Volts));
    Arrays.stream(profiles).forEach(profile -> profile.updateState(mechanism_State));
    Arrays.stream(feedbacks).forEach(feedback -> feedback.updateState(mechanism_State));
    Logger.recordOutput(logName + "/_Profile", "SysIdRoutine");
    Logger.recordOutput(logName + "/_Feedback", "SysIdRoutine");
    StateUtil.recordNullOutput(logName + "/" + goalStateLogName, goal_State);
    StateUtil.recordNullOutput(logName + "/" + nextStateLogName, next_State);
    StateUtil.overrideOutput(logName + "/" + inputStateLogName, voltage_State, input_State);
    controller.setInput(voltage_State);
  }

  public Command getSysIdCommand(SysIdType type) {
    return SysIdUtil.getSysIdCommand(sysId, type);
  }

  public Output_State getState() {
    return mechanism_State;
  }

  public class ComponentStatesLogger implements LoggableStateInputs {
    @Override
    public State[] getStates() {
      return componentStates;
    }

    @Override
    public void setStates(State[] states) {
      componentStates = states;
    }

    @Override
    public String[] getStateNames() {
      return componentNames;
    }
  }

  public static class MechanismConfig<Output_State extends State> {
    /** outputsLogName, tuningLogName, and start_State must be defined */
    public MechanismConstants<Output_State> constants;

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

    public Output_State starting_State;

    public Function<State[], Output_State> componentsToState;
    public double sysIdRampRate_VPs = 1;
    public double sysIdStepVoltage_V = 7;
    public double sysIdTimeout_s = 10;
  }
}
