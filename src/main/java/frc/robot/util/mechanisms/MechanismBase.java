package frc.robot.util.mechanisms;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.util.States.State;
import frc.robot.util.States.StateValue;
import frc.robot.util.States.Volt_State;
import frc.robot.util.SysIdUtil;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.components.bases.ComponentBase;
import frc.robot.util.components.bases.ComponentControllerBase;
import frc.robot.util.components.bases.ComponentSimBase;
import frc.robot.util.components.bases.ComponentSimControllerBase;
import frc.robot.util.components.bases.ComponentStates.ComponentState;
import frc.robot.util.control_functions.ControlFunctionBase;
import frc.robot.util.logging.LogUtil;
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

  private State[] componentStates;
  private final String[] componentNames;
  private final Function<State[], Output_State> componentsToState;
  private Output_State mechanism_State;

  private final ControlFunctionBase[] profiles;
  private final ControlFunctionBase[] feedbacks;
  private int activeProfile = 0;
  private int activeFeedback = 0;
  private Class<State> goalStateUnits;
  private Class<State> nextStateUnits;
  private Class<State> inputStateUnits;

  private final SysIdRoutine sysId;

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
      for (State state : component.getState()) {
        componentStates[i] = state;
        i++;
      }
    }
    Logger.processInputs(logName, componentStatesLogger);
    mechanism_State = componentsToState.apply(componentStates);
    for (StateValue<?> value : mechanism_State.getValues())
      Logger.recordOutput(
          logName + "/State/" + toSuffix(value.getUnit()),
          value.getClass().cast(value.getValue()));
    // Logger.recordOutput("Tuning/test", 1);
    for (ComponentSimBase simulator : simulators) {
      simulator.updateState(RobotConstants.CODE_PERIOD_s);
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
    State next_State = null;
    State input_State = null;
    for (int i = 0; i < profiles.length; i++) {
      if (i == activeProfile) {
        next_State = profiles[i].calculate(goal_State, mechanism_State);
      } else {
        profiles[i].updateState(mechanism_State);
      }
      // TODO: add check and exception for if no profile is active
    }
    for (int i = 0; i < feedbacks.length; i++) {
      if (i == activeFeedback) {
        input_State = feedbacks[i].calculate(next_State, mechanism_State);
      } else {
        feedbacks[i].updateState(mechanism_State);
      }
      // TODO: add check and exception for if no feedback is active
    }
    Logger.recordOutput(
        logName + "/Active_Profile", profiles[activeProfile].getControlFunctionName());
    Logger.recordOutput(
        logName + "/Active_Feedback", feedbacks[activeFeedback].getControlFunctionName());

    for 
    if (goalStateUnits != goal_State.getUnits()) {
      for (Unit unit : goalStateUnits) {
        Logger.recordOutput(logName + "/Goal/" + toSuffix(unit), Double.NaN);
      }
      goalStateUnits = goal_State.getUnits();
    }
    if (nextStateUnits != next_State.getUnits()) {
      for (Unit unit : nextStateUnits) {
        Logger.recordOutput(logName + "/Next/" + toSuffix(unit), Double.NaN);
      }
      nextStateUnits = next_State.getUnits();
    }
    if (inputStateUnits != input_State.getUnits()) {
      for (Unit unit : inputStateUnits) {
        Logger.recordOutput(logName + "/Input/" + toSuffix(unit), Double.NaN);
      }
      inputStateUnits = input_State.getUnits();
    }

    for (int i = 0; i < goalStateUnits.length; i++)
      Logger.recordOutput(
          logName + "/Goal" + toSuffix(goalStateUnits[i]), goal_State.getValues()[i]);
    for (int i = 0; i < nextStateUnits.length; i++)
      Logger.recordOutput(
          logName + "/Next" + toSuffix(nextStateUnits[i]), next_State.getValues()[i]);
    for (int i = 0; i < inputStateUnits.length; i++)
      Logger.recordOutput(
          logName + "/Input" + toSuffix(inputStateUnits[i]), input_State.getValues()[i]);
    controller.setInput(input_State);
  }

  protected void setVoltageSysId(Voltage V) {
    Volt_State voltage_State = new Volt_State(V.in(Volts));
    Arrays.stream(profiles).forEach(profile -> profile.updateState(mechanism_State));
    Arrays.stream(feedbacks).forEach(feedback -> feedback.updateState(mechanism_State));

    Logger.recordOutput(logName + "/Active_Profile", "SysIdRoutine");
    Logger.recordOutput(logName + "/Active_Feedback", "SysIdRoutine");
    for (int i = 0; i < goalStateUnits.length; i++)
      Logger.recordOutput(logName + "/Goal" + toSuffix(goalStateUnits[i]), Double.NaN);
    for (int i = 0; i < nextStateUnits.length; i++)
      Logger.recordOutput(logName + "/Next" + toSuffix(nextStateUnits[i]), Double.NaN);
    for (int i = 0; i < inputStateUnits.length; i++)
      Logger.recordOutput(logName + "/Input" + toSuffix(inputStateUnits[i]), Double.NaN);
    Logger.recordOutput(logName + "/Input" + toSuffix(Volts), voltage_State.V());
    controller.setInput(voltage_State);
  }

  public Command getSysIdCommand(SysIdType type) {
    return SysIdUtil.getSysIdCommand(sysId, type);
  }

  public Output_State getState() {
    return mechanism_State;
  }

  private String toSuffix(Unit unit) {
    return LogUtil.toSuffix(unit.symbol()) + "   " + unit.name();
  }

  public class ComponentStatesLogger implements LoggableInputs {
    @Override
    public void toLog(LogTable table) {
      for (int i = 0; i < componentStates.length; i++) {
        // make this work for everything compatable with advantagekit
        if (StructSerializable.class.isAssignableFrom(componentStates[i].getClass())) {
          table.put(componentNames[i], (StructSerializable) componentStates[i]);
        } else if (Record.class.isAssignableFrom(componentStates[i].getClass())) {
          table.put(componentNames[i], (Record) componentStates[i]);
        }
      }
    }

    @Override
    public void fromLog(LogTable table) {
      for (int i = 0; i < componentStates.length; i++) {
        if (StructSerializable.class.isAssignableFrom(componentStates[i].getClass())) {
          componentStates[i] =
              (State) table.get(componentNames[i], (StructSerializable) componentStates[i]);
        } else if (Record.class.isAssignableFrom(componentStates[i].getClass())) {
          componentStates[i] = (State) table.get(componentNames[i], (Record) componentStates[i]);
        }
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

  public static class MechanismConfig<Output_State> {
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

    public Function<State[], Output_State> componentsToState;
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
