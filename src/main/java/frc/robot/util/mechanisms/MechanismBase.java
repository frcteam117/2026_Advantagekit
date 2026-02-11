package frc.robot.util.mechanisms;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.util.StateUtil;
import frc.robot.util.StateUtil.BaseState;
import frc.robot.util.StateUtil.State;
import frc.robot.util.StateUtil.StateValue;
import frc.robot.util.StateUtil.Volt_State;
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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

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
  private State goal_State = new BaseState();
  private State next_State = new BaseState();
  private State input_State = new BaseState();

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
    StateUtil.recordOutput(logName + "/0_State/", mechanism_State);
    for (ComponentSimBase simulator : simulators) {
      simulator.updateState(RobotConstants.CODE_PERIOD_s);
    }
  }

  public void setGoal(State goalState, int usedProfile, int usedFeedback) {
    activeProfile = usedProfile;
    activeFeedback = usedFeedback;
    setGoal(goalState);
  }

  public void setGoal(State goal_State) {
    final State oldNext_State = next_State;
    final State oldInput_State = input_State;

    // Logging goal_State
    StateUtil.overrideOutput(logName + "/1_Goal/", goal_State, this.goal_State);
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
    StateUtil.overrideOutput(logName + "/2_Next/", next_State, oldNext_State);

    // Calculating and logging input_State
    for (int i = 0; i < feedbacks.length; i++) {
      if (i == activeFeedback) {
        input_State = feedbacks[i].calculate(next_State, mechanism_State);
      } else {
        feedbacks[i].updateState(mechanism_State);
      }
    }
    Logger.recordOutput(logName + "/_Feedback", feedbacks[activeFeedback].getControlFunctionName());
    StateUtil.overrideOutput(logName + "/3_Input/", input_State, oldInput_State);

    // Giving input_State to the controller
    controller.setInput(input_State);
  }

  protected void setVoltageSysId(Voltage V) {
    Volt_State voltage_State = new Volt_State(V.in(Volts));
    Arrays.stream(profiles).forEach(profile -> profile.updateState(mechanism_State));
    Arrays.stream(feedbacks).forEach(feedback -> feedback.updateState(mechanism_State));
    Logger.recordOutput(logName + "/_Profile", "SysIdRoutine");
    Logger.recordOutput(logName + "/_Feedback", "SysIdRoutine");
    StateUtil.recordNullOutput(logName + "/1_Goal/", goal_State);
    StateUtil.recordNullOutput(logName + "/2_Next/", next_State);
    StateUtil.overrideOutput(logName + "/3_Input/", voltage_State, input_State);
    controller.setInput(voltage_State);
  }

  public Command getSysIdCommand(SysIdType type) {
    return SysIdUtil.getSysIdCommand(sysId, type);
  }

  public Output_State getState() {
    return mechanism_State;
  }

  // private String toSuffix(Unit unit) {
  //   return LogUtil.toSuffix(unit.symbol()) + "  " + unit.name();
  // }

  public class ComponentStatesLogger implements LoggableInputs {
    @SuppressWarnings("unchecked")
    @Override
    public void toLog(LogTable table) {
      for (int i = 0; i < componentStates.length; i++) {
        for (StateValue value : componentStates[i].getValues()) {
          String newKey = componentNames[i] + "/" + StateUtil.toSuffix(value);
          if (value.getValue() instanceof byte[]) {
            table.put(newKey, (byte[]) value.getValue());
          } else if (value.getValue() instanceof byte[][]) {
            table.put(newKey, (byte[][]) value.getValue());
          } else if (value.getValue() instanceof Boolean) {
            table.put(newKey, (Boolean) value.getValue());
          } else if (value.getValue() instanceof BooleanSupplier) {
            table.put(newKey, ((BooleanSupplier) value.getValue()).getAsBoolean());
          } else if (value.getValue() instanceof boolean[]) {
            table.put(newKey, (boolean[]) value.getValue());
          } else if (value.getValue() instanceof boolean[][]) {
            table.put(newKey, (boolean[][]) value.getValue());
          } else if (value.getValue() instanceof Integer) {
            if (value.getUnit() == null) {
              table.put(newKey, (Integer) value.getValue());
            } else {
              table.put(newKey, (Integer) value.getValue(), value.getUnit().name());
            }
          } else if (value.getValue() instanceof IntSupplier) {
            table.put(newKey, ((IntSupplier) value.getValue()).getAsInt());
          } else if (value.getValue() instanceof int[]) {
            table.put(newKey, (int[]) value.getValue());
          } else if (value.getValue() instanceof int[][]) {
            table.put(newKey, (int[][]) value.getValue());
          } else if (value.getValue() instanceof Long) {
            if (value.getUnit() == null) {
              table.put(newKey, (Long) value.getValue());
            } else {
              table.put(newKey, (Long) value.getValue(), value.getUnit().name());
            }
          } else if (value.getValue() instanceof LongSupplier) {
            table.put(newKey, ((LongSupplier) value.getValue()).getAsLong());
          } else if (value.getValue() instanceof long[]) {
            table.put(newKey, (long[]) value.getValue());
          } else if (value.getValue() instanceof long[][]) {
            table.put(newKey, (long[][]) value.getValue());
          } else if (value.getValue() instanceof Float) {
            if (value.getUnit() == null) {
              table.put(newKey, (Float) value.getValue());
            } else {
              table.put(newKey, (Float) value.getValue(), value.getUnit().name());
            }
          } else if (value.getValue() instanceof float[]) {
            table.put(newKey, (float[]) value.getValue());
          } else if (value.getValue() instanceof float[][]) {
            table.put(newKey, (float[][]) value.getValue());
          } else if (value.getValue() instanceof Double) {
            if (value.getUnit() == null) {
              table.put(newKey, (Double) value.getValue());
            } else {
              table.put(newKey, (Double) value.getValue(), value.getUnit().name());
            }
          } else if (value.getValue() instanceof DoubleSupplier) {
            table.put(newKey, ((DoubleSupplier) value.getValue()).getAsDouble());
          } else if (value.getValue() instanceof double[]) {
            table.put(newKey, (double[]) value.getValue());
          } else if (value.getValue() instanceof double[][]) {
            table.put(newKey, (double[][]) value.getValue());
          } else if (value.getValue() instanceof String) {
            table.put(newKey, (String) value.getValue());
          } else if (value.getValue() instanceof String[]) {
            table.put(newKey, (String[]) value.getValue());
          } else if (value.getValue() instanceof String[][]) {
            table.put(newKey, (String[][]) value.getValue());
          } else if (value.getValue() instanceof Enum) {
            table.put(newKey, (Enum.class.cast(value.getValue())));
          } else if (value.getValue() instanceof Enum[]) {
            table.put(newKey, (Enum[]) value.getValue());
          } else if (value.getValue() instanceof Enum[][]) {
            table.put(newKey, (Enum[][]) value.getValue());
          } else if (value.getValue() instanceof Measure) {
            if (value.getUnit() == null) {
              table.put(newKey, (Measure<?>) value.getValue());
            } else {
              table.put(
                  newKey,
                  ((Measure<?>) value.getValue()).in(value.getUnit()),
                  value.getUnit().name());
            }
          } else if (value.getValue() instanceof WPISerializable) {
            table.put(newKey, (WPISerializable) value.getValue());
          } else if (value.getValue() instanceof StructSerializable) {
            table.put(newKey, (StructSerializable) value.getValue());
          } else if (value.getValue() instanceof StructSerializable[]) {
            table.put(newKey, (StructSerializable[]) value.getValue());
          } else if (value.getValue() instanceof StructSerializable[][]) {
            table.put(newKey, (StructSerializable[][]) value.getValue());
          } else if (value.getValue() instanceof Record) {
            table.put(newKey, (Record) value.getValue());
          } else if (value.getValue() instanceof Record[]) {
            table.put(newKey, (Record[]) value.getValue());
          } else if (value.getValue() instanceof Record[][]) {
            table.put(newKey, (Record[][]) value.getValue());
          } else if (value.getValue() instanceof LoggedMechanism2d) {
            return;
          } else if (value.getValue() instanceof Color) {
            table.put(newKey, (Color) value.getValue());
          }
        }
      }
    }

    @SuppressWarnings("unchecked")
    @Override
    public void fromLog(LogTable table) {
      for (int i = 0; i < componentStates.length; i++) {
        StateValue[] values = componentStates[i].getValues();
        for (int j = 0; j < values.length; j++) {
          String newKey = componentNames[i] + "/" + StateUtil.toSuffix(values[j]);
          if (values[j].getValue() instanceof byte[]) {
            values[j] = new StateValue(
                table.get(newKey, (byte[]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof byte[][]) {
            values[j] = new StateValue(
                table.get(newKey, (byte[][]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof Boolean) {
            values[j] = new StateValue(
                table.get(newKey, (Boolean) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof BooleanSupplier) {
            boolean value =
                table.get(newKey, ((BooleanSupplier) values[j].getValue()).getAsBoolean());
            values[j] = new StateValue(
                (BooleanSupplier) () -> value, values[j].getName(), values[j].getUnit());
          } else if (values[j].getValue() instanceof boolean[]) {
            values[j] = new StateValue(
                table.get(newKey, (boolean[]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof boolean[][]) {
            values[j] = new StateValue(
                table.get(newKey, (boolean[][]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof Integer) {
            values[j] = new StateValue(
                table.get(newKey, (Integer) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof IntSupplier) {
            int value = table.get(newKey, ((IntSupplier) values[j].getValue()).getAsInt());
            values[j] =
                new StateValue((IntSupplier) () -> value, values[j].getName(), values[j].getUnit());
          } else if (values[j].getValue() instanceof int[]) {
            values[j] = new StateValue(
                table.get(newKey, (int[]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof int[][]) {
            values[j] = new StateValue(
                table.get(newKey, (int[][]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof Long) {
            values[j] = new StateValue(
                table.get(newKey, (Long) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof LongSupplier) {
            long value = table.get(newKey, ((LongSupplier) values[j].getValue()).getAsLong());
            values[j] = new StateValue(
                (LongSupplier) () -> value, values[j].getName(), values[j].getUnit());
          } else if (values[j].getValue() instanceof long[]) {
            values[j] = new StateValue(
                table.get(newKey, (long[]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof long[][]) {
            values[j] = new StateValue(
                table.get(newKey, (long[][]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof Float) {
            values[j] = new StateValue(
                table.get(newKey, (Float) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof float[]) {
            values[j] = new StateValue(
                table.get(newKey, (float[]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof float[][]) {
            values[j] = new StateValue(
                table.get(newKey, (float[][]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof Double) {
            values[j] = new StateValue(
                table.get(newKey, (Double) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof DoubleSupplier) {
            double value = table.get(newKey, ((DoubleSupplier) values[j].getValue()).getAsDouble());
            values[j] = new StateValue(
                (DoubleSupplier) () -> value, values[j].getName(), values[j].getUnit());
          } else if (values[j].getValue() instanceof double[]) {
            values[j] = new StateValue(
                table.get(newKey, (double[]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof double[][]) {
            values[j] = new StateValue(
                table.get(newKey, (double[][]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof String) {
            values[j] = new StateValue(
                table.get(newKey, (String) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof String[]) {
            values[j] = new StateValue(
                table.get(newKey, (String[]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof String[][]) {
            values[j] = new StateValue(
                table.get(newKey, (String[][]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof Enum) {
            values[j] = new StateValue(
                table.get(newKey, (Enum.class.cast(values[j].getValue()))),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof Enum[]) {
            values[j] = new StateValue(
                table.get(newKey, (Enum[]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof Enum[][]) {
            values[j] = new StateValue(
                table.get(newKey, (Enum[][]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof Measure) {
            if (values[j].getUnit() == null) {
              values[j] = new StateValue(
                  table.get(newKey, (Measure<?>) values[j].getValue()),
                  values[j].getName(),
                  values[j].getUnit());
            } else {
              values[j] = new StateValue(
                  table.get(newKey, ((Measure<?>) values[j].getValue()).in(values[j].getUnit())),
                  values[j].getName(),
                  values[j].getUnit());
            }
          } else if (values[j].getValue() instanceof WPISerializable) {
            values[j] = new StateValue(
                table.get(newKey, (WPISerializable) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof StructSerializable) {
            values[j] = new StateValue(
                table.get(newKey, (StructSerializable) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof StructSerializable[]) {
            values[j] = new StateValue(
                table.get(newKey, (StructSerializable[]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof StructSerializable[][]) {
            values[j] = new StateValue(
                table.get(newKey, (StructSerializable[][]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof Record) {
            values[j] = new StateValue(
                table.get(newKey, (Record) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof Record[]) {
            values[j] = new StateValue(
                table.get(newKey, (Record[]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof Record[][]) {
            values[j] = new StateValue(
                table.get(newKey, (Record[][]) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          } else if (values[j].getValue() instanceof LoggedMechanism2d) {
            return;
          } else if (values[j].getValue() instanceof Color) {
            values[j] = new StateValue(
                table.get(newKey, (Color) values[j].getValue()),
                values[j].getName(),
                values[j].getUnit());
          }
        }
        componentStates[i] = componentStates[i].create(values);
      }
    }
  }

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
  }
}
