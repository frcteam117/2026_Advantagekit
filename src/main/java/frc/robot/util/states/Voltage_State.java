package frc.robot.util.states;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import java.security.InvalidParameterException;

public class Voltage_State implements State {
  private final StateValue value;

  public Voltage_State(double V) {
    this(V, "", Volts);
  }

  public Voltage_State(double V, String logName) {
    this(V, logName, Volts);
  }

  public Voltage_State(double V, VoltageUnit unit) {
    this(V, "", unit);
  }

  public Voltage_State(double V, String logName, VoltageUnit unit) {
    value = new StateValue(V, logName, unit);
  }

  public Voltage_State(StateValue... values) {
    if (!(values[0].getUnit() instanceof VoltageUnit)) {
      throw new InvalidParameterException(
          values[0].getUnit().toString() + " is not a valid unit of voltage.");
    }
    if (!StateUtil.isDoubleConvertable(values[0])) {
      throw new InvalidParameterException(
          values[0].getValue().toString() + " cannot be converted to a double.");
    }
    value = values[0];
  }

  public double V() {
    return StateUtil.getValueAsDouble(value);
  }

  @Override
  public StateValue[] getValues() {
    return new StateValue[] {value};
  }

  @Override
  public Voltage_State createNew(StateValue... values) {
    return new Voltage_State(values);
  }
}
