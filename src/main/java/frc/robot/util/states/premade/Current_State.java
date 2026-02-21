package frc.robot.util.states.premade;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.CurrentUnit;
import frc.robot.util.states.State;
import frc.robot.util.states.StateUtil;
import frc.robot.util.states.StateValue;
import java.security.InvalidParameterException;

public class Current_State implements State {
  private final StateValue value;

  public Current_State(double A) {
    this(A, "", Amps);
  }

  public Current_State(double A, String logName) {
    this(A, logName, Amps);
  }

  public Current_State(double A, CurrentUnit unit) {
    this(A, "", unit);
  }

  public Current_State(double A, String logName, CurrentUnit unit) {
    value = new StateValue(A, logName, unit);
  }

  public Current_State(StateValue... values) {
    if (!(values[0].getUnit() instanceof CurrentUnit)) {
      throw new InvalidParameterException(
          values[0].getUnit().toString() + " is not a valid unit of current.");
    }
    if (!StateUtil.isDoubleConvertable(values[0])) {
      throw new InvalidParameterException(
          values[0].getValue().toString() + " cannot be converted to a double.");
    }
    value = values[0];
  }

  public double A() {
    return StateUtil.getValueAsDouble(value);
  }

  @Override
  public StateValue[] getValues() {
    return new StateValue[] {value};
  }

  @Override
  public Current_State createNew(StateValue... values) {
    return new Current_State(values);
  }
}
