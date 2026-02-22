package frc.robot.util.states.bases;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.Unit;
import frc.robot.util.states.State;
import frc.robot.util.states.StateUtil;
import frc.robot.util.states.StateValue;
import java.security.InvalidParameterException;

public interface Acc_State extends State {
  public abstract StateValue accValue();

  public default double acc() {
    return StateUtil.getValueAsDouble(this.accValue());
  }

  /** Gets the acceleration value in the unit given if possible. Otherwise returns the value as a double */
  public default double acc(Unit unit) {
    return StateUtil.getValueAsDouble(this.accValue(), unit);
  }

  public static Acc_State create(StateValue... values) {
    if (values.length != 1
        || !StateUtil.isDoubleConvertable(values[0])
        || !(values[0].getUnit() == null
            || values[0].getUnit() instanceof LinearAccelerationUnit
            || values[0].getUnit() instanceof AngularAccelerationUnit)) {
      // TODO: make null a valid value for unit
      throw new InvalidParameterException();
    }
    return new Acc_State() {
      public StateValue accValue() {
        return values[0];
      }

      public StateValue[] getValues() {
        return values;
      }

      public Acc_State createNew(StateValue... values) {
        return Acc_State.create(values);
      }
    };
  }
}
