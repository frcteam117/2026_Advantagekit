package frc.robot.util.states.bases;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Unit;
import frc.robot.util.states.State;
import frc.robot.util.states.StateUtil;
import frc.robot.util.states.StateValue;
import java.security.InvalidParameterException;

public interface Vel_State extends State {
  public abstract StateValue velValue();

  public default double vel() {
    return StateUtil.getValueAsDouble(this.velValue());
  }

  public default double vel(Unit unit) {
    return StateUtil.getValueAsDouble(this.velValue(), unit);
  }

  public static Vel_State create(StateValue... values) {
    if (values.length != 1
        || !StateUtil.isDoubleConvertable(values[0])
        || !(values[0].getUnit() == null
            || values[0].getUnit() instanceof LinearVelocityUnit
            || values[0].getUnit() instanceof AngularVelocityUnit)) {
      // TODO: make null a valid value for unit
      throw new InvalidParameterException();
    }
    return new Vel_State() {
      public StateValue velValue() {
        return values[0];
      }

      public StateValue[] getValues() {
        return values;
      }

      public Vel_State createNew(StateValue... values) {
        return Vel_State.create(values);
      }
    };
  }
}
