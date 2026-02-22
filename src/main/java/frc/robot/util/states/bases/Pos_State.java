package frc.robot.util.states.bases;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import frc.robot.util.states.State;
import frc.robot.util.states.StateUtil;
import frc.robot.util.states.StateValue;
import java.security.InvalidParameterException;

public interface Pos_State extends State {
  public abstract StateValue posValue();

  public default double pos() {
    return StateUtil.getValueAsDouble(this.posValue());
  }

  public default double pos(Unit unit) {
    return StateUtil.getValueAsDouble(this.posValue(), unit);
  }

  public static Pos_State create(StateValue... values) {
    if (values.length != 1
        || !StateUtil.isDoubleConvertable(values[0])
        || !(values[0].getUnit() == null
            || values[0].getUnit() instanceof DistanceUnit
            || values[0].getUnit() instanceof AngleUnit)) {
      // TODO: make null a valid value for unit
      throw new InvalidParameterException();
    }
    return new Pos_State() {
      public StateValue posValue() {
        return values[0];
      }

      public StateValue[] getValues() {
        return values;
      }

      public Pos_State createNew(StateValue... values) {
        return Pos_State.create(values);
      }
    };
  }
}
