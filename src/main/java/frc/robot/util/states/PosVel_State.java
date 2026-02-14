package frc.robot.util.states;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import java.security.InvalidParameterException;

public interface PosVel_State extends Pos_State, Vel_State {
  public static PosVel_State create(StateValue... values) {
    if (values.length != 2
        || !StateUtil.isDoubleConvertable(values[0])
        || !(values[0].getUnit() == null
            || values[0].getUnit() instanceof DistanceUnit
            || values[0].getUnit() instanceof AngleUnit)
        || !StateUtil.isDoubleConvertable(values[1])
        || !(values[1].getUnit() == null
            || values[1].getUnit() instanceof LinearVelocityUnit
            || values[1].getUnit() instanceof AngularVelocityUnit)) {
      // TODO: make null a valid value for unit
      throw new InvalidParameterException();
    }
    return new PosVel_State() {
      public StateValue posValue() {
        return values[0];
      }

      public StateValue velValue() {
        return values[1];
      }

      public StateValue[] getValues() {
        return values;
      }

      public PosVel_State createNew(StateValue... values) {
        return PosVel_State.create(values);
      }
    };
  }
}
