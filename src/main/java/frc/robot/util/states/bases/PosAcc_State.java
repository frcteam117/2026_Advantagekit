package frc.robot.util.states.bases;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import frc.robot.util.states.StateUtil;
import frc.robot.util.states.StateValue;
import java.security.InvalidParameterException;

public interface PosAcc_State extends Pos_State, Acc_State {
  public static PosAcc_State create(StateValue... values) {
    if (values.length != 2
        || !StateUtil.isDoubleConvertable(values[0])
        || !(values[0].getUnit() == null
            || values[0].getUnit() instanceof DistanceUnit
            || values[0].getUnit() instanceof AngleUnit)
        || !StateUtil.isDoubleConvertable(values[1])
        || !(values[1].getUnit() == null
            || values[1].getUnit() instanceof LinearAccelerationUnit
            || values[1].getUnit() instanceof AngularAccelerationUnit)) {
      // TODO: make null a valid value for unit
      throw new InvalidParameterException();
    }
    return new PosAcc_State() {
      public StateValue posValue() {
        return values[0];
      }

      public StateValue accValue() {
        return values[1];
      }

      public StateValue[] getValues() {
        return values;
      }

      public PosAcc_State createNew(StateValue... values) {
        return PosAcc_State.create(values);
      }
    };
  }
}
