package frc.robot.util.states.bases;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import frc.robot.util.states.StateUtil;
import frc.robot.util.states.StateValue;
import java.security.InvalidParameterException;

public interface VelAcc_State extends Vel_State, Acc_State {
  public static VelAcc_State create(StateValue... values) {
    if (values.length != 2
        || !StateUtil.isDoubleConvertable(values[0])
        || !(values[0].getUnit() == null
            || values[0].getUnit() instanceof LinearVelocityUnit
            || values[0].getUnit() instanceof AngularVelocityUnit)
        || !StateUtil.isDoubleConvertable(values[1])
        || !(values[1].getUnit() == null
            || values[1].getUnit() instanceof LinearAccelerationUnit
            || values[1].getUnit() instanceof AngularAccelerationUnit)) {
      // TODO: make null a valid value for unit
      throw new InvalidParameterException();
    }
    return new VelAcc_State() {
      public StateValue velValue() {
        return values[0];
      }

      public StateValue accValue() {
        return values[1];
      }

      public StateValue[] getValues() {
        return values;
      }

      public VelAcc_State createNew(StateValue... values) {
        return VelAcc_State.create(values);
      }
    };
  }
}
