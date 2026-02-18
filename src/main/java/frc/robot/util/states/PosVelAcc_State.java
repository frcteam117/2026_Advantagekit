package frc.robot.util.states;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import java.security.InvalidParameterException;

public interface PosVelAcc_State extends PosVel_State, PosAcc_State, VelAcc_State {
  public static PosVelAcc_State create(StateValue... values) {
    if (values.length != 3
        || !StateUtil.isDoubleConvertable(values[0])
        || !(values[0].getUnit() == null
            || values[0].getUnit() instanceof DistanceUnit
            || values[0].getUnit() instanceof AngleUnit)
        || !StateUtil.isDoubleConvertable(values[1])
        || !(values[1].getUnit() == null
            || values[1].getUnit() instanceof LinearVelocityUnit
            || values[1].getUnit() instanceof AngularVelocityUnit)
        || !StateUtil.isDoubleConvertable(values[2])
        || !(values[2].getUnit() == null
            || values[2].getUnit() instanceof LinearAccelerationUnit
            || values[2].getUnit() instanceof AngularAccelerationUnit)) {
      // TODO: make null a valid value for unit
      throw new InvalidParameterException();
    }
    return new PosVelAcc_State() {
      public StateValue posValue() {
        return values[0];
      }

      public StateValue velValue() {
        return values[1];
      }

      public StateValue accValue() {
        return values[2];
      }

      public StateValue[] getValues() {
        return values;
      }

      public PosVelAcc_State createNew(StateValue... values) {
        return PosVelAcc_State.create(values);
      }
    };
  }
}
