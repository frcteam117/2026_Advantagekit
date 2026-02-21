package frc.robot.util.states.premade;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import frc.robot.util.states.StateUtil;
import frc.robot.util.states.StateValue;
import frc.robot.util.states.bases.PosVel_State;

public class RadPosVel_State implements PosVel_State {
  private final StateValue[] values;

  public RadPosVel_State(double rad, double radPs) {
    values =
        new StateValue[] {new StateValue(rad, Radians), new StateValue(radPs, RadiansPerSecond)};
  }

  public RadPosVel_State(double rad, String posName, double radPs, String velName) {
    values = new StateValue[] {
      new StateValue(rad, posName, Radians), new StateValue(radPs, velName, RadiansPerSecond)
    };
  }

  public RadPosVel_State(Measure<AngleUnit> pos, Measure<AngularVelocityUnit> vel) {
    values = new StateValue[] {new StateValue(pos), new StateValue(vel)};
  }

  public RadPosVel_State(
      Measure<AngleUnit> pos, String posName, Measure<AngularVelocityUnit> vel, String velName) {
    values = new StateValue[] {new StateValue(pos, posName), new StateValue(vel, velName)};
  }

  public StateValue posValue() {
    return values[0];
  }

  public StateValue velValue() {
    return values[1];
  }

  public StateValue[] getValues() {
    return values;
  }

  public RadPosVel_State createNew(StateValue... values) {
    return new RadPosVel_State(
        StateUtil.getValueAsDouble(values[0], Radians),
        values[0].getName(),
        StateUtil.getValueAsDouble(values[1], RadiansPerSecond),
        values[1].getName());
  }
}
