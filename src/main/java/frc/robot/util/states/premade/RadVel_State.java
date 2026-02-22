package frc.robot.util.states.premade;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import frc.robot.util.states.StateUtil;
import frc.robot.util.states.StateValue;
import frc.robot.util.states.bases.Vel_State;

public class RadVel_State implements Vel_State {
  private final StateValue value;

  public RadVel_State(double radPs) {
    value = new StateValue(radPs, RadiansPerSecond);
  }

  public RadVel_State(double radPs, String name) {
    value = new StateValue(radPs, name, RadiansPerSecond);
  }

  public RadVel_State(Measure<AngularVelocityUnit> vel) {
    value = new StateValue(vel, RadiansPerSecond);
  }

  public RadVel_State(Measure<AngularVelocityUnit> vel, String name) {
    value = new StateValue(vel, name, RadiansPerSecond);
  }

  public StateValue velValue() {
    return value;
  }

  public StateValue[] getValues() {
    return new StateValue[] {value};
  }

  public RadVel_State createNew(StateValue... values) {
    return new RadVel_State(
        StateUtil.getValueAsDouble(values[0], RadiansPerSecond), values[0].getName());
  }
}
