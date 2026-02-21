package frc.robot.util.states.premade;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import frc.robot.util.states.StateUtil;
import frc.robot.util.states.StateValue;
import frc.robot.util.states.bases.Pos_State;

public class RadPos_State implements Pos_State {
  private final StateValue value;

  public RadPos_State(double rad) {
    value = new StateValue(rad, Radians);
  }

  public RadPos_State(double rad, String name) {
    value = new StateValue(rad, name, Radians);
  }

  public RadPos_State(Measure<AngleUnit> pos) {
    value = new StateValue(pos, RadiansPerSecond);
  }

  public RadPos_State(Measure<AngleUnit> pos, String name) {
    value = new StateValue(pos, name, RadiansPerSecond);
  }

  public StateValue posValue() {
    return value;
  }

  public StateValue[] getValues() {
    return new StateValue[] {value};
  }

  public RadPos_State createNew(StateValue... values) {
    return new RadPos_State(StateUtil.getValueAsDouble(values[0], Radians), values[0].getName());
  }
}
