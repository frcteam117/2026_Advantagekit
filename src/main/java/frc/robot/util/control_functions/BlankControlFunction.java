package frc.robot.util.control_functions;

import frc.robot.util.States.State;

public class BlankControlFunction extends ControlFunctionBase {
  private final String name;

  public BlankControlFunction() {
    name = "BlankControlFunction";
  }

  public BlankControlFunction(String name) {
    this.name = name;
  }

  public State calculate(State goal_State, State mechanism_State) {
    return goal_State;
  }

  public void updateState(State mechanism_State) {}

  public void resetState(State mechanism_State) {}

  @Override
  public String getControlFunctionName() {
    return name;
  }

  @Override
  public void setTunable(boolean isTunable) {}
}
