package frc.robot.util.components.bases;

import frc.robot.util.states.State;

public interface ComponentBase {
  public abstract String[] getComponentNames();

  public abstract State[] getState();
}
